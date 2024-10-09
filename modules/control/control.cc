/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/control/control.h"

#include <iomanip>
#include <string>

#include "ros/include/std_msgs/String.h"

#include "modules/localization/proto/localization.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

std::string Control::Name() const { return FLAGS_control_node_name; }

Status Control::Init() {
  // 获取初始化时间
  init_time_ = Clock::NowInSeconds();

  // 输出日志，表示控制模块正在初始化
  AINFO << "Control init, starting ...";

  // 从配置文件加载控制配置，若加载失败则抛出错误信息
  CHECK(common::util::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;

  // 输出日志，表示配置文件已成功加载
  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";

  // 初始化适配器管理器
  AdapterManager::Init(FLAGS_control_adapter_config_filename);

  // 创建一个监控日志缓冲区，用于记录系统的运行状态
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

   // set controller // 设置控制器
  if (!controller_agent_.Init(&control_conf_).ok()) {
    // 如果控制器初始化失败，记录错误信息并返回错误状态
    std::string error_msg = "Control init controller failed! Stopping...";
    buffer.ERROR(error_msg);
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }

  // lock it in case for after sub, init_vehicle not ready, but msg trigger come// 检查各模块是否已初始化，防止在车辆状态未就绪时被触发
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";  // 检查定位模块
  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";  // 检查底盘模块
  CHECK(AdapterManager::GetPlanning()) << "Planning is not initialized.";  // 检查规划模块
  CHECK(AdapterManager::GetPad()) << "Pad is not initialized.";  // 检查控制板模块
  CHECK(AdapterManager::GetMonitor()) << "Monitor is not initialized.";  // 检查监控模块
  CHECK(AdapterManager::GetControlCommand())
      << "ControlCommand publisher is not initialized.";  // 检查控制命令发布器

  // 注册控制板回调函数，用于响应控制板的输入
  AdapterManager::AddPadCallback(&Control::OnPad, this);
  // 注册监控回调函数，用于监控系统状态
  AdapterManager::AddMonitorCallback(&Control::OnMonitor, this);

  // 返回初始化成功的状态
  return Status::OK();
}

Status Control::Start() {
  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  // 重置车辆状态，通过发送命令来设置初始状态
  // 由于通道广播的延迟，设置短暂的睡眠时间（大约80毫秒延迟），此处延长至1000毫秒以确保通道就绪
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // should init_vehicle first, let car enter work status, then use status msg
  // trigger control
  // 应先初始化车辆状态，使车辆进入工作状态，之后通过状态消息触发控制
  AINFO << "Control default driving action is "
        << DrivingAction_Name(control_conf_.action());

  // 设置默认驾驶行为为配置文件中定义的操作
  pad_msg_.set_action(control_conf_.action());

  // 创建定时器，按照控制配置中的周期触发控制逻辑
  timer_ = AdapterManager::CreateTimer(
      ros::Duration(control_conf_.control_period()), &Control::OnTimer, this);

  // 输出日志，表示控制模块初始化完成
  AINFO << "Control init done!";

  // 记录控制模块已启动的信息到监控日志中
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("control started");

  // 返回启动成功的状态
  return Status::OK();
}

void Control::OnPad(const PadMessage &pad) {
  pad_msg_ = pad;
  ADEBUG << "Received Pad Msg:" << pad.DebugString();
  AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";

  // do something according to pad message
  if (pad_msg_.action() == DrivingAction::RESET) {
    AINFO << "Control received RESET action!";
    estop_ = false;
  }
  pad_received_ = true;
}

void Control::OnMonitor(
    const common::monitor::MonitorMessage &monitor_message) {
  for (const auto &item : monitor_message.item()) {
    if (item.log_level() == MonitorMessageItem::FATAL) {
      estop_ = true;
      return;
    }
  }
}

Status Control::ProduceControlCommand(ControlCommand *control_command) {
  // 检查输入数据的状态
  Status status = CheckInput();

  // check data // 如果输入数据有问题，设置错误信息并停止
  if (!status.ok()) {
    AERROR_EVERY(100) << "Control input data failed: "
                      << status.error_message();
    control_command->mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);  // 禁止车辆进入自动驾驶
    control_command->mutable_engage_advice()->set_reason(
        status.error_message());  // 设置失败的原因
    estop_ = true;  // 触发紧急停止
  } else {
    // 检查时间戳的有效性
    Status status_ts = CheckTimestamp();
    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";  // 输入消息超时
      // estop_ = true;
      status = status_ts;
      // 如果底盘不在完全自动驾驶模式，则禁止进入自动驾驶
      if (chassis_.driving_mode() !=
          apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        control_command->mutable_engage_advice()->set_advice(
            apollo::common::EngageAdvice::DISALLOW_ENGAGE);  // 禁止自动驾驶
        control_command->mutable_engage_advice()->set_reason(
            status.error_message());  // 设置原因
      }
    } else {
      // 如果检查通过，准备进入自动驾驶
      control_command->mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::READY_TO_ENGAGE);
    }
  }

  // check estop  // 检查是否触发了紧急停止
  estop_ = estop_ || trajectory_.estop().is_estop();

  // if planning set estop, then no control process triggered // 如果未触发紧急停止，继续控制流程
  if (!estop_) {
    // 如果车辆处于手动模式，重置控制器
    if (chassis_.driving_mode() == Chassis::COMPLETE_MANUAL) {
      controller_agent_.Reset();
      AINFO_EVERY(100) << "Reset Controllers in Manual Mode";
    }

    // 将调试信息填充到控制命令中
    auto debug = control_command->mutable_debug()->mutable_input_debug();
    debug->mutable_localization_header()->CopyFrom(localization_.header());
    debug->mutable_canbus_header()->CopyFrom(chassis_.header());
    debug->mutable_trajectory_header()->CopyFrom(trajectory_.header());

    // 调用控制器主函数生成控制命令
    Status status_compute = controller_agent_.ComputeControlCommand(
        &localization_, &chassis_, &trajectory_, control_command);

    // 如果控制计算失败，记录错误信息并触发紧急停止
    if (!status_compute.ok()) {
      AERROR << "Control main function failed"
             << " with localization: " << localization_.ShortDebugString()
             << " with chassis: " << chassis_.ShortDebugString()
             << " with trajectory: " << trajectory_.ShortDebugString()
             << " with cmd: " << control_command->ShortDebugString()
             << " status:" << status_compute.error_message();
      estop_ = true;  // 触发紧急停止
      status = status_compute;
    }
  }

  // 如果触发了紧急停止，设置车辆的紧急停止命令
  if (estop_) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
     // set Estop command
    control_command->set_speed(0);  // 速度设为0
    control_command->set_throttle(0);  // 油门设为0
    control_command->set_brake(control_conf_.soft_estop_brake());  // 刹车设置为软刹车值
    control_command->set_gear_location(Chassis::GEAR_DRIVE);  // 设置档位为驱动档
  }

    // check signal // 检查是否有车辆信号信息，并将其加入控制命令中
  if (trajectory_.decision().has_vehicle_signal()) {
    control_command->mutable_signal()->CopyFrom(
        trajectory_.decision().vehicle_signal());
  }

  // 返回状态
  return status;
}

void Control::OnTimer(const ros::TimerEvent &) {
  // 记录当前时间戳，表示定时器回调的开始时间
  double start_timestamp = Clock::NowInSeconds();

  // 检查是否处于控制测试模式，并且测试时间是否超过设定的持续时间
  if (FLAGS_is_control_test_mode && FLAGS_control_test_duration > 0 &&
      (start_timestamp - init_time_) > FLAGS_control_test_duration) {
    // 如果测试时间已超过设定的持续时间，输出日志并关闭ROS系统
    AERROR << "Control finished testing. exit";
    ros::shutdown();  // 关闭系统
  }

  // 创建一个用于存储控制命令的对象
  ControlCommand control_command;

  // 调用 ProduceControlCommand 函数生成控制命令，并获取状态
  Status status = ProduceControlCommand(&control_command);
  // 如果状态不是OK，输出错误信息
  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();

  // 记录当前时间戳，表示定时器回调的结束时间
  double end_timestamp = Clock::NowInSeconds();

  // 检查是否接收到控制板消息
  if (pad_received_) {
    // 如果接收到控制板消息，则将 pad_msg_ 复制到控制命令中的 pad_msg_ 字段
    control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
    // 标记控制板消息已处理完毕
    pad_received_ = false;
  }

  // 计算控制周期的总耗时（毫秒）
  const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
  // 设置控制命令中的延迟统计信息，包括总耗时
  control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  // 判断控制周期时间是否超出配置的控制周期
  control_command.mutable_latency_stats()->set_total_time_exceeded(
      time_diff_ms < control_conf_.control_period());
  // 输出调试信息，显示当前控制周期的耗时
  ADEBUG << "control cycle time is: " << time_diff_ms << " ms.";
  // 将状态信息保存到控制命令的头部
  status.Save(control_command.mutable_header()->mutable_status());

  // 发送生成的控制命令
  SendCmd(&control_command);
}

Status Control::CheckInput() {
  AdapterManager::Observe();
  auto localization_adapter = AdapterManager::GetLocalization();
  if (localization_adapter->Empty()) {
    AWARN_EVERY(100) << "No Localization msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No localization msg");
  }
  localization_ = localization_adapter->GetLatestObserved();
  ADEBUG << "Received localization:" << localization_.ShortDebugString();

  auto chassis_adapter = AdapterManager::GetChassis();
  if (chassis_adapter->Empty()) {
    AWARN_EVERY(100) << "No Chassis msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No chassis msg");
  }
  chassis_ = chassis_adapter->GetLatestObserved();
  ADEBUG << "Received chassis:" << chassis_.ShortDebugString();

  auto trajectory_adapter = AdapterManager::GetPlanning();
  if (trajectory_adapter->Empty()) {
    AWARN_EVERY(100) << "No planning msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No planning msg");
  }
  trajectory_ = trajectory_adapter->GetLatestObserved();
  if (!trajectory_.estop().is_estop() &&
      trajectory_.trajectory_point_size() == 0) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "planning has no trajectory point.");
  }

  for (auto &trajectory_point : *trajectory_.mutable_trajectory_point()) {
    if (trajectory_point.v() < control_conf_.minimum_speed_resolution()) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }

  VehicleStateProvider::instance()->Update(localization_, chassis_);

  return Status::OK();
}

Status Control::CheckTimestamp() {
  if (!FLAGS_enable_input_timestamp_check || FLAGS_is_control_test_mode) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }
  double current_timestamp = Clock::NowInSeconds();
  double localization_diff =
      current_timestamp - localization_.header().timestamp_sec();
  if (localization_diff >
      (FLAGS_max_localization_miss_num * control_conf_.localization_period())) {
    AERROR << "Localization msg lost for " << std::setprecision(6)
           << localization_diff << "s";
    common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR("Localization msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }

  double chassis_diff = current_timestamp - chassis_.header().timestamp_sec();
  if (chassis_diff >
      (FLAGS_max_chassis_miss_num * control_conf_.chassis_period())) {
    AERROR << "Chassis msg lost for " << std::setprecision(6) << chassis_diff
           << "s";
    common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR("Chassis msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }

  double trajectory_diff =
      current_timestamp - trajectory_.header().timestamp_sec();
  if (trajectory_diff >
      (FLAGS_max_planning_miss_num * control_conf_.trajectory_period())) {
    AERROR << "Trajectory msg lost for " << std::setprecision(6)
           << trajectory_diff << "s";
    common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR("Trajectory msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Trajectory msg timeout");
  }
  return Status::OK();
}

void Control::SendCmd(ControlCommand *control_command) {
  // set header // 设置控制命令的头部信息
  AdapterManager::FillControlCommandHeader(Name(), control_command);

  // 输出控制命令的简要调试信息
  ADEBUG << control_command->ShortDebugString();

  // 如果处于控制测试模式，跳过发布控制命令
  if (FLAGS_is_control_test_mode) {
    ADEBUG << "Skip publish control command in test mode";
    return;
  }

  // 发布控制命令，将其发送到相关模块
  AdapterManager::PublishControlCommand(*control_command);
}

void Control::Stop() {}

}  // namespace control
}  // namespace apollo
