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

#include "modules/planning/planning.h"

#include <algorithm>
#include <list>
#include <vector>

#include "google/protobuf/repeated_field.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_thread_pool.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/common/trajectory/trajectory_stitcher.h"
#include "modules/planning/planner/em/em_planner.h"
#include "modules/planning/planner/lattice/lattice_planner.h"
#include "modules/planning/planner/navi/navi_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/planning/tasks/traffic_decider/traffic_decider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;

Planning::~Planning() { Stop(); }

std::string Planning::Name() const { return "planning"; }

// // 检查适配器是否已注册的宏
// #define CHECK_ADAPTER(NAME)                                              \
//   if (AdapterManager::Get##NAME() == nullptr) {                          \  // 如果适配器为空（未注册）
//     AERROR << #NAME << " is not registered";                             \  // 输出错误日志，指明适配器未注册
//     return Status(ErrorCode::PLANNING_ERROR, #NAME " is not registered"); \  // 返回错误状态，提示适配器未注册
//   }
// // 条件检查适配器是否已注册的宏
// #define CHECK_ADAPTER_IF(CONDITION, NAME) \
//   if (CONDITION) CHECK_ADAPTER(NAME)  // 如果满足条件，则调用 CHECK_ADAPTER 宏进行检查

#define CHECK_ADAPTER(NAME)                                              \
  if (AdapterManager::Get##NAME() == nullptr) {                          \
    AERROR << #NAME << " is not registered";                             \
    return Status(ErrorCode::PLANNING_ERROR, #NAME " is not registerd"); \
  }

//! CHECK_ADAPTER(Adapter) 会被展开为：
// if (AdapterManager::GetAdapter() == nullptr) {                          
//     AERROR << "Adapter" << " is not registered";                             
//     return Status(ErrorCode::PLANNING_ERROR, "Adapter is not registered"); 
// }


#define CHECK_ADAPTER_IF(CONDITION, NAME) \
  if (CONDITION) CHECK_ADAPTER(NAME)

// 注册规划器
void Planning::RegisterPlanners() {
  // 注册 RTKReplayPlanner 规划器
  planner_factory_.Register(
      PlanningConfig::RTK, []() -> Planner* { return new RTKReplayPlanner(); });
  
  // 注册 EMPlanner 规划器
  planner_factory_.Register(PlanningConfig::EM,
                            []() -> Planner* { return new EMPlanner(); });
  
  // 注册 LatticePlanner 规划器
  planner_factory_.Register(PlanningConfig::LATTICE,
                            []() -> Planner* { return new LatticePlanner(); });
  
  // 注册 NaviPlanner 规划器
  planner_factory_.Register(PlanningConfig::NAVI,
                            []() -> Planner* { return new NaviPlanner(); });
}

// 初始化规划框架
Status Planning::InitFrame(const uint32_t sequence_num,  // 序列号
                           const TrajectoryPoint& planning_start_point,  // 规划开始的轨迹点
                           const double start_time,  // 开始时间
                           const VehicleState& vehicle_state) {  // 车辆状态
  // 创建新的 Frame 实例并重置 frame_ 指针
  frame_.reset(new Frame(sequence_num, planning_start_point, start_time,
                         vehicle_state, reference_line_provider_.get()));
  
  // 初始化 Frame，并获取状态
  auto status = frame_->Init();
  // 检查初始化是否成功
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();  // 记录错误日志
    return status;  // 返回错误状态
  }
  return Status::OK();  // 返回成功状态
}

// 重置临时停车状态
void Planning::ResetPullOver(const routing::RoutingResponse& response) {
  // 获取当前规划状态中的临时停车信息
  auto* pull_over =
      util::GetPlanningStatus()->mutable_planning_state()->mutable_pull_over();
  
  // 如果没有上一个路由信息，则更新为当前路由信息并清除临时停车状态
  if (!last_routing_.has_header()) {
    last_routing_ = response;  // 更新最后的路由信息
    pull_over->Clear();  // 清除临时停车状态
    return;  // 结束函数
  }

  // 如果当前不在临时停车状态，直接返回
  if (!pull_over->in_pull_over()) {
    return;
  }

  // 如果检测到新的路由信息
  if (hdmap::PncMap::IsNewRouting(last_routing_, response)) {
    pull_over->Clear();  // 清除临时停车状态
    last_routing_ = response;  // 更新最后的路由信息
    AINFO << "Cleared Pull Over Status after received new routing";  // 记录日志
  }
}

Status Planning::Init() {
  // 从配置文件加载规划配置
  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                               &config_))
      << "failed to load planning config file " << FLAGS_planning_config_file;

  // 从配置文件加载交通规则配置
  CHECK(apollo::common::util::GetProtoFromFile(
      FLAGS_traffic_rule_config_filename, &traffic_rule_configs_))
      << "Failed to load traffic rule config file "
      << FLAGS_traffic_rule_config_filename;

  // 初始化规划线程池
  PlanningThreadPool::instance()->Init();

  // 清除规划状态
  util::GetPlanningStatus()->Clear();

  // 初始化适配器管理器，如果尚未初始化
  if (!AdapterManager::Initialized()) {
    AdapterManager::Init(FLAGS_planning_adapter_config_filename);
  }
  
  // 检查必需的适配器是否已连接
  CHECK_ADAPTER(Localization);           // 本地化适配器
  CHECK_ADAPTER(Chassis);                // 底盘适配器
  CHECK_ADAPTER(RoutingResponse);        // 路由响应适配器
  CHECK_ADAPTER(RoutingRequest);         // 路由请求适配器
  CHECK_ADAPTER_IF(FLAGS_use_navigation_mode, RelativeMap); // 相对地图适配器（在导航模式下）
  CHECK_ADAPTER_IF(FLAGS_use_navigation_mode && FLAGS_enable_prediction,
                   PerceptionObstacles); // 感知障碍适配器（在导航模式和预测启用下）
  CHECK_ADAPTER_IF(FLAGS_enable_prediction, Prediction); // 预测适配器（在预测启用下）
  CHECK_ADAPTER(TrafficLightDetection); // 红绿灯检测适配器

  // 如果不使用导航模式，加载地图
  if (!FLAGS_use_navigation_mode) {
    hdmap_ = HDMapUtil::BaseMapPtr();
    CHECK(hdmap_) << "Failed to load map";
    // Prefer "std::make_unique" to direct use of "new".
    // Reference "https://herbsutter.com/gotw/_102/" for details.
    reference_line_provider_ = std::make_unique<ReferenceLineProvider>(hdmap_);
  }

  // 注册规划器
  RegisterPlanners();
  
  // 创建具体的规划器对象
  planner_ = planner_factory_.CreateObject(config_.planner_type());
  if (!planner_) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }

  // 调用规划器的初始化函数
  return planner_->Init(config_);
}

// 检查车辆状态是否有效
bool Planning::IsVehicleStateValid(const VehicleState& vehicle_state) {
  // 检查车辆状态的各个属性是否为 NaN（非数字）
  if (std::isnan(vehicle_state.x()) ||  // 检查 x 坐标
      std::isnan(vehicle_state.y()) ||  // 检查 y 坐标
      std::isnan(vehicle_state.z()) ||  // 检查 z 坐标
      std::isnan(vehicle_state.heading()) ||  // 检查航向角
      std::isnan(vehicle_state.kappa()) ||  // 检查曲率
      std::isnan(vehicle_state.linear_velocity()) ||  // 检查线速度
      std::isnan(vehicle_state.linear_acceleration())) {  // 检查线加速度
    return false;  // 如果有任意属性为 NaN，返回 false
  }
  return true;  // 所有属性有效，返回 true
}

Status Planning::Start() {
  // 创建一个定时器，每隔1.0 / FLAGS_planning_loop_rate秒触发一次OnTimer函数
  timer_ = AdapterManager::CreateTimer(
      ros::Duration(1.0 / FLAGS_planning_loop_rate), &Planning::OnTimer, this);
  
  // The "reference_line_provider_" may not be created yet in navigation mode.
  // It is necessary to check its existence.
  // 在导航模式下，"reference_line_provider_"可能尚未创建，因此需要检查其存在性
  if (reference_line_provider_) {
    reference_line_provider_->Start(); // 启动参考线提供者
  }
  
  // 记录开始时间
  start_time_ = Clock::NowInSeconds();
  AINFO << "Planning started";
  return Status::OK();
}

// 定时器事件回调函数
void Planning::OnTimer(const ros::TimerEvent&) {
  RunOnce();  // 执行一次规划操作

  // 检查是否处于测试模式，并判断测试持续时间
  if (FLAGS_planning_test_mode && FLAGS_test_duration > 0.0 &&
      Clock::NowInSeconds() - start_time_ > FLAGS_test_duration) {
    ros::shutdown();  // 如果超过测试时长，关闭 ROS 节点
  }
}

// 发布规划的轨迹数据
void Planning::PublishPlanningPb(ADCTrajectory* trajectory_pb,  // 轨迹数据的指针
                                 double timestamp) {  // 时间戳
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);  // 设置轨迹数据的时间戳

  // TODO(all): integrate reverse gear // TODO: 整合倒车档
  trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);  // 设置为前进档

  // 检查是否有有效的路由响应
  if (AdapterManager::GetRoutingResponse() &&
      !AdapterManager::GetRoutingResponse()->Empty()) {
    // 从最新观察到的路由响应中复制路由头信息
    trajectory_pb->mutable_routing_header()->CopyFrom(
        AdapterManager::GetRoutingResponse()->GetLatestObserved().header());
  }

  // 如果使用导航模式且轨迹点为空，设置备用巡航轨迹
  if (FLAGS_use_navigation_mode &&
      trajectory_pb->trajectory_point_size() == 0) {
    SetFallbackCruiseTrajectory(trajectory_pb);
  }

  // NOTICE:
  // Since we are using the time at each cycle beginning as timestamp, the
  // relative time of each trajectory point should be modified so that we can
  // use the current timestamp in header.
  // 注意：
  // 由于我们在每个周期开始时使用时间戳，因此需要修改每个轨迹点的相对时间，
  // 以便我们可以在头信息中使用当前时间戳。

  // auto* trajectory_points = trajectory_pb.mutable_trajectory_point();
  if (!FLAGS_planning_test_mode) {
    const double dt = timestamp - Clock::NowInSeconds();  // 计算时间差
    // 更新每个轨迹点的相对时间
    for (auto& p : *trajectory_pb->mutable_trajectory_point()) {
      p.set_relative_time(p.relative_time() + dt);
    }
  }
  
  Publish(trajectory_pb);  // 发布轨迹数据
}

void Planning::RunOnce() {
   // snapshot all coming data // 快照所有传入的数据
  AdapterManager::Observe();

  const double start_timestamp = Clock::NowInSeconds(); // 获取当前时间戳

  ADCTrajectory not_ready_pb; // 创建不准备状态的轨迹消息
  auto* not_ready = not_ready_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  // 检查各个组件是否准备就绪
  if (AdapterManager::GetLocalization()->Empty()) {
    not_ready->set_reason("localization not ready");
  } else if (AdapterManager::GetChassis()->Empty()) {
    not_ready->set_reason("chassis not ready");
  } else if (!FLAGS_use_navigation_mode &&
             AdapterManager::GetRoutingResponse()->Empty()) {
    not_ready->set_reason("routing not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  }

  // 如果有未准备的原因，记录错误并跳过此次规划周期
  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }

  // 如果使用导航模式，每个周期都重新创建参考线提供者
  if (FLAGS_use_navigation_mode) {
    // recreate reference line provider in every cycle
    hdmap_ = HDMapUtil::BaseMapPtr();
    // Prefer "std::make_unique" to direct use of "new".
    // Reference "https://herbsutter.com/gotw/_102/" for details.
    reference_line_provider_ = std::make_unique<ReferenceLineProvider>(hdmap_);
  }

  // localization
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Get localization:" << localization.DebugString();

  // chassis
  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  ADEBUG << "Get chassis:" << chassis.DebugString();

  // 更新车辆状态提供者
  Status status =
      VehicleStateProvider::instance()->Update(localization, chassis);

  // 如果使用导航模式，检查并更新车辆状态
  if (FLAGS_use_navigation_mode) {
    const auto& vehicle_state_abs =
        VehicleStateProvider::instance()->vehicle_state();

    if (IsVehicleStateValid(last_vehicle_state_abs_pos_)) {
      // 计算车辆状态变化并调整轨迹
      auto x_diff = vehicle_state_abs.x() - last_vehicle_state_abs_pos_.x();
      auto y_diff = vehicle_state_abs.y() - last_vehicle_state_abs_pos_.y();
      auto theta_diff = vehicle_state_abs.heading()
          - last_vehicle_state_abs_pos_.heading();
      TrajectoryStitcher::TransformLastPublishedTrajectory(-x_diff, -y_diff,
          -theta_diff, last_publishable_trajectory_.get());
    }
    last_vehicle_state_abs_pos_ = vehicle_state_abs;

    // 设置车辆配置
    VehicleStateProvider::instance()->set_vehicle_config(0.0, 0.0, 0.0);
  }

  // 获取当前车辆状态
  VehicleState vehicle_state =
      VehicleStateProvider::instance()->vehicle_state();

  // 在当前时间戳下估算 (x, y) 坐标
  // 这个估算仅在当前时间与车辆状态时间戳之间的差异很小（20毫秒）时有效。
  // 当差异过大时，估算是无效的。
    // estimate (x, y) at current timestamp
  // This estimate is only valid if the current time and vehicle state timestamp
  // differs only a small amount (20ms). When the different is too large, the
  // estimation is invalid.
  DCHECK_GE(start_timestamp, vehicle_state.timestamp());
  if (FLAGS_estimate_current_vehicle_state &&
      start_timestamp - vehicle_state.timestamp() < 0.020) {
    auto future_xy = VehicleStateProvider::instance()->EstimateFuturePosition(
        start_timestamp - vehicle_state.timestamp());
    vehicle_state.set_x(future_xy.x());
    vehicle_state.set_y(future_xy.y());
    vehicle_state.set_timestamp(start_timestamp);
  }

  // 检查车辆状态是否有效
  if (!status.ok() || !IsVehicleStateValid(vehicle_state)) {
    std::string msg("Update VehicleStateProvider failed");
    AERROR << msg;
    not_ready->set_reason(msg);
    status.Save(not_ready_pb.mutable_header()->mutable_status());
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }

  // 更新参考线提供者的路由响应
  if (!FLAGS_use_navigation_mode &&
      !reference_line_provider_->UpdateRoutingResponse(
          AdapterManager::GetRoutingResponse()->GetLatestObserved())) {
    std::string msg("Failed to update routing in reference line provider");
    AERROR << msg;
    not_ready->set_reason(msg);
    status.Save(not_ready_pb.mutable_header()->mutable_status());
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }

  // 如果启用了预测，检查预测适配器是否有数据
  if (FLAGS_enable_prediction && AdapterManager::GetPrediction()->Empty()) {
    AWARN_EVERY(100) << "prediction is enabled but no prediction provided";
  }

  // Update reference line provider and reset pull over if necessary // 更新参考线提供者并在必要时重置停车状态
  if (!FLAGS_use_navigation_mode) {
    reference_line_provider_->UpdateVehicleState(vehicle_state);
    ResetPullOver(AdapterManager::GetRoutingResponse()->GetLatestObserved());
  }

  const double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;

  bool is_replan = false; // 标记是否需要重新规划
  std::vector<TrajectoryPoint> stitching_trajectory; // 计算轨迹 //拼接轨迹
  stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
      vehicle_state, start_timestamp, planning_cycle_time,
      last_publishable_trajectory_.get(), &is_replan);

  // 初始化帧
  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;
  status = InitFrame(frame_num, stitching_trajectory.back(), start_timestamp,
                     vehicle_state);
  if (!frame_) {
    std::string msg("Failed to init frame");
    AERROR << msg;
    not_ready->set_reason(msg);
    status.Save(not_ready_pb.mutable_header()->mutable_status());
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }

  // 记录轨迹
  auto* trajectory_pb = frame_->mutable_trajectory();
  if (FLAGS_enable_record_debug) {
    frame_->RecordInputDebug(trajectory_pb->mutable_debug());
  }
  trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);
  
  // 检查状态是否正常
  if (!status.ok()) {
    AERROR << status.ToString();
    if (FLAGS_publish_estop) {
      // 在发布紧急停车信号时，添加更多信息

      // 因为函数 "Control::ProduceControlCommand()" 会通过以下代码检查
      // "estop" 信号（control.cc中的第170行）：
      // estop_ = estop_ || trajectory_.estop().is_estop();
      // 我们应该添加更多信息以确保触发紧急停车信号。
      // Because the function "Control::ProduceControlCommand()" checks the
      // "estop" signal with the following line (Line 170 in control.cc):
      // estop_ = estop_ || trajectory_.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      ADCTrajectory estop_trajectory;
      EStop* estop = estop_trajectory.mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
      status.Save(estop_trajectory.mutable_header()->mutable_status());
      PublishPlanningPb(&estop_trajectory, start_timestamp);
    } else {
      trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
      status.Save(trajectory_pb->mutable_header()->mutable_status());
      PublishPlanningPb(trajectory_pb, start_timestamp);
    }

    auto seq_num = frame_->SequenceNum();
    FrameHistory::instance()->Add(seq_num, std::move(frame_));

    return;
  }

  // 遍历参考线信息，执行交通决策
  for (auto& ref_line_info : frame_->reference_line_info()) {
    TrafficDecider traffic_decider;
    traffic_decider.Init(traffic_rule_configs_);
    auto traffic_status = traffic_decider.Execute(frame_.get(), &ref_line_info);
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      AWARN << "Reference line " << ref_line_info.Lanes().Id()
            << " traffic decider failed";
      continue;
    }
  }

  // 开始规划
  status = Plan(start_timestamp, stitching_trajectory, trajectory_pb);

  const auto time_diff_ms = (Clock::NowInSeconds() - start_timestamp) * 1000;
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

  trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: "
         << trajectory_pb->latency_stats().DebugString();

  // 记录参考线提供者的延迟
  auto* ref_line_task =
      trajectory_pb->mutable_latency_stats()->add_task_stats();
  ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                             1000.0);
  ref_line_task->set_name("ReferenceLineProvider");

  // 如果规划失败，处理错误
  if (!status.ok()) {
    status.Save(trajectory_pb->mutable_header()->mutable_status());
    AERROR << "Planning failed:" << status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      // Because the function "Control::ProduceControlCommand()" checks the
      // "estop" signal with the following line (Line 170 in control.cc):
      // estop_ = estop_ || trajectory_.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      EStop* estop = trajectory_pb->mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
    }
  }

  trajectory_pb->set_is_replan(is_replan); // 设置是否重新规划
  PublishPlanningPb(trajectory_pb, start_timestamp); // 发布规划消息
  ADEBUG << "Planning pb:" << trajectory_pb->header().DebugString();

  auto seq_num = frame_->SequenceNum();
  FrameHistory::instance()->Add(seq_num, std::move(frame_)); // 保存帧历史
}

// 设置备用巡航轨迹
void Planning::SetFallbackCruiseTrajectory(ADCTrajectory* cruise_trajectory) {
  CHECK_NOTNULL(cruise_trajectory);  // 检查 cruise_trajectory 指针不为空

  const double v = VehicleStateProvider::instance()->linear_velocity();  // 获取当前车辆的线速度
  
  // 根据备用巡航时间生成轨迹点
  for (double t = 0.0; t < FLAGS_navigation_fallback_cruise_time; t += 0.1) {
    const double s = t * v;  // 计算当前位置 s（行驶距离）

    // 创建一个新的轨迹点并添加到巡航轨迹中
    auto* cruise_point = cruise_trajectory->add_trajectory_point();
    cruise_point->mutable_path_point()->CopyFrom(
        common::util::MakePathPoint(s, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));  // 设置路径点
    cruise_point->mutable_path_point()->set_s(s);  // 设置行驶距离 s
    cruise_point->set_v(v);  // 设置线速度
    cruise_point->set_a(0.0);  // 设置加速度为 0（恒速巡航）
    cruise_point->set_relative_time(t);  // 设置相对时间
  }
}

// 停止规划
void Planning::Stop() {
  AERROR << "Planning Stop is called";  // 记录停止规划的日志信息
  
  // 停止规划线程池（注释掉，未启用）
  // PlanningThreadPool::instance()->Stop();

  // 如果存在参考线提供者，则停止它
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }

  // 重置可发布的轨迹、框架和规划器指针
  last_publishable_trajectory_.reset(nullptr);
  frame_.reset(nullptr);
  planner_.reset(nullptr);

  // 清除框架历史记录
  FrameHistory::instance()->Clear();
}

// 设置最后可发布的轨迹
void Planning::SetLastPublishableTrajectory(
    const ADCTrajectory& adc_trajectory) {
  // 重置 last_publishable_trajectory_ 指针，创建新的可发布轨迹
  last_publishable_trajectory_.reset(new PublishableTrajectory(adc_trajectory));
}

// 导出参考线调试信息
void Planning::ExportReferenceLineDebug(planning_internal::Debug* debug) {
  // 如果调试记录未启用，则直接返回
  if (!FLAGS_enable_record_debug) {
    return;
  }

  // 遍历当前框架中的所有参考线信息
  for (auto& reference_line_info : frame_->reference_line_info()) {
    // 创建并设置参考线调试信息
    auto rl_debug = debug->mutable_planning_data()->add_reference_line();
    rl_debug->set_id(reference_line_info.Lanes().Id());  // 设置参考线ID
    rl_debug->set_length(reference_line_info.reference_line().Length());  // 设置参考线长度
    rl_debug->set_cost(reference_line_info.Cost());  // 设置参考线成本
    rl_debug->set_is_change_lane_path(reference_line_info.IsChangeLanePath());  // 设置是否为变道路径
    rl_debug->set_is_drivable(reference_line_info.IsDrivable());  // 设置是否可驾驶
    rl_debug->set_is_protected(reference_line_info.GetRightOfWayStatus() == ADCTrajectory::PROTECTED);  // 设置是否有优先通行权
  }
}

Status Planning::Plan(const double current_time_stamp,
                      const std::vector<TrajectoryPoint>& stitching_trajectory,
                      ADCTrajectory* trajectory_pb) {
  auto* ptr_debug = trajectory_pb->mutable_debug();
  // 如果启用了调试记录，则将最后的拼接轨迹点复制到调试信息中
  if (FLAGS_enable_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
  }

  // 调用规划器进行路径规划
  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get());

  // 导出参考线调试信息
  ExportReferenceLineDebug(ptr_debug);

  // 查找最佳的可行驶参考线信息
  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  if (!best_ref_info) {
    std::string msg("planner failed to make a driving plan");
    AERROR << msg;
    // 如果有上一个可发布的轨迹，则清空
    if (last_publishable_trajectory_) {
      last_publishable_trajectory_->Clear();
    }
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  
  // 将最佳参考线的信息合并到调试信息中
  ptr_debug->MergeFrom(best_ref_info->debug());
  trajectory_pb->mutable_latency_stats()->MergeFrom(
      best_ref_info->latency_stats());
  // set right of way status // 设置优先通行状态
  trajectory_pb->set_right_of_way_status(best_ref_info->GetRightOfWayStatus());
  
  // 将目标车道ID添加到轨迹中
  for (const auto& id : best_ref_info->TargetLaneId()) {
    trajectory_pb->add_lane_id()->CopyFrom(id);
  }

  // 导出决策信息
  best_ref_info->ExportDecision(trajectory_pb->mutable_decision());

  // Add debug information.// 添加调试信息
  if (FLAGS_enable_record_debug) {
    auto* reference_line = ptr_debug->mutable_planning_data()->add_path();
    reference_line->set_name("planning_reference_line");
    const auto& reference_points =
        best_ref_info->reference_line().reference_points();
    double s = 0.0; // 路径长度
    double prev_x = 0.0; // 上一个点的x坐标
    double prev_y = 0.0; // 上一个点的y坐标
    bool empty_path = true; // 是否为空路径
    for (const auto& reference_point : reference_points) {
      auto* path_point = reference_line->add_path_point();
      path_point->set_x(reference_point.x());
      path_point->set_y(reference_point.y());
      path_point->set_theta(reference_point.heading());
      path_point->set_kappa(reference_point.kappa());
      path_point->set_dkappa(reference_point.dkappa());
      if (empty_path) {
        path_point->set_s(0.0);
        empty_path = false;
      } else {
        double dx = reference_point.x() - prev_x; // x方向的变化
        double dy = reference_point.y() - prev_y; // y方向的变化
        s += std::hypot(dx, dy); // 计算路径长度
        path_point->set_s(s);
      }
      prev_x = reference_point.x(); // 更新上一个点的x坐标
      prev_y = reference_point.y(); // 更新上一个点的y坐标
    }
  }

  // 创建可发布轨迹对象
  last_publishable_trajectory_.reset(new PublishableTrajectory(
      current_time_stamp, best_ref_info->trajectory()));

  ADEBUG << "current_time_stamp: " << std::to_string(current_time_stamp);

  // 将拼接轨迹的点添加到可发布轨迹中
  last_publishable_trajectory_->PrependTrajectoryPoints(
      stitching_trajectory.begin(), stitching_trajectory.end() - 1);

  // 调试信息：输出可发布轨迹中的点
  for (size_t i = 0; i < last_publishable_trajectory_->NumOfPoints(); ++i) {
    if (last_publishable_trajectory_->TrajectoryPointAt(i).relative_time() >
        FLAGS_trajectory_time_high_density_period) {
      break;
    }
    ADEBUG << last_publishable_trajectory_->TrajectoryPointAt(i)
                  .ShortDebugString();
  }

  // 将可发布轨迹填充到protobuf中
  last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory_pb);

  // 导出介入建议
  best_ref_info->ExportEngageAdvice(trajectory_pb->mutable_engage_advice());

  return status; // 返回状态
}

}  // namespace planning
}  // namespace apollo
