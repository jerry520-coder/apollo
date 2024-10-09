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

#include "modules/planning/planner/em/em_planner.h"

#include <fstream>
#include <limits>
#include <utility>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/tasks/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/tasks/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/tasks/path_decider/path_decider.h"
#include "modules/planning/tasks/poly_st_speed/poly_st_speed_optimizer.h"
#include "modules/planning/tasks/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/tasks/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/tasks/speed_decider/speed_decider.h"

namespace apollo {
namespace planning {

using common::ErrorCode;
using common::SLPoint;
using common::SpeedPoint;
using common::Status;
using common::TrajectoryPoint;
using common::adapter::AdapterManager;
using common::math::Vec2d;
using common::time::Clock;

namespace {
constexpr double kPathOptimizationFallbackClost = 2e4;
constexpr double kSpeedOptimizationFallbackClost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

void EMPlanner::RegisterTasks() {
  // 注册动态规划多项式路径优化任务
  task_factory_.Register(DP_POLY_PATH_OPTIMIZER,
                         []() -> Task* { return new DpPolyPathOptimizer(); });

  // 注册路径选择任务
  task_factory_.Register(PATH_DECIDER,
                         []() -> Task* { return new PathDecider(); });

  // 注册动态规划时速优化任务
  task_factory_.Register(DP_ST_SPEED_OPTIMIZER,
                         []() -> Task* { return new DpStSpeedOptimizer(); });

  // 注册速度选择任务
  task_factory_.Register(SPEED_DECIDER,
                         []() -> Task* { return new SpeedDecider(); });

  // 注册二次规划样条时速优化任务
  task_factory_.Register(QP_SPLINE_ST_SPEED_OPTIMIZER, []() -> Task* {
    return new QpSplineStSpeedOptimizer();
  });

  // 注册多项式时速优化任务
  task_factory_.Register(POLY_ST_SPEED_OPTIMIZER,
                         []() -> Task* { return new PolyStSpeedOptimizer(); });
}

Status EMPlanner::Init(const PlanningConfig& config) {
  AINFO << "In EMPlanner::Init()";  // 日志输出，表示初始化函数开始

  // 注册所有任务
  RegisterTasks();

  // 根据配置创建任务并存储到 tasks_ 向量中
  for (const auto task : config.em_planner_config().task()) {
    tasks_.emplace_back(
        task_factory_.CreateObject(static_cast<TaskType>(task))); // 创建任务对象
    AINFO << "Created task:" << tasks_.back()->Name();  // 日志输出，显示创建的任务名称
  }

  // 初始化每个任务
  for (auto& task : tasks_) {
    if (!task->Init(config)) {  // 调用任务的 Init 函数进行初始化
      std::string msg(
          common::util::StrCat("Init task[", task->Name(), "] failed.")); // 构造错误信息
      AERROR << msg;  // 日志输出，记录错误
      return Status(ErrorCode::PLANNING_ERROR, msg);  // 返回错误状态
    }
  }

  return Status::OK();  // 初始化成功，返回 OK 状态
}

void EMPlanner::RecordObstacleDebugInfo(
    ReferenceLineInfo* reference_line_info) {
  // 检查是否启用调试信息记录
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";  // 日志输出，跳过记录
    return;
  }

  auto ptr_debug = reference_line_info->mutable_debug();  // 获取调试信息的指针

  const auto path_decision = reference_line_info->path_decision();  // 获取路径决策信息
  for (const auto path_obstacle : path_decision->path_obstacles().Items()) {
    // 为每个障碍物添加调试信息
    auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
    obstacle_debug->set_id(path_obstacle->Id());  // 设置障碍物 ID
    obstacle_debug->mutable_sl_boundary()->CopyFrom(
        path_obstacle->PerceptionSLBoundary());  // 复制障碍物的 SL 边界信息

    const auto& decider_tags = path_obstacle->decider_tags();  // 获取决策标签
    const auto& decisions = path_obstacle->decisions();  // 获取决策信息

    // 检查决策标签和决策数量是否一致
    if (decider_tags.size() != decisions.size()) {
      AERROR << "decider_tags size: " << decider_tags.size()
             << " different from decisions size:" << decisions.size();  // 记录错误信息
    }

    // 记录每个决策标签及其对应的决策
    for (size_t i = 0; i < decider_tags.size(); ++i) {
      auto decision_tag = obstacle_debug->add_decision_tag();
      decision_tag->set_decider_tag(decider_tags[i]);  // 设置决策标签
      decision_tag->mutable_decision()->CopyFrom(decisions[i]);  // 复制决策信息
    }
  }
}

void EMPlanner::RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                                const std::string& name,
                                const double time_diff_ms) {
  // 检查是否启用调试信息记录
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";  // 日志输出，跳过记录
    return;
  }

  // 检查参考线信息是否为空
  if (reference_line_info == nullptr) {
    AERROR << "Reference line info is null.";  // 记录错误信息
    return;
  }

  // 获取延迟统计信息的指针
  auto ptr_latency_stats = reference_line_info->mutable_latency_stats();

  // 添加新的任务统计信息
  auto ptr_stats = ptr_latency_stats->add_task_stats();
  ptr_stats->set_name(name);  // 设置任务名称
  ptr_stats->set_time_ms(time_diff_ms);  // 设置时间差（毫秒）
}

Status EMPlanner::Plan(const TrajectoryPoint& planning_start_point,  // 规划开始的轨迹点
                       Frame* frame) {  // 当前的规划框架
  bool has_drivable_reference_line = false;  // 标记是否找到可行驶的参考线
  bool disable_low_priority_path = false;  // 标记是否禁用低优先级路径
  auto status = 
      Status(ErrorCode::PLANNING_ERROR, "reference line not drivable");  // 初始化状态为错误，表示参考线不可行驶

  // 遍历当前框架中的所有参考线信息
  for (auto& reference_line_info : frame->reference_line_info()) {
    // 如果禁用低优先级路径，则将当前参考线标记为不可行驶
    if (disable_low_priority_path) {
      reference_line_info.SetDrivable(false);
    }
    // 如果当前参考线不可行驶，则跳过
    if (!reference_line_info.IsDrivable()) {
      continue;
    }
    
    // 在当前可行驶的参考线上进行规划
    auto cur_status = 
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);
    
    // 如果规划成功且当前参考线仍然可行驶
    if (cur_status.ok() && reference_line_info.IsDrivable()) {
      has_drivable_reference_line = true;  // 标记找到可行驶的参考线
      // 如果设置了优先变道且当前参考线为变道路径且其成本低于直行路径成本
      if (FLAGS_prioritize_change_lane &&
          reference_line_info.IsChangeLanePath() &&
          reference_line_info.Cost() < kStraightForwardLineCost) {
        disable_low_priority_path = true;  // 禁用低优先级路径
      }
    } else {
      // 如果规划失败或当前参考线不可行驶，则将其标记为不可行驶
      reference_line_info.SetDrivable(false);
    }
  }
  
  // 如果找到可行驶的参考线，则返回成功状态；否则返回初始化的错误状态
  return has_drivable_reference_line ? Status::OK() : status;
}

Status EMPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  // 如果当前不是变道路径，则增加直线行驶的成本
  if (!reference_line_info->IsChangeLanePath()) {
    reference_line_info->AddCost(kStraightForwardLineCost);
  }
  
  ADEBUG << "planning start point:" << planning_start_point.DebugString();  // 输出规划起始点的信息
  auto* heuristic_speed_data = reference_line_info->mutable_speed_data();  // 获取可变的速度数据
  auto speed_profile = GenerateInitSpeedProfile(planning_start_point, reference_line_info);  // 生成初始速度配置文件

  // 如果速度配置文件为空，则使用热启动速度配置
  if (speed_profile.empty()) {
    speed_profile = GenerateSpeedHotStart(planning_start_point);
    ADEBUG << "Using dummy hot start for speed vector";  // 输出使用热启动的调试信息
  }
  heuristic_speed_data->set_speed_vector(speed_profile);  // 设置速度数据

  auto ret = Status::OK();  // 初始化返回状态

  // 遍历并执行优化任务
  for (auto& optimizer : tasks_) {
    const double start_timestamp = Clock::NowInSeconds();  // 记录开始时间
    ret = optimizer->Execute(frame, reference_line_info);  // 执行优化任务
    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << optimizer->Name()
             << "], Error message: " << ret.error_message();  // 输出错误信息
      break;  // 退出循环
    }
    const double end_timestamp = Clock::NowInSeconds();  // 记录结束时间
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;  // 计算耗时（毫秒）

    ADEBUG << "after optimizer " << optimizer->Name() << ":"
           << reference_line_info->PathSpeedDebugString() << std::endl;  // 输出优化后的速度路径信息
    ADEBUG << optimizer->Name() << " time spend: " << time_diff_ms << " ms.";  // 输出优化任务耗时

    RecordDebugInfo(reference_line_info, optimizer->Name(), time_diff_ms);  // 记录调试信息
  }

  RecordObstacleDebugInfo(reference_line_info);  // 记录障碍物调试信息

  // 如果路径数据为空，则进行路径回退
  if (reference_line_info->path_data().Empty()) {
    ADEBUG << "Path fallback.";  // 输出路径回退的信息
    GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());  // 生成回退路径配置
    reference_line_info->AddCost(kPathOptimizationFallbackClost);  // 增加路径优化回退的成本
  }

  // 如果返回状态不正确或速度数据为空，则进行速度回退
  if (!ret.ok() || reference_line_info->speed_data().Empty()) {
    ADEBUG << "Speed fallback.";  // 输出速度回退的信息
    GenerateFallbackSpeedProfile(reference_line_info,
                                 reference_line_info->mutable_speed_data());  // 生成回退速度配置
    reference_line_info->AddCost(kSpeedOptimizationFallbackClost);  // 增加速度优化回退的成本
  }

  DiscretizedTrajectory trajectory;  // 初始化轨迹对象
  // 合并路径和速度配置生成轨迹
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) {
    std::string msg("Fail to aggregate planning trajectory.");  // 输出错误信息
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);  // 返回规划错误状态
  }

  // 检查路径障碍物信息
  for (const auto* path_obstacle :
       reference_line_info->path_decision()->path_obstacles().Items()) {
    if (path_obstacle->obstacle()->IsVirtual()) {
      continue;  // 跳过虚拟障碍物
    }
    if (!path_obstacle->obstacle()->IsStatic()) {
      continue;  // 跳过动态障碍物
    }
    // 如果障碍物的纵向决策包含停止，增加静态障碍物成本
    if (path_obstacle->LongitudinalDecision().has_stop()) {
      constexpr double kRefrenceLineStaticObsCost = 1e3;
      reference_line_info->AddCost(kRefrenceLineStaticObsCost);  // 增加成本
    }
  }

  // 如果启用了轨迹检查，则验证当前规划轨迹
  if (FLAGS_enable_trajectory_check) {
    if (!ConstraintChecker::ValidTrajectory(trajectory)) {
      std::string msg("Failed to validate current planning trajectory.");  // 输出错误信息
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);  // 返回规划错误状态
    }
  }

  reference_line_info->SetTrajectory(trajectory);  // 设置最终轨迹
  reference_line_info->SetDrivable(true);  // 设置可行驶状态
  return Status::OK();  // 返回成功状态
}

std::vector<SpeedPoint> EMPlanner::GenerateInitSpeedProfile(
    const TrajectoryPoint& planning_init_point,
    const ReferenceLineInfo* reference_line_info) {
  std::vector<SpeedPoint> speed_profile;  // 初始化速度配置文件
  const auto* last_frame = FrameHistory::instance()->Latest();  // 获取最近的帧
  if (!last_frame) {
    AWARN << "last frame is empty";  // 日志警告：最近帧为空
    return speed_profile;  // 返回空的速度配置文件
  }
  const ReferenceLineInfo* last_reference_line_info =
      last_frame->DriveReferenceLineInfo();  // 获取最近帧的参考线信息
  if (!last_reference_line_info) {
    ADEBUG << "last reference line info is empty";  // 调试信息：最近参考线信息为空
    return speed_profile;  // 返回空的速度配置文件
  }
  if (!reference_line_info->IsStartFrom(*last_reference_line_info)) {
    ADEBUG << "Current reference line is not started previous drived line";  // 调试信息：当前参考线未从上一个驱动线开始
    return speed_profile;  // 返回空的速度配置文件
  }
  const auto& last_speed_vector =
      last_reference_line_info->speed_data().speed_vector();  // 获取最近速度数据

  if (!last_speed_vector.empty()) {
    const auto& last_init_point = last_frame->PlanningStartPoint().path_point();  // 获取最近帧的起始点
    Vec2d last_xy_point(last_init_point.x(), last_init_point.y());  // 最近点的二维坐标
    SLPoint last_sl_point;
    // 将最近点从 xy 坐标转换为 sl 坐标
    if (!last_reference_line_info->reference_line().XYToSL(last_xy_point, &last_sl_point)) {
      AERROR << "Fail to transfer xy to sl when init speed profile";  // 错误信息：转换失败
    }

    Vec2d xy_point(planning_init_point.path_point().x(),
                   planning_init_point.path_point().y());  // 当前点的二维坐标
    SLPoint sl_point;
    // 将当前点从 xy 坐标转换为 sl 坐标
    if (!last_reference_line_info->reference_line().XYToSL(xy_point, &sl_point)) {
      AERROR << "Fail to transfer xy to sl when init speed profile";  // 错误信息：转换失败
    }

    double s_diff = sl_point.s() - last_sl_point.s();  // 计算 s 坐标的差值
    double start_time = 0.0;  // 初始化开始时间
    double start_s = 0.0;  // 初始化开始 s 坐标
    bool is_updated_start = false;  // 标记开始点是否已更新
    for (const auto& speed_point : last_speed_vector) {
      if (speed_point.s() < s_diff) {
        continue;  // 跳过小于差值的速度点
      }
      if (!is_updated_start) {
        start_time = speed_point.t();  // 更新开始时间
        start_s = speed_point.s();  // 更新开始 s 坐标
        is_updated_start = true;  // 标记为已更新
      }
      SpeedPoint refined_speed_point;  // 创建精炼的速度点
      refined_speed_point.set_s(speed_point.s() - start_s);  // 设置精炼后的 s 坐标
      refined_speed_point.set_t(speed_point.t() - start_time);  // 设置精炼后的时间
      refined_speed_point.set_v(speed_point.v());  // 设置速度
      refined_speed_point.set_a(speed_point.a());  // 设置加速度
      refined_speed_point.set_da(speed_point.da());  // 设置加速度变化
      speed_profile.push_back(std::move(refined_speed_point));  // 添加到速度配置文件
    }
  }
  return speed_profile;  // 返回生成的速度配置文件
}

// This is a dummy simple hot start, need refine later // 这是一个简单的热启动函数，需要后续进行优化
std::vector<SpeedPoint> EMPlanner::GenerateSpeedHotStart(
    const TrajectoryPoint& planning_init_point) {
  std::vector<SpeedPoint> hot_start_speed_profile;  // 存储速度配置文件
  double s = 0.0;  // 初始化s值
  double t = 0.0;  // 初始化时间
  // 将初始化点的速度限制在5.0到规划的最高速度限制之间
  double v = common::math::Clamp(planning_init_point.v(), 5.0,
                                 FLAGS_planning_upper_speed_limit);
  
  // 在时间范围内生成速度点
  while (t < FLAGS_trajectory_time_length) {
    SpeedPoint speed_point;  // 创建速度点对象
    speed_point.set_s(s);    // 设置s值
    speed_point.set_t(t);    // 设置时间t
    speed_point.set_v(v);    // 设置速度v

    hot_start_speed_profile.push_back(std::move(speed_point));  // 将速度点添加到配置文件中

    t += FLAGS_trajectory_time_min_interval;  // 更新时间
    s += v * FLAGS_trajectory_time_min_interval;  // 更新s值，基于当前速度和时间间隔
  }
  return hot_start_speed_profile;  // 返回生成的速度配置文件
}

void EMPlanner::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {
  // 获取当前车辆点和边界信息
  auto adc_point = reference_line_info->AdcPlanningPoint();
  double adc_s = reference_line_info->AdcSlBoundary().end_s();
  const double max_s = 150.0;  // 最大s值
  const double unit_s = 1.0;    // s增量

  // projection of adc point onto reference line // 将车辆点投影到参考线
  const auto& adc_ref_point =
      reference_line_info->reference_line().GetReferencePoint(adc_s);

  // 确保车辆点有路径点信息
  DCHECK(adc_point.has_path_point());
  const double dx = adc_point.path_point().x() - adc_ref_point.x();  // x方向偏移
  const double dy = adc_point.path_point().y() - adc_ref_point.y();  // y方向偏移

  std::vector<common::PathPoint> path_points;
  // 从当前s值开始，生成未来的路径点
  for (double s = adc_s; s < max_s; s += unit_s) {
    const auto& ref_point =
        reference_line_info->reference_line().GetReferencePoint(adc_s);
    // 创建路径点，应用偏移
    common::PathPoint path_point = common::util::MakePathPoint(
        ref_point.x() + dx, ref_point.y() + dy, 0.0, ref_point.heading(),
        ref_point.kappa(), ref_point.dkappa(), 0.0);
    path_point.set_s(s);  // 设置s值

    path_points.push_back(std::move(path_point));  // 添加到路径点集合
  }
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));  // 设置路径数据
}

void EMPlanner::GenerateFallbackSpeedProfile(
    const ReferenceLineInfo* reference_line_info, SpeedData* speed_data) {
  // 使用多项式生成减速速度配置文件
  *speed_data = GenerateStopProfileFromPolynomial(
      reference_line_info->AdcPlanningPoint().v(),  // 获取当前速度
      reference_line_info->AdcPlanningPoint().a());  // 获取当前加速度

  // 如果生成的速度配置文件为空，则使用另一种方式生成
  if (speed_data->Empty()) {
    *speed_data = GenerateStopProfile(reference_line_info->AdcPlanningPoint().v(),
                                       reference_line_info->AdcPlanningPoint().a());
  }
}

SpeedData EMPlanner::GenerateStopProfile(const double init_speed,
                                         const double init_acc) const {
  AERROR << "Slowing down the car.";  // 输出调试信息，表示正在减速
  SpeedData speed_data;  // 创建速度数据对象

  const double kFixedJerk = -1.0;  // 固定的加加速度
  const double first_point_acc = std::fmin(0.0, init_acc);  // 第一个点的加速度

  const double max_t = 3.0;  // 最大时间
  const double unit_t = 0.02;  // 时间步长

  double pre_s = 0.0;  // 前一个位移
  // 计算中间时间和中间位移
  const double t_mid =
      (FLAGS_slowdown_profile_deceleration - first_point_acc) / kFixedJerk;
  const double s_mid = init_speed * t_mid +
                       0.5 * first_point_acc * t_mid * t_mid +
                       1.0 / 6.0 * kFixedJerk * t_mid * t_mid * t_mid;
  const double v_mid =
      init_speed + first_point_acc * t_mid + 0.5 * kFixedJerk * t_mid * t_mid;

  // 遍历时间，从0到最大时间
  for (double t = 0.0; t < max_t; t += unit_t) {
    double s = 0.0;  // 当前位移
    double v = 0.0;  // 当前速度

    // 如果当前时间小于等于中间时间
    if (t <= t_mid) {
      // 计算当前位移和速度
      s = std::fmax(pre_s, init_speed * t + 0.5 * first_point_acc * t * t +
                               1.0 / 6.0 * kFixedJerk * t * t * t);
      v = std::fmax(0.0, init_speed + first_point_acc * t + 0.5 * kFixedJerk * t * t);
      const double a = first_point_acc + kFixedJerk * t;  // 计算当前加速度
      speed_data.AppendSpeedPoint(s, t, v, a, 0.0);  // 添加速度点
      pre_s = s;  // 更新前一个位移
    } else {
      // 如果当前时间大于中间时间
      s = std::fmax(pre_s, s_mid + v_mid * (t - t_mid) +
                               0.5 * FLAGS_slowdown_profile_deceleration *
                                   (t - t_mid) * (t - t_mid));
      v = std::fmax(0.0,
                    v_mid + (t - t_mid) * FLAGS_slowdown_profile_deceleration);
      speed_data.AppendSpeedPoint(s, t, v, FLAGS_slowdown_profile_deceleration,
                                  0.0);  // 添加速度点
    }
    pre_s = s;  // 更新前一个位移
  }
  return speed_data;  // 返回生成的速度数据
}

SpeedData EMPlanner::GenerateStopProfileFromPolynomial(
    const double init_speed, const double init_acc) const {
  AERROR << "Slowing down the car with polynomial.";  // 输出调试信息，表示正在使用多项式减速
  constexpr double kMaxT = 4.0;  // 设置最大时间参数

  // 遍历时间参数，从2.0到kMaxT，每次增加0.5
  for (double t = 2.0; t <= kMaxT; t += 0.5) {
    // 遍历目标距离，从0到50，每次增加1.0
    for (double s = 0.0; s < 50.0; s += 1.0) {
      // 创建五次多项式曲线
      QuinticPolynomialCurve1d curve(0.0, init_speed, init_acc, s, 0.0, 0.0, t);
      // 检查曲线是否有效
      if (!IsValidProfile(curve)) {
        continue;  // 如果无效，继续下一个循环
      }

      constexpr double kUnitT = 0.02;  // 设置单位时间步长
      SpeedData speed_data;  // 创建速度数据对象

      // 遍历曲线的时间参数，从0到t，每次增加kUnitT
      for (double curve_t = 0.0; curve_t <= t; curve_t += kUnitT) {
        // 计算当前时间t的位移、速度、加速度和加速度变化率
        const double curve_s = curve.Evaluate(0, curve_t);
        const double curve_v = curve.Evaluate(1, curve_t);
        const double curve_a = curve.Evaluate(2, curve_t);
        const double curve_da = curve.Evaluate(3, curve_t);
        // 将计算结果添加到速度数据中
        speed_data.AppendSpeedPoint(curve_s, curve_t, curve_v, curve_a, curve_da);
      }
      return speed_data;  // 返回生成的速度数据
    }
  }
  return SpeedData();  // 如果没有有效的曲线，返回空的速度数据
}

bool EMPlanner::IsValidProfile(const QuinticPolynomialCurve1d& curve) const {
  // 遍历曲线的参数长度，检查速度和加速度的有效性
  for (double evaluate_t = 0.1; evaluate_t <= curve.ParamLength();
       evaluate_t += 0.2) {
    const double v = curve.Evaluate(1, evaluate_t);  // 计算当前参数t的速度
    const double a = curve.Evaluate(2, evaluate_t);  // 计算当前参数t的加速度
    constexpr double kEpsilon = 1e-3;  // 定义一个小的容忍值

    // 检查速度是否小于零或加速度是否小于-5.0
    if (v < -kEpsilon || a < -5.0) {
      return false;  // 如果不满足条件，则返回false
    }
  }
  return true;  // 如果所有点均有效，则返回true
}

}  // namespace planning
}  // namespace apollo
