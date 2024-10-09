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

/**
 * @file
 **/

#include "modules/planning/planner/lattice/lattice_planner.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/collision_checker.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/lattice/behavior/path_time_graph.h"
#include "modules/planning/lattice/behavior/prediction_querier.h"
#include "modules/planning/lattice/trajectory1d/lattice_trajectory1d.h"
#include "modules/planning/lattice/trajectory_generation/backup_trajectory_generator.h"
#include "modules/planning/lattice/trajectory_generation/trajectory1d_generator.h"
#include "modules/planning/lattice/trajectory_generation/trajectory_combiner.h"
#include "modules/planning/lattice/trajectory_generation/trajectory_evaluator.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::adapter::AdapterManager;
using apollo::common::math::PathMatcher;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::time::Clock;

namespace {

// 将参考点转换为离散化的路径点
std::vector<PathPoint> ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points) {  // 输入为一系列参考点
  double s = 0.0;  // 初始化 Frenet s 坐标
  std::vector<PathPoint> path_points;  // 存储离散化后的路径点

  // 遍历所有参考点
  for (const auto& ref_point : ref_points) {
    PathPoint path_point;  // 创建一个新的路径点
    path_point.set_x(ref_point.x());  // 设置路径点的 x 坐标
    path_point.set_y(ref_point.y());  // 设置路径点的 y 坐标
    path_point.set_theta(ref_point.heading());  // 设置路径点的朝向角度
    path_point.set_kappa(ref_point.kappa());  // 设置路径点的曲率
    path_point.set_dkappa(ref_point.dkappa());  // 设置路径点的曲率导数

    // 如果路径点集合不为空，计算当前路径点与上一个路径点的距离
    if (!path_points.empty()) {
      double dx = path_point.x() - path_points.back().x();  // x 坐标差
      double dy = path_point.y() - path_points.back().y();  // y 坐标差
      s += std::sqrt(dx * dx + dy * dy);  // 更新 Frenet s 坐标
    }
    path_point.set_s(s);  // 设置路径点的 s 坐标
    path_points.push_back(std::move(path_point));  // 将路径点添加到集合中
  }
  return path_points;  // 返回离散化后的路径点集合
}

// 计算初始的 Frenet 状态
void ComputeInitFrenetState(const PathPoint& matched_point,  // 匹配的路径点
                            const TrajectoryPoint& cartesian_state,  // 轨迹点（笛卡尔坐标系）
                            std::array<double, 3>* ptr_s,  // 指向存储 Frenet s 状态的指针
                            std::array<double, 3>* ptr_d) {  // 指向存储 Frenet d 状态的指针
  // 调用 CartesianFrenetConverter 将笛卡尔坐标转换为 Frenet 坐标
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s(),  // 匹配点的 s 坐标
      matched_point.x(),  // 匹配点的 x 坐标
      matched_point.y(),  // 匹配点的 y 坐标
      matched_point.theta(),  // 匹配点的朝向角度
      matched_point.kappa(),  // 匹配点的曲率
      matched_point.dkappa(),  // 匹配点的曲率导数
      cartesian_state.path_point().x(),  // 当前轨迹点的 x 坐标
      cartesian_state.path_point().y(),  // 当前轨迹点的 y 坐标
      cartesian_state.v(),  // 当前轨迹点的速度
      cartesian_state.a(),  // 当前轨迹点的加速度
      cartesian_state.path_point().theta(),  // 当前轨迹点的朝向角度
      cartesian_state.path_point().kappa(),  // 当前轨迹点的曲率
      ptr_s,  // 指向 Frenet s 状态的指针
      ptr_d   // 指向 Frenet d 状态的指针
  );
}

}  // namespace

Status LatticePlanner::Plan(const TrajectoryPoint& planning_start_point,
                            Frame* frame) {
  // 记录成功规划的参考线数量
  std::size_t success_line_count = 0;
  // 参考线索引
  std::size_t index = 0;

  // 遍历帧中的所有参考线信息
  for (auto& reference_line_info : frame->reference_line_info()) {
    // 设置非优先参考线的成本
    if (index != 0) {
      reference_line_info.SetPriorityCost(
          FLAGS_cost_non_priority_reference_line);
    } else {
      // 第一个参考线的优先成本设置为0
      reference_line_info.SetPriorityCost(0.0);
    }

    // 在参考线上进行规划
    auto status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

    // 检查规划状态是否成功
    if (status != Status::OK()) {
      // 如果是变道路径，记录错误信息
      if (reference_line_info.IsChangeLanePath()) {
        AERROR << "Planner failed to change lane to "
               << reference_line_info.Lanes().Id();
      } else {
        // 记录其他规划失败的情况
        AERROR << "Planner failed to " << reference_line_info.Lanes().Id();
      }
    } else {
      // 成功规划，增加成功数量
      success_line_count += 1;
    }
    // 索引自增
    ++index;
  }

  // 如果至少有一条参考线成功规划，返回成功状态
  if (success_line_count > 0) {
    return Status::OK();
  }
  // 如果没有任何参考线成功规划，返回错误状态
  return Status(ErrorCode::PLANNING_ERROR,
                "Failed to plan on any reference line.");
}

Status LatticePlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  static std::size_t num_planning_cycles = 0;  // 规划周期计数
  static std::size_t num_planning_succeeded_cycles = 0;  // 成功规划周期计数

  double start_time = Clock::NowInSeconds();  // 记录开始时间
  double current_time = start_time;  // 当前时间

  ADEBUG << "Number of planning cycles: " << num_planning_cycles << " "
         << num_planning_succeeded_cycles;
  ++num_planning_cycles;  // 增加规划周期计数

  reference_line_info->set_is_on_reference_line();  // 标记为在参考线上
   // 1. obtain a reference line and transform it to the PathPoint format. // 1. 获取参考线并转换为 PathPoint 格式
  auto ptr_reference_line =
      std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(
          reference_line_info->reference_line().reference_points()));

  // 2. compute the matched point of the init planning point on the reference line. // 2. 计算初始化规划点在参考线上的匹配点
  PathPoint matched_point = PathMatcher::MatchToPath(
      *ptr_reference_line, planning_init_point.path_point().x(),
      planning_init_point.path_point().y());

  // 3. according to the matched point, compute the init state in Frenet frame.// 3. 根据匹配点计算 Frenet 坐标系下的初始化状态
  std::array<double, 3> init_s;  // s 方向状态
  std::array<double, 3> init_d;  // d 方向状态
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

  ADEBUG << "ReferenceLine and Frenet Conversion Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();  // 更新当前时间

  auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(
      frame->obstacles(), ptr_reference_line);  // 创建预测查询器

   // 4. parse the decision and get the planning target.// 4. 解析决策并获取规划目标
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
      ptr_prediction_querier->GetObstacles(),
      *ptr_reference_line,
      reference_line_info,
      init_s[0], init_s[0] + FLAGS_decision_horizon,
      0.0, FLAGS_trajectory_time_length);

  PlanningTarget planning_target = reference_line_info->planning_target();
  if (planning_target.has_stop_point()) {
    ADEBUG << "Planning target stop s: " << planning_target.stop_point().s()
           << "Current ego s: " << init_s[0];  // 如果有停止点，记录目标和当前状态
  }

  ADEBUG << "Decision_Time = " << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();  // 更新当前时间

  // 5. generate 1d trajectory bundle for longitudinal and lateral respectively. // 5. 生成纵向和横向的 1D 轨迹包
  Trajectory1dGenerator trajectory1d_generator(
      init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;  // 纵向轨迹包
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;  // 横向轨迹包
  trajectory1d_generator.GenerateTrajectoryBundles(
      planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

  ADEBUG << "Trajectory_Generation_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();  // 更新当前时间


  // 6. first, evaluate the feasibility of the 1d trajectories according to
  // dynamic constraints.
  //   second, evaluate the feasible longitudinal and lateral trajectory pairs
  //   and sort them according to the cost.
  // 6. 评估 1D 轨迹的可行性
  //   先评估动态约束下的可行性
  //   然后评估可行的纵向和横向轨迹对并按成本排序
  TrajectoryEvaluator trajectory_evaluator(
      init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
      ptr_path_time_graph, ptr_reference_line);

  ADEBUG << "Trajectory_Evaluator_Construction_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();  // 更新当前时间

  ADEBUG << "number of trajectory pairs = "
         << trajectory_evaluator.num_of_trajectory_pairs()
         << "  number_lon_traj = " << lon_trajectory1d_bundle.size()
         << "  number_lat_traj = " << lat_trajectory1d_bundle.size();

  // Get instance of collision checker and constraint checker // 获取碰撞检查器和约束检查器实例
  CollisionChecker collision_checker(frame->obstacles(), init_s[0], init_d[0],
                                     *ptr_reference_line, reference_line_info,
                                     ptr_path_time_graph);
  // 7. always get the best pair of trajectories to combine; return the first
  // collision-free trajectory.
  // 7. 始终获取最佳轨迹对进行组合；返回第一个无碰撞的轨迹
  std::size_t constraint_failure_count = 0;  // 约束失败计数
  std::size_t collision_failure_count = 0;  // 碰撞失败计数
  std::size_t combined_constraint_failure_count = 0;  // 组合约束失败计数

  std::size_t num_lattice_traj = 0;  // 有效轨迹计数
  while (trajectory_evaluator.has_more_trajectory_pairs()) {
    double trajectory_pair_cost = 
        trajectory_evaluator.top_trajectory_pair_cost();  // 获取当前轨迹对的成本
    // For auto tuning // 用于自动调优
    std::vector<double> trajectory_pair_cost_components;
    if (FLAGS_enable_auto_tuning) {
      trajectory_pair_cost_components = 
          trajectory_evaluator.top_trajectory_pair_component_cost();
      ADEBUG << "TrajectoryPairComponentCost";
      ADEBUG << "travel_cost = " << trajectory_pair_cost_components[0];
      ADEBUG << "jerk_cost = " << trajectory_pair_cost_components[1];
      ADEBUG << "obstacle_cost = " << trajectory_pair_cost_components[2];
      ADEBUG << "lateral_cost = " << trajectory_pair_cost_components[3];
    }
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();  // 获取下一个轨迹对

    // combine two 1d trajectories to one 2d trajectory // 将两个 1D 轨迹组合为一个 2D 轨迹
    auto combined_trajectory = TrajectoryCombiner::Combine(
        *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second,
        planning_init_point.relative_time());

    // check longitudinal and lateral acceleration
    // considering trajectory curvatures
    // 检查纵向和横向加速度，考虑轨迹曲率
    if (!ConstraintChecker::ValidTrajectory(combined_trajectory)) {
      ++combined_constraint_failure_count;  // 增加组合约束失败计数
      continue;  // 如果无效，则继续下一个轨迹对
    }

    // check collision with other obstacles // 检查与其他障碍物的碰撞
    if (collision_checker.InCollision(combined_trajectory)) {
      ++collision_failure_count;  // 增加碰撞失败计数
      continue;  // 如果碰撞，则继续下一个轨迹对
    }

    // put combine trajectory into debug data // 将组合轨迹放入调试数据中
    const auto& combined_trajectory_points = 
        combined_trajectory.trajectory_points();
    num_lattice_traj += 1;  // 有效轨迹计数加一
    reference_line_info->SetTrajectory(combined_trajectory);  // 设置参考线轨迹
    reference_line_info->SetCost(reference_line_info->PriorityCost() + 
                                trajectory_pair_cost);  // 设置轨迹成本
    reference_line_info->SetDrivable(true);  // 设置可行驶标志

    // Auto Tuning // 自动调优
    if (AdapterManager::GetLocalization() == nullptr) {
      AERROR << "Auto tuning failed since no localization is available.";
    } else if (FLAGS_enable_auto_tuning) {
      // 1. Get future trajectory from localization // 1. 从定位中获取未来轨迹
      DiscretizedTrajectory future_trajectory = GetFutureTrajectory();
      // 2. Map future trajectory to lon-lat trajectory pair // 2. 将未来轨迹映射到纵向-横向轨迹对
      std::vector<common::SpeedPoint> lon_future_trajectory;
      std::vector<common::FrenetFramePoint> lat_future_trajectory;
      if (!MapFutureTrajectoryToSL(future_trajectory, *ptr_reference_line,
                                   &lon_future_trajectory,
                                   &lat_future_trajectory)) {
        AERROR << "Auto tuning failed since no mapping "
               << "from future trajectory to lon-lat";
      }
      // 3. evaluate cost // 3. 评估成本
      std::vector<double> future_traj_component_cost;
      trajectory_evaluator.EvaluateDiscreteTrajectory(
          planning_target, lon_future_trajectory, lat_future_trajectory,
          &future_traj_component_cost);

      // 4. emit // 4. 生成调试数据
      planning_internal::PlanningData* ptr_debug =
          reference_line_info->mutable_debug()->mutable_planning_data();

      apollo::planning_internal::AutoTuningTrainingData auto_tuning_data;

      // 将学生成本组件添加到调试数据
      for (double student_cost_component : trajectory_pair_cost_components) {
        auto_tuning_data.mutable_student_component()->add_cost_component(
            student_cost_component);
      }

      // 将教师成本组件添加到调试数据
      for (double teacher_cost_component : future_traj_component_cost) {
        auto_tuning_data.mutable_teacher_component()->add_cost_component(
            teacher_cost_component);
      }

      ptr_debug->mutable_auto_tuning_training_data()->CopyFrom(
          auto_tuning_data);  // 复制调试数据
    }

    // Print the chosen end condition and start condition // 打印选择的结束条件和开始条件
    ADEBUG << "Starting Lon. State: s = " << init_s[0] << " ds = " << init_s[1] 
          << " dds = " << init_s[2];
    // cast // 动态转换轨迹指针
    auto lattice_traj_ptr =
        std::dynamic_pointer_cast<LatticeTrajectory1d>(trajectory_pair.first);
    if (!lattice_traj_ptr) {
      ADEBUG << "Dynamically casting trajectory1d ptr. failed.";
    }

    if (lattice_traj_ptr->has_target_position()) {
      ADEBUG << "Ending Lon. State s = " << lattice_traj_ptr->target_position()
             << " ds = " << lattice_traj_ptr->target_velocity()
             << " t = " << lattice_traj_ptr->target_time();
    }

    // 打印输入信息
    ADEBUG << "InputPose";
    ADEBUG << "XY: " << planning_init_point.ShortDebugString();
    ADEBUG << "S: (" << init_s[0] << ", " << init_s[1] << "," << init_s[2] << ")";
    ADEBUG << "L: (" << init_d[0] << ", " << init_d[1] << "," << init_d[2] << ")";

    ADEBUG << "Reference_line_priority_cost = " << reference_line_info->PriorityCost();
    ADEBUG << "Total_Trajectory_Cost = " << trajectory_pair_cost;
    ADEBUG << "OutputTrajectory";
    for (uint i = 0; i < 10; ++i) {
      ADEBUG << combined_trajectory_points[i].ShortDebugString();  // 打印组合轨迹前10个点
    }

    break;  // 找到有效轨迹后跳出循环

    /*
    auto combined_trajectory_path =
        ptr_debug->mutable_planning_data()->add_trajectory_path();
    for (uint i = 0; i < combined_trajectory_points.size(); ++i) {
      combined_trajectory_path->add_trajectory_point()->CopyFrom(
          combined_trajectory_points[i]);
    }
    combined_trajectory_path->set_lattice_trajectory_cost(trajectory_pair_cost);
    */
  }

  ADEBUG << "Trajectory_Evaluation_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;

  ADEBUG << "Step CombineTrajectory Succeeded";

  // 打印约束和碰撞失败统计信息
  ADEBUG << "1d trajectory not valid for constraint [" << constraint_failure_count << "] times";
  ADEBUG << "Combined trajectory not valid for [" << combined_constraint_failure_count << "] times";
  ADEBUG << "Trajectory not valid for collision [" << collision_failure_count << "] times";
  ADEBUG << "Total_Lattice_Planning_Frame_Time = "
         << (Clock::NowInSeconds() - start_time) * 1000;

  // 如果有有效的轨迹，返回成功状态
  if (num_lattice_traj > 0) {
    ADEBUG << "Planning succeeded";
    num_planning_succeeded_cycles += 1;  // 成功规划周期计数增加
    reference_line_info->SetDrivable(true);  // 设置可行驶标志
    return Status::OK();
  } else {
    // 如果没有有效轨迹，返回失败状态
    AERROR << "Planning failed";
    if (FLAGS_enable_backup_trajectory) {
      AERROR << "Use backup trajectory";  // 如果启用备用轨迹，生成备用轨迹
      BackupTrajectoryGenerator backup_trajectory_generator(
          init_s, init_d, planning_init_point.relative_time(),
          std::make_shared<CollisionChecker>(collision_checker),
          &trajectory1d_generator);
      DiscretizedTrajectory trajectory =
          backup_trajectory_generator.GenerateTrajectory(*ptr_reference_line);

      reference_line_info->AddCost(FLAGS_backup_trajectory_cost);  // 添加备用轨迹成本
      reference_line_info->SetTrajectory(trajectory);  // 设置备用轨迹
      reference_line_info->SetDrivable(true);  // 设置可行驶标志
      return Status::OK();  // 返回成功状态
    } else {
      reference_line_info->SetCost(std::numeric_limits<double>::infinity());  // 设置成本为无穷大
    }
    return Status(ErrorCode::PLANNING_ERROR, "No feasible trajectories");  // 返回规划错误状态
  }
}

DiscretizedTrajectory LatticePlanner::GetFutureTrajectory() const {
   // localizatio // 获取最新的定位信息
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Get localization:" << localization.DebugString();  // 输出定位信息调试信息

  std::vector<TrajectoryPoint> traj_pts;  // 存储轨迹点的容器
  // 遍历定位信息中的每个轨迹点
  for (const auto& traj_pt : localization.trajectory_point()) {
    traj_pts.emplace_back(traj_pt);  // 将轨迹点添加到容器中
  }

  // 创建离散化轨迹并返回
  DiscretizedTrajectory ret(traj_pts);
  return ret;
}

bool LatticePlanner::MapFutureTrajectoryToSL(
    const DiscretizedTrajectory& future_trajectory,
    const std::vector<PathPoint>& discretized_reference_line,
    std::vector<apollo::common::SpeedPoint>* st_points,
    std::vector<apollo::common::FrenetFramePoint>* sl_points) {
  // 检查离散化参考线是否为空
  if (0 == discretized_reference_line.size()) {
    AERROR << "MapFutureTrajectoryToSL error";  // 输出错误信息
    return false;  // 返回失败状态
  }

  // 遍历未来轨迹中的每一个轨迹点
  for (const common::TrajectoryPoint& trajectory_point :
       future_trajectory.trajectory_points()) {
    const PathPoint& path_point = trajectory_point.path_point();  // 获取轨迹点的路径点

    // 将路径点匹配到离散化参考线
    PathPoint matched_point = PathMatcher::MatchToPath(
        discretized_reference_line, path_point.x(), path_point.y());

    // 初始化Frenet坐标系中的状态
    std::array<double, 3> pose_s;  // s坐标系状态
    std::array<double, 3> pose_d;  // d坐标系状态
    ComputeInitFrenetState(matched_point, trajectory_point, &pose_s, &pose_d);

    // 创建速度点和Frenet帧点
    apollo::common::SpeedPoint st_point;
    apollo::common::FrenetFramePoint sl_point;

    // 设置s坐标系的状态
    st_point.set_s(pose_s[0]);  // 设置s位置
    st_point.set_t(trajectory_point.relative_time());  // 设置相对时间
    st_point.set_v(pose_s[1]);  // 设置速度
    st_point.set_a(pose_s[2]);  // 设置加速度（不设置加加速度da）

    // 设置d坐标系的状态
    sl_point.set_s(pose_s[0]);  // 设置s位置
    sl_point.set_l(pose_d[0]);  // 设置l位置
    sl_point.set_dl(pose_d[0]);  // 设置l的变化率
    sl_point.set_ddl(pose_d[0]);  // 设置l的加速度

    // 将计算得到的速度点和Frenet帧点添加到对应的输出向量中
    st_points->emplace_back(std::move(st_point));
    sl_points->emplace_back(std::move(sl_point));
  }

  return true;  // 返回成功状态
}

}  // namespace planning
}  // namespace apollo
