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

#ifndef MODULES_PLANNING_PLANNER_PLANNER_H_
#define MODULES_PLANNING_PLANNER_PLANNER_H_

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/proto/planning_config.pb.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class Planner
 * @brief Planner is a base class for specific planners.
 *        It contains a pure virtual function Plan which must be implemented in
 * derived class.
 */

/**
 * @class Planner
 * @brief Planner 是一个具体规划器的基类。
 *        它包含一个纯虚函数 Plan，该函数必须在派生类中实现。
 */
class Planner {
 public:
  /**
   * @brief Constructor
   */
  Planner() = default;

  /**
   * @brief Destructor
   */
  virtual ~Planner() = default;

  virtual apollo::common::Status Init(const PlanningConfig& config) = 0;

  /**
   * @brief Compute trajectories for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */

   /**
 * @brief 计算执行轨迹。
 * @param planning_init_point 规划开始的轨迹点。
 * @param frame 当前规划帧。
 * @return 如果规划成功则返回OK；否则返回错误。
 */
  virtual apollo::common::Status Plan(
      const common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;

  /**
   * @brief Compute a trajectory for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */

  /**
 * @brief 计算执行轨迹。
 * @param planning_init_point 规划开始的轨迹点。
 * @param frame 当前规划帧。
 * @param reference_line_info 计算得到的参考线。
 * @return 如果规划成功则返回OK；否则返回错误。
 */
  virtual apollo::common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) = 0;
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNER_PLANNER_H_ */
