/**
 * @file freespace_hybrid_example.h
 * @brief An example of a feespace motion planning with OMPL then TrajOpt
 *
 * @author Levi Armstrong
 * @date March 16, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ABB_PLANNER_EXAMPLE_H
#define ABB_PLANNER_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/example.h>

namespace tesseract_ros_examples
{
  /**
 * @brief An example of a robot leveraging OMPL RRTConnect to generate a freespace motion trajectory the TrajOpt to
 * optimize
 */
  class AbbPlannerExample : public Example
  {
  public:
    AbbPlannerExample(const ros::NodeHandle &nh, bool plotting, bool rviz, double range, double planning_time);
    ~AbbPlannerExample() override = default;
    AbbPlannerExample(const AbbPlannerExample &) = default;
    AbbPlannerExample &operator=(const AbbPlannerExample &) = default;
    AbbPlannerExample(AbbPlannerExample &&) = default;
    AbbPlannerExample &operator=(AbbPlannerExample &&) = default;

    tesseract_common::VectorIsometry3d getPoses();

    bool run() override;

  private:
    ros::NodeHandle nh_;
    double range_;
    double planning_time_;
  };

} // namespace tesseract_ros_examples

#endif // TESSERACT_ROS_FREESPACE_HYBRID_EXAMPLE_H
