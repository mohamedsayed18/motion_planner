/**
 * @file interface utils.h
 * @brief Utilities that depend on the planners.
 *
 * @author Matthew Powelson
 * @date July 23, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_INTERFACE_UTILS_H
#define TESSERACT_MOTION_PLANNERS_INTERFACE_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>

namespace tesseract_planning
{
/** @brief Provided for backwards compatibility */
static CompositeInstruction generateSeed(const CompositeInstruction& instructions,
                                         const tesseract_scene_graph::SceneState& current_state,
                                         const tesseract_environment::Environment::ConstPtr& env,
                                         double state_longest_valid_segment_length = 5 * M_PI / 180,
                                         double translation_longest_valid_segment_length = 0.15,
                                         double rotation_longest_valid_segment_length = 5 * M_PI / 180,
                                         int min_steps = 1)
{
  // Fill out request and response
  PlannerRequest request;
  request.instructions = instructions;
  request.env_state = current_state;
  request.env = env;
  PlannerResponse response;

  // Set up planner
  SimpleMotionPlanner planner;

  auto profile = std::make_shared<SimplePlannerLVSPlanProfile>(state_longest_valid_segment_length,
                                                               translation_longest_valid_segment_length,
                                                               rotation_longest_valid_segment_length,
                                                               min_steps);
  planner.plan_profiles[instructions.getProfile()] = profile;
  auto flat = flattenProgram(instructions);
  for (const auto& i : flat)
    if (isPlanInstruction(i.get()))
      planner.plan_profiles[i.get().as<PlanInstruction>().getProfile()] = profile;

  // Solve
  planner.solve(request, response);

  return response.results;
}

}  // namespace tesseract_planning

#endif  // TESSERACT_PLANNING_UTILS_H
