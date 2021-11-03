/**
 * @file utils.h
 * @brief Tesseract OMPL planner utility functions
 *
 * @author Levi Armstrong
 * @date February 17, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_UTILS_H
#define TESSERACT_MOTION_PLANNERS_OMPL_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <Eigen/Geometry>
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_motion_planners/ompl/ompl_problem.h>

namespace tesseract_planning
{
Eigen::Map<Eigen::VectorXd> RealVectorStateSpaceExtractor(const ompl::base::State* s1, unsigned dimension);

#ifndef OMPL_LESS_1_4_0
Eigen::Map<Eigen::VectorXd> ConstrainedStateSpaceExtractor(const ompl::base::State* s1);
#endif

/**
 * @brief Convert an ompl path to tesseract TrajArray
 * @param path OMPL Path
 * @param extractor This function understands the type of state space and converts it to an eigen vector.
 * @return Tesseract TrajArray
 */
tesseract_common::TrajArray toTrajArray(const ompl::geometric::PathGeometric& path,
                                        const OMPLStateExtractor& extractor);

/**
 * @brief Given longest valid fraction and length it will set the correct information of the state space
 * @param state_space_ptr OMPL State Space
 * @param longest_valid_segment_fraction
 * @param longest_valid_segment_length
 */
void processLongestValidSegment(const ompl::base::StateSpacePtr& state_space_ptr,
                                double longest_valid_segment_fraction,
                                double longest_valid_segment_length);

/**
 * @brief Given collision check config set ompl longest_valid_segment_fraction
 * @param state_space_ptr OMPL State Space
 * @param collision_check_config
 */
void processLongestValidSegment(const ompl::base::StateSpacePtr& state_space_ptr,
                                const tesseract_collision::CollisionCheckConfig& collision_check_config);

/**
 * @brief For the provided problem check if the state is in collision
 * @param prob The OMPL Problem
 * @param state The joint state
 * @param contact_map Map of contact results. Will be empty if return true
 * @return True if in collision otherwise false
 */
bool checkStateInCollision(OMPLProblem& prob,
                           const Eigen::VectorXd& state,
                           tesseract_collision::ContactResultMap& contact_map);

/**
 * @brief For the provided problem check if the state is in collision
 * @param prob The OMPL Problem
 * @param state The joint state
 * @return True if in collision otherwise false
 */
bool checkStateInCollision(OMPLProblem& prob, const Eigen::VectorXd& state);

/**
 * @brief Default State sampler which uses the weights information to scale the sampled state. This is use full
 * when you state space has mixed units like meters and radian.
 * @param space The ompl state space.
 * @return OMPL state sampler shared pointer
 */
ompl::base::StateSamplerPtr allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space,
                                                                const Eigen::VectorXd& weights,
                                                                const Eigen::MatrixX2d& limits);

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_UTILS_H
