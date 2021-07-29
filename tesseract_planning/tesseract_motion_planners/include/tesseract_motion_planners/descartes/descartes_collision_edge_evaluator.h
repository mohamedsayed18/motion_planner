/**
 * @file descartes_collision_edge_evaluator.h
 * @brief Tesseract Descartes Collision Edge Evaluator Implementation
 *
 * @author Levi Armstrong
 * @date December 18, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_EDGE_EVALUATOR_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_EDGE_EVALUATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
#include <descartes_light/core/edge_evaluator.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
template <typename FloatType>
class DescartesCollisionEdgeEvaluator : public descartes_light::EdgeEvaluator<FloatType>
{
public:
  DescartesCollisionEdgeEvaluator(const tesseract_environment::Environment::ConstPtr& collision_env,
                                  std::vector<std::string> active_links,
                                  std::vector<std::string> joint_names,
                                  tesseract_collision::CollisionCheckConfig config,
                                  bool allow_collision = false,
                                  bool debug = false);

  std::pair<bool, FloatType> evaluate(const descartes_light::State<FloatType>& start,
                                      const descartes_light::State<FloatType>& end) const override;

protected:
  /** @brief The tesseract state solver */
  tesseract_environment::StateSolver::ConstPtr state_solver_;
  /** @brief The allowed collision matrix */
  tesseract_scene_graph::AllowedCollisionMatrix acm_;
  /** @brief A vector of active link names */
  std::vector<std::string> active_link_names_;
  /** @brief A vector of joint names */
  std::vector<std::string> joint_names_;
  /** @brief The discrete contact manager */
  tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager_;
  /** @brief The discrete contact manager */
  tesseract_collision::ContinuousContactManager::Ptr continuous_contact_manager_;
  /** @brief The minimum allowed collision distance */
  tesseract_collision::CollisionCheckConfig collision_check_config_;
  /** @brief If true and no valid edges are found it will return the one with the lowest cost */
  bool allow_collision_;
  /** @brief Enable debug information to be printed to the terminal */
  bool debug_;

  /**
   * @brief Check if two links are allowed to be in collision
   * @param a The name of the first link
   * @param b The name of the second link
   * @return True if allowed to be in collision, otherwise false
   */
  bool isContactAllowed(const std::string& a, const std::string& b) const;

  // The member variables below are to cache the contact manager based on thread ID. Currently descartes is multi
  // threaded but the methods used to implement collision checking are not thread safe. To prevent
  // reconstructing the collision environment for every check this will cache a contact manager
  // based on its thread ID.

  /** @brief Contact manager caching mutex */
  mutable std::mutex mutex_;

  /** @brief The continuous contact manager cache */
  mutable std::map<unsigned long int, tesseract_collision::ContinuousContactManager::Ptr> continuous_contact_managers_;

  /** @brief The discrete contact manager cache */
  mutable std::map<unsigned long int, tesseract_collision::DiscreteContactManager::Ptr> discrete_contact_managers_;

  /** @brief The state solver manager cache */
  mutable std::map<unsigned long int, tesseract_environment::StateSolver::Ptr> state_solver_managers_;

  /**
   * @brief Perform a continuous collision check between two states
   * @param segment Trajectory containing two states
   * @param results Store results from collision check.
   * @return True if in collision otherwise false
   */
  bool continuousCollisionCheck(std::vector<tesseract_collision::ContactResultMap>& results,
                                const tesseract_common::TrajArray& segment,
                                bool find_best) const;

  /**
   * @brief Perform a continuous discrete check between two states
   * @param segment Trajectory containing two states
   * @param results Store results from collision check.
   * @return True if in collision otherwise false
   */
  bool discreteCollisionCheck(std::vector<tesseract_collision::ContactResultMap>& results,
                              const tesseract_common::TrajArray& segment,
                              bool find_best) const;
};

using DescartesCollisionEdgeEvaluatorF = DescartesCollisionEdgeEvaluator<float>;
using DescartesCollisionEdgeEvaluatorD = DescartesCollisionEdgeEvaluator<double>;

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_EDGE_EVALUATOR_H
