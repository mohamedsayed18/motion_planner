/**
 * @file ompl_default_plan_profile.h
 * @brief Tesseract OMPL default plan profile
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_OMPL_DEFAULT_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_OMPL_OMPL_DEFAULT_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/utils.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/ompl/types.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::OMPLDefaultPlanProfile)
%ignore tesseract_planning::OMPLDefaultPlanProfile::allocWeightedRealVectorStateSampler;
#endif  // SWIG

namespace tesseract_planning
{
/**
 * @brief OMPL does not support the concept of multi waypoint planning like descartes and trajopt. Because of this
 * every plan instruction will be its a seperate ompl motion plan and therefore planning information is relevent
 * for this motion planner in the profile.
 */
class OMPLDefaultPlanProfile : public OMPLPlanProfile
{
public:
  using Ptr = std::shared_ptr<OMPLDefaultPlanProfile>;
  using ConstPtr = std::shared_ptr<const OMPLDefaultPlanProfile>;

  OMPLDefaultPlanProfile() = default;
  ~OMPLDefaultPlanProfile() override = default;
  OMPLDefaultPlanProfile(const OMPLDefaultPlanProfile&) = default;
  OMPLDefaultPlanProfile& operator=(const OMPLDefaultPlanProfile&) = default;
  OMPLDefaultPlanProfile(OMPLDefaultPlanProfile&&) noexcept = default;
  OMPLDefaultPlanProfile& operator=(OMPLDefaultPlanProfile&&) noexcept = default;
  OMPLDefaultPlanProfile(const tinyxml2::XMLElement& xml_element);

  /** @brief The OMPL state space to use when planning */
  OMPLProblemStateSpace state_space{ OMPLProblemStateSpace::REAL_STATE_SPACE };

  /** @brief Max planning time allowed in seconds */
  double planning_time = 5.0;

  /** @brief The max number of solutions. If max solutions are hit it will exit even if other threads are running. */
  int max_solutions = 10;

  /**
   * @brief Simplify trajectory.
   *
   * Note: If set to true it ignores n_output_states and returns the simplest trajectory.
   */
  bool simplify = false;

  /**
   * @brief This uses all available planning time to create the most optimized trajectory given the objective function.
   *
   * This is required because not all OMPL planners are optimize graph planners. If the planner you choose is an
   * optimize graph planner then setting this to true has no affect. In the case of non-optimize planners they still
   * use the OptimizeObjective function but only when searching the graph to find the most optimize solution based
   * on the provided optimize objective function. In the case of these type of planners like RRT and RRTConnect if set
   * to true it will leverage all planning time to keep finding solutions up to your max solutions count to find the
   * most optimal solution.
   */
  bool optimize = true;

  /**
   * @brief The planner configurators
   *
   * This defaults to two RRTConnectConfigurator
   *
   * This will create a new thread for each planner configurator provided. T
   */
  std::vector<OMPLPlannerConfigurator::ConstPtr> planners = { std::make_shared<const RRTConnectConfigurator>(),
                                                              std::make_shared<const RRTConnectConfigurator>() };

  /** @brief The collision check configuration */
  tesseract_collision::CollisionCheckConfig collision_check_config;

  /** @brief The state sampler allocator. This can be null and it will use Tesseract default state sampler allocator. */
  StateSamplerAllocator state_sampler_allocator;

  /** @brief Set the optimization objective function allocator. Default is to minimize path length */
  OptimizationObjectiveAllocator optimization_objective_allocator;

  /** @brief The ompl state validity checker. If nullptr and collision checking enabled it uses
   * StateCollisionValidator */
  StateValidityCheckerAllocator svc_allocator;

  /** @brief The ompl motion validator. If nullptr and continuous collision checking enabled it used
   * ContinuousMotionValidator */
  MotionValidatorAllocator mv_allocator;

  void setup(OMPLProblem& prob) const override;

  void applyGoalStates(OMPLProblem& prob,
                       const Eigen::Isometry3d& cartesian_waypoint,
                       const Instruction& parent_instruction,
                       const ManipulatorInfo& manip_info,
                       const std::vector<std::string>& active_links,
                       int index) const override;

  void applyGoalStates(OMPLProblem& prob,
                       const Eigen::VectorXd& joint_waypoint,
                       const Instruction& parent_instruction,
                       const ManipulatorInfo& manip_info,
                       const std::vector<std::string>& active_links,
                       int index) const override;

  void applyStartStates(OMPLProblem& prob,
                        const Eigen::Isometry3d& cartesian_waypoint,
                        const Instruction& parent_instruction,
                        const ManipulatorInfo& manip_info,
                        const std::vector<std::string>& active_links,
                        int index) const override;

  void applyStartStates(OMPLProblem& prob,
                        const Eigen::VectorXd& joint_waypoint,
                        const Instruction& parent_instruction,
                        const ManipulatorInfo& manip_info,
                        const std::vector<std::string>& active_links,
                        int index) const override;

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const override;

protected:
  ompl::base::StateValidityCheckerPtr processStateValidator(OMPLProblem& prob) const;
  void processMotionValidator(OMPLProblem& prob,
                              const ompl::base::StateValidityCheckerPtr& svc_without_collision) const;
  void processOptimizationObjective(OMPLProblem& prob) const;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_OMPL_OMPL_DEFAULT_PLAN_PROFILE_H
