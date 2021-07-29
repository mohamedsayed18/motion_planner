/**
 * @file fix_state_collisions_task_generator.cpp
 * @brief Process generator for process that pushes plan instructions to be out of collision
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <trajopt/problem_description.hpp>
#include <tesseract_common/timer.h>
#include <tesseract_environment/core/utils.h>

#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/task_generators/fix_state_collision_task_generator.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/utils/filter_functions.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_process_managers/task_generators/fix_state_collision_task_generator.h>

namespace tesseract_planning
{
bool StateInCollision(const Eigen::Ref<const Eigen::VectorXd>& start_pos,
                      const TaskInput& input,
                      const FixStateCollisionProfile& profile,
                      tesseract_collision::ContactResultMap& contacts)
{
  using namespace tesseract_collision;
  using namespace tesseract_environment;

  auto env = input.env;
  auto kin = env->getManipulatorManager()->getFwdKinematicSolver(input.manip_info.manipulator);

  std::vector<ContactResultMap> collisions;
  DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();
  AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setCollisionMarginData(profile.collision_check_config.collision_margin_data);
  collisions.clear();

  tesseract_environment::EnvState::Ptr state = env->getState(kin->getJointNames(), start_pos);
  if (!checkTrajectoryState(collisions, *manager, state, profile.collision_check_config))
  {
    CONSOLE_BRIDGE_logDebug("No collisions found");
    if (profile.collision_check_config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
      CONSOLE_BRIDGE_logDebug("StateInCollision does not support longest valid segment logic");
    return false;
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Waypoint is not contact free!");
    for (std::size_t i = 0; i < collisions.size(); i++)
      for (const auto& contact_vec : collisions[i])
      {
        for (const auto& contact : contact_vec.second)
          CONSOLE_BRIDGE_logDebug(("timestep: " + std::to_string(i) + " Links: " + contact.link_names[0] + ", " +
                                   contact.link_names[1] + " Dist: " + std::to_string(contact.distance))
                                      .c_str());
        contacts = collisions[i];
      }
  }

  return true;
}

bool WaypointInCollision(const Waypoint& waypoint,
                         const TaskInput& input,
                         const FixStateCollisionProfile& profile,
                         tesseract_collision::ContactResultMap& contacts)
{
  // Get position associated with waypoint
  Eigen::VectorXd start_pos;
  try
  {
    start_pos = getJointPosition(waypoint);
  }
  catch (std::runtime_error& e)
  {
    CONSOLE_BRIDGE_logError("WaypointInCollision error: %s", e.what());
    return false;
  }
  return StateInCollision(start_pos, input, profile, contacts);
}

bool MoveWaypointFromCollisionTrajopt(Waypoint& waypoint,
                                      const TaskInput& input,
                                      const FixStateCollisionProfile& profile)
{
  using namespace trajopt;

  // Get position associated with waypoint
  Eigen::VectorXd start_pos;
  try
  {
    start_pos = getJointPosition(waypoint);
  }
  catch (std::runtime_error& e)
  {
    CONSOLE_BRIDGE_logError("MoveWaypointFromCollision error: %s", e.what());
    return false;
  }
  std::size_t num_jnts = static_cast<std::size_t>(start_pos.size());

  // Setup trajopt problem with basic info
  ProblemConstructionInfo pci(input.env);
  pci.basic_info.n_steps = 1;
  pci.basic_info.manip = input.manip_info.manipulator;
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.getManipulator(pci.basic_info.manip);

  // Initialize trajectory to waypoint position
  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = tesseract_common::TrajArray(1, start_pos.size());
  pci.init_info.data.row(0) = start_pos.transpose();

  // Add constraint that position is allowed to be within a tolerance of the original position
  {
    Eigen::MatrixX2d limits = pci.getManipulator(pci.basic_info.manip)->getLimits().joint_limits;
    Eigen::VectorXd range = limits.col(1).array() - limits.col(0).array();
    Eigen::VectorXd pos_tolerance = range * profile.jiggle_factor;
    Eigen::VectorXd neg_tolerance = range * -profile.jiggle_factor;

    auto jp = std::make_shared<JointPosTermInfo>();
    jp->coeffs = std::vector<double>(num_jnts, 1.0);
    jp->targets = std::vector<double>(start_pos.data(), start_pos.data() + start_pos.size());
    jp->upper_tols = std::vector<double>(pos_tolerance.data(), pos_tolerance.data() + pos_tolerance.size());
    jp->lower_tols = std::vector<double>(neg_tolerance.data(), neg_tolerance.data() + neg_tolerance.size());
    jp->first_step = 0;
    jp->last_step = 0;
    jp->name = "joint_pos";
    jp->term_type = TT_CNT;
    pci.cnt_infos.push_back(jp);
  }

  // Add a constraint that it must not be in collision
  {
    auto collision = std::make_shared<CollisionTermInfo>();
    collision->name = "collision";
    collision->term_type = TT_CNT;
    collision->evaluator_type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
    collision->first_step = 0;
    collision->last_step = 0;
    collision->info = createSafetyMarginDataVector(
        pci.basic_info.n_steps, profile.collision_check_config.collision_margin_data.getMaxCollisionMargin(), 1);
    collision->use_weighted_sum = true;
    pci.cnt_infos.push_back(collision);
  }
  // Add an additional cost to collisions to help it converge
  {
    auto collision = std::make_shared<CollisionTermInfo>();
    collision->name = "collision";
    collision->term_type = TT_COST;
    collision->evaluator_type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
    collision->first_step = 0;
    collision->last_step = 0;
    collision->info = createSafetyMarginDataVector(
        pci.basic_info.n_steps, profile.collision_check_config.collision_margin_data.getMaxCollisionMargin(), 20);
    collision->use_weighted_sum = true;
    pci.cost_infos.push_back(collision);
  }
  auto prob = ConstructProblem(pci);

  // Run trajopt optimization
  sco::BasicTrustRegionSQP opt(prob);
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();
  if (opt.results().status != sco::OptStatus::OPT_CONVERGED)
  {
    CONSOLE_BRIDGE_logError("MoveWaypointFromCollision did not converge");
    return false;
  }
  Eigen::VectorXd results(start_pos.size());
  results = getTraj(opt.x(), prob->GetVars()).row(0);
  return setJointPosition(waypoint, results);
}

bool MoveWaypointFromCollisionRandomSampler(Waypoint& waypoint,
                                            const TaskInput& input,
                                            const FixStateCollisionProfile& profile)
{
  // Get position associated with waypoint
  Eigen::VectorXd start_pos;
  try
  {
    start_pos = getJointPosition(waypoint);
  }
  catch (std::runtime_error& e)
  {
    CONSOLE_BRIDGE_logError("MoveWaypointFromCollision error: %s", e.what());
    return false;
  }

  const auto kin = input.env->getManipulatorManager()->getFwdKinematicSolver(input.manip_info.manipulator);
  Eigen::MatrixXd limits = kin->getLimits().joint_limits;
  Eigen::VectorXd range = limits.col(1).array() - limits.col(0).array();

  assert(start_pos.size() == range.size());
  for (int i = 0; i < profile.sampling_attempts; i++)
  {
    Eigen::VectorXd start_sampled_pos =
        start_pos + Eigen::VectorXd::Random(start_pos.size()).cwiseProduct(range) * profile.jiggle_factor;

    // Make sure it doesn't violate joint limits
    Eigen::VectorXd sampled_pos = start_sampled_pos;
    sampled_pos = sampled_pos.cwiseMax(limits.col(0));
    sampled_pos = sampled_pos.cwiseMin(limits.col(1));

    tesseract_collision::ContactResultMap contacts;
    if (!StateInCollision(sampled_pos, input, profile, contacts))
    {
      return setJointPosition(waypoint, sampled_pos);
    }
  }

  return false;
}

bool ApplyCorrectionWorkflow(Waypoint& waypoint,
                             const TaskInput& input,
                             const FixStateCollisionProfile& profile,
                             tesseract_collision::ContactResultMap& contacts)
{
  for (const auto& method : profile.correction_workflow)
  {
    switch (method)
    {
      case FixStateCollisionProfile::CorrectionMethod::NONE:
        return false;  // No correction and in collision, so return false
      case FixStateCollisionProfile::CorrectionMethod::TRAJOPT:
        if (MoveWaypointFromCollisionTrajopt(waypoint, input, profile))
          return true;
        break;
      case FixStateCollisionProfile::CorrectionMethod::RANDOM_SAMPLER:
        if (MoveWaypointFromCollisionRandomSampler(waypoint, input, profile))
          return true;
        break;
    }
  }
  // If all methods have tried without returning, then correction failed
  WaypointInCollision(waypoint, input, profile, contacts);
  return false;
}

FixStateCollisionTaskGenerator::FixStateCollisionTaskGenerator(std::string name) : TaskGenerator(std::move(name))
{
  // Register default profile
  auto default_profile = std::make_shared<FixStateCollisionProfile>();
  default_profile->collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  default_profile->collision_check_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  composite_profiles["DEFAULT"] = default_profile;
}

int FixStateCollisionTaskGenerator::conditionalProcess(TaskInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_shared<FixStateCollisionTaskInfo>(unique_id, name_);
  info->return_value = 0;
  input.addTaskInfo(info);
  tesseract_common::Timer timer;
  timer.start();
  saveInputs(info, input);

  // --------------------
  // Check that inputs are valid
  // --------------------
  const Instruction* input_intruction = input.getInstruction();
  if (!isCompositeInstruction(*(input_intruction)))
  {
    info->message = "Input seed to FixStateCollision must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    saveOutputs(info, input);
    info->elapsed_time = timer.elapsedSeconds();
    return 0;
  }

  const auto& ci = input_intruction->as<CompositeInstruction>();

  // Get Composite Profile
  std::string profile = ci.getProfile();
  profile = getProfileString(profile, name_, input.composite_profile_remapping);
  auto cur_composite_profile =
      getProfile<FixStateCollisionProfile>(profile, composite_profiles, std::make_shared<FixStateCollisionProfile>());
  cur_composite_profile = applyProfileOverrides(name_, cur_composite_profile, ci.profile_overrides);

  switch (cur_composite_profile->mode)
  {
    case FixStateCollisionProfile::Settings::START_ONLY:
    {
      const PlanInstruction* instr_const_ptr = getFirstPlanInstruction(ci);
      if (instr_const_ptr)
      {
        PlanInstruction* mutable_instruction = const_cast<PlanInstruction*>(instr_const_ptr);
        info->contact_results.resize(1);
        if (WaypointInCollision(
                mutable_instruction->getWaypoint(), input, *cur_composite_profile, info->contact_results[0]))
        {
          CONSOLE_BRIDGE_logInform("FixStateCollisionTaskGenerator is modifying the const input instructions");
          if (!ApplyCorrectionWorkflow(
                  mutable_instruction->getWaypoint(), input, *cur_composite_profile, info->contact_results[0]))
          {
            saveOutputs(info, input);
            info->elapsed_time = timer.elapsedSeconds();
            return 0;
          }
        }
      }
    }
    break;
    case FixStateCollisionProfile::Settings::END_ONLY:
    {
      const PlanInstruction* instr_const_ptr = getLastPlanInstruction(ci);
      if (instr_const_ptr)
      {
        PlanInstruction* mutable_instruction = const_cast<PlanInstruction*>(instr_const_ptr);
        info->contact_results.resize(1);
        if (WaypointInCollision(
                mutable_instruction->getWaypoint(), input, *cur_composite_profile, info->contact_results[0]))
        {
          CONSOLE_BRIDGE_logInform("FixStateCollisionTaskGenerator is modifying the const input instructions");
          if (!ApplyCorrectionWorkflow(
                  mutable_instruction->getWaypoint(), input, *cur_composite_profile, info->contact_results[0]))
          {
            saveOutputs(info, input);
            info->elapsed_time = timer.elapsedSeconds();
            return 0;
          }
        }
      }
    }
    break;
    case FixStateCollisionProfile::Settings::ALL:
    {
      auto flattened = flatten(ci, planFilter);
      info->contact_results.resize(flattened.size());
      if (flattened.empty())
      {
        CONSOLE_BRIDGE_logWarn("FixStateCollisionTaskGenerator found no PlanInstructions to process");
        info->return_value = 1;
        return 1;
      }

      bool in_collision = false;
      std::vector<bool> in_collision_vec(flattened.size());
      for (std::size_t i = 0; i < flattened.size(); i++)
      {
        in_collision_vec[i] = WaypointInCollision(flattened[i].get().as<PlanInstruction>().getWaypoint(),
                                                  input,
                                                  *cur_composite_profile,
                                                  info->contact_results[i]);
        in_collision |= in_collision_vec[i];
      }
      if (!in_collision)
        break;

      CONSOLE_BRIDGE_logInform("FixStateCollisionTaskGenerator is modifying the const input instructions");
      for (std::size_t i = 0; i < flattened.size(); i++)
      {
        if (in_collision_vec[i])
        {
          const Instruction* instr_const_ptr = &flattened[i].get();
          Instruction* mutable_instruction = const_cast<Instruction*>(instr_const_ptr);
          auto& plan = mutable_instruction->as<PlanInstruction>();

          if (!ApplyCorrectionWorkflow(plan.getWaypoint(), input, *cur_composite_profile, info->contact_results[i]))
          {
            saveOutputs(info, input);
            info->elapsed_time = timer.elapsedSeconds();
            return 0;
          }
        }
      }
    }
    break;
    case FixStateCollisionProfile::Settings::DISABLED:
      info->return_value = 1;
      saveOutputs(info, input);
      info->elapsed_time = timer.elapsedSeconds();
      return 1;
  }

  CONSOLE_BRIDGE_logDebug("FixStateCollisionTaskGenerator succeeded");
  info->return_value = 1;
  saveOutputs(info, input);
  info->elapsed_time = timer.elapsedSeconds();
  return 1;
}

void FixStateCollisionTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

FixStateCollisionTaskInfo::FixStateCollisionTaskInfo(std::size_t unique_id, std::string name)
  : TaskInfo(unique_id, std::move(name))
{
}

}  // namespace tesseract_planning
