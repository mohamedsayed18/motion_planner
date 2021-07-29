/**
 * @file ompl_planner_tests.cpp
 * @brief This contains unit test for the tesseract descartes planner
 *
 * @author Levi Armstrong, Jonathan Meyer
 * @date September 16, 2019
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>

#include <ompl/util/RandomNumbers.h>

#include <functional>
#include <cmath>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/interface_utils.h>

using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;
using namespace tesseract_geometry;
using namespace tesseract_kinematics;
using namespace tesseract_planning;

const static int SEED = 1;
const static std::vector<double> start_state = { -0.5, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
const static std::vector<double> end_state = { 0.5, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

static void addBox(tesseract_environment::Environment& env)
{
  Link link_1("box_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.5, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Box>(0.4, 0.001, 0.4);
  link_1.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_1.collision.push_back(collision);

  Joint joint_1("joint_n1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = link_1.getName();
  joint_1.type = JointType::FIXED;

  env.addLink(link_1, joint_1);
}

template <typename Configurator>
class OMPLTestFixture : public ::testing::Test
{
public:
  OMPLTestFixture() { configurator = std::make_shared<Configurator>(); }
  using ::testing::Test::Test;
  std::shared_ptr<Configurator> configurator;
  OMPLMotionPlanner ompl_planner;
};

using Implementations = ::testing::Types<tesseract_planning::SBLConfigurator,
                                         tesseract_planning::PRMConfigurator,
                                         tesseract_planning::PRMstarConfigurator,
                                         tesseract_planning::LazyPRMstarConfigurator,
                                         tesseract_planning::ESTConfigurator,
                                         tesseract_planning::LBKPIECE1Configurator,
                                         tesseract_planning::BKPIECE1Configurator,
                                         tesseract_planning::KPIECE1Configurator,
                                         // tesseract_planning::RRTConfigurator,
                                         // tesseract_planning::RRTstarConfigurator,
                                         // tesseract_planning::SPARSConfigurator,
                                         // tesseract_planning::TRRTConfigurator,
                                         tesseract_planning::RRTConnectConfigurator>;

TYPED_TEST_CASE(OMPLTestFixture, Implementations);

TYPED_TEST(OMPLTestFixture, OMPLFreespacePlannerUnit)
{
  EXPECT_EQ(ompl::RNG::getSeed(), SEED) << "Randomization seed does not match expected: " << ompl::RNG::getSeed()
                                        << " vs. " << SEED;

  // Step 1: Load scene and srdf
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  Environment::Ptr env = std::make_shared<Environment>();
  tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_path, srdf_path, locator));

  ManipulatorInfo manip;
  manip.manipulator = "manipulator";

  // Step 2: Add box to environment
  addBox(*env);

  // Step 3: Create ompl planner config and populate it
  auto fwd_kin = env->getManipulatorManager()->getFwdKinematicSolver(manip.manipulator);
  auto inv_kin = env->getManipulatorManager()->getInvKinematicSolver(manip.manipulator);

  // Synchronize the inverse kinematics with the forward kinematics
  inv_kin->synchronize(fwd_kin);

  auto cur_state = env->getCurrentState();

  // Specify a start waypoint
  JointWaypoint wp1(fwd_kin->getJointNames(),
                    Eigen::Map<const Eigen::VectorXd>(start_state.data(), static_cast<long>(start_state.size())));

  // Specify a end waypoint
  JointWaypoint wp2(fwd_kin->getJointNames(),
                    Eigen::Map<const Eigen::VectorXd>(end_state.data(), static_cast<long>(end_state.size())));

  // Define Start Instruction
  PlanInstruction start_instruction(wp1, PlanInstructionType::START, "TEST_PROFILE");

  // Define Plan Instructions
  PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, "TEST_PROFILE");
  PlanInstruction plan_f2(wp1, PlanInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a program
  CompositeInstruction program;
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(manip);
  program.push_back(plan_f1);
  program.push_back(plan_f2);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, env, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<OMPLDefaultPlanProfile>();
  plan_profile->collision_check_config.collision_margin_override_type =
      tesseract_collision::CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN;
  plan_profile->collision_check_config.collision_margin_data.setDefaultCollisionMargin(0.025);
  plan_profile->collision_check_config.longest_valid_segment_length = 0.1;
  plan_profile->collision_check_config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  plan_profile->planning_time = 10;
  plan_profile->optimize = false;
  plan_profile->max_solutions = 2;
  plan_profile->simplify = false;
  plan_profile->planners = { this->configurator, this->configurator };

  // Create Planner
  OMPLMotionPlanner ompl_planner;
  ompl_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
  ompl_planner.problem_generator = &DefaultOMPLProblemGenerator;

  // Create Planner Request
  PlannerRequest request;
  request.instructions = program;
  request.seed = seed;
  request.env = env;
  request.env_state = env->getCurrentState();

  PlannerResponse planner_response;
  auto status = ompl_planner.solve(request, planner_response);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("CI Error: %s", status.message().c_str());
  }

  EXPECT_TRUE(&status);
  EXPECT_TRUE(planner_response.results.hasStartInstruction());
  EXPECT_EQ(getMoveInstructionCount(planner_response.results), 21);  // 10 per segment + start a instruction
  EXPECT_EQ(planner_response.results.size(), 2);
  EXPECT_TRUE(wp1.isApprox(getJointPosition(getFirstMoveInstruction(planner_response.results)->getWaypoint()), 1e-5));
  EXPECT_TRUE(wp2.isApprox(
      getJointPosition(
          getLastMoveInstruction(planner_response.results.front().as<CompositeInstruction>())->getWaypoint()),
      1e-5));
  EXPECT_TRUE(wp1.isApprox(getJointPosition(getLastMoveInstruction(planner_response.results)->getWaypoint()), 1e-5));

  // Check for start state in collision error
  std::vector<double> swp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };

  // Define New Start Instruction
  wp1 = Eigen::Map<const Eigen::VectorXd>(swp.data(), static_cast<long>(swp.size()));
  wp1.joint_names = fwd_kin->getJointNames();

  start_instruction = PlanInstruction(wp1, PlanInstructionType::START, "TEST_PROFILE");

  // Create a new program
  program = CompositeInstruction();
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(manip);
  program.push_back(plan_f1);

  // Create a new seed
  seed = generateSeed(program, cur_state, env, 3.14, 1.0, 3.14, 10);

  // Update Configuration
  request.instructions = program;
  request.seed = seed;
  request.data = nullptr;  // Note: Nust clear the saved problem or it will use it instead.
  ompl_planner.plan_profiles["TEST_PROFILE"] = plan_profile;

  // Solve
  status = ompl_planner.solve(request, planner_response);
  EXPECT_FALSE(status);

  // Check for end state in collision error
  swp = start_state;
  std::vector<double> ewp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };

  wp1 = Eigen::Map<const Eigen::VectorXd>(swp.data(), static_cast<long>(swp.size()));
  wp1.joint_names = fwd_kin->getJointNames();

  wp2 = Eigen::Map<const Eigen::VectorXd>(ewp.data(), static_cast<long>(ewp.size()));
  wp2.joint_names = fwd_kin->getJointNames();

  // Define Start Instruction
  start_instruction = PlanInstruction(wp1, PlanInstructionType::START, "TEST_PROFILE");

  // Define Plan Instructions
  plan_f1 = PlanInstruction(wp2, PlanInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a new program
  program = CompositeInstruction();
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(manip);
  program.push_back(plan_f1);

  // Create a new seed
  seed = generateSeed(program, cur_state, env, 3.14, 1.0, 3.14, 10);

  // Update Configuration
  request.instructions = program;
  request.seed = seed;
  request.data = nullptr;  // Note: Nust clear the saved problem or it will use it instead.
  ompl_planner.plan_profiles["TEST_PROFILE"] = plan_profile;

  // Set new configuration and solve
  status = ompl_planner.solve(request, planner_response);
  EXPECT_FALSE(status);
}

TYPED_TEST(OMPLTestFixture, OMPLFreespaceCartesianGoalPlannerUnit)
{
  EXPECT_EQ(ompl::RNG::getSeed(), SEED) << "Randomization seed does not match expected: " << ompl::RNG::getSeed()
                                        << " vs. " << SEED;

  // Step 1: Load scene and srdf
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  Environment::Ptr env = std::make_shared<Environment>();
  tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_path, srdf_path, locator));

  // Set manipulator
  ManipulatorInfo manip;
  manip.manipulator = "manipulator";

  // Step 2: Add box to environment
  addBox(*(env));

  // Step 3: Create ompl planner config and populate it
  auto fwd_kin = env->getManipulatorManager()->getFwdKinematicSolver(manip.manipulator);
  auto inv_kin = env->getManipulatorManager()->getInvKinematicSolver(manip.manipulator);

  // Synchronize the inverse kinematics with the forward kinematics
  inv_kin->synchronize(fwd_kin);

  auto cur_state = env->getCurrentState();

  // Specify a start waypoint
  JointWaypoint wp1(fwd_kin->getJointNames(),
                    Eigen::Map<const Eigen::VectorXd>(start_state.data(), static_cast<long>(start_state.size())));

  // Specify a end waypoint
  auto goal_jv = Eigen::Map<const Eigen::VectorXd>(end_state.data(), static_cast<long>(end_state.size()));
  Eigen::Isometry3d goal = fwd_kin->calcFwdKin(goal_jv);
  CartesianWaypoint wp2 = goal;

  // Define Start Instruction
  PlanInstruction start_instruction(wp1, PlanInstructionType::START, "TEST_PROFILE");

  // Define Plan Instructions
  PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a program
  CompositeInstruction program;
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(manip);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, env, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<OMPLDefaultPlanProfile>();
  plan_profile->collision_check_config.collision_margin_override_type =
      tesseract_collision::CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN;
  plan_profile->collision_check_config.collision_margin_data.setDefaultCollisionMargin(0.02);
  plan_profile->collision_check_config.longest_valid_segment_length = 0.1;
  plan_profile->collision_check_config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  plan_profile->planning_time = 10;
  plan_profile->optimize = false;
  plan_profile->max_solutions = 2;
  plan_profile->simplify = false;
  plan_profile->planners = { this->configurator, this->configurator };

  // Create Planner
  OMPLMotionPlanner ompl_planner;
  ompl_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
  ompl_planner.problem_generator = &DefaultOMPLProblemGenerator;

  // Create Planner Request
  PlannerRequest request;
  request.instructions = program;
  request.seed = seed;
  request.env = env;
  request.env_state = env->getCurrentState();

  PlannerResponse planner_response;
  auto status = ompl_planner.solve(request, planner_response);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("CI Error: %s", status.message().c_str());
  }
  EXPECT_TRUE(&status);
  EXPECT_TRUE(planner_response.results.hasStartInstruction());
  EXPECT_EQ(getMoveInstructionCount(planner_response.results), 11);
  EXPECT_TRUE(wp1.isApprox(getJointPosition(getFirstMoveInstruction(planner_response.results)->getWaypoint()), 1e-5));

  Eigen::Isometry3d check_goal =
      fwd_kin->calcFwdKin(getJointPosition(getLastMoveInstruction(planner_response.results)->getWaypoint()));
  EXPECT_TRUE(wp2.isApprox(check_goal, 1e-3));
}

TYPED_TEST(OMPLTestFixture, OMPLFreespaceCartesianStartPlannerUnit)
{
  EXPECT_EQ(ompl::RNG::getSeed(), SEED) << "Randomization seed does not match expected: " << ompl::RNG::getSeed()
                                        << " vs. " << SEED;

  // Step 1: Load scene and srdf
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  Environment::Ptr env = std::make_shared<Environment>();
  tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_path, srdf_path, locator));

  // Set manipulator
  ManipulatorInfo manip;
  manip.manipulator = "manipulator";

  // Step 2: Add box to environment
  addBox(*(env));

  // Step 3: Create ompl planner config and populate it
  auto fwd_kin = env->getManipulatorManager()->getFwdKinematicSolver(manip.manipulator);
  auto inv_kin = env->getManipulatorManager()->getInvKinematicSolver(manip.manipulator);

  // Synchronize the inverse kinematics with the forward kinematics
  inv_kin->synchronize(fwd_kin);

  auto cur_state = env->getCurrentState();

  // Specify a start waypoint
  auto start_jv = Eigen::Map<const Eigen::VectorXd>(start_state.data(), static_cast<long>(start_state.size()));
  Eigen::Isometry3d start = fwd_kin->calcFwdKin(start_jv);
  CartesianWaypoint wp1 = start;

  // Specify a end waypoint
  JointWaypoint wp2(fwd_kin->getJointNames(),
                    Eigen::Map<const Eigen::VectorXd>(end_state.data(), static_cast<long>(end_state.size())));

  // Define Start Instruction
  PlanInstruction start_instruction(wp1, PlanInstructionType::START, "TEST_PROFILE");

  // Define Plan Instructions
  PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, "TEST_PROFILE");

  // Create a program
  CompositeInstruction program;
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(manip);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, env, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<OMPLDefaultPlanProfile>();
  plan_profile->collision_check_config.collision_margin_override_type =
      tesseract_collision::CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN;
  plan_profile->collision_check_config.collision_margin_data.setDefaultCollisionMargin(0.02);
  plan_profile->collision_check_config.longest_valid_segment_length = 0.1;
  plan_profile->collision_check_config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  plan_profile->planning_time = 10;
  plan_profile->optimize = false;
  plan_profile->max_solutions = 2;
  plan_profile->simplify = false;
  plan_profile->planners = { this->configurator, this->configurator };

  // Create Planner
  OMPLMotionPlanner ompl_planner;
  ompl_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
  ompl_planner.problem_generator = &DefaultOMPLProblemGenerator;

  // Create Planner Request
  PlannerRequest request;
  request.instructions = program;
  request.seed = seed;
  request.env = env;
  request.env_state = env->getCurrentState();

  PlannerResponse planner_response;
  auto status = ompl_planner.solve(request, planner_response);

  if (!status)
  {
    CONSOLE_BRIDGE_logError("CI Error: %s", status.message().c_str());
  }

  EXPECT_TRUE(&status);
  EXPECT_TRUE(planner_response.results.hasStartInstruction());
  EXPECT_EQ(getMoveInstructionCount(planner_response.results), 11);
  EXPECT_TRUE(wp2.isApprox(getJointPosition(getLastMoveInstruction(planner_response.results)->getWaypoint()), 1e-5));

  Eigen::Isometry3d check_start =
      fwd_kin->calcFwdKin(getJointPosition(getFirstMoveInstruction(planner_response.results)->getWaypoint()));
  EXPECT_TRUE(wp1.isApprox(check_start, 1e-3));
}

// TEST(OMPLMultiPlanner, OMPLMultiPlannerUnit)  // NOLINT
//{
//  EXPECT_EQ(ompl::RNG::getSeed(), SEED) << "Randomization seed does not match expected: " << ompl::RNG::getSeed()
//                                        << " vs. " << SEED;

//  // Step 1: Load scene and srdf
//  tesseract_scene_graph::ResourceLocator::Ptr locator =
//      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
//  Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
//  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
//  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
//  EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));

//  // Step 2: Add box to environment
//  addBox(*(env));

//  // Step 3: Create ompl planner config and populate it
//  auto kin = env->getManipulatorManager()->getFwdKinematicSolver(manip.manipulator);
//  std::vector<double> swp = start_state;
//  std::vector<double> ewp = end_state;

//  OMPLMotionPlanner ompl_planner;

//  std::vector<OMPLPlannerConfigurator::ConstPtr> planners = { std::make_shared<SBLConfigurator>(),
//                                                              std::make_shared<RRTConnectConfigurator>() };
//  auto ompl_config = std::make_shared<OMPLPlannerFreespaceConfig>(tesseract, manip.manipulator, planners);

//  ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp,
//  kin->getJointNames()); ompl_config->end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp,
//  kin->getJointNames()); ompl_config->collision_safety_margin = 0.02; ompl_config->planning_time = 5.0;
//  ompl_config->max_solutions = 2;
//  ompl_config->longest_valid_segment_fraction = 0.01;

//  ompl_config->collision_continuous = true;
//  ompl_config->collision_check = true;
//  ompl_config->simplify = false;
//  ompl_config->n_output_states = 50;

//  // Set the planner configuration
//  ompl_planner.setConfiguration(ompl_config);

//  tesseract_motion_planners::PlannerResponse ompl_planning_response;
//  tesseract_common::StatusCode status = ompl_planner.solve(ompl_planning_response);

//  if (!status)
//  {
//    CONSOLE_BRIDGE_logError("CI Error: %s", status.message().c_str());
//  }
//  EXPECT_TRUE(&status);
//  EXPECT_EQ(ompl_planning_response.joint_trajectory.trajectory.rows(), ompl_config->n_output_states);

//  // Check for start state in collision error
//  swp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };
//  ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp,
//  kin->getJointNames());

//  ompl_planner.setConfiguration(ompl_config);
//  status = ompl_planner.solve(ompl_planning_response);

//  EXPECT_FALSE(status);

//  // Check for start state in collision error
//  swp = start_state;
//  ewp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };
//  ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp,
//  kin->getJointNames()); ompl_config->end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp,
//  kin->getJointNames());

//  ompl_planner.setConfiguration(ompl_config);
//  status = ompl_planner.solve(ompl_planning_response);

//  EXPECT_FALSE(status);

//  // Reset start and end waypoints
//  swp = start_state;
//  ewp = end_state;
//  ompl_config->start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(swp,
//  kin->getJointNames()); ompl_config->end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp,
//  kin->getJointNames());
//}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Set the randomization seed for the planners to get repeatable results
  ompl::RNG::setSeed(SEED);

  return RUN_ALL_TESTS();
}
