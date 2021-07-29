/**
 * @file descartes_planner_tests.cpp
 * @brief This contains unit test for the tesseract descartes planner
 *
 * @author Levi Armstrong
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
#include <gtest/gtest.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <tesseract_kinematics/core/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>

#include <tesseract_command_language/utils/utils.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>

using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_planning;
using namespace tesseract_kinematics;
using namespace descartes_light;

const bool DEBUG = false;

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

class TesseractPlanningDescartesUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  ManipulatorInfo manip;

  void SetUp() override
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    Environment::Ptr env = std::make_shared<Environment>();
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
    EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_path, srdf_path, locator));
    env_ = env;

    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "OPWInvKin";
    manip.working_frame = "base_link";
  }
};

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerFixedPoses)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto fwd_kin = env_->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  auto inv_kin = env_->getManipulatorManager()->getInvKinematicSolver("manipulator");

  // Synchronize the inverse kinematics with the forward kinematics
  inv_kin->synchronize(fwd_kin);

  auto cur_state = env_->getCurrentState();

  // Specify a start waypoint
  CartesianWaypoint wp1 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Specify a end waypoint
  CartesianWaypoint wp2 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Define Start Instruction
  PlanInstruction start_instruction(wp1, PlanInstructionType::START, "TEST_PROFILE", manip);

  // Define Plan Instructions
  PlanInstruction plan_f1(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE", manip);

  // Create a program
  CompositeInstruction program;
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(manip);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner;
  plan_profile->num_threads = 1;
  single_descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
  single_descartes_planner.problem_generator = &DefaultDescartesProblemGenerator<double>;

  // Create Planning Request
  PlannerRequest request;
  request.seed = seed;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  PlannerResponse single_planner_response;
  auto single_status = single_descartes_planner.solve(request, single_planner_response);
  EXPECT_TRUE(&single_status);

  CompositeInstruction official_results = single_planner_response.results;

  for (int i = 0; i < 10; ++i)
  {
    // Test the problem generator
    {
      auto problem = DefaultDescartesProblemGenerator<double>(
          single_descartes_planner.getName(), request, single_descartes_planner.plan_profiles);
      EXPECT_EQ(problem->samplers.size(), 11);
      EXPECT_EQ(problem->edge_evaluators.size(), 10);
    }

    DescartesMotionPlannerD descartes_planner;
    plan_profile->num_threads = 4;
    descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
    descartes_planner.problem_generator = &DefaultDescartesProblemGenerator<double>;

    PlannerResponse planner_response;
    auto status = descartes_planner.solve(request, planner_response);

    if (DEBUG)
    {
      std::cout << "Request Instructions:" << std::endl;
      request.instructions.print();
      std::cout << "Request Seed:" << std::endl;
      request.seed.print();
      std::cout << "Single Planner Response Results:" << std::endl;
      single_planner_response.results.print();
      std::cout << "Threaded Planner Response Results:" << std::endl;
      planner_response.results.print();
    }

    EXPECT_TRUE(&status);
    EXPECT_EQ(official_results.size(), planner_response.results.size());
    for (std::size_t j = 0; j < official_results.size(); ++j)
    {
      if (isCompositeInstruction(official_results[j]))
      {
        const auto& sub_official = official_results[j].as<CompositeInstruction>();
        const auto& sub = planner_response.results[j].as<CompositeInstruction>();
        for (std::size_t k = 0; k < sub.size(); ++k)
        {
          if (isCompositeInstruction(sub_official[k]))
          {
            EXPECT_TRUE(false);
          }
          else if (isMoveInstruction(sub_official[k]))
          {
            const auto& mv_official = sub_official[k].as<MoveInstruction>();
            const auto& mv = sub[k].as<MoveInstruction>();
            EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
          }
        }
      }
      else if (isMoveInstruction(official_results[j]))
      {
        const auto& mv_official = official_results[j].as<MoveInstruction>();
        const auto& mv = request.seed[j].as<MoveInstruction>();
        EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
      }
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerAxialSymetric)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto fwd_kin = env_->getManipulatorManager()->getFwdKinematicSolver(manip.manipulator);
  auto inv_kin = env_->getManipulatorManager()->getInvKinematicSolver(manip.manipulator);

  // Synchronize the inverse kinematics with the forward kinematics
  inv_kin->synchronize(fwd_kin);

  auto cur_state = env_->getCurrentState();

  // Specify a start waypoint
  CartesianWaypoint wp1 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Specify a end waypoint
  CartesianWaypoint wp2 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Define Start Instruction
  PlanInstruction start_instruction(wp1, PlanInstructionType::START, "TEST_PROFILE", manip);

  // Define Plan Instructions
  PlanInstruction plan_f1(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE", manip);

  // Create a program
  CompositeInstruction program;
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(manip);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
  // Make this a tool z-axis free sampler
  plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) {
    return tesseract_planning::sampleToolAxis(tool_pose, M_PI_4, Eigen::Vector3d(0, 0, 1));
  };

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner;
  plan_profile->num_threads = 1;
  single_descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
  single_descartes_planner.problem_generator = &DefaultDescartesProblemGenerator<double>;

  // Create Planning Request
  PlannerRequest request;
  request.seed = seed;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  auto problem = DefaultDescartesProblemGenerator<double>(
      single_descartes_planner.getName(), request, single_descartes_planner.plan_profiles);
  problem->num_threads = 1;
  EXPECT_EQ(problem->samplers.size(), 11);
  EXPECT_EQ(problem->edge_evaluators.size(), 10);

  PlannerResponse single_planner_response;
  auto single_status = single_descartes_planner.solve(request, single_planner_response);
  EXPECT_TRUE(&single_status);

  CompositeInstruction official_results = request.seed;

  for (int i = 0; i < 10; ++i)
  {
    DescartesMotionPlannerD descartes_planner;
    plan_profile->num_threads = 4;
    descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
    descartes_planner.problem_generator = &DefaultDescartesProblemGenerator<double>;

    PlannerResponse planner_response;
    request.seed = seed;  // reset seed to the original seed
    auto status = descartes_planner.solve(request, planner_response);
    EXPECT_TRUE(&status);
    EXPECT_TRUE(official_results.size() == request.seed.size());
    for (std::size_t j = 0; j < official_results.size(); ++j)
    {
      if (isCompositeInstruction(official_results[j]))
      {
        const auto& sub_official = official_results[j].as<CompositeInstruction>();
        const auto& sub = request.seed[j].as<CompositeInstruction>();
        for (std::size_t k = 0; k < sub.size(); ++k)
        {
          if (isCompositeInstruction(sub_official[k]))
          {
            EXPECT_TRUE(false);
          }
          else if (isMoveInstruction(sub_official[k]))
          {
            const auto& mv_official = sub_official[k].as<MoveInstruction>();
            const auto& mv = sub[k].as<MoveInstruction>();
            EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
          }
        }
      }
      else if (isMoveInstruction(official_results[j]))
      {
        const auto& mv_official = official_results[j].as<MoveInstruction>();
        const auto& mv = request.seed[j].as<MoveInstruction>();
        EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
      }
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerCollisionEdgeEvaluator)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  auto fwd_kin = env_->getManipulatorManager()->getFwdKinematicSolver(manip.manipulator);
  auto inv_kin = env_->getManipulatorManager()->getInvKinematicSolver(manip.manipulator);

  // Synchronize the inverse kinematics with the forward kinematics
  inv_kin->synchronize(fwd_kin);

  auto cur_state = env_->getCurrentState();

  // Specify a start waypoint
  CartesianWaypoint wp1 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.10, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Specify a end waypoint
  CartesianWaypoint wp2 =
      Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .10, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

  // Define Start Instruction
  PlanInstruction start_instruction(wp1, PlanInstructionType::START, "TEST_PROFILE", manip);

  // Define Plan Instructions
  PlanInstruction plan_f1(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE", manip);

  // Create a program
  CompositeInstruction program;
  program.setStartInstruction(start_instruction);
  program.setManipulatorInfo(manip);
  program.push_back(plan_f1);

  // Create a seed
  CompositeInstruction seed = generateSeed(program, cur_state, env_, 3.14, 1.0, 3.14, 2);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
  // Make this a tool z-axis free sampler
  plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) {
    return tesseract_planning::sampleToolAxis(tool_pose, 60 * M_PI * 180.0, Eigen::Vector3d(0, 0, 1));
  };
  plan_profile->enable_edge_collision = true;  // Add collision edge evaluator

  // Create Planning Request
  PlannerRequest request;
  request.seed = seed;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner;
  plan_profile->num_threads = 1;
  single_descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;

  // Test Problem size - TODO: Make dedicated unit test for DefaultDescartesProblemGenerator
  auto problem = DefaultDescartesProblemGenerator<double>(
      single_descartes_planner.getName(), request, single_descartes_planner.plan_profiles);
  EXPECT_EQ(problem->samplers.size(), 3);
  EXPECT_EQ(problem->edge_evaluators.size(), 2);
  EXPECT_EQ(problem->num_threads, 1);

  PlannerResponse single_planner_response;
  auto single_status = single_descartes_planner.solve(request, single_planner_response);
  EXPECT_TRUE(&single_status);

  CompositeInstruction official_results = request.seed;

  for (int i = 0; i < 10; ++i)
  {
    DescartesMotionPlannerD descartes_planner;
    plan_profile->num_threads = 4;
    descartes_planner.plan_profiles["TEST_PROFILE"] = plan_profile;
    descartes_planner.problem_generator = &DefaultDescartesProblemGenerator<double>;

    PlannerResponse planner_response;
    request.seed = seed;  // reset seed to the original seed
    auto status = descartes_planner.solve(request, planner_response);
    EXPECT_TRUE(&status);
    EXPECT_TRUE(official_results.size() == request.seed.size());
    for (std::size_t j = 0; j < official_results.size(); ++j)
    {
      if (isCompositeInstruction(official_results[j]))
      {
        const auto& sub_official = official_results[j].as<CompositeInstruction>();
        const auto& sub = request.seed[j].as<CompositeInstruction>();
        for (std::size_t k = 0; k < sub.size(); ++k)
        {
          if (isCompositeInstruction(sub_official[k]))
          {
            EXPECT_TRUE(false);
          }
          else if (isMoveInstruction(sub_official[k]))
          {
            const auto& mv_official = sub_official[k].as<MoveInstruction>();
            const auto& mv = sub[k].as<MoveInstruction>();
            EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
          }
        }
      }
      else if (isMoveInstruction(official_results[j]))
      {
        const auto& mv_official = official_results[j].as<MoveInstruction>();
        const auto& mv = request.seed[j].as<MoveInstruction>();
        EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
      }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
