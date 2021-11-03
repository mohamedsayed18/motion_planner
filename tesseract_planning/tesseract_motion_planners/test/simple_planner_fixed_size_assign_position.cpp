/**
 * @file simple_planner_fixed_size_assign_position.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date July 28, 2020
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
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>

using namespace tesseract_environment;
using namespace tesseract_planning;

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

class TesseractPlanningSimplePlannerFixedSizeAssignPositionUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  ManipulatorInfo manip_info_;
  std::vector<std::string> joint_names_;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract_common::SimpleResourceLocator>(locateResource);
    Environment::Ptr env = std::make_shared<Environment>();
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;

    manip_info_.tcp_frame = "tool0";
    manip_info_.working_frame = "base_link";
    manip_info_.manipulator = "manipulator";
    joint_names_ = env_->getJointGroup("manipulator")->getJointNames();
  }
};

TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignPositionUnit, JointCartesian_AssignJointPosition)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();
  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity();
  PlanInstruction instr1(wp1, PlanInstructionType::START, "TEST_PROFILE", manip_info_);
  PlanInstruction instr2(wp2, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  SimplePlannerFixedSizeAssignPlanProfile profile(10, 10);
  auto composite = profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_EQ(composite.size(), 10);
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.as<MoveInstruction>().getWaypoint()));
    const auto& mi = c.as<MoveInstruction>();
    EXPECT_TRUE(wp1.isApprox(mi.getWaypoint().as<StateWaypoint>().position, 1e-5));
    EXPECT_EQ(c.as<MoveInstruction>().getProfile(), instr2.getProfile());
  }
}

TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignPositionUnit, CartesianJoint_AssignJointPosition)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity();
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Zero(7));
  PlanInstruction instr1(wp1, PlanInstructionType::START, "TEST_PROFILE", manip_info_);
  PlanInstruction instr2(wp2, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  SimplePlannerFixedSizeAssignPlanProfile profile(10, 10);
  auto composite = profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_EQ(composite.size(), 10);
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.as<MoveInstruction>().getWaypoint()));
    const auto& mi = c.as<MoveInstruction>();
    EXPECT_TRUE(wp2.isApprox(mi.getWaypoint().as<StateWaypoint>().position, 1e-5));
    EXPECT_EQ(c.as<MoveInstruction>().getProfile(), instr2.getProfile());
  }
}

TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignPositionUnit, CartesianCartesian_AssignJointPosition)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity();
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity();
  PlanInstruction instr1(wp1, PlanInstructionType::START, "TEST_PROFILE", manip_info_);
  PlanInstruction instr2(wp2, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  SimplePlannerFixedSizeAssignPlanProfile profile(10, 10);
  auto composite = profile.generate(instr1, instr2, request, ManipulatorInfo());
  auto fwd_kin = env_->getJointGroup(manip_info_.manipulator);
  Eigen::VectorXd position = request.env_state.getJointValues(fwd_kin->getJointNames());
  EXPECT_EQ(composite.size(), 10);
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.as<MoveInstruction>().getWaypoint()));
    const auto& mi = c.as<MoveInstruction>();
    EXPECT_TRUE(position.isApprox(mi.getWaypoint().as<StateWaypoint>().position, 1e-5));
    EXPECT_EQ(c.as<MoveInstruction>().getProfile(), instr2.getProfile());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
