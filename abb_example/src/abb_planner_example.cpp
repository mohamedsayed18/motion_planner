/**
 * @file freespace_ompl_example.cpp
 * @brief An example of a feespace motion planning with OMPL.
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/abb_planner_example.h>
#include <tesseract_environment/utils.h>
#include <tesseract_environment/commands.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_visualization;
using namespace tesseract_visualization;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

namespace tesseract_ros_examples
{
  AbbPlannerExample::AbbPlannerExample(const ros::NodeHandle &nh,
                                       bool plotting,
                                       bool rviz,
                                       double range,
                                       double planning_time)
      : Example(plotting, rviz), nh_(nh), range_(range), planning_time_(planning_time)
  {
  }

  tesseract_common::VectorIsometry3d AbbPlannerExample::getPoses()
  {
    tesseract_common::VectorIsometry3d path; // results
    std::ifstream indata;

    std::string filename = ros::package::getPath("abb_example") + "/config/abb_waypoints.csv";
    indata.open(filename);
    std::string line;
    while (std::getline(indata, line))
    {
      std::stringstream linestream(line);
      std::string cell;
      Eigen::Matrix<double, 7, 1> xyzijk;
      int i = 0;
      while (getline(linestream, cell, ','))
      {
        xyzijk(i) = std::stod(cell);
        i++;
      }
      Eigen::Isometry3d current_pose;
      current_pose.translation() = Eigen::Vector3d(xyzijk(0), xyzijk(1), xyzijk(2));
      current_pose.linear() = Eigen::Quaterniond(xyzijk(3), xyzijk(4), xyzijk(5), xyzijk(6)).toRotationMatrix();

      path.push_back(current_pose);
    }

    indata.close();
    return path;
  }

  bool AbbPlannerExample::run()
  {
    using tesseract_planning::CartesianWaypoint;
    using tesseract_planning::CompositeInstruction;
    using tesseract_planning::CompositeInstructionOrder;
    using tesseract_planning::Instruction;
    using tesseract_planning::ManipulatorInfo;
    using tesseract_planning::PlanInstruction;
    using tesseract_planning::PlanInstructionType;
    using tesseract_planning::ProcessPlanningFuture;
    using tesseract_planning::ProcessPlanningRequest;
    using tesseract_planning::ProcessPlanningServer;
    using tesseract_planning::StateWaypoint;
    using tesseract_planning::Waypoint;
    using tesseract_planning_server::ROSProcessEnvironmentCache;

    // Initial setup
    std::string urdf_xml_string, srdf_xml_string;
    nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
    nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

    tesseract_common::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    if (!env_->init(urdf_xml_string, srdf_xml_string, locator))
      return false;

    // Create monitor
    monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
    if (rviz_)
      monitor_->startPublishingEnvironment();

    // Create plotting tool
    ROSPlottingPtr plotter = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());
    if (rviz_)
      plotter->waitForConnection();

    // Set the robot initial state
    std::vector<std::string> joint_names;
    joint_names.push_back("joint_1");
    joint_names.push_back("joint_2");
    joint_names.push_back("joint_3");
    joint_names.push_back("joint_4");
    joint_names.push_back("joint_5");
    joint_names.push_back("joint_6");
    joint_names.push_back("positioner_base_joint");
    joint_names.push_back("positioner_joint_1");

    Eigen::VectorXd joint_start_pos(8);
    joint_start_pos(0) = -0.4;
    joint_start_pos(1) = 0.2762;
    joint_start_pos(2) = 0.0;
    joint_start_pos(3) = -1.3348;
    joint_start_pos(4) = 0.0;
    joint_start_pos(5) = 1.4959;
    joint_start_pos(6) = 0;
    joint_start_pos(7) = 0;

    env_->setState(joint_names, joint_start_pos);

    // Create manipulator information for program
    ManipulatorInfo mi("manipulator", "positioner_tool0", "tool0");

    // Create Program
    CompositeInstruction program("FREESPACE", CompositeInstructionOrder::ORDERED, mi);

    // Waypoints for the program
    //TODO: The start point should match the joint state, we can use getState to know the position of the robot
    Waypoint wp_start = StateWaypoint(joint_names, joint_start_pos);
    Waypoint wp0 = CartesianWaypoint(getPoses()[0]);
    Waypoint wp1 = CartesianWaypoint(getPoses()[1]);
    Waypoint wp2 = CartesianWaypoint(getPoses()[2]);
    Waypoint wp3 = CartesianWaypoint(getPoses()[3]);

    // Plan motion from start to end
    PlanInstruction start_instruction(wp_start, PlanInstructionType::START);
    program.setStartInstruction(start_instruction);

    PlanInstruction plan_f0(wp0, PlanInstructionType::FREESPACE, "FREESPACE");
    PlanInstruction plan_f1(wp1, PlanInstructionType::FREESPACE, "FREESPACE");
    PlanInstruction plan_f2(wp2, PlanInstructionType::FREESPACE, "FREESPACE");
    PlanInstruction plan_f3(wp3, PlanInstructionType::FREESPACE, "FREESPACE");
    PlanInstruction plan_f4(wp0, PlanInstructionType::FREESPACE, "FREESPACE");

    // Add Instructions to program
    program.push_back(plan_f0);
    program.push_back(plan_f1);
    program.push_back(plan_f2);
    program.push_back(plan_f3);
    // program.push_back(plan_f4);

    ROS_INFO("basic cartesian motion with abb");

    plotter->waitForInput("Hit enter to send motion request!");

    // Create Process Planning Server
    ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
    planning_server.loadDefaultProcessPlanners();

    // Create Process Planning Request
    ProcessPlanningRequest request;
    request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
    request.instructions = Instruction(program);

    // Print Diagnostics
    request.instructions.print("Program: ");

    // Solve process plan
    ProcessPlanningFuture response = planning_server.run(request);
    planning_server.waitForAll();

    // Plot Process Trajectory
    if (rviz_ && plotter != nullptr && plotter->isConnected())
    {
      plotter->waitForInput();
      const auto &ci = response.results->as<tesseract_planning::CompositeInstruction>();
      tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(ci, *env_);
      tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(ci);
      plotter->plotMarker(ToolpathMarker(toolpath));
      plotter->plotTrajectory(trajectory, *env_->getStateSolver());
    }

    ROS_INFO("Final trajectory is collision free");
    return true;
  }
} // namespace tesseract_ros_examples
