# ABB planner example

In the example the robot follows a group of waypoints read from a file.

```cpp
  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.push_back("joint_1");
  joint_names.push_back("joint_2");
  joint_names.push_back("joint_3");
  joint_names.push_back("joint_4");
  joint_names.push_back("joint_5");
  joint_names.push_back("joint_6");

  Eigen::VectorXd joint_start_pos(6);
  joint_start_pos(0) = -0.4;
  joint_start_pos(1) = 0.2762;
  joint_start_pos(2) = 0.0;
  joint_start_pos(3) = -1.3348;
  joint_start_pos(4) = 0.0;
  joint_start_pos(5) = 1.4959;

  env_->setState(joint_names, joint_start_pos);
```

We define the joint names as mentioned in the robot description (xacro/urdf) and
set them to initial values to be the start position.

```cpp
// Create Program
CompositeInstruction program("FREESPACE", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));
```

We create a program give it a name, and manipulator

```cpp
  Waypoint wp_start = StateWaypoint(joint_names, joint_start_pos);
  Waypoint wp0 = CartesianWaypoint(getPoses()[0]);
  Waypoint wp1 = CartesianWaypoint(getPoses()[1]);
  Waypoint wp2 = CartesianWaypoint(getPoses()[2]);
  Waypoint wp3 = CartesianWaypoint(getPoses()[3]);
```

To define the waypoints or the positions of the robot we can use `StateWaypoint` which takes the joint states/values
or we can use `CartesianWaypoint` which an eigen matrix defining the pose (position/orientation)

`getPoses` reads data from file and return a list of the waypoints.

```cpp
  // Plan motion from start to end
  PlanInstruction start_instruction(wp_start, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  PlanInstruction plan_f0(wp0, PlanInstructionType::FREESPACE, "FREESPACE");
  PlanInstruction plan_f1(wp1, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_f2(wp2, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_f3(wp3, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_f4(wp0, PlanInstructionType::LINEAR, "FREESPACE");
```

We create a PlanInstruction for every waypoint we defined.

```cpp
  // Add Instructions to program
  program.push_back(plan_f0);
  program.push_back(plan_f1);
  program.push_back(plan_f2);
  program.push_back(plan_f3);
  program.push_back(plan_f4);
```

Then We add these PlanInstructions to the program.

```cpp
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  planning_server.loadDefaultProcessPlanners();
```

We create a planning server.

```cpp
  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  request.instructions = Instruction(program);
```

Then we create a request object and add the program to its instructions attribute

```cpp
  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();
```

Then we pass the request to the planning server to get the response.
