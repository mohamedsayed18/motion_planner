#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>

#include <tesseract_process_managers/core/task_input.h>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_process_managers/taskflow_generators/raster_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_only_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_only_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_dt_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_waad_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_waad_dt_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/cartesian_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/freespace_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/descartes_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>
#include <tesseract_process_managers/task_generators/seed_min_length_task_generator.h>

#include "raster_example_program.h"
#include "raster_dt_example_program.h"
#include "raster_waad_example_program.h"
#include "raster_waad_dt_example_program.h"
#include "freespace_example_program.h"

using namespace tesseract_kinematics;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
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

class TesseractProcessManagerUnit : public ::testing::Test
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

TEST_F(TesseractProcessManagerUnit, SeedMinLengthTaskGeneratorTest)
{
  tesseract_planning::CompositeInstruction program = freespaceExampleProgramABB();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  // Define the Process Input
  auto cur_state = env_->getCurrentState();
  CompositeInstruction seed = generateSeed(program, cur_state, env_);

  Instruction program_instruction = program;
  Instruction seed_instruction = seed;

  long current_length = getMoveInstructionCount(seed);
  TaskInput input(env_, &program_instruction, program.getManipulatorInfo(), &seed_instruction, true, nullptr);

  SeedMinLengthTaskGenerator smlpg(current_length);
  EXPECT_TRUE(smlpg.conditionalProcess(input, 1) == 1);
  long final_length = getMoveInstructionCount(input.getResults()->as<CompositeInstruction>());
  EXPECT_TRUE(final_length == current_length);

  SeedMinLengthTaskGenerator smlpg2(2 * current_length);
  EXPECT_TRUE(smlpg2.conditionalProcess(input, 2) == 1);
  long final_length2 = getMoveInstructionCount(input.getResults()->as<CompositeInstruction>());
  EXPECT_TRUE(final_length2 >= (2 * current_length));

  seed_instruction = seed;
  TaskInput input2(env_, &program_instruction, program.getManipulatorInfo(), &seed_instruction, true, nullptr);

  SeedMinLengthTaskGenerator smlpg3(3 * current_length);
  EXPECT_TRUE(smlpg3.conditionalProcess(input, 3) == 1);
  long final_length3 = getMoveInstructionCount(input2.getResults()->as<CompositeInstruction>());
  EXPECT_TRUE(final_length3 >= (3 * current_length));
}

TEST_F(TesseractProcessManagerUnit, RasterSimpleMotionPlannerFixedSizeAssignPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  tesseract_planning::CompositeInstruction program = rasterExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  PlannerResponse response;
  interpolator->plan_profiles[process_profile] = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  interpolator->plan_profiles[freespace_profile] = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto pcnt = getPlanInstructionCount(request.instructions);
  auto mcnt = getMoveInstructionCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, RasterSimpleMotionPlannerDefaultLVSPlanProfileTest)
{
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  tesseract_planning::CompositeInstruction program = rasterExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  PlannerResponse response;
  interpolator->plan_profiles[process_profile] = std::make_shared<SimplePlannerLVSPlanProfile>();
  interpolator->plan_profiles[freespace_profile] = std::make_shared<SimplePlannerLVSPlanProfile>();
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto mcnt = getMoveInstructionCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(161, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, FreespaceSimpleMotionPlannerFixedSizeAssignPlanProfileTest)
{
  CompositeInstruction program = freespaceExampleProgramABB();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  PlannerResponse response;
  interpolator->plan_profiles[DEFAULT_PROFILE_KEY] = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto pcnt = getPlanInstructionCount(request.instructions);
  auto mcnt = getMoveInstructionCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, FreespaceSimpleMotionPlannerDefaultLVSPlanProfileTest)
{
  CompositeInstruction program = freespaceExampleProgramABB();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  PlannerResponse response;
  interpolator->plan_profiles[DEFAULT_PROFILE_KEY] = std::make_shared<SimplePlannerLVSPlanProfile>();
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto mcnt = getMoveInstructionCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // 32 move instruction.
  EXPECT_EQ(37, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, RasterProcessManagerDefaultPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterProcessManagerDefaultLVSPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterGlobalProcessManagerDefaultPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_G_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterGlobalProcessManagerDefaultLVSPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_G_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyProcessManagerDefaultPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_O_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyProcessManagerDefaultLVSPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_O_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyGlobalProcessManagerDefaultPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_O_G_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyGlobalProcessManagerDefaultLVSPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_O_G_FT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterDTProcessManagerDefaultPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_DT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterDTExampleProgram(freespace_profile, process_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterDTProcessManagerDefaultLVSPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_DT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterDTExampleProgram(freespace_profile, process_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADProcessManagerDefaultPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_WAAD_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";

  CompositeInstruction program =
      rasterWAADExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(approach_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(departure_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADProcessManagerDefaultLVSPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_WAAD_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";

  CompositeInstruction program =
      rasterWAADExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(approach_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(departure_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADDTProcessManagerDefaultPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_WAAD_DT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";

  CompositeInstruction program =
      rasterWAADDTExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerFixedSizeAssignPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(approach_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(departure_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADDTProcessManagerDefaultLVSPlanProfileTest)
{
  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env_), 1);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_WAAD_DT_PLANNER_NAME;

  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";

  CompositeInstruction program =
      rasterWAADDTExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(freespace_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(approach_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(process_profile, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(departure_profile, default_simple_plan_profile);

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Confirm that the task is finished
  EXPECT_TRUE(response.ready());

  // Solve
  EXPECT_TRUE(response.interface->isSuccessful());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
