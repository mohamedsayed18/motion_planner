/**
 * @file raster_waad_taskflow.cpp
 * @brief Plans raster paths with approach and departure
 *
 * @author Levi Armstrong
 * @date August 28, 2020
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
#include <functional>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/taskflow_generators/raster_waad_taskflow.h>

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>

#include <tesseract_common/utils.h>

using namespace tesseract_planning;

RasterWAADTaskflow::RasterWAADTaskflow(TaskflowGenerator::UPtr freespace_taskflow_generator,
                                       TaskflowGenerator::UPtr transition_taskflow_generator,
                                       TaskflowGenerator::UPtr raster_taskflow_generator,
                                       std::string name)
  : freespace_taskflow_generator_(std::move(freespace_taskflow_generator))
  , transition_taskflow_generator_(std::move(transition_taskflow_generator))
  , raster_taskflow_generator_(std::move(raster_taskflow_generator))
  , name_(std::move(name))
{
}

const std::string& RasterWAADTaskflow::getName() const { return name_; }

TaskflowContainer RasterWAADTaskflow::generateTaskflow(TaskInput input, TaskflowVoidFn done_cb, TaskflowVoidFn error_cb)
{
  // This should make all of the isComposite checks so that you can safely cast below
  if (!checkTaskInput(input))
  {
    CONSOLE_BRIDGE_logError("Invalid Process Input");
    throw std::runtime_error("Invalid Process Input");
  }

  TaskflowContainer container;
  container.taskflow = std::make_unique<tf::Taskflow>(name_);
  container.input = container.taskflow->emplace([]() {}).name(name_ + ": Input Task");
  std::vector<std::array<tf::Task, 3>> raster_tasks;

  // Generate all of the raster tasks. They don't depend on anything
  std::size_t raster_idx = 0;
  for (std::size_t idx = 1; idx < input.size() - 1; idx += 2)
  {
    // Get the last plan instruction of the approach
    assert(isCompositeInstruction(*(input[idx][0].getInstruction())));
    const auto& aci = input[idx][0].getInstruction()->as<CompositeInstruction>();
    const auto* ali = getLastPlanInstruction(aci);
    assert(ali != nullptr);

    // Create the process taskflow
    TaskInput task_input = input[idx][1];
    task_input.setStartInstruction(*ali);
    TaskflowContainer sub_container1 = raster_taskflow_generator_->generateTaskflow(
        task_input,
        [=]() { successTask(input, name_, task_input.getInstruction()->getDescription(), done_cb); },
        [=]() { failureTask(input, name_, task_input.getInstruction()->getDescription(), error_cb); });

    auto process_step =
        container.taskflow->composed_of(*(sub_container1.taskflow)).name("raster_" + std::to_string(raster_idx + 1));
    container.containers.push_back(std::move(sub_container1));

    // Create Departure Taskflow
    TaskInput departure_input = input[idx][2];
    departure_input.setStartInstruction(std::vector<std::size_t>({ idx, 1 }));
    TaskflowContainer sub_container2 = raster_taskflow_generator_->generateTaskflow(
        departure_input,
        [=]() { successTask(input, name_, departure_input.getInstruction()->getDescription(), done_cb); },
        [=]() { failureTask(input, name_, departure_input.getInstruction()->getDescription(), error_cb); });

    auto departure_step =
        container.taskflow->composed_of(*(sub_container2.taskflow)).name("departure_" + std::to_string(raster_idx + 1));
    container.containers.push_back(std::move(sub_container2));

    // Get Start Plan Instruction for approach
    Instruction start_instruction{ NullInstruction() };
    if (idx == 1)
    {
      assert(isCompositeInstruction(*(input[0].getInstruction())));
      const auto& ci = input[0].getInstruction()->as<CompositeInstruction>();
      const auto* li = getLastPlanInstruction(ci);
      assert(li != nullptr);
      start_instruction = *li;
    }
    else
    {
      assert(isCompositeInstruction(*(input[idx - 1].getInstruction())));
      const auto& tci = input[idx - 1].getInstruction()->as<CompositeInstruction>();
      const auto* li = getLastPlanInstruction(tci);
      assert(li != nullptr);
      start_instruction = *li;
    }

    // Create the departure taskflow
    start_instruction.as<PlanInstruction>().setPlanType(PlanInstructionType::START);
    TaskInput approach_input = input[idx][0];
    approach_input.setStartInstruction(start_instruction);
    approach_input.setEndInstruction(std::vector<std::size_t>({ idx, 1 }));
    TaskflowContainer sub_container0 = raster_taskflow_generator_->generateTaskflow(
        approach_input,
        [=]() { successTask(input, name_, approach_input.getInstruction()->getDescription(), done_cb); },
        [=]() { failureTask(input, name_, approach_input.getInstruction()->getDescription(), error_cb); });

    auto approach_step =
        container.taskflow->composed_of(*(sub_container0.taskflow)).name("approach_" + std::to_string(raster_idx + 1));
    container.containers.push_back(std::move(sub_container0));

    // Each approach and departure depend on raster
    approach_step.succeed(process_step);
    departure_step.succeed(process_step);
    container.input.precede(process_step);

    raster_tasks.push_back(std::array<tf::Task, 3>({ approach_step, process_step, departure_step }));
    raster_idx++;
  }

  // Loop over all transitions
  std::size_t transition_idx = 0;
  for (std::size_t input_idx = 2; input_idx < input.size() - 2; input_idx += 2)
  {
    // This use to extract the start and end, but things were changed so the seed is generated as part of the
    // taskflow. So the seed is only a skeleton and does not contain move instructions. So instead we provide the
    // composite and let the generateTaskflow extract the start and end waypoint from the composite. This is also more
    // robust because planners could modify composite size, which is rare but does happen when using OMPL where it is
    // not possible to simplify the trajectory to the desired number of states.
    TaskInput transition_input = input[input_idx];
    transition_input.setStartInstruction(std::vector<std::size_t>({ input_idx - 1, 2 }));
    transition_input.setEndInstruction(std::vector<std::size_t>({ input_idx + 1, 0 }));
    TaskflowContainer sub_container = transition_taskflow_generator_->generateTaskflow(
        transition_input,
        [=]() { successTask(input, name_, transition_input.getInstruction()->getDescription(), done_cb); },
        [=]() { failureTask(input, name_, transition_input.getInstruction()->getDescription(), error_cb); });

    auto transition_step = container.taskflow->composed_of(*(sub_container.taskflow))
                               .name("transition_" + std::to_string(transition_idx + 1));
    container.containers.push_back(std::move(sub_container));
    // Each transition is independent and thus depends only on the adjacent rasters approach and departure
    transition_step.succeed(raster_tasks[transition_idx][2]);
    transition_step.succeed(raster_tasks[transition_idx + 1][0]);

    transition_idx++;
  }

  // Plan from_start - preceded by the first raster
  TaskInput from_start_input = input[0];
  from_start_input.setStartInstruction(input.getInstruction()->as<CompositeInstruction>().getStartInstruction());
  from_start_input.setEndInstruction(std::vector<std::size_t>({ 1, 0 }));
  TaskflowContainer sub_container1 = freespace_taskflow_generator_->generateTaskflow(
      from_start_input,
      [=]() { successTask(input, name_, from_start_input.getInstruction()->getDescription(), done_cb); },
      [=]() { failureTask(input, name_, from_start_input.getInstruction()->getDescription(), error_cb); });

  auto from_start = container.taskflow->composed_of(*(sub_container1.taskflow)).name("from_start");
  container.containers.push_back(std::move(sub_container1));
  raster_tasks[0][0].precede(from_start);

  // Plan to_end - preceded by the last raster
  TaskInput to_end_input = input[input.size() - 1];
  to_end_input.setStartInstruction(std::vector<std::size_t>({ input.size() - 2, 2 }));
  TaskflowContainer sub_container2 = freespace_taskflow_generator_->generateTaskflow(
      to_end_input,
      [=]() { successTask(input, name_, to_end_input.getInstruction()->getDescription(), done_cb); },
      [=]() { failureTask(input, name_, to_end_input.getInstruction()->getDescription(), error_cb); });

  auto to_end = container.taskflow->composed_of(*(sub_container2.taskflow)).name("to_end");
  container.containers.push_back(std::move(sub_container2));
  raster_tasks.back()[2].precede(to_end);

  return container;
}

bool RasterWAADTaskflow::checkTaskInput(const tesseract_planning::TaskInput& input)
{
  // -------------
  // Check Input
  // -------------
  if (!input.env)
  {
    CONSOLE_BRIDGE_logError("TaskInput env is a nullptr");
    return false;
  }

  // Check the overall input
  const Instruction* input_instruction = input.getInstruction();
  if (!isCompositeInstruction(*input_instruction))
  {
    CONSOLE_BRIDGE_logError("TaskInput Invalid: input.instructions should be a composite");
    return false;
  }
  const auto& composite = input_instruction->as<CompositeInstruction>();

  // Check that it has a start instruction
  if (!composite.hasStartInstruction() && isNullInstruction(input.getStartInstruction()))
  {
    CONSOLE_BRIDGE_logError("TaskInput Invalid: input.instructions should have a start instruction");
    return false;
  }

  // Check from_start
  if (!isCompositeInstruction(composite.at(0)))
  {
    CONSOLE_BRIDGE_logError("TaskInput Invalid: from_start should be a composite");
    return false;
  }

  // Check rasters and transitions
  for (std::size_t index = 1; index < composite.size() - 1; index++)
  {
    // Both rasters and transitions should be a composite
    if (!isCompositeInstruction(composite.at(index)))
    {
      CONSOLE_BRIDGE_logError("TaskInput Invalid: Both rasters and transitions should be a composite");
      return false;
    }

    // Convert to composite
    const auto& step = composite.at(index).as<CompositeInstruction>();

    // Odd numbers are raster segments
    if (index % 2 == 1)
    {
      // Raster must have three composites approach, raster and departure
      if (step.size() != 3)
      {
        CONSOLE_BRIDGE_logError("TaskInput Invalid: Rasters must have three element approach, raster and departure");
        return false;
      }
      // The approach should be a composite
      if (!isCompositeInstruction(step.at(0)))
      {
        CONSOLE_BRIDGE_logError("TaskInput Invalid: The raster approach should be a composite");
        return false;
      }

      // The process should be a composite
      if (!isCompositeInstruction(step.at(1)))
      {
        CONSOLE_BRIDGE_logError("TaskInput Invalid: The process should be a composite");
        return false;
      }

      // The departure should be a composite
      if (!isCompositeInstruction(step.at(2)))
      {
        CONSOLE_BRIDGE_logError("TaskInput Invalid: The departure should be a composite");
        return false;
      }
    }
  }

  // Check to_end
  if (!isCompositeInstruction(composite.back()))
  {
    CONSOLE_BRIDGE_logError("TaskInput Invalid: to_end should be a composite");
    return false;
  };

  return true;
}
