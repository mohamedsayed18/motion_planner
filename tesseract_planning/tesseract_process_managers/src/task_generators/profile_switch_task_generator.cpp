/**
 * @file profile_switch_task_generator.h
 * @brief Process generator that returns a value based on the profile
 *
 * @author Matthew Powelson
 * @date October 26. 2020
 * @version TODO
 * @bug No known bugs
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
#include <tesseract_common/timer.h>

#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/task_generators/profile_switch_task_generator.h>
#include <tesseract_command_language/constants.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
ProfileSwitchProfile::ProfileSwitchProfile(const int& return_value) : return_value(return_value) {}

ProfileSwitchTaskGenerator::ProfileSwitchTaskGenerator(std::string name) : TaskGenerator(std::move(name))
{
  // Register default profile
  composite_profiles[DEFAULT_PROFILE_KEY] = std::make_shared<ProfileSwitchProfile>();
}

int ProfileSwitchTaskGenerator::conditionalProcess(TaskInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_shared<ProfileSwitchTaskInfo>(unique_id, name_);
  info->return_value = 0;
  input.addTaskInfo(info);
  tesseract_common::Timer timer;
  timer.start();
  saveInputs(*info, input);

  // --------------------
  // Check that inputs are valid
  // --------------------
  const Instruction* input_instruction = input.getInstruction();
  if (!isCompositeInstruction(*input_instruction))
  {
    CONSOLE_BRIDGE_logError("Input instruction to ProfileSwitch must be a composite instruction. Returning 0");
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    return 0;
  }

  const auto& ci = input_instruction->as<CompositeInstruction>();

  // Get Composite Profile
  std::string profile = ci.getProfile();
  profile = getProfileString(profile, name_, input.composite_profile_remapping);
  auto cur_composite_profile =
      getProfile<ProfileSwitchProfile>(profile, composite_profiles, std::make_shared<ProfileSwitchProfile>());
  cur_composite_profile = applyProfileOverrides(name_, cur_composite_profile, ci.profile_overrides);

  // Return the value specified in the profile
  CONSOLE_BRIDGE_logDebug("ProfileSwitchProfile returning %d", cur_composite_profile->return_value);
  return cur_composite_profile->return_value;
}

void ProfileSwitchTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

ProfileSwitchTaskInfo::ProfileSwitchTaskInfo(std::size_t unique_id, std::string name)
  : TaskInfo(unique_id, std::move(name))
{
}
}  // namespace tesseract_planning
