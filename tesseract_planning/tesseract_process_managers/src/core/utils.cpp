/**
 * @file utils.h
 * @brief Tesseract process managers utility functions
 *
 * @author Matthew Powelson
 * @author Levi Armstrong
 * @date July 15. 2020
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

#include <tesseract_process_managers/core/utils.h>

namespace tesseract_planning
{
void successTask(const TaskInput& /*instruction*/,
                 const std::string& name,
                 const std::string& message,
                 const TaskflowVoidFn& user_callback)
{
  CONSOLE_BRIDGE_logInform("%s Successful: %s", name.c_str(), message.c_str());
  if (user_callback)
    user_callback();
}

void failureTask(TaskInput instruction,
                 const std::string& name,
                 const std::string& message,
                 const TaskflowVoidFn& user_callback)
{
  // Call abort on the process input
  instruction.abort();

  using namespace backward;
  StackTrace st; st.load_here(32);
  Printer p; p.print(st);
  // Print an error if this is the first failure
  CONSOLE_BRIDGE_logError("%s Failure: %s", name.c_str(), message.c_str());
  if (user_callback)
    user_callback();
}

bool isCompositeEmpty(const CompositeInstruction& composite)
{
  if (composite.empty())
    return true;

  for (const auto& i : composite)
  {
    if (isCompositeInstruction(i))
    {
      const auto& sub_composite = i.as<CompositeInstruction>();
      if (isCompositeEmpty(sub_composite))
        return true;
    }
  }

  return false;
}

int hasSeedTask(TaskInput input)
{
  if (input.has_seed)
    return 1;

  assert(isCompositeInstruction(*(input.getResults())));
  if (isCompositeInstruction(*(input.getResults())))
  {
    const auto& composite = input.getResults()->as<CompositeInstruction>();
    if (isCompositeEmpty(composite))
    {
      CONSOLE_BRIDGE_logDebug("Seed is empty!");
      return 0;
    }
  }
  return 1;
}

void saveInputs(TaskInfo::Ptr info, TaskInput& input)
{
  if (input.save_io)
  {
    info->environment = input.env->clone();
    info->instructions_input = *input.getInstruction();
    info->results_input = *input.getResults();
  }
}

void saveOutputs(TaskInfo::Ptr info, TaskInput& input)
{
  if (input.save_io)
  {
    info->instructions_output = *input.getInstruction();
    info->results_output = *input.getResults();
  }
}
}  // namespace tesseract_planning
