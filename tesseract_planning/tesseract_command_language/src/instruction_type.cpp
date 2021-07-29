/**
 * @file instruction_type.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/null_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
bool isCommentInstruction(const Instruction& instruction)
{
  UNUSED(instruction);
  // TODO: Implement CommentInstruction
  return false;
}

bool isVariableInstruction(const Instruction& instruction)
{
  UNUSED(instruction);
  // TODO: Implement VariableInstruction
  return false;
}

bool isAnalogInstruction(const Instruction& instruction)
{
  UNUSED(instruction);
  // TODO: Implement AnalogInstruction
  return false;
}

bool isIOInstruction(const Instruction& instruction)
{
  UNUSED(instruction);
  // TODO: Implement IOInstruction
  return false;
}

bool isCompositeInstruction(const Instruction& instruction)
{
  return (instruction.getType() == std::type_index(typeid(CompositeInstruction)));
}

bool isMoveInstruction(const Instruction& instruction)
{
  return (instruction.getType() == std::type_index(typeid(MoveInstruction)));
}

bool isPlanInstruction(const Instruction& instruction)
{
  return (instruction.getType() == std::type_index(typeid(PlanInstruction)));
}

bool isNullInstruction(const Instruction& instruction)
{
  return (instruction.getType() == std::type_index(typeid(NullInstruction)));
}

}  // namespace tesseract_planning
