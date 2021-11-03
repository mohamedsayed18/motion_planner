/**
 * @file move_instruction.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/waypoint_type.h>

namespace tesseract_planning
{
MoveInstruction::MoveInstruction(Waypoint waypoint,
                                 MoveInstructionType type,
                                 std::string profile,
                                 ManipulatorInfo manipulator_info)
  : move_type_(type)
  , profile_(std::move(profile))
  , waypoint_(std::move(waypoint))
  , manipulator_info_(std::move(manipulator_info))
{
}

void MoveInstruction::setWaypoint(Waypoint waypoint)
{
  if (!isStateWaypoint(waypoint))
    CONSOLE_BRIDGE_logWarn("MoveInstruction usually expects to be provided a State Waypoint!");

  waypoint_ = std::move(waypoint);
}

Waypoint& MoveInstruction::getWaypoint() { return waypoint_; }
const Waypoint& MoveInstruction::getWaypoint() const { return waypoint_; }

void MoveInstruction::setManipulatorInfo(ManipulatorInfo info) { manipulator_info_ = std::move(info); }
const ManipulatorInfo& MoveInstruction::getManipulatorInfo() const { return manipulator_info_; }
ManipulatorInfo& MoveInstruction::getManipulatorInfo() { return manipulator_info_; }

void MoveInstruction::setProfile(const std::string& profile)
{
  profile_ = (profile.empty()) ? DEFAULT_PROFILE_KEY : profile;
}
const std::string& MoveInstruction::getProfile() const { return profile_; }

const std::string& MoveInstruction::getDescription() const { return description_; }

void MoveInstruction::setDescription(const std::string& description) { description_ = description; }

void MoveInstruction::print(const std::string& prefix) const
{
  std::cout << prefix + "Move Instruction, Move Type: " << static_cast<int>(move_type_) << ", ";
  getWaypoint().print();
  std::cout << ", Description: " << getDescription() << std::endl;
}

void MoveInstruction::setMoveType(MoveInstructionType move_type) { move_type_ = move_type; }

MoveInstructionType MoveInstruction::getMoveType() const { return move_type_; }

bool MoveInstruction::isLinear() const { return (move_type_ == MoveInstructionType::LINEAR); }

bool MoveInstruction::isFreespace() const { return (move_type_ == MoveInstructionType::FREESPACE); }

bool MoveInstruction::isCircular() const { return (move_type_ == MoveInstructionType::CIRCULAR); }

bool MoveInstruction::isStart() const { return (move_type_ == MoveInstructionType::START); }

bool MoveInstruction::operator==(const MoveInstruction& rhs) const
{
  bool equal = true;
  equal &= (static_cast<int>(move_type_) == static_cast<int>(rhs.move_type_));
  equal &= (waypoint_ == rhs.waypoint_);
  equal &= (manipulator_info_ == rhs.manipulator_info_);
  equal &= (profile_ == rhs.profile_);  // NO LINT
  return equal;
}

bool MoveInstruction::operator!=(const MoveInstruction& rhs) const { return !operator==(rhs); }

template <class Archive>
void MoveInstruction::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("move_type", move_type_);
  ar& boost::serialization::make_nvp("description", description_);
  ar& boost::serialization::make_nvp("profile", profile_);
  ar& boost::serialization::make_nvp("waypoint", waypoint_);
  ar& boost::serialization::make_nvp("manipulator_info", manipulator_info_);
}

}  // namespace tesseract_planning

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
template void tesseract_planning::MoveInstruction::serialize(boost::archive::xml_oarchive& ar,
                                                             const unsigned int version);
template void tesseract_planning::MoveInstruction::serialize(boost::archive::xml_iarchive& ar,
                                                             const unsigned int version);

TESSERACT_INSTRUCTION_EXPORT_IMPLEMENT(tesseract_planning::MoveInstruction);
