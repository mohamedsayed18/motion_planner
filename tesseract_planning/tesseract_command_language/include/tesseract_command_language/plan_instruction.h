/**
 * @file plan_instruction.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_PLAN_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_PLAN_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/null_waypoint.h>
#include <tesseract_command_language/constants.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/types.h>

namespace tesseract_planning
{
enum class PlanInstructionType : int
{
  LINEAR = 0,
  FREESPACE = 1,
  CIRCULAR = 2,
  START = 3
};

class PlanInstruction
{
public:
  PlanInstruction() = default;  // Required for boost serialization do not use
  PlanInstruction(Waypoint waypoint,
                  PlanInstructionType type,
                  std::string profile = DEFAULT_PROFILE_KEY,
                  ManipulatorInfo manipulator_info = ManipulatorInfo());

  void setWaypoint(Waypoint waypoint);
  Waypoint& getWaypoint();
  const Waypoint& getWaypoint() const;

  void setManipulatorInfo(ManipulatorInfo info);
  const ManipulatorInfo& getManipulatorInfo() const;
  ManipulatorInfo& getManipulatorInfo();

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  /** @brief Dictionary of profiles that will override named profiles for a specific task*/
  ProfileDictionary::Ptr profile_overrides;

  const std::string& getDescription() const;

  void setDescription(const std::string& description);

  void print(const std::string& prefix = "") const;

  PlanInstructionType getPlanType() const;

  void setPlanType(PlanInstructionType type);

  bool isLinear() const;

  bool isFreespace() const;

  bool isCircular() const;

  bool isStart() const;

  bool operator==(const PlanInstruction& rhs) const;
  bool operator!=(const PlanInstruction& rhs) const;

private:
  PlanInstructionType plan_type_{ PlanInstructionType::START };

  /** @brief The assigned waypoint (Cartesian or Joint) */
  Waypoint waypoint_{ NullWaypoint() };

  /** @brief The profile used for this plan instruction */
  std::string profile_{ DEFAULT_PROFILE_KEY };

  /** @brief Contains information about the manipulator associated with this instruction*/
  ManipulatorInfo manipulator_info_;

  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Plan Instruction" };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_instruction_type(PlanInstruction)
#else
TESSERACT_INSTRUCTION_EXPORT_KEY(tesseract_planning::PlanInstruction);
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_PLAN_INSTRUCTION_H
