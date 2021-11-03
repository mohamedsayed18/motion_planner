/**
 * @file composite_instruction.cpp
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
#include <stdexcept>
#include <iostream>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/null_instruction.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
CompositeInstruction::CompositeInstruction(std::string profile,
                                           CompositeInstructionOrder order,
                                           ManipulatorInfo manipulator_info)
  : manipulator_info_(std::move(manipulator_info)), profile_(std::move(profile)), order_(order)
{
}

CompositeInstructionOrder CompositeInstruction::getOrder() const { return order_; }

const std::string& CompositeInstruction::getDescription() const { return description_; }

void CompositeInstruction::setDescription(const std::string& description) { description_ = description; }

void CompositeInstruction::setProfile(const std::string& profile)
{
  profile_ = (profile.empty()) ? DEFAULT_PROFILE_KEY : profile;
}
const std::string& CompositeInstruction::getProfile() const { return profile_; }

void CompositeInstruction::setManipulatorInfo(ManipulatorInfo info) { manipulator_info_ = std::move(info); }
const ManipulatorInfo& CompositeInstruction::getManipulatorInfo() const { return manipulator_info_; }
ManipulatorInfo& CompositeInstruction::getManipulatorInfo() { return manipulator_info_; }

void CompositeInstruction::setStartInstruction(Instruction instruction) { start_instruction_ = std::move(instruction); }

void CompositeInstruction::resetStartInstruction() { start_instruction_ = NullInstruction(); }

const Instruction& CompositeInstruction::getStartInstruction() const { return start_instruction_; }

Instruction& CompositeInstruction::getStartInstruction() { return start_instruction_; }

bool CompositeInstruction::hasStartInstruction() const { return (!isNullInstruction(start_instruction_)); }

void CompositeInstruction::setInstructions(std::vector<tesseract_planning::Instruction> instructions)
{
  container_.swap(instructions);
}
std::vector<tesseract_planning::Instruction>& CompositeInstruction::getInstructions() { return container_; }
const std::vector<tesseract_planning::Instruction>& CompositeInstruction::getInstructions() const { return container_; }

void CompositeInstruction::print(const std::string& prefix) const
{
  std::cout << prefix + "Composite Instruction, Description: " << getDescription() << std::endl;
  std::cout << prefix + "--- Start Instruction, Description: " << start_instruction_.getDescription() << std::endl;
  std::cout << prefix + "{" << std::endl;
  for (const auto& i : *this)
    i.print(prefix + "  ");
  std::cout << prefix + "}" << std::endl;
}

bool CompositeInstruction::operator==(const CompositeInstruction& rhs) const
{
  bool equal = true;
  equal &= (static_cast<int>(order_) == static_cast<int>(rhs.order_));
  equal &= (profile_ == rhs.profile_);  // NOLINT
  equal &= (manipulator_info_ == rhs.manipulator_info_);
  equal &= (start_instruction_ == rhs.start_instruction_);
  equal &= (container_.size() == rhs.container_.size());
  if (equal)
  {
    for (std::size_t i = 0; i < container_.size(); ++i)
    {
      equal &= (container_[i] == rhs.container_[i]);

      if (!equal)
        break;
    }
  }
  return equal;
}

bool CompositeInstruction::operator!=(const CompositeInstruction& rhs) const { return !operator==(rhs); }

///////////////
// Iterators //
///////////////
CompositeInstruction::iterator CompositeInstruction::begin() { return container_.begin(); }
CompositeInstruction::const_iterator CompositeInstruction::begin() const { return container_.begin(); }
CompositeInstruction::iterator CompositeInstruction::end() { return container_.end(); }
CompositeInstruction::const_iterator CompositeInstruction::end() const { return container_.end(); }
CompositeInstruction::reverse_iterator CompositeInstruction::rbegin() { return container_.rbegin(); }
CompositeInstruction::const_reverse_iterator CompositeInstruction::rbegin() const { return container_.rbegin(); }
CompositeInstruction::reverse_iterator CompositeInstruction::rend() { return container_.rend(); }
CompositeInstruction::const_reverse_iterator CompositeInstruction::rend() const { return container_.rend(); }
CompositeInstruction::const_iterator CompositeInstruction::cbegin() const { return container_.cbegin(); }
CompositeInstruction::const_iterator CompositeInstruction::cend() const { return container_.cend(); }
CompositeInstruction::const_reverse_iterator CompositeInstruction::crbegin() const { return container_.crbegin(); }
CompositeInstruction::const_reverse_iterator CompositeInstruction::crend() const { return container_.crend(); }

//////////////
// Capacity //
//////////////
bool CompositeInstruction::empty() const { return container_.empty(); }
CompositeInstruction::size_type CompositeInstruction::size() const { return container_.size(); }
CompositeInstruction::size_type CompositeInstruction::max_size() const { return container_.max_size(); }
void CompositeInstruction::reserve(size_type n) { container_.reserve(n); }
CompositeInstruction::size_type CompositeInstruction::capacity() const { return container_.capacity(); }
void CompositeInstruction::shrink_to_fit() { container_.shrink_to_fit(); }

////////////////////
// Element Access //
////////////////////
CompositeInstruction::reference CompositeInstruction::front() { return container_.front(); }
CompositeInstruction::const_reference CompositeInstruction::front() const { return container_.front(); }
CompositeInstruction::reference CompositeInstruction::back() { return container_.back(); }
CompositeInstruction::const_reference CompositeInstruction::back() const { return container_.back(); }
CompositeInstruction::reference CompositeInstruction::at(size_type n) { return container_.at(n); }
CompositeInstruction::const_reference CompositeInstruction::at(size_type n) const { return container_.at(n); }
CompositeInstruction::pointer CompositeInstruction::data() { return container_.data(); }
CompositeInstruction::const_pointer CompositeInstruction::data() const { return container_.data(); }
CompositeInstruction::reference CompositeInstruction::operator[](size_type pos) { return container_[pos]; }
CompositeInstruction::const_reference CompositeInstruction::operator[](size_type pos) const { return container_[pos]; };

///////////////
// Modifiers //
///////////////
void CompositeInstruction::clear() { container_.clear(); }
CompositeInstruction::iterator CompositeInstruction::insert(const_iterator p, const value_type& x)
{
  return container_.insert(p, x);
}
CompositeInstruction::iterator CompositeInstruction::insert(const_iterator p, value_type&& x)
{
  return container_.insert(p, x);
}
CompositeInstruction::iterator CompositeInstruction::insert(const_iterator p, std::initializer_list<value_type> l)
{
  return container_.insert(p, l);
}

template <class... Args>
CompositeInstruction::iterator CompositeInstruction::emplace(const_iterator pos, Args&&... args)
{
  return container_.emplace(pos, std::forward<Args>(args)...);
}

CompositeInstruction::iterator CompositeInstruction::erase(const_iterator p) { return container_.erase(p); }
CompositeInstruction::iterator CompositeInstruction::erase(const_iterator first, const_iterator last)
{
  return container_.erase(first, last);
}
void CompositeInstruction::push_back(const value_type& x) { container_.push_back(x); }
void CompositeInstruction::push_back(const value_type&& x) { container_.push_back(x); }

template <typename... Args>
#if __cplusplus > 201402L
CompositeInstruction::reference CompositeInstruction::emplace_back(Args&&... args)
{
  return container_.emplace_back(std::forward<Args>(args)...);
}
#else
void CompositeInstruction::emplace_back(Args&&... args)
{
  container_.emplace_back(std::forward<Args>(args)...);
}
#endif

void CompositeInstruction::pop_back() { container_.pop_back(); }
void CompositeInstruction::swap(std::vector<value_type>& other) { container_.swap(other); }

template <class Archive>
void CompositeInstruction::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("description", description_);
  ar& boost::serialization::make_nvp("manipulator_info", manipulator_info_);
  ar& boost::serialization::make_nvp("profile", profile_);
  ar& boost::serialization::make_nvp("order", order_);
  ar& boost::serialization::make_nvp("start_instruction", start_instruction_);
  ar& boost::serialization::make_nvp("container", container_);
}

}  // namespace tesseract_planning

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
template void tesseract_planning::CompositeInstruction::serialize(boost::archive::xml_oarchive& ar,
                                                                  const unsigned int version);
template void tesseract_planning::CompositeInstruction::serialize(boost::archive::xml_iarchive& ar,
                                                                  const unsigned int version);

TESSERACT_INSTRUCTION_EXPORT_IMPLEMENT(tesseract_planning::CompositeInstruction);
