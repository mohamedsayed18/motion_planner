/**
 * @file composite_instruction.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/null_instruction.h>
#include <tesseract_command_language/constants.h>
#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/types.h>

namespace tesseract_planning
{
enum class CompositeInstructionOrder
{
  ORDERED,               // Must go in forward
  UNORDERED,             // Any order is allowed
  ORDERED_AND_REVERABLE  // Can go forward or reverse the order
};

class CompositeInstruction
{
public:
  CompositeInstruction(std::string profile = DEFAULT_PROFILE_KEY,
                       CompositeInstructionOrder order = CompositeInstructionOrder::ORDERED,
                       ManipulatorInfo manipulator_info = ManipulatorInfo());

  CompositeInstructionOrder getOrder() const;

  void setDescription(const std::string& description);
  const std::string& getDescription() const;

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  /** @brief Dictionary of profiles that will override named profiles for a specific task*/
  ProfileDictionary::Ptr profile_overrides;

  void setManipulatorInfo(ManipulatorInfo info);
  const ManipulatorInfo& getManipulatorInfo() const;
  ManipulatorInfo& getManipulatorInfo();

  void setStartInstruction(Instruction instruction);

  void resetStartInstruction();
  const Instruction& getStartInstruction() const;
  Instruction& getStartInstruction();
  bool hasStartInstruction() const;

  void setInstructions(std::vector<tesseract_planning::Instruction> instructions);
  std::vector<tesseract_planning::Instruction>& getInstructions();
  const std::vector<tesseract_planning::Instruction>& getInstructions() const;

  void print(const std::string& prefix = "") const;

  bool operator==(const CompositeInstruction& rhs) const;

  bool operator!=(const CompositeInstruction& rhs) const;

  // C++ container support

  /** value_type */
  using value_type = Instruction;
  /** pointer */
  using pointer = typename std::vector<value_type>::pointer;
  /** const_pointer */
  using const_pointer = typename std::vector<value_type>::const_pointer;
  /** reference */
  using reference = typename std::vector<value_type>::reference;
  /** const_reference */
  using const_reference = typename std::vector<value_type>::const_reference;
  /** size_type */
  using size_type = typename std::vector<value_type>::size_type;
  /** difference_type */
  using difference_type = typename std::vector<value_type>::difference_type;
#ifndef SWIG
  /** iterator */
  using iterator = typename std::vector<value_type>::iterator;
  /** const_iterator */
  using const_iterator = typename std::vector<value_type>::const_iterator;
  /** reverse_iterator */
  using reverse_iterator = typename std::vector<value_type>::reverse_iterator;
  /** const_reverse_iterator */
  using const_reverse_iterator = typename std::vector<value_type>::const_reverse_iterator;
#endif

#ifndef SWIG

  template <class InputIt>
  CompositeInstruction(InputIt first, InputIt last) : container_(first, last)
  {
  }

#endif  // SWIG

#ifndef SWIG
  ///////////////
  // Iterators //
  ///////////////
  /** @brief returns an iterator to the beginning */
  iterator begin();
  /** @brief returns an iterator to the beginning */
  const_iterator begin() const;
  /** @brief returns an iterator to the end */
  iterator end();
  /** @brief returns an iterator to the end */
  const_iterator end() const;
  /** @brief returns a reverse iterator to the beginning */
  reverse_iterator rbegin();
  /** @brief returns a reverse iterator to the beginning */
  const_reverse_iterator rbegin() const;
  /** @brief returns a reverse iterator to the end */
  reverse_iterator rend();
  /** @brief returns a reverse iterator to the end */
  const_reverse_iterator rend() const;
  /** @brief returns an iterator to the beginning */
  const_iterator cbegin() const;
  /** @brief returns an iterator to the end */
  const_iterator cend() const;
  /** @brief returns a reverse iterator to the beginning */
  const_reverse_iterator crbegin() const;
  /** @brief returns a reverse iterator to the end */
  const_reverse_iterator crend() const;

  //////////////
  // Capacity //
  //////////////
  /** @brief checks whether the container is empty */
  bool empty() const;
  /** @brief returns the number of elements */
  size_type size() const;
  /** @brief returns the maximum possible number of elements */
  size_type max_size() const;
  /** @brief reserve number of elements */
  void reserve(size_type n);
  /** @brief returns the number of elements that can be held in currently allocated storage */
  size_type capacity() const;
  /** @brief reduces memory usage by freeing unused memory  */
  void shrink_to_fit();

  ////////////////////
  // Element Access //
  ////////////////////
  /** @brief access the first element */
  reference front();
  /** @brief access the first element */
  const_reference front() const;
  /** @brief access the last element */
  reference back();
  /** @brief access the last element */
  const_reference back() const;
  /** @brief access specified element with bounds checking */
  reference at(size_type n);
  /** @brief access specified element with bounds checking */
  const_reference at(size_type n) const;
  /** @brief direct access to the underlying array  */
  pointer data();
  /** @brief direct access to the underlying array  */
  const_pointer data() const;
  /** @brief access specified element */
  reference operator[](size_type pos);
  /** @brief access specified element */
  const_reference operator[](size_type pos) const;

  ///////////////
  // Modifiers //
  ///////////////
  /** @brief clears the contents */
  void clear();
  /** @brief inserts element */
  iterator insert(const_iterator p, const value_type& x);
  iterator insert(const_iterator p, value_type&& x);
  iterator insert(const_iterator p, std::initializer_list<value_type> l);
  template <class InputIt>
  void insert(const_iterator pos, InputIt first, InputIt last)
  {
    container_.insert(pos, first, last);
  }

  /** @brief constructs element in-place */
  template <class... Args>
  iterator emplace(const_iterator pos, Args&&... args);

  /** @brief erases element */
  iterator erase(const_iterator p);
  iterator erase(const_iterator first, const_iterator last);
  /** @brief adds an element to the end */
  void push_back(const value_type& x);
  void push_back(const value_type&& x);

  /** @brief constructs an element in-place at the end  */
  template <typename... Args>
#if __cplusplus > 201402L
  reference emplace_back(Args&&... args);
#else
  void emplace_back(Args&&... args);
#endif

  /** @brief removes the last element */
  void pop_back();
  /** @brief swaps the contents  */
  void swap(std::vector<value_type>& other);

#endif  // SWIG

#ifdef SWIG
  %ignore get_allocator;
  %ignore resize;
  %ignore assign;
  %ignore swap;
  %ignore insert;
  %ignore CompositeInstruction();
  %ignore CompositeInstruction(const CompositeInstruction&);
  %ignore CompositeInstruction(size_type);
  %ignore CompositeInstruction(size_type, value_type const &); 
  %swig_vector_methods(tesseract_planning::CompositeInstruction)
  %std_vector_methods(CompositeInstruction)
#endif  // SWIG

private:
  std::vector<value_type> container_;

  /** @brief The description of the instruction */
  std::string description_{ "Tesseract Composite Instruction" };

  /** @brief Contains information about the manipulator associated with this instruction*/
  ManipulatorInfo manipulator_info_;

  /**
   * @brief The profile applied its child plan instructions
   *
   * If it has a child composite instruction it uses the child composites profile for that section
   */
  std::string profile_{ DEFAULT_PROFILE_KEY };

  /** @brief The order of the composite instruction */
  CompositeInstructionOrder order_{ CompositeInstructionOrder::ORDERED };

  /**
   * @brief The start instruction to use for composite instruction. This should be of type PlanInstruction or
   * MoveInstruction but is stored as type Instruction because it is not required
   *
   * If not provided, the planner should use the current state of the robot is used and defined as fixed.
   */
  value_type start_instruction_{ NullInstruction() };

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_instruction_type(CompositeInstruction)
#else
TESSERACT_INSTRUCTION_EXPORT_KEY(tesseract_planning::CompositeInstruction);
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
