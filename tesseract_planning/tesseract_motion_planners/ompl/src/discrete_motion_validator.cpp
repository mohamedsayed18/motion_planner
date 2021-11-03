/**
 * @file discrete_motion_validator.cpp
 * @brief Tesseract OMPL planner discrete collision check between two states
 *
 * @author Jonathan Meyer, Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <ompl/base/SpaceInformation.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>

namespace tesseract_planning
{
DiscreteMotionValidator::DiscreteMotionValidator(const ompl::base::SpaceInformationPtr& space_info)
  : MotionValidator(space_info)
{
}

bool DiscreteMotionValidator::checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  std::pair<ompl::base::State*, double> dummy = { nullptr, 0.0 };
  return checkMotion(s1, s2, dummy);
}

bool DiscreteMotionValidator::checkMotion(const ompl::base::State* s1,
                                          const ompl::base::State* s2,
                                          std::pair<ompl::base::State*, double>& lastValid) const
{
  const ompl::base::StateSpace& state_space = *si_->getStateSpace();

  unsigned n_steps = state_space.validSegmentCount(s1, s2);
  bool is_valid = true;

  if (n_steps > 1)
  {
    ompl::base::State* end_interp = si_->allocState();
    for (unsigned i = 1; i < n_steps; ++i)
    {
      state_space.interpolate(s1, s2, static_cast<double>(i) / static_cast<double>(n_steps), end_interp);

      if (!si_->isValid(end_interp))
      {
        lastValid.second = static_cast<double>(i - 1) / static_cast<double>(n_steps);
        if (lastValid.first != nullptr)
          state_space.interpolate(s1, s2, lastValid.second, lastValid.first);

        is_valid = false;
        break;
      }
    }
    si_->freeState(end_interp);
  }

  if (is_valid)
  {
    if (!si_->isValid(s2))
    {
      lastValid.second = static_cast<double>(n_steps - 1) / static_cast<double>(n_steps);
      if (lastValid.first != nullptr)
        state_space.interpolate(s1, s2, lastValid.second, lastValid.first);

      is_valid = false;
    }
  }

  return is_valid;
}
}  // namespace tesseract_planning
