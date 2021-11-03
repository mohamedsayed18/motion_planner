/**
 * @file serialization.h
 * @brief Provides boost exports for waypoint and instruction implementation for serialization
 * Also includes utility function
 *   - toArchiveStringXML
 *   - toArchiveFileXML
 *   - fromArchiveStringXML
 *   - fromArchiveFileXML
 *
 * @author Levi Armstrong
 * @date February 24, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_COMMAND_LANGUAGE_SERIALIZATION_H
#define TESSERACT_COMMAND_LANGUAGE_SERIALIZATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <sstream>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
struct Serialization
{
  template <typename SerializableType>
  static std::string toArchiveStringXML(const SerializableType& archive_type, const std::string& name = "")
  {
    std::stringstream ss;
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      boost::archive::xml_oarchive oa(ss);

      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << boost::serialization::make_nvp<SerializableType>("archive_type",
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << boost::serialization::make_nvp<SerializableType>(name.c_str(),
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return ss.str();
  }

  template <typename SerializableType>
  static bool toArchiveFileXML(const SerializableType& archive_type,
                               const std::string& file_path,
                               const std::string& name = "")
  {
    std::ofstream os(file_path);
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      boost::archive::xml_oarchive oa(os);
      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << boost::serialization::make_nvp<SerializableType>("archive_type",
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << boost::serialization::make_nvp<SerializableType>(name.c_str(),
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return true;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveStringXML(const std::string& archive_xml)
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::stringstream ss(archive_xml);
      boost::archive::xml_iarchive ia(ss);
      ia >> BOOST_SERIALIZATION_NVP(archive_type);
    }

    return archive_type;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveFileXML(const std::string& file_path)
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::ifstream ifs(file_path);
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);
      ia >> BOOST_SERIALIZATION_NVP(archive_type);
    }

    return archive_type;
  }
};

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_SERIALIZATION_H
