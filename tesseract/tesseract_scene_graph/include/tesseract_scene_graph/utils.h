/**
 * @file utils.h
 * @brief Tesseract Scene Graph utility functions
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_SCENE_GRAPH_UTILS_H
#define TESSERACT_SCENE_GRAPH_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/allowed_collision_matrix.h>

namespace tesseract_scene_graph
{
/**
 * @brief Gets allowed collisions for a set of link names.
 * @param link_names Vector of link names for which we want the allowed collisions
 * @param acm_entries Entries in the ACM. Get this with AllowedCollisionMatrix::getAllAllowedCollisions()
 * @param remove_duplicates If true, duplicates will be removed. Default: true
 * @return vector of links that are allowed to collide with links given
 */
std::vector<std::string> getAllowedCollisions(const std::vector<std::string>& link_names,
                                              const AllowedCollisionEntries& acm_entries,
                                              bool remove_duplicates = true);

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_UTILS_H
