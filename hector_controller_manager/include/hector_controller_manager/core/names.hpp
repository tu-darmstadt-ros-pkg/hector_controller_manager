// Copyright 2026 Team Hector
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HECTOR_CONTROLLER_MANAGER__CORE__NAMES_HPP_
#define HECTOR_CONTROLLER_MANAGER__CORE__NAMES_HPP_

#include <algorithm>
#include <iterator>
#include <string>

#include "hector_controller_manager/core/model.hpp"

/// Set helpers for names.
namespace hector_controller_manager::core
{
inline Names merged(Names a, const Names & b)
{
  a.insert(b.begin(), b.end());
  return a;
}

inline Names without(Names a, const Names & b)
{
  for (const auto & name : b)
  {
    a.erase(name);
  }
  return a;
}

inline Names intersection(const Names & a, const Names & b)
{
  Names result;
  std::set_intersection(
    a.begin(), a.end(), b.begin(), b.end(), std::inserter(result, result.end()));
  return result;
}

inline bool is_subset(const Names & a, const Names & b)
{
  return std::includes(b.begin(), b.end(), a.begin(), a.end());
}

inline std::string text(const Names & names)
{
  std::string result;
  for (const auto & name : names)
  {
    result += (result.empty() ? "'" : ", '") + name + "'";
  }
  return result.empty() ? "nothing" : result;
}

}  // namespace hector_controller_manager::core

#endif  // HECTOR_CONTROLLER_MANAGER__CORE__NAMES_HPP_
