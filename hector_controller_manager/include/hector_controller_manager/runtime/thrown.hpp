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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__THROWN_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__THROWN_HPP_

#include <exception>
#include <optional>
#include <string>

namespace hector_controller_manager::runtime
{
/// Runs `work` and returns the text of what it threw, or nothing if it returned. It allocates, so
/// RT keeps its own handlers.
template <typename Work>
std::optional<std::string> thrown_by(Work && work)
{
  try
  {
    work();
    return std::nullopt;
  }
  catch (const std::exception & e)
  {
    return e.what();
  }
  catch (...)
  {
    return "an exception of unknown type";
  }
}

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__THROWN_HPP_
