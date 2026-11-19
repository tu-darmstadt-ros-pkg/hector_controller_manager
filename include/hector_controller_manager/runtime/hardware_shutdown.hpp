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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__HARDWARE_SHUTDOWN_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__HARDWARE_SHUTDOWN_HPP_

#include <map>
#include <optional>
#include <string>

#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/runtime/hardware_runtime.hpp"

namespace hector_controller_manager::runtime
{
/// Deactivates and shuts down every component on its owner, all in parallel, so a slow one does
/// not use up the time of the others. Returns the state each one ended in, none for a component
/// whose call still runs at the deadline.
std::map<std::string, std::optional<core::Lifecycle>> shut_down_all(
  const std::map<std::string, HardwareRuntime *> & hardware, Clock::time_point deadline);

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__HARDWARE_SHUTDOWN_HPP_
