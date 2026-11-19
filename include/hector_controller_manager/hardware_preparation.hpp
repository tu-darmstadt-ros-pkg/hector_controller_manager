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

#ifndef HECTOR_CONTROLLER_MANAGER__HARDWARE_PREPARATION_HPP_
#define HECTOR_CONTROLLER_MANAGER__HARDWARE_PREPARATION_HPP_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/supervisor.hpp"

namespace hector_controller_manager
{
struct PreparedHardware
{
  /// With the controller manager's update rate as rw_rate and the limit parameters applied.
  hardware_interface::HardwareInfo info;
  /// Whether the commands are clamped to the joint limits.
  bool limit_commands;
};

struct HardwarePreparation
{
  std::vector<PreparedHardware> hardware;
  /// Why the description is rejected.
  std::optional<std::string> error;
};

/// Parses the components of a robot description and checks them against the configuration and the
/// initial states, before any plugin is created.
HardwarePreparation prepare_hardware(
  const std::string & description, const SupervisorConfig & config,
  const std::map<std::string, core::Lifecycle> & initial);

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__HARDWARE_PREPARATION_HPP_
