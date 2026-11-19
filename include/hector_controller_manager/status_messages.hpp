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

#ifndef HECTOR_CONTROLLER_MANAGER__STATUS_MESSAGES_HPP_
#define HECTOR_CONTROLLER_MANAGER__STATUS_MESSAGES_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "controller_manager_msgs/msg/controller_state.hpp"
#include "controller_manager_msgs/msg/hardware_component_state.hpp"
#include "controller_manager_msgs/msg/hardware_interface.hpp"
#include "hector_controller_manager/supervisor.hpp"
#include "lifecycle_msgs/msg/state.hpp"

// The status snapshot as the list services and `~/activity` report it
namespace hector_controller_manager
{
/// The transition state while a call still runs or RT already stopped or faulted the entity.
lifecycle_msgs::msg::State state_message(const ControllerStatus & controller);
lifecycle_msgs::msg::State state_message(const HardwareStatus & hardware);

/// The target of a hardware state request, active and inactive only. The id decides, the label
/// only for an unknown id.
std::optional<core::Lifecycle> lifecycle_of(const lifecycle_msgs::msg::State & state);

const HardwareStatus * find_hardware(const Status & status, const std::string & name);

/// Without a stamp, so equal states compare equal.
controller_manager_msgs::msg::ControllerManagerActivity activity_message(const Status & status);

std::vector<controller_manager_msgs::msg::ControllerState> controller_states(
  const Status & status, uint16_t update_rate);

std::vector<controller_manager_msgs::msg::HardwareComponentState> hardware_component_states(
  const Status & status, uint16_t rw_rate);

/// The interfaces of every component and the references and exported states of every controller.
void hardware_interfaces(
  const Status & status, std::vector<controller_manager_msgs::msg::HardwareInterface> & commands,
  std::vector<controller_manager_msgs::msg::HardwareInterface> & states);

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__STATUS_MESSAGES_HPP_
