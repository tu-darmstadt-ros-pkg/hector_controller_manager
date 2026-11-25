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

#include "hector_controller_manager/status_messages.hpp"

#include <algorithm>
#include <map>

#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hector_controller_manager/runtime/lifecycle_state.hpp"

namespace hector_controller_manager
{
namespace
{
using core::Lifecycle;
using StateMsg = lifecycle_msgs::msg::State;
namespace names = hardware_interface::lifecycle_state_names;

lifecycle_msgs::msg::State state_message(Lifecycle state, bool leaving)
{
  lifecycle_msgs::msg::State message;
  if (leaving)
  {
    // A callback still runs or RT stopped the entity, it is in or right before its transition
    switch (state)
    {
      case Lifecycle::Active:
        message.id = StateMsg::TRANSITION_STATE_DEACTIVATING;
        message.label = "deactivating";
        return message;
      case Lifecycle::Inactive:
        message.id = StateMsg::TRANSITION_STATE_ACTIVATING;
        message.label = "activating";
        return message;
      case Lifecycle::Unconfigured:
      case Lifecycle::Finalized:
        message.id = StateMsg::TRANSITION_STATE_CONFIGURING;
        message.label = "configuring";
        return message;
    }
  }
  const auto primary = runtime::primary_state(state);
  message.id = primary.id();
  message.label = primary.label();
  return message;
}

controller_manager_msgs::msg::HardwareInterface interface_entry(
  const std::string & name, bool available, bool claimed)
{
  controller_manager_msgs::msg::HardwareInterface entry;
  entry.name = name;
  entry.data_type = "double";
  entry.is_available = available;
  entry.is_claimed = claimed;
  return entry;
}

bool is_claimed(const Status & status, const std::string & interface)
{
  for (const auto & controller : status.controllers)
  {
    const auto & claimed = controller.model.claims.command;
    if (
      controller.model.state == Lifecycle::Active &&
      std::find(claimed.begin(), claimed.end(), interface) != claimed.end())
    {
      return true;
    }
  }
  return false;
}

void append_interfaces(
  const Status & status, const HardwareStatus & hardware,
  std::vector<controller_manager_msgs::msg::HardwareInterface> & commands,
  std::vector<controller_manager_msgs::msg::HardwareInterface> & states)
{
  const bool active = hardware.model.state == Lifecycle::Active;
  const bool readable = active || hardware.model.state == Lifecycle::Inactive;
  for (const auto & interface : hardware.model.command_interfaces)
  {
    commands.push_back(interface_entry(interface.name, active, is_claimed(status, interface.name)));
  }
  for (const auto & interface : hardware.model.state_interfaces)
  {
    states.push_back(interface_entry(interface, readable, false));
  }
}
}  // namespace

/// RT already stopped an active controller or faulted an active component, its on_deactivate or
/// on_error follows.
lifecycle_msgs::msg::State state_message(const ControllerStatus & controller)
{
  const auto & model = controller.model;
  const bool stopped =
    model.state == Lifecycle::Active &&
    controller.runtime->failed_activation.load(std::memory_order_acquire) == model.activation;
  return state_message(model.state, model.quarantined || stopped);
}

lifecycle_msgs::msg::State state_message(const HardwareStatus & hardware)
{
  const auto & model = hardware.model;
  const bool faulted =
    model.state == Lifecycle::Active && hardware.runtime->fault() != core::HardwareFault::None;
  return state_message(model.state, model.quarantined || faulted);
}

std::optional<Lifecycle> lifecycle_of(const lifecycle_msgs::msg::State & state)
{
  if (state.id == StateMsg::PRIMARY_STATE_UNKNOWN)
  {
    if (state.label == names::ACTIVE)
    {
      return Lifecycle::Active;
    }
    if (state.label == names::INACTIVE)
    {
      return Lifecycle::Inactive;
    }
    return std::nullopt;
  }
  if (state.id == StateMsg::PRIMARY_STATE_ACTIVE)
  {
    return Lifecycle::Active;
  }
  if (state.id == StateMsg::PRIMARY_STATE_INACTIVE)
  {
    return Lifecycle::Inactive;
  }
  return std::nullopt;
}

const HardwareStatus * find_hardware(const Status & status, const std::string & name)
{
  const auto it = std::find_if(
    status.hardware.begin(), status.hardware.end(),
    [&](const HardwareStatus & hardware) { return hardware.name == name; });
  return it == status.hardware.end() ? nullptr : &*it;
}

controller_manager_msgs::msg::ControllerManagerActivity activity_message(const Status & status)
{
  controller_manager_msgs::msg::ControllerManagerActivity message;
  for (const auto & controller : status.controllers)
  {
    controller_manager_msgs::msg::NamedLifecycleState named;
    named.name = controller.name;
    named.state = state_message(controller);
    message.controllers.push_back(named);
  }
  for (const auto & hardware : status.hardware)
  {
    controller_manager_msgs::msg::NamedLifecycleState named;
    named.name = hardware.name;
    named.state = state_message(hardware);
    message.hardware_components.push_back(named);
  }
  return message;
}

std::vector<controller_manager_msgs::msg::ControllerState> controller_states(
  const Status & status, uint16_t update_rate)
{
  std::map<std::string, std::string> reference_owner;
  for (const auto & controller : status.controllers)
  {
    for (const auto & reference : controller.model.references)
    {
      reference_owner.emplace(reference, controller.name);
    }
  }
  std::vector<controller_manager_msgs::msg::ControllerState> states;
  for (const auto & controller : status.controllers)
  {
    const auto & model = controller.model;
    controller_manager_msgs::msg::ControllerState state;
    state.name = controller.name;
    state.type = controller.type;
    state.state = state_message(controller).label;
    state.is_async = false;
    state.update_rate = update_rate;
    state.claimed_interfaces = model.claims.command;
    state.required_command_interfaces = model.interfaces.command;
    state.required_state_interfaces = model.interfaces.state;
    state.is_chainable = model.chainable;
    state.is_chained = model.chained;
    state.reference_interfaces = model.references;
    state.exported_state_interfaces = model.exported_states;
    std::map<std::string, std::vector<std::string>> connections;
    for (const auto & interface : model.interfaces.command)
    {
      if (const auto it = reference_owner.find(interface); it != reference_owner.end())
      {
        connections[it->second].push_back(interface);
      }
    }
    for (auto & [name, references] : connections)
    {
      controller_manager_msgs::msg::ChainConnection connection;
      connection.name = name;
      connection.reference_interfaces = references;
      state.chain_connections.push_back(connection);
    }
    states.push_back(state);
  }
  return states;
}

std::vector<controller_manager_msgs::msg::HardwareComponentState> hardware_component_states(
  const Status & status, uint16_t rw_rate)
{
  std::vector<controller_manager_msgs::msg::HardwareComponentState> components;
  for (const auto & hardware : status.hardware)
  {
    controller_manager_msgs::msg::HardwareComponentState component;
    component.name = hardware.name;
    component.type = hardware.info.type;
    component.is_async = hardware.info.is_async;
    component.rw_rate = rw_rate;
    component.plugin_name = hardware.info.hardware_plugin_name;
    component.state = state_message(hardware);
    append_interfaces(status, hardware, component.command_interfaces, component.state_interfaces);
    components.push_back(component);
  }
  return components;
}

void hardware_interfaces(
  const Status & status, std::vector<controller_manager_msgs::msg::HardwareInterface> & commands,
  std::vector<controller_manager_msgs::msg::HardwareInterface> & states)
{
  for (const auto & hardware : status.hardware)
  {
    append_interfaces(status, hardware, commands, states);
  }
  for (const auto & controller : status.controllers)
  {
    const bool active = controller.model.state == Lifecycle::Active;
    for (const auto & reference : controller.model.references)
    {
      commands.push_back(interface_entry(reference, active, is_claimed(status, reference)));
    }
    for (const auto & exported : controller.model.exported_states)
    {
      states.push_back(interface_entry(exported, active, false));
    }
  }
}

}  // namespace hector_controller_manager
