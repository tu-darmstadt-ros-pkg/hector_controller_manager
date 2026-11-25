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

#include "hector_controller_manager/core/model.hpp"

#include <algorithm>
#include <stdexcept>

namespace hector_controller_manager::core
{
const char * to_string(Lifecycle state)
{
  switch (state)
  {
    case Lifecycle::Unconfigured:
      return "unconfigured";
    case Lifecycle::Inactive:
      return "inactive";
    case Lifecycle::Active:
      return "active";
    case Lifecycle::Finalized:
      return "finalized";
  }
  return "unknown";
}

InterfaceIndex::InterfaceIndex(const Model & model)
{
  const auto add = [this](const std::string & name, InterfaceOwner owner)
  {
    const bool is_command = owner.kind == InterfaceOwner::Kind::HardwareCommand ||
                            owner.kind == InterfaceOwner::Kind::Reference;
    if (!(is_command ? commands_ : states_).emplace(name, std::move(owner)).second)
    {
      throw std::invalid_argument("Interface '" + name + "' is exported twice");
    }
  };
  for (const auto & [name, hw] : model.hardware)
  {
    for (const auto & interface : hw.command_interfaces)
    {
      add(interface.name, {InterfaceOwner::Kind::HardwareCommand, name, interface.joint});
    }
    for (const auto & interface : hw.state_interfaces)
    {
      add(interface, {InterfaceOwner::Kind::HardwareState, name, {}});
    }
  }
  for (const auto & [name, controller] : model.controllers)
  {
    for (const auto & interface : controller.references)
    {
      add(interface, {InterfaceOwner::Kind::Reference, name, {}});
    }
    for (const auto & interface : controller.exported_states)
    {
      add(interface, {InterfaceOwner::Kind::ExportedState, name, {}});
    }
  }
}

namespace
{
const InterfaceOwner * find(
  const std::map<std::string, InterfaceOwner> & owners, const std::string & name)
{
  const auto it = owners.find(name);
  return it == owners.end() ? nullptr : &it->second;
}
}  // namespace

const InterfaceOwner * InterfaceIndex::command(const std::string & name) const
{
  return find(commands_, name);
}

const InterfaceOwner * InterfaceIndex::hardware_command(const std::string & name) const
{
  const auto * owner = command(name);
  return owner && owner->kind == InterfaceOwner::Kind::HardwareCommand ? owner : nullptr;
}

const InterfaceOwner * InterfaceIndex::state(const std::string & name) const
{
  return find(states_, name);
}

bool commandable(const Hardware & hw)
{
  return hw.state == Lifecycle::Active && hw.fault == HardwareFault::None && !hw.quarantined;
}

bool readable(const Hardware & hw)
{
  return (hw.state == Lifecycle::Active || hw.state == Lifecycle::Inactive) &&
         hw.fault == HardwareFault::None && !hw.quarantined;
}

bool is_running_member(const Controller & controller)
{
  return controller.state == Lifecycle::Active && !controller.failed && !controller.quarantined;
}

Names running_members(const Model & model)
{
  Names names;
  for (const auto & [name, controller] : model.controllers)
  {
    if (is_running_member(controller))
    {
      names.insert(name);
    }
  }
  return names;
}

bool activated(const Controller & controller, uint64_t min_activation)
{
  return is_running_member(controller) && controller.activation >= min_activation;
}

bool estop_stops(const Controller & controller)
{
  return !controller.is_protected && !controller.interfaces.command.empty();
}

bool holds_claims(const Controller & controller)
{
  return controller.state == Lifecycle::Active || controller.quarantined;
}

const ControllerEntry * command_claimer(const Model & model, const std::string & interface)
{
  for (const auto & entry : model.controllers)
  {
    const auto & claimed = entry.second.claims.command;
    if (
      holds_claims(entry.second) &&
      std::find(claimed.begin(), claimed.end(), interface) != claimed.end())
    {
      return &entry;
    }
  }
  return nullptr;
}

std::optional<std::string> hung_claimer(const Model & model, const Hardware & hw)
{
  for (const auto & interface : hw.command_interfaces)
  {
    const auto * claimer = command_claimer(model, interface.name);
    if (claimer && claimer->second.state == Lifecycle::Active && claimer->second.quarantined)
    {
      return claimer->first;
    }
  }
  return std::nullopt;
}

Names names_of(const Desired & desired)
{
  Names names;
  for (const auto & [name, unused] : desired.active)
  {
    names.insert(name);
  }
  return names;
}

std::map<std::string, Claims> claims_of(const Desired & desired)
{
  std::map<std::string, Claims> claims;
  for (const auto & [name, want] : desired.active)
  {
    claims.emplace(name, want.claims);
  }
  return claims;
}

}  // namespace hector_controller_manager::core
