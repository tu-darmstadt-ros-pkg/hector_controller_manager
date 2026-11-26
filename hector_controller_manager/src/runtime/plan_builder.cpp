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

#include "hector_controller_manager/runtime/plan_builder.hpp"

#include "hector_controller_manager/core/chain_rules.hpp"

namespace hector_controller_manager::runtime
{
using core::Names;

std::shared_ptr<Plan> build_plan(
  const core::Model & model,
  const std::map<std::string, std::shared_ptr<ControllerRuntime>> & controllers,
  const std::map<std::string, std::shared_ptr<HardwareRuntime>> & hardware, uint64_t generation)
{
  auto plan = std::make_shared<Plan>();
  plan->generation = generation;
  const core::InterfaceIndex index(model);
  std::map<std::string, core::Claims> claims;
  for (const auto & [name, controller] : model.controllers)
  {
    if (controller.running)
    {
      claims.emplace(name, controller.claims);
    }
  }
  const core::ChainGraph graph(index, claims);
  const auto order = graph.update_order();
  std::map<std::string, uint32_t> position;
  for (const auto & name : order)
  {
    position.emplace(name, static_cast<uint32_t>(plan->controllers.size()));
    const auto & controller = model.controllers.at(name);
    plan->controllers.push_back(
      {controllers.at(name).get(), controller.activation, {}, core::estop_stops(controller)});
    plan->keep_alive.push_back(controllers.at(name));
  }
  const auto scope_of = [&](const Names & seeds)
  {
    Plan::FailureScope scope;
    scope.controllers.begin = static_cast<uint32_t>(plan->scope_controllers.size());
    scope.holds.begin = static_cast<uint32_t>(plan->scope_holds.size());
    for (const auto & member : core::stop_closure(graph, seeds))
    {
      const auto it = position.find(member);
      if (it == position.end())
      {
        continue;
      }
      plan->scope_controllers.push_back(it->second);
      for (const auto & interface : model.controllers.at(member).claims.command)
      {
        if (const auto * owner = index.hardware_command(interface))
        {
          const auto & component = hardware.at(owner->owner);
          plan->scope_holds.push_back({component.get(), *component->command_index(interface)});
        }
      }
    }
    scope.controllers.end = static_cast<uint32_t>(plan->scope_controllers.size());
    scope.holds.end = static_cast<uint32_t>(plan->scope_holds.size());
    return scope;
  };
  for (auto & entry : plan->controllers)
  {
    entry.scope = scope_of({entry.runtime->name()});
  }
  for (const auto & [name, unused] : model.hardware)
  {
    Names seeds;
    for (const auto & claimer : core::claimers_of_hardware(model, index, name))
    {
      if (claims.count(claimer))
      {
        seeds.insert(claimer);
      }
    }
    const auto & component = hardware.at(name);
    std::vector<Plan::Claim> claimed(component->command_interfaces().size());
    for (const auto & claimer : seeds)
    {
      for (const auto & interface : claims.at(claimer).command)
      {
        const auto * owner = index.hardware_command(interface);
        if (owner && owner->owner == name)
        {
          // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the component owns the interface
          claimed[*component->command_index(interface)] = {
            controllers.at(claimer).get(), model.controllers.at(claimer).activation};
        }
      }
    }
    plan->hardware.push_back(
      {component.get(), !seeds.empty(), scope_of(seeds), std::move(claimed)});
    plan->keep_alive.push_back(hardware.at(name));
  }
  return plan;
}

}  // namespace hector_controller_manager::runtime
