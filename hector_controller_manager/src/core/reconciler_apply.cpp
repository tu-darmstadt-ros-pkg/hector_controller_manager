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

#include "hector_controller_manager/core/reconciler.hpp"

#include <algorithm>
#include <optional>
#include <type_traits>
#include <utility>

#include "hector_controller_manager/core/chain_rules.hpp"
#include "hector_controller_manager/core/names.hpp"

namespace hector_controller_manager::core
{
namespace
{
using Kind = InterfaceOwner::Kind;

void apply_actual(Model & actual, const Step & step, const Outcome & outcome)
{
  const auto returned = [&] { return outcome.result != CallResult::TimedOut; };
  std::visit(
    [&](const auto & executed)
    {
      using T = std::decay_t<decltype(executed)>;
      if constexpr (std::is_same_v<T, step::Hold>)
      {
        for (const auto & [name, interfaces] : executed.interfaces)
        {
          auto & hw = actual.hardware.at(name);
          for (const auto & interface : interfaces)
          {
            hw.live.erase(interface);
          }
        }
      }
      else if constexpr (std::is_same_v<T, step::Take>)
      {
        for (const auto & name : executed.hardware)
        {
          actual.hardware.at(name).taken = true;
        }
      }
      else if constexpr (std::is_same_v<T, step::SwitchModes>)
      {
        for (const auto & [name, modes] : executed.modes)
        {
          auto & hw = actual.hardware.at(name);
          for (const auto & interface : modes.stop)
          {
            hw.live.erase(interface);
          }
          for (const auto & interface : modes.start)
          {
            if (!outcome.faulted_hardware.count(name) && !outcome.skipped.count(interface))
            {
              hw.live.insert(interface);
            }
          }
          // A call given up on keeps the writes blocked until the next lifecycle transition
          hw.taken = hw.taken && outcome.timed_out_hardware.count(name);
        }
      }
      else if constexpr (std::is_same_v<T, step::Detach>)
      {
        for (const auto & name : executed.controllers)
        {
          actual.controllers.at(name).running = false;
          actual.controllers.at(name).updated = false;
        }
      }
      else if constexpr (std::is_same_v<T, step::Deactivate>)
      {
        auto & controller = actual.controllers.at(executed.controller);
        if (!returned())
        {
          ++controller.quarantined;
          return;
        }
        controller.state = outcome.result == CallResult::Ok ? Lifecycle::Inactive : outcome.state;
        controller.claims = {};
        controller.failed = false;
      }
      else if constexpr (std::is_same_v<T, step::Hardware>)
      {
        auto & hw = actual.hardware.at(executed.hardware);
        if (!returned())
        {
          ++hw.quarantined;
          return;
        }
        hw.state = outcome.state;
        hw.fault = HardwareFault::None;
        hw.live.clear();
        hw.taken = false;
      }
      else if constexpr (std::is_same_v<T, step::SetChained>)
      {
        if (!returned())
        {
          ++actual.controllers.at(executed.controller).quarantined;
        }
        else if (outcome.result == CallResult::Ok)
        {
          actual.controllers.at(executed.controller).chained = executed.chained;
        }
      }
      else if constexpr (std::is_same_v<T, step::Activate>)
      {
        auto & controller = actual.controllers.at(executed.controller);
        if (!returned())
        {
          ++controller.quarantined;
          controller.claims = executed.claims;
          return;
        }
        if (outcome.result == CallResult::Ok)
        {
          controller.state = Lifecycle::Active;
          ++controller.activation;
          controller.claims = executed.claims;
          controller.failed = false;
        }
        else
        {
          controller.state = outcome.state;
          controller.claims = {};
        }
      }
      else if constexpr (std::is_same_v<T, step::Attach>)
      {
        for (const auto & name : executed.controllers)
        {
          actual.controllers.at(name).running = true;
          actual.controllers.at(name).updated = false;
        }
      }
      else if constexpr (std::is_same_v<T, step::AwaitUpdate>)
      {
        for (const auto & name : executed.controllers)
        {
          actual.controllers.at(name).updated = !outcome.not_updated;
        }
      }
    },
    step);
}

Lifecycle expected_state(step::HardwareTransition transition)
{
  switch (transition)
  {
    case step::HardwareTransition::Configure:
    case step::HardwareTransition::Deactivate:
      return Lifecycle::Inactive;
    case step::HardwareTransition::Activate:
      return Lifecycle::Active;
    case step::HardwareTransition::Cleanup:
    case step::HardwareTransition::Error:
      return Lifecycle::Unconfigured;
    case step::HardwareTransition::Shutdown:
      return Lifecycle::Finalized;
  }
  return Lifecycle::Finalized;
}
}  // namespace

Failures apply(Model & actual, Desired & desired, const Step & step, const Outcome & outcome)
{
  const auto * hardware_step = std::get_if<step::Hardware>(&step);
  // Fault handling keeps its target, nothing comes back up because of a fault
  const bool fault_driven =
    hardware_step && actual.hardware.at(hardware_step->hardware).fault != HardwareFault::None;
  apply_actual(actual, step, outcome);
  apply(actual, GivenUp{{}, outcome.timed_out_hardware});
  if (fault_driven)
  {
    // Fault handling that ended below its target, through on_error, stays there
    const auto & component = actual.hardware.at(hardware_step->hardware);
    const auto target = desired.hardware.find(hardware_step->hardware);
    if (
      target != desired.hardware.end() && target->second != Lifecycle::Finalized &&
      component.state < target->second)
    {
      target->second = component.state;
    }
  }
  Names failed;
  Names refused;
  if (const auto * activate = std::get_if<step::Activate>(&step))
  {
    if (outcome.result != CallResult::Ok)
    {
      failed.insert(activate->controller);
    }
  }
  else if (const auto * deactivate = std::get_if<step::Deactivate>(&step))
  {
    // A controller that did not stop cleanly is not started again, a restart is dropped
    if (outcome.result != CallResult::Ok)
    {
      failed.insert(deactivate->controller);
    }
  }
  else if (const auto * chained = std::get_if<step::SetChained>(&step))
  {
    if (outcome.result == CallResult::TimedOut)
    {
      failed.insert(chained->controller);
    }
    else if (outcome.result != CallResult::Ok)
    {
      refused.insert(chained->controller);
    }
  }
  else if (const auto * hw = std::get_if<step::Hardware>(&step))
  {
    const auto & component = actual.hardware.at(hw->hardware);
    if (outcome.result == CallResult::TimedOut || component.state != expected_state(hw->transition))
    {
      // A late success of a requested transition is undone toward the state it started from
      if (!fault_driven)
      {
        desired.hardware[hw->hardware] = component.state;
      }
      failed.insert(hw->hardware);
      // A failed transition can leave the component below what its claimers need, like a fault
      const InterfaceIndex index(actual);
      for (const auto & [name, controller] : actual.controllers)
      {
        if (!holds_claims(controller))
        {
          continue;
        }
        for_each_claim(
          controller.claims, index,
          [&](const std::string &, const InterfaceOwner & owner)
          {
            const bool usable =
              component.state == Lifecycle::Active ||
              (owner.kind == Kind::HardwareState && component.state == Lifecycle::Inactive);
            if (is_hardware_kind(owner.kind) && owner.owner == hw->hardware && !usable)
            {
              failed.insert(name);
            }
          });
      }
    }
  }
  std::map<std::string, HardwareFault> faulted;
  for (const auto & name : outcome.faulted_hardware)
  {
    faulted.emplace(name, HardwareFault::Error);
  }
  apply_failures(actual, desired, {}, faulted);
  shrink(actual, desired, merged(failed, refused));
  return {merged(failed, outcome.faulted_hardware), refused};
}

void apply(Model & actual, const GivenUp & given_up)
{
  for (const auto & name : given_up.controllers)
  {
    ++actual.controllers.at(name).quarantined;
  }
  for (const auto & name : given_up.hardware)
  {
    ++actual.hardware.at(name).quarantined;
  }
}

void apply(Model & actual, const Returned & returned)
{
  const auto release = [](uint32_t & quarantined) { quarantined -= quarantined > 0 ? 1 : 0; };
  std::visit(
    [&](const auto & event)
    {
      using T = std::decay_t<decltype(event)>;
      if constexpr (std::is_same_v<T, returned::ControllerCallback>)
      {
        auto & controller = actual.controllers.at(event.controller);
        release(controller.quarantined);
        if (event.state == Lifecycle::Active && controller.state != Lifecycle::Active)
        {
          ++controller.activation;
        }
        controller.state = event.state;
        // Like a callback that returned in time, it ends the activation a failure refers to
        controller.failed = false;
        if (event.state != Lifecycle::Active)
        {
          controller.claims = {};
        }
      }
      else if constexpr (std::is_same_v<T, returned::HardwareCallback>)
      {
        auto & hw = actual.hardware.at(event.hardware);
        release(hw.quarantined);
        hw.state = event.state;
        hw.fault = HardwareFault::None;
        hw.live.clear();
        hw.taken = false;
      }
      else if constexpr (std::is_same_v<T, returned::ChainedMode>)
      {
        auto & controller = actual.controllers.at(event.controller);
        release(controller.quarantined);
        controller.chained = event.chained.value_or(controller.chained);
      }
      else if constexpr (std::is_same_v<T, returned::Released>)
      {
        // An unloaded controller is gone with its quarantine
        for (const auto & name : event.controllers)
        {
          if (const auto it = actual.controllers.find(name); it != actual.controllers.end())
          {
            release(it->second.quarantined);
          }
        }
        for (const auto & name : event.hardware)
        {
          release(actual.hardware.at(name).quarantined);
        }
      }
    },
    returned);
}

void shrink(const Model & actual, Desired & desired, const Names & seeds)
{
  for (const auto & name :
       stop_closure(ChainGraph(InterfaceIndex(actual), claims_of(desired)), seeds))
  {
    desired.active.erase(name);
  }
}

void apply_failures(
  Model & actual, Desired & desired, const Names & failed_controllers,
  const std::map<std::string, HardwareFault> & faulted_hardware)
{
  const InterfaceIndex index(actual);
  Names seeds = failed_controllers;
  for (const auto & name : failed_controllers)
  {
    actual.controllers.at(name).failed = true;
  }
  for (const auto & [name, fault] : faulted_hardware)
  {
    actual.hardware.at(name).fault = fault;
    desired.hardware[name] =
      fault == HardwareFault::Deactivate ? Lifecycle::Inactive : Lifecycle::Unconfigured;
    seeds.merge(claimers_holding(actual, index, name));
    // Also what is about to start on it
    for (const auto & [controller, want] : desired.active)
    {
      if (claims_hardware(index, want.claims, name))
      {
        seeds.insert(controller);
      }
    }
  }
  shrink(actual, desired, seeds);
}

std::vector<std::string> check_invariants(const Model & actual)
{
  std::vector<std::string> violations;
  const InterfaceIndex index(actual);
  std::map<std::string, std::string> claimer;
  for (const auto & [name, controller] : actual.controllers)
  {
    if (controller.running && (controller.state != Lifecycle::Active || controller.quarantined))
    {
      violations.push_back(name + " runs while not active");
    }
    if (!holds_claims(controller))
    {
      continue;
    }
    for (const auto & interface : controller.claims.command)
    {
      const auto [it, inserted] = claimer.emplace(interface, name);
      if (!inserted)
      {
        violations.push_back(interface + " is claimed by " + it->second + " and " + name);
      }
    }
    // A quarantined user is out of the running set, its providers may stop
    if (controller.state != Lifecycle::Active || controller.quarantined)
    {
      continue;
    }
    for_each_claim(
      controller.claims, index,
      [&](const std::string &, const InterfaceOwner & owner)
      {
        if (
          is_provider_kind(owner.kind) &&
          actual.controllers.at(owner.owner).state != Lifecycle::Active)
        {
          violations.push_back(name + " is active over inactive provider " + owner.owner);
        }
      });
  }
  for (const auto & [name, hw] : actual.hardware)
  {
    std::map<std::string, std::string> joint_claimer;
    for (const auto & interface : hw.command_interfaces)
    {
      const auto * claimer = command_claimer(actual, interface.name);
      if (!hw.live.count(interface.name) || !claimer)
      {
        continue;
      }
      const auto [it, first] = joint_claimer.emplace(interface.joint, claimer->first);
      if (!first && it->second != claimer->first)
      {
        violations.push_back(
          "joint " + interface.joint + " of " + name + " is live in modes of " + it->second +
          " and " + claimer->first);
      }
    }
    // A quarantined, taken or faulted component writes nothing. Its owner is in a call or its
    // writes are blocked, a faulted one's until its next lifecycle transition
    const bool writes_nothing = hw.quarantined || hw.taken || hw.fault != HardwareFault::None;
    for (const auto & interface : writes_nothing ? Names{} : hw.live)
    {
      const auto * claimer = command_claimer(actual, interface);
      const bool applied_by_runner =
        claimer && claimer->second.state == Lifecycle::Active &&
        ((claimer->second.running && claimer->second.updated) || claimer->second.failed);
      if (!applied_by_runner)
      {
        violations.push_back(interface + " of " + name + " is live without an updated claimer");
      }
    }
  }
  return violations;
}

}  // namespace hector_controller_manager::core
