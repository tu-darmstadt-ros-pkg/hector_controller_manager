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

const DesiredController * wanted(const Desired & desired, const std::string & name)
{
  const auto it = desired.active.find(name);
  return it == desired.active.end() ? nullptr : &it->second;
}

/// Active in the activation desired asks for.
bool wanted_and_activated(const Controller & controller, const DesiredController * want)
{
  return want && activated(controller, want->min_activation);
}

bool stopping(const Controller & controller, const DesiredController * want)
{
  return controller.state == Lifecycle::Active && !controller.quarantined &&
         !wanted_and_activated(controller, want);
}

/// Active users of the provider's references or exported states. A quarantined user is out of
/// the running set and its hardware holds, so it does not keep its providers running.
Names users_holding(const Model & model, const InterfaceIndex & index, const std::string & provider)
{
  Names users;
  for (const auto & [name, controller] : model.controllers)
  {
    if (controller.state != Lifecycle::Active || controller.quarantined)
    {
      continue;
    }
    for_each_claim(
      controller.claims, index,
      [&](const std::string &, const InterfaceOwner & owner)
      {
        if (is_provider_kind(owner.kind) && owner.owner == provider)
        {
          users.insert(name);
        }
      });
  }
  return users;
}

bool desired_chained(
  const Desired & desired, const InterfaceIndex & index, const std::string & name)
{
  return std::any_of(
    desired.active.begin(), desired.active.end(),
    [&](const auto & entry) { return providers_written(index, entry.second.claims).count(name); });
}

Lifecycle desired_hardware_state(
  const Desired & desired, const std::string & name, Lifecycle fallback)
{
  const auto it = desired.hardware.find(name);
  return it == desired.hardware.end() ? fallback : it->second;
}

const CommandInterface * find_command(const Hardware & hw, const std::string & interface)
{
  const auto it = std::find_if(
    hw.command_interfaces.begin(), hw.command_interfaces.end(),
    [&](const CommandInterface & candidate) { return candidate.name == interface; });
  return it == hw.command_interfaces.end() ? nullptr : &*it;
}

/// A controller desired after the switch commands the joint of the interface, in any mode.
bool has_successor(const Desired & desired, const Hardware & hw, const CommandInterface & interface)
{
  for (const auto & [unused, want] : desired.active)
  {
    for (const auto & claimed : want.claims.command)
    {
      const auto * other = find_command(hw, claimed);
      if (other && other->joint == interface.joint)
      {
        return true;
      }
    }
  }
  return false;
}

bool keeps_running(const Model & actual, const Desired & desired, const std::string & interface)
{
  const auto * claimer = command_claimer(actual, interface);
  return claimer && claimer->second.running &&
         !stopping(claimer->second, wanted(desired, claimer->first));
}

/// The claimer of the live interface stops as planned, a successor desired `after` the switch
/// takes the joint over, and NaN can clear what the claimer leaves behind. Without a successor the
/// joint holds before Detach.
bool hands_over(
  const Model & actual, const Desired & desired, const Desired & after, const Hardware & hw,
  const std::string & interface)
{
  const auto * claimer = command_claimer(actual, interface);
  const auto * command = find_command(hw, interface);
  return claimer && claimer->second.running && !claimer->second.failed &&
         stopping(claimer->second, wanted(desired, claimer->first)) && command &&
         command->takes_nan && has_successor(after, hw, *command);
}

/// Taking blocks every write of the component, so no claimer of it may keep running.
bool may_take(
  const Model & actual, const Desired & desired, const Desired & after, const Hardware & hw)
{
  if (hw.taken || hw.quarantined || hw.fault != HardwareFault::None)
  {
    return false;
  }
  bool handing_over = false;
  for (const auto & interface : hw.live)
  {
    if (keeps_running(actual, desired, interface))
    {
      return false;
    }
    handing_over = handing_over || hands_over(actual, desired, after, hw, interface);
  }
  return handing_over;
}

std::optional<step::Hold> hold_step(
  const Model & actual, const Desired & desired, const Desired & after)
{
  step::Hold hold;
  for (const auto & [name, hw] : actual.hardware)
  {
    // A switch job still runs there, its writes stay blocked until a lifecycle transition
    if (hw.quarantined)
    {
      continue;
    }
    const bool takes = may_take(actual, desired, after, hw);
    for (const auto & interface : hw.live)
    {
      // A taken component stops what nobody claims any more in its single mode switch, the scope
      // of a failure holds at once
      const auto * claimer = command_claimer(actual, interface);
      const bool failed = claimer && claimer->second.failed;
      // A detached claimer whose stop was given up keeps the joint until it returns
      const bool hung = claimer && claimer->second.state == Lifecycle::Active &&
                        claimer->second.quarantined && !claimer->second.running;
      const bool deferred = (hw.taken && !hung) || (takes && !failed);
      if (!deferred && !keeps_running(actual, desired, interface))
      {
        hold.interfaces[name].push_back(interface);
      }
    }
  }
  return hold.interfaces.empty() ? std::nullopt : std::optional(hold);
}

std::optional<step::Take> take_step(
  const Model & actual, const Desired & desired, const Desired & after)
{
  step::Take take;
  for (const auto & [name, hw] : actual.hardware)
  {
    if (may_take(actual, desired, after, hw))
    {
      take.hardware.insert(name);
    }
  }
  return take.hardware.empty() ? std::nullopt : std::optional(take);
}

std::optional<step::Hardware> hardware_down(
  const Model & actual, const Desired & desired, const InterfaceIndex & index)
{
  for (const auto & [name, hw] : actual.hardware)
  {
    if (hw.quarantined || hw.state == Lifecycle::Finalized)
    {
      continue;
    }
    const Lifecycle target = desired_hardware_state(desired, name, hw.state);
    // Readers may stay on an inactive component, the planner starts them there as well
    const bool deactivates = hw.fault == HardwareFault::None && hw.state == Lifecycle::Active &&
                             target == Lifecycle::Inactive;
    const Names holding = claimers_holding(actual, index, name);
    const bool blocked =
      deactivates
        ? std::any_of(
            holding.begin(), holding.end(), [&](const std::string & claimer)
            { return commands_hardware(index, actual.controllers.at(claimer).claims, name); })
        : !holding.empty();
    if (blocked)
    {
      continue;
    }
    if (hw.fault != HardwareFault::None)
    {
      const bool deactivate =
        hw.fault == HardwareFault::Deactivate && hw.state == Lifecycle::Active;
      return step::Hardware{
        name, deactivate ? step::HardwareTransition::Deactivate : step::HardwareTransition::Error};
    }
    if (target == Lifecycle::Finalized)
    {
      return step::Hardware{
        name, hw.state == Lifecycle::Active ? step::HardwareTransition::Deactivate
                                            : step::HardwareTransition::Shutdown};
    }
    if (target == Lifecycle::Inactive && hw.state == Lifecycle::Active)
    {
      return step::Hardware{name, step::HardwareTransition::Deactivate};
    }
    if (target == Lifecycle::Unconfigured && hw.state != Lifecycle::Unconfigured)
    {
      return step::Hardware{
        name, hw.state == Lifecycle::Active ? step::HardwareTransition::Deactivate
                                            : step::HardwareTransition::Cleanup};
    }
  }
  return std::nullopt;
}

std::optional<step::Hardware> hardware_up(const Model & actual, const Desired & desired)
{
  for (const auto & [name, hw] : actual.hardware)
  {
    if (hw.quarantined || hw.fault != HardwareFault::None || hw.state == Lifecycle::Finalized)
    {
      continue;
    }
    const Lifecycle target = desired_hardware_state(desired, name, hw.state);
    if (target == Lifecycle::Finalized || target == hw.state)
    {
      continue;
    }
    if (hw.state == Lifecycle::Unconfigured)
    {
      return step::Hardware{name, step::HardwareTransition::Configure};
    }
    if (hw.state == Lifecycle::Inactive && target == Lifecycle::Active)
    {
      return step::Hardware{name, step::HardwareTransition::Activate};
    }
  }
  return std::nullopt;
}

bool may_activate(
  const Model & actual, const Desired & desired, const InterfaceIndex & index,
  const std::string & name, const DesiredController & want)
{
  bool ready = true;
  for_each_claim(
    want.claims, index,
    [&](const std::string & interface, const InterfaceOwner & owner)
    {
      if (is_provider_kind(owner.kind))
      {
        ready = ready && wanted_and_activated(
                           actual.controllers.at(owner.owner), wanted(desired, owner.owner));
      }
      else
      {
        const auto & hw = actual.hardware.at(owner.owner);
        ready = ready && (owner.kind == Kind::HardwareState ? readable(hw) : commandable(hw));
      }
      if (owner.kind != Kind::ExportedState && owner.kind != Kind::HardwareState)
      {
        const auto * other = command_claimer(actual, interface);
        ready = ready && (!other || other->first == name);
      }
    });
  return ready;
}

/// What desired still misses. Quarantined entities count only if `include_quarantined`, together
/// with a taken component that waits for its quarantined claimer.
std::string unmet(const Model & actual, const Desired & desired, bool include_quarantined)
{
  Names missing;
  for (const auto & [name, want] : desired.active)
  {
    const auto it = actual.controllers.find(name);
    if (
      it == actual.controllers.end() || !wanted_and_activated(it->second, &want) ||
      !it->second.running || !it->second.updated)
    {
      missing.insert(name);
    }
  }
  for (const auto & [name, controller] : actual.controllers)
  {
    if (
      controller.quarantined ? include_quarantined
                             : controller.state == Lifecycle::Active && !desired.active.count(name))
    {
      missing.insert(name);
    }
  }
  for (const auto & [name, hw] : actual.hardware)
  {
    if (!include_quarantined && (hw.quarantined || (hw.taken && hung_claimer(actual, hw))))
    {
      continue;
    }
    const Lifecycle target = desired_hardware_state(desired, name, hw.state);
    const bool settled =
      hw.state == Lifecycle::Finalized ||
      (hw.state == target && hw.fault == HardwareFault::None && !hw.quarantined && !hw.taken);
    if (!settled)
    {
      missing.insert(name);
    }
  }
  return missing.empty() ? std::string() : text(missing);
}

/// An active provider in the wrong chained mode only changes it through a restart, with the same
/// restart scope as a planned one. The restart removes its scope from desired for this step only,
/// its successor is the requested activation.
Desired without_chained_mode_restarts(
  const Model & actual, const Desired & requested, const InterfaceIndex & index)
{
  Names mismatched;
  for (const auto & [name, controller] : actual.controllers)
  {
    const bool chained = requested.active.count(name) && desired_chained(requested, index, name);
    if (
      controller.chainable && controller.state == Lifecycle::Active &&
      controller.chained != chained)
    {
      mismatched.insert(name);
    }
  }
  Desired desired = requested;
  if (!mismatched.empty())
  {
    for (const auto & name : stop_closure(ChainGraph(index, claims_of(requested)), mismatched))
    {
      const auto it = actual.controllers.find(name);
      if (it != actual.controllers.end() && it->second.state == Lifecycle::Active)
      {
        desired.active.erase(name);
      }
    }
  }
  return desired;
}

/// Starts the interfaces of updated claimers and gives taken components back.
std::optional<step::SwitchModes> switch_modes_step(
  const Model & actual, const Desired & desired, const InterfaceIndex & index)
{
  step::SwitchModes switch_modes;
  const auto updated_claimer = [&](const std::string & interface)
  {
    const auto * claimer = command_claimer(actual, interface);
    return claimer && claimer->second.running && claimer->second.updated &&
           wanted_and_activated(claimer->second, wanted(desired, claimer->first));
  };
  for (const auto & [name, controller] : actual.controllers)
  {
    if (
      !controller.running || !controller.updated ||
      !wanted_and_activated(controller, wanted(desired, name)))
    {
      continue;
    }
    for (const auto & interface : controller.claims.command)
    {
      if (const auto * owner = index.hardware_command(interface))
      {
        const auto & hw = actual.hardware.at(owner->owner);
        if (!hw.live.count(interface) && hw.fault == HardwareFault::None && !hw.quarantined)
        {
          switch_modes.modes[owner->owner].start.push_back(interface);
        }
      }
    }
  }
  for (const auto & [name, hw] : actual.hardware)
  {
    if (!hw.taken || hw.quarantined || hw.fault != HardwareFault::None)
    {
      continue;
    }
    if (hung_claimer(actual, hw))
    {
      switch_modes.modes.erase(name);
      continue;
    }
    // Given back even without a mode change
    auto & modes = switch_modes.modes[name];
    for (const auto & interface : hw.live)
    {
      if (!updated_claimer(interface))
      {
        modes.stop.push_back(interface);
      }
    }
  }
  if (switch_modes.modes.empty())
  {
    return std::nullopt;
  }
  return switch_modes;
}

const char * to_string(step::HardwareTransition transition)
{
  switch (transition)
  {
    case step::HardwareTransition::Configure:
      return "configure";
    case step::HardwareTransition::Activate:
      return "activate";
    case step::HardwareTransition::Deactivate:
      return "deactivate";
    case step::HardwareTransition::Cleanup:
      return "clean up";
    case step::HardwareTransition::Error:
      return "on_error";
    case step::HardwareTransition::Shutdown:
      return "shut down";
  }
  return "unknown";
}

}  // namespace

bool reached_apart_from_quarantine(const Model & actual, const Desired & desired)
{
  const Step step = next_step(actual, desired);
  return std::holds_alternative<Done>(step) ||
         (std::holds_alternative<Blocked>(step) && unmet(actual, desired, false).empty());
}

std::string describe(const Step & step)
{
  const auto interfaces = [](const InterfacesByHardware & by_hardware)
  {
    std::string result;
    for (const auto & [name, list] : by_hardware)
    {
      result += " " + name + "[";
      for (const auto & interface : list)
      {
        result += (result.back() == '[' ? "" : " ") + interface;
      }
      result += "]";
    }
    return result;
  };
  return std::visit(
    [&](const auto & executed) -> std::string
    {
      using T = std::decay_t<decltype(executed)>;
      if constexpr (std::is_same_v<T, Done>)
      {
        return "done";
      }
      else if constexpr (std::is_same_v<T, Blocked>)
      {
        return "blocked: " + executed.reason;
      }
      else if constexpr (std::is_same_v<T, step::Hold>)
      {
        return "hold" + interfaces(executed.interfaces);
      }
      else if constexpr (std::is_same_v<T, step::Take>)
      {
        return "take " + text(executed.hardware);
      }
      else if constexpr (std::is_same_v<T, step::SwitchModes>)
      {
        InterfacesByHardware start;
        InterfacesByHardware stop;
        for (const auto & [name, modes] : executed.modes)
        {
          start[name] = modes.start;
          stop[name] = modes.stop;
        }
        return "switch modes, start" + interfaces(start) + ", stop" + interfaces(stop);
      }
      else if constexpr (std::is_same_v<T, step::Detach>)
      {
        return "detach " + text(executed.controllers);
      }
      else if constexpr (std::is_same_v<T, step::Attach>)
      {
        return "attach " + text(executed.controllers);
      }
      else if constexpr (std::is_same_v<T, step::AwaitUpdate>)
      {
        return "await update " + text(executed.controllers);
      }
      else if constexpr (std::is_same_v<T, step::Deactivate>)
      {
        return "deactivate " + executed.controller;
      }
      else if constexpr (std::is_same_v<T, step::Activate>)
      {
        return "activate " + executed.controller;
      }
      else if constexpr (std::is_same_v<T, step::SetChained>)
      {
        return "set chained mode of " + executed.controller + (executed.chained ? " on" : " off");
      }
      else
      {
        return std::string(to_string(executed.transition)) + " hardware " + executed.hardware;
      }
    },
    step);
}

Step next_step(const Model & actual, const Desired & requested)
{
  const InterfaceIndex index(actual);
  const Desired desired = without_chained_mode_restarts(actual, requested, index);
  if (auto hold = hold_step(actual, desired, requested))
  {
    return *hold;
  }
  if (auto take = take_step(actual, desired, requested))
  {
    return *take;
  }
  step::Detach detach;
  for (const auto & [name, controller] : actual.controllers)
  {
    if (
      controller.running && (controller.quarantined || stopping(controller, wanted(desired, name))))
    {
      detach.controllers.insert(name);
    }
  }
  if (!detach.controllers.empty())
  {
    return detach;
  }
  for (const auto & [name, controller] : actual.controllers)
  {
    if (
      stopping(controller, wanted(desired, name)) &&
      without(users_holding(actual, index, name), {name}).empty())
    {
      return step::Deactivate{name};
    }
  }
  if (auto down = hardware_down(actual, desired, index))
  {
    return *down;
  }
  // Only a controller about to start gets its chained mode, so a refusal shrinks desired
  for (const auto & [name, want] : requested.active)
  {
    const auto it = actual.controllers.find(name);
    const bool chained = desired_chained(requested, index, name);
    if (
      it != actual.controllers.end() && it->second.chainable &&
      it->second.state == Lifecycle::Inactive && !it->second.quarantined &&
      it->second.chained != chained)
    {
      return step::SetChained{name, chained};
    }
  }
  if (auto up = hardware_up(actual, desired))
  {
    return *up;
  }
  for (const auto & [name, want] : requested.active)
  {
    const auto it = actual.controllers.find(name);
    if (
      it != actual.controllers.end() && it->second.state == Lifecycle::Inactive &&
      !it->second.quarantined && may_activate(actual, desired, index, name, want))
    {
      return step::Activate{name, want.claims};
    }
  }
  step::Attach attach;
  step::AwaitUpdate await_update;
  for (const auto & [name, controller] : actual.controllers)
  {
    if (wanted_and_activated(controller, wanted(desired, name)))
    {
      if (!controller.running)
      {
        attach.controllers.insert(name);
      }
      else if (!controller.updated)
      {
        await_update.controllers.insert(name);
      }
    }
  }
  if (!attach.controllers.empty())
  {
    return attach;
  }
  if (!await_update.controllers.empty())
  {
    return await_update;
  }
  if (auto switch_modes = switch_modes_step(actual, desired, index))
  {
    return *switch_modes;
  }
  const std::string missing = unmet(actual, requested, true);
  if (missing.empty())
  {
    return Done{};
  }
  return Blocked{"Not reached: " + missing};
}

}  // namespace hector_controller_manager::core
