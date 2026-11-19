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
bool activated(const Controller & controller, const DesiredController * want)
{
  return want && is_running_member(controller) && controller.activation >= want->min_activation;
}

bool stopping(const Controller & controller, const DesiredController * want)
{
  return controller.state == Lifecycle::Active && !controller.quarantined &&
         !activated(controller, want);
}

/// The plugin holds interfaces, active or quarantined in on_activate with interfaces assigned.
bool holds_claims(const Controller & controller)
{
  return controller.state == Lifecycle::Active || controller.quarantined;
}

bool is_provider_kind(Kind kind) { return kind == Kind::Reference || kind == Kind::ExportedState; }

bool is_hardware_kind(Kind kind)
{
  return kind == Kind::HardwareCommand || kind == Kind::HardwareState;
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

Names claimers_holding(const Model & model, const InterfaceIndex & index, const std::string & hw)
{
  Names claimers;
  for (const auto & name : claimers_of_hardware(model, index, hw))
  {
    if (holds_claims(model.controllers.at(name)))
    {
      claimers.insert(name);
    }
  }
  return claimers;
}

using ControllerEntry = std::pair<const std::string, Controller>;

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

/// A claimer that is detached but not yet stopped may still write its handles, from an update RT is
/// stuck in or from its own on_deactivate, also of an interface held meanwhile.
bool claimer_may_write(const Model & actual, const Hardware & hw)
{
  return std::any_of(
    hw.command_interfaces.begin(), hw.command_interfaces.end(),
    [&](const CommandInterface & interface)
    {
      const auto * claimer = command_claimer(actual, interface.name);
      return claimer && claimer->second.state == Lifecycle::Active && claimer->second.quarantined;
    });
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
    if (
      hw.quarantined || hw.state == Lifecycle::Finalized ||
      !claimers_holding(actual, index, name).empty())
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
    const Lifecycle target = desired_hardware_state(desired, name, hw.state);
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
        ready =
          ready && activated(actual.controllers.at(owner.owner), wanted(desired, owner.owner));
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
      it == actual.controllers.end() || !activated(it->second, &want) || !it->second.running ||
      !it->second.updated)
    {
      missing.insert(name);
    }
  }
  for (const auto & [name, controller] : actual.controllers)
  {
    if (controller.quarantined && !include_quarantined)
    {
      continue;
    }
    if (
      controller.quarantined ||
      (controller.state == Lifecycle::Active && !desired.active.count(name)))
    {
      missing.insert(name);
    }
  }
  for (const auto & [name, hw] : actual.hardware)
  {
    if (!include_quarantined && (hw.quarantined || (hw.taken && claimer_may_write(actual, hw))))
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
           activated(claimer->second, wanted(desired, claimer->first));
  };
  for (const auto & [name, controller] : actual.controllers)
  {
    if (!controller.running || !controller.updated || !activated(controller, wanted(desired, name)))
    {
      continue;
    }
    for (const auto & interface : controller.claims.command)
    {
      const auto * owner = index.command(interface);
      if (owner && owner->kind == Kind::HardwareCommand)
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
    if (claimer_may_write(actual, hw))
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
        static const char * NAMES[] = {"configure", "activate", "deactivate",
                                       "clean up",  "on_error", "shut down"};
        return std::string(NAMES[static_cast<int>(executed.transition)]) + " hardware " +
               executed.hardware;
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
    if (activated(controller, wanted(desired, name)))
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

namespace
{
void apply_actual(Model & actual, const Step & step, const Outcome & outcome)
{
  const auto returned = [&] { return outcome.result != CallResult::TimedOut; };
  std::visit(
    [&](const auto & executed)
    {
      using T = std::decay_t<decltype(executed)>;
      const auto faults = [&](const std::string & name, Hardware & hw)
      {
        if (outcome.faulted_hardware.count(name))
        {
          hw.fault = HardwareFault::Error;
        }
        if (outcome.timed_out_hardware.count(name))
        {
          ++hw.quarantined;
        }
      };
      if constexpr (std::is_same_v<T, step::Hold>)
      {
        for (const auto & [name, interfaces] : executed.interfaces)
        {
          auto & hw = actual.hardware.at(name);
          faults(name, hw);
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
          auto & hw = actual.hardware.at(name);
          faults(name, hw);
          hw.taken = true;
        }
      }
      else if constexpr (std::is_same_v<T, step::SwitchModes>)
      {
        for (const auto & [name, modes] : executed.modes)
        {
          auto & hw = actual.hardware.at(name);
          faults(name, hw);
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
      else if constexpr (std::is_same_v<T, returned::ModeSwitch>)
      {
        release(actual.hardware.at(event.hardware).quarantined);
      }
      else if constexpr (std::is_same_v<T, returned::ChainedMode>)
      {
        auto & controller = actual.controllers.at(event.controller);
        release(controller.quarantined);
        controller.chained = event.chained.value_or(controller.chained);
      }
      else if constexpr (std::is_same_v<T, returned::Detached>)
      {
        // An unloaded controller is gone with its quarantine
        for (const auto & name : event.controllers)
        {
          if (const auto it = actual.controllers.find(name); it != actual.controllers.end())
          {
            release(it->second.quarantined);
          }
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
    // A quarantined or taken component writes nothing, its owner is in a call or its writes are
    // blocked
    for (const auto & interface : hw.quarantined || hw.taken ? Names{} : hw.live)
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
