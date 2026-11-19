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

#include "hector_controller_manager/core/planner.hpp"

#include <algorithm>
#include <optional>
#include <stdexcept>
#include <utility>

#include "hector_controller_manager/core/chain_rules.hpp"
#include "hector_controller_manager/core/names.hpp"

namespace hector_controller_manager::core
{
namespace
{
using Kind = InterfaceOwner::Kind;

/// Controllers whose references or strictly requested exported states the controller needs.
Names needed_providers(const InterfaceIndex & index, const Controller & controller)
{
  Names providers = providers_written(index, {controller.interfaces.command, {}});
  if (!controller.interfaces.best_effort_states)
  {
    for (const auto & interface : controller.interfaces.state)
    {
      if (const auto * owner = index.state(interface); owner && owner->kind == Kind::ExportedState)
      {
        providers.insert(owner->owner);
      }
    }
  }
  return providers;
}

std::optional<Rejection> check_requested(const Model & model, const SwitchRequest & request)
{
  for (const auto & name : merged(request.activate, request.deactivate))
  {
    const auto it = model.controllers.find(name);
    if (it == model.controllers.end())
    {
      return Rejection{"Controller '" + name + "' is not loaded"};
    }
    if (it->second.quarantined)
    {
      return Rejection{"Controller '" + name + "' is quarantined, a callback of it still runs"};
    }
  }
  for (const auto & name : request.activate)
  {
    const auto & controller = model.controllers.at(name);
    if (!is_running_member(controller) && controller.state != Lifecycle::Inactive)
    {
      return Rejection{
        "Controller '" + name + "' cannot be activated, it is " +
        (controller.failed ? std::string("stopping after a failure")
                           : to_string(controller.state))};
    }
  }
  return std::nullopt;
}

/// For AUTO and FORCE_AUTO the inactive providers the started controllers need, transitively.
std::optional<Rejection> add_providers(
  const Model & model, const InterfaceIndex & index, const Names & active, const Names & deactivate,
  Names & start)
{
  std::vector<std::string> pending(start.begin(), start.end());
  while (!pending.empty())
  {
    const std::string name = pending.back();
    pending.pop_back();
    for (const auto & provider : needed_providers(index, model.controllers.at(name)))
    {
      if (deactivate.count(provider))
      {
        return Rejection{
          "'" + name + "' needs '" + provider + "', which is requested to be deactivated"};
      }
      if (active.count(provider))
      {
        continue;
      }
      const auto & candidate = model.controllers.at(provider);
      if (candidate.state != Lifecycle::Inactive || candidate.quarantined)
      {
        return Rejection{
          "'" + name + "' needs '" + provider + "', which is " + to_string(candidate.state)};
      }
      if (start.insert(provider).second)
      {
        pending.push_back(provider);
      }
    }
  }
  return std::nullopt;
}

/// For FORCE_AUTO the active controllers claiming a command interface or another mode of a joint
/// that the started controllers need.
Names conflicts(
  const Model & model, const InterfaceIndex & index, const Names & active, const Names & start,
  const Names & requested)
{
  Names needed;
  Names joints;
  for (const auto & name : start)
  {
    for (const auto & interface : model.controllers.at(name).interfaces.command)
    {
      needed.insert(interface);
      if (const auto * owner = index.hardware_command(interface))
      {
        joints.insert(joint_key(*owner));
      }
    }
  }
  Names result;
  for (const auto & name : without(active, merged(start, requested)))
  {
    for (const auto & interface : model.controllers.at(name).claims.command)
    {
      const auto * owner = index.hardware_command(interface);
      const bool same_joint = owner && joints.count(joint_key(*owner));
      if (needed.count(interface) || same_joint)
      {
        result.insert(name);
      }
    }
  }
  return result;
}

/// Every command interface has one claimer, quarantined controllers keep theirs, and the command
/// interfaces of one joint belong to one controller, because a joint runs one mode at a time.
std::optional<Rejection> check_exclusive(
  const Model & model, const InterfaceIndex & index, const std::map<std::string, Claims> & claims)
{
  std::map<std::string, std::string> claimer;
  std::map<std::string, std::string> joint_claimer;
  const auto claim = [&](
                       const std::string & name, const Claims & claimed,
                       bool quarantined) -> std::optional<Rejection>
  {
    const std::string label = quarantined ? name + "' (quarantined)" : name + "'";
    for (const auto & interface : claimed.command)
    {
      const auto [it, inserted] = claimer.emplace(interface, label);
      if (!inserted)
      {
        return Rejection{"'" + interface + "' is claimed by '" + it->second + " and '" + label};
      }
      if (const auto * owner = index.hardware_command(interface))
      {
        const auto [joint, first] = joint_claimer.emplace(joint_key(*owner), label);
        if (!first && joint->second != label)
        {
          return Rejection{
            "'" + joint->second + " and '" + label + " command joint '" + owner->joint +
            "' in different modes"};
        }
      }
    }
    return std::nullopt;
  };
  for (const auto & [name, controller] : model.controllers)
  {
    if (controller.quarantined)
    {
      if (auto rejection = claim(name, controller.claims, true))
      {
        return rejection;
      }
    }
  }
  for (const auto & [name, claimed] : claims)
  {
    if (auto rejection = claim(name, claimed, false))
    {
      return rejection;
    }
  }
  return std::nullopt;
}

std::map<std::string, Claims> actual_claims(const Model & model, const Names & names)
{
  std::map<std::string, Claims> claims;
  for (const auto & name : names)
  {
    claims.emplace(name, model.controllers.at(name).claims);
  }
  return claims;
}

/// Adds to `leaving` the users of a leaving controller, which FORCE_AUTO stops with it and which
/// reject the switch otherwise. A best effort reader of a leaving provider stays and goes to
/// `reresolved`, it restarts without that state.
std::optional<Rejection> add_users(
  const Model & model, const ChainGraph & current, const SwitchRequest & request, Names & leaving,
  Names & reresolved)
{
  while (true)
  {
    Names in_way;
    for (const auto & name : leaving)
    {
      for (const auto & user : without(current.users_of(name), leaving))
      {
        const bool only_reads = !current.written_by(user).count(name);
        const bool best_effort = model.controllers.at(user).interfaces.best_effort_states;
        (only_reads && best_effort ? reresolved : in_way).insert(user);
      }
    }
    if (in_way.empty())
    {
      return std::nullopt;
    }
    if (request.strictness != Strictness::ForceAuto)
    {
      return Rejection{
        "Deactivating " + text(leaving) + " requires deactivating " + text(in_way) +
        ", which uses them"};
    }
    if (const Names requested = intersection(in_way, request.activate); !requested.empty())
    {
      return Rejection{
        "Deactivating " + text(leaving) + " requires deactivating " + text(requested) +
        ", which the request activates"};
    }
    leaving.merge(in_way);
  }
}

/// The running controllers that stay in `target` but restart, the `seeds` and each chained one
/// whose writer departs, with their restart scope.
Names departed_writer_restarts(
  const ChainGraph & current, const ChainGraph & next, const Names & running, const Names & target,
  Names seeds)
{
  const Names stay = intersection(running, target);
  for (const auto & name : stay)
  {
    if (!next.writers_of(name).empty() && !is_subset(current.writers_of(name), target))
    {
      seeds.insert(name);
    }
  }
  return intersection(stop_closure(next, seeds), stay);
}

}  // namespace

std::variant<Claims, Rejection> resolve_claims(
  const Model & model, const InterfaceIndex & index, const std::string & name, const Names & active)
{
  const auto & controller = model.controllers.at(name);
  Claims claims;
  for (const auto & interface : controller.interfaces.command)
  {
    const auto * owner = index.command(interface);
    if (!owner)
    {
      return Rejection{"'" + name + "' claims the unknown command interface '" + interface + "'"};
    }
    if (owner->kind == Kind::HardwareCommand)
    {
      const auto & hw = model.hardware.at(owner->owner);
      if (!commandable(hw))
      {
        return Rejection{
          "'" + name + "' claims '" + interface + "' of hardware '" + owner->owner +
          "', which is not active or has a fault"};
      }
      if (const auto hung = hung_claimer(model, hw); hung && hw.taken)
      {
        return Rejection{
          "'" + name + "' claims '" + interface + "' of hardware '" + owner->owner +
          "', which waits for the quarantined controller '" + *hung + "'"};
      }
    }
    else if (owner->kind != Kind::Reference || !active.count(owner->owner))
    {
      return Rejection{
        "'" + name + "' claims '" + interface + "', but its provider '" + owner->owner +
        "' does not stay active"};
    }
    claims.command.push_back(interface);
  }
  for (const auto & interface : controller.interfaces.state)
  {
    const auto * owner = index.state(interface);
    const bool available =
      owner && ((owner->kind == Kind::HardwareState && readable(model.hardware.at(owner->owner))) ||
                (owner->kind == Kind::ExportedState && active.count(owner->owner)));
    if (available)
    {
      claims.state.push_back(interface);
    }
    else if (!controller.interfaces.best_effort_states)
    {
      return Rejection{"'" + name + "' claims the unavailable state interface '" + interface + "'"};
    }
  }
  return claims;
}

std::variant<Names, Rejection> auto_start_set(const Model & model, const std::string & controller)
{
  SwitchRequest request;
  request.activate = {controller};
  if (auto rejection = check_requested(model, request))
  {
    return *rejection;
  }
  const InterfaceIndex index(model);
  Names start{controller};
  if (auto rejection = add_providers(model, index, running_members(model), {}, start))
  {
    return *rejection;
  }
  return start;
}

StartupSelection select_startup(
  const Model & model, const std::vector<std::string> & wanted, bool restore,
  const std::optional<std::string> & estop_cause)
{
  const InterfaceIndex index(model);
  // A hardware interface conflicts per joint, a reference only with itself
  const auto key = [&](const std::string & interface)
  {
    const auto * owner = index.hardware_command(interface);
    return owner ? joint_key(*owner) : interface;
  };
  const Names running = running_members(model);
  Names held;
  for (const auto & [name, controller] : model.controllers)
  {
    if (controller.state == Lifecycle::Active)
    {
      for (const auto & interface : controller.claims.command)
      {
        held.insert(key(interface));
      }
    }
  }
  const auto skip_reason = [&](const std::string & name) -> std::optional<std::string>
  {
    const auto & controller = model.controllers.at(name);
    const auto & command = controller.interfaces.command;
    if (restore && controller.is_protected)
    {
      return "protected";
    }
    if (estop_cause && estop_stops(controller))
    {
      return "e-stop engaged by " + *estop_cause;
    }
    if (
      std::any_of(
        command.begin(), command.end(),
        [&](const std::string & interface) { return held.count(key(interface)); }))
    {
      return "a running controller claims its joint or reference";
    }
    return std::nullopt;
  };
  StartupSelection selection;
  for (const auto & name : wanted)
  {
    const auto it = model.controllers.find(name);
    if (it == model.controllers.end())
    {
      selection.activate.insert(name);
      continue;
    }
    // A quarantined one may still hang in its stop, it is skipped with the reason
    if (it->second.state == Lifecycle::Active && !it->second.quarantined)
    {
      continue;
    }
    if (const auto reason = skip_reason(name))
    {
      selection.skipped.push_back("'" + name + "' (" + *reason + ")");
      continue;
    }
    // AUTO also starts the providers it needs, so one skipped provider skips its users
    const auto start = auto_start_set(model, name);
    if (const auto * rejection = std::get_if<Rejection>(&start))
    {
      selection.skipped.push_back("'" + name + "' (" + rejection->reason + ")");
      continue;
    }
    const auto & members = std::get<Names>(start);
    std::string needs;
    for (const auto & provider : without(members, {name}))
    {
      if (const auto reason = skip_reason(provider); reason && needs.empty())
      {
        needs = "it needs '" + provider + "', which is skipped: " + *reason;
      }
    }
    // The claim check of the switch, so hardware that is still down does not reject the whole set
    for (const auto & member : members)
    {
      const auto claims = resolve_claims(model, index, member, merged(running, members));
      if (const auto * rejection = std::get_if<Rejection>(&claims); rejection && needs.empty())
      {
        needs = rejection->reason;
      }
    }
    if (needs.empty())
    {
      selection.activate.insert(name);
    }
    else
    {
      selection.skipped.push_back("'" + name + "' (" + needs + ")");
    }
  }
  return selection;
}

void restart_for_departed_writers(const Model & model, const Desired & before, Desired & desired)
{
  const InterfaceIndex index(model);
  const Names running = running_members(model);
  const auto wanted = claims_of(desired);
  const Names target = names_of(desired);
  std::map<std::string, Claims> writers;
  for (const auto & [name, controller] : model.controllers)
  {
    if (controller.state == Lifecycle::Active)
    {
      writers.emplace(name, controller.claims);
    }
  }
  for (const auto & [name, want] : before.active)
  {
    writers.emplace(name, want.claims);
  }
  const ChainGraph current(index, writers);
  const ChainGraph next(index, wanted);
  for (const auto & name : departed_writer_restarts(current, next, running, target, {}))
  {
    auto & want = desired.active.at(name);
    want.min_activation = std::max(want.min_activation, model.controllers.at(name).activation + 1);
  }
}

Names resolve_starting(const Model & model, Desired & desired)
{
  const InterfaceIndex index(model);
  const Names target = names_of(desired);
  Names unresolved;
  for (auto & [name, want] : desired.active)
  {
    const auto & controller = model.controllers.at(name);
    if (activated(controller, want.min_activation))
    {
      continue;
    }
    auto resolved = resolve_claims(model, index, name, target);
    if (std::holds_alternative<Rejection>(resolved))
    {
      unresolved.insert(name);
      continue;
    }
    want.claims = std::get<Claims>(std::move(resolved));
  }
  return unresolved;
}

std::variant<SwitchPlan, Rejection> plan_switch(
  const Model & model, const std::map<std::string, Lifecycle> & hardware,
  const SwitchRequest & request)
{
  if (auto rejection = check_requested(model, request))
  {
    return *rejection;
  }
  const InterfaceIndex index(model);
  const Names active = running_members(model);
  const Names restart_requested =
    intersection(intersection(request.activate, request.deactivate), active);
  Names leaving = without(intersection(request.deactivate, active), request.activate);
  Names start = without(request.activate, active);

  if (request.strictness != Strictness::Strict)
  {
    if (auto rejection = add_providers(model, index, active, request.deactivate, start))
    {
      return *rejection;
    }
  }
  if (request.strictness == Strictness::ForceAuto)
  {
    leaving.merge(conflicts(model, index, active, start, request.activate));
  }

  const ChainGraph current(index, actual_claims(model, active));
  Names reresolved;
  if (auto rejection = add_users(model, current, request, leaving, reresolved))
  {
    return *rejection;
  }

  for (const auto & [name, controller] : model.controllers)
  {
    const Names touched = intersection(providers_written(index, controller.claims), start);
    if (controller.quarantined && !touched.empty())
    {
      return Rejection{
        "Quarantined '" + name + "' may still write the references of " + text(touched) +
        ", they cannot start until it returns"};
    }
  }
  const Names target = merged(without(active, leaving), start);
  std::map<std::string, Claims> claims = actual_claims(model, without(target, start));
  reresolved = intersection(reresolved, target);
  for (const auto & name : merged(start, merged(restart_requested, reresolved)))
  {
    auto resolved = resolve_claims(model, index, name, target);
    if (auto * rejection = std::get_if<Rejection>(&resolved))
    {
      return *rejection;
    }
    claims[name] = std::get<Claims>(std::move(resolved));
  }

  if (auto rejection = check_exclusive(model, index, claims))
  {
    return *rejection;
  }
  const ChainGraph next(index, claims);
  try
  {
    next.check_acyclic();
  }
  catch (const std::invalid_argument & e)
  {
    return Rejection{e.what()};
  }

  Names seeds = merged(restart_requested, reresolved);
  for (const auto & name : intersection(active, target))
  {
    const auto & controller = model.controllers.at(name);
    // A chained mode changes only through a restart
    if (controller.chainable && next.writers_of(name).empty() == controller.chained)
    {
      seeds.insert(name);
    }
  }
  const Names restarted = departed_writer_restarts(current, next, active, target, seeds);

  SwitchPlan plan;
  plan.stopped = merged(without(active, target), restarted);
  for (const auto & name : without(plan.stopped, request.deactivate))
  {
    if (model.controllers.at(name).is_protected)
    {
      return Rejection{
        "The switch would stop protected '" + name + "', deactivate it explicitly to stop it"};
    }
  }
  plan.started = merged(start, restarted);
  for (const auto & name : target)
  {
    const auto & controller = model.controllers.at(name);
    const uint64_t activation = controller.activation + (plan.started.count(name) ? 1 : 0);
    plan.desired.active.emplace(name, DesiredController{activation, claims.at(name)});
  }
  plan.desired.hardware = hardware;
  const auto add_hardware_interfaces = [&](const Claims & claim, auto member)
  {
    for (const auto & interface : claim.command)
    {
      if (const auto * owner = index.hardware_command(interface))
      {
        (plan.validation[owner->owner].*member).push_back(interface);
      }
    }
  };
  for (const auto & name : plan.started)
  {
    add_hardware_interfaces(claims.at(name), &ModeSwitch::start);
  }
  for (const auto & name : plan.stopped)
  {
    add_hardware_interfaces(model.controllers.at(name).claims, &ModeSwitch::stop);
  }
  return plan;
}

}  // namespace hector_controller_manager::core
