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

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iterator>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "fixtures.hpp"
#include "hector_controller_manager/core/chain_rules.hpp"
#include "hector_controller_manager/core/names.hpp"
#include "hector_controller_manager/core/planner.hpp"
#include "hector_controller_manager/core/reconciler.hpp"

/// The small scope explorer covers topologies of up to four controllers on two hardware components,
/// requests in every strictness and a single failure of any step, checked against a brute-force
/// oracle and the invariants after every step. It injects hardware faults only as Error on active
/// hardware and never fails a hardware transition, so fault handling below its target is covered by
/// the reconciler's unit tests instead.
namespace hector_controller_manager::core
{
namespace
{
/// Hardware command claim of one controller, none, two modes of joint a on h0, or joint b or c of
/// h1.
const char * const HARDWARE_CLAIMS[] = {"", "a/position", "a/velocity", "b/position", "c/position"};
/// Choices of the exhaustive run, one joint per component, so every claimer of a component hands
/// over together.
const std::vector<int> ONE_JOINT_CHOICES = {0, 1, 2, 3};
/// Two claimers on two joints of one component, so one may keep running next to a handover.
const std::vector<int> TWO_JOINT_CHOICES = {0, 3, 4};

enum Edge : int
{
  NONE = 0,
  WRITE = 1,
  READ = 2,
};

struct Shape
{
  int hardware = 0;
  /// Edge to every controller with a lower index.
  std::vector<Edge> edges;
  bool best_effort = false;
};

using Topology = std::vector<Shape>;
/// A controller's activation, by its name and activation count.
using Activation = std::pair<std::string, uint64_t>;
using Activations = std::set<Activation>;

std::string name_of(size_t index) { return "c" + std::to_string(index); }

Model build(const Topology & topology)
{
  Model model;
  testing::add_hardware(model, "h0", {"a"}, {"position", "velocity"}, {"a/position"});
  testing::add_hardware(model, "h1", {"b", "c"}, {"position"}, {"b/position"});
  for (size_t i = 0; i < topology.size(); ++i)
  {
    const auto & shape = topology[i];
    testing::ControllerSpec spec{name_of(i), true, {}, {}, {"r"}, {"s"}, shape.best_effort};
    if (shape.hardware != 0)
    {
      spec.command.push_back(HARDWARE_CLAIMS[shape.hardware]);
    }
    for (size_t j = 0; j < shape.edges.size(); ++j)
    {
      if (shape.edges[j] == WRITE)
      {
        spec.command.push_back(name_of(j) + "/r");
      }
      if (shape.edges[j] == READ)
      {
        spec.state.push_back(name_of(j) + "/s");
      }
    }
    if (shape.best_effort)
    {
      spec.state.push_back("b/position");
      spec.state.push_back("b/missing");
    }
    testing::add_controller(model, spec);
  }
  return model;
}

/// Calls visit for every topology of the given size.
void for_each_topology(
  size_t size, const std::vector<int> & choices,
  const std::function<void(const Topology &)> & visit)
{
  Topology topology(size);
  const std::function<void(size_t)> place = [&](size_t index)
  {
    if (index == size)
    {
      visit(topology);
      return;
    }
    auto & shape = topology[index];
    shape.edges.assign(index, NONE);
    size_t edge_combinations = 1;
    for (size_t j = 0; j < index; ++j)
    {
      edge_combinations *= 3;
    }
    for (const int hardware : choices)
    {
      for (size_t edges = 0; edges < edge_combinations; ++edges)
      {
        for (const bool best_effort : {false, true})
        {
          // Best effort reads are one shape of the first controller, enough to cover them
          if (best_effort && index != 0)
          {
            continue;
          }
          shape.hardware = hardware;
          shape.best_effort = best_effort;
          size_t code = edges;
          for (size_t j = 0; j < index; ++j)
          {
            shape.edges[j] = static_cast<Edge>(code % 3);
            code /= 3;
          }
          place(index + 1);
        }
      }
    }
  };
  place(0);
}

Topology random_topology(std::mt19937 & random, size_t size)
{
  Topology topology(size);
  for (size_t i = 0; i < size; ++i)
  {
    topology[i].hardware = static_cast<int>(random() % std::size(HARDWARE_CLAIMS));
    topology[i].best_effort = random() % 4 == 0;
    for (size_t j = 0; j < i; ++j)
    {
      topology[i].edges.push_back(static_cast<Edge>(random() % 3));
    }
  }
  return topology;
}

bool hardware_fine(const Hardware & hw, bool command)
{
  const bool state_ok =
    hw.state == Lifecycle::Active || (!command && hw.state == Lifecycle::Inactive);
  return state_ok && hw.fault == HardwareFault::None && !hw.quarantined;
}

/// Whether the STRICT target is valid, by brute force straight from the definition.
bool strict_target_valid(const Model & model, const Names & target)
{
  const InterfaceIndex index(model);
  std::map<std::string, std::string> claimer;
  std::map<std::string, std::string> joint_claimer;
  for (const auto & name : target)
  {
    const auto & controller = model.controllers.at(name);
    if (controller.state != Lifecycle::Inactive && controller.state != Lifecycle::Active)
    {
      return false;
    }
    for (const auto & interface : controller.interfaces.command)
    {
      const auto * owner = index.command(interface);
      if (!owner)
      {
        return false;
      }
      if (owner->kind == InterfaceOwner::Kind::Reference && !target.count(owner->owner))
      {
        return false;
      }
      if (
        owner->kind == InterfaceOwner::Kind::HardwareCommand &&
        !hardware_fine(model.hardware.at(owner->owner), true))
      {
        return false;
      }
      if (owner->kind == InterfaceOwner::Kind::HardwareCommand)
      {
        const auto [joint, first] = joint_claimer.emplace(owner->owner + owner->joint, name);
        if (!first && joint->second != name)
        {
          return false;
        }
      }
      if (!claimer.emplace(interface, name).second)
      {
        return false;
      }
    }
    for (const auto & interface : controller.interfaces.state)
    {
      const auto * owner = index.state(interface);
      const bool available = owner && (owner->kind == InterfaceOwner::Kind::HardwareState
                                         ? hardware_fine(model.hardware.at(owner->owner), false)
                                         : target.count(owner->owner) > 0);
      if (!available && !controller.interfaces.best_effort_states)
      {
        return false;
      }
    }
  }
  return true;
}

enum class Failure
{
  None,
  StepFails,
  /// Every step fails, which shows termination under any outcome.
  EveryStepFails,
  ControllerFails,
  HardwareFails,
  /// The hardware faulted before the request and its fault handling still waits.
  FaultPending,
  /// The activation or deactivation at the step is given up at its deadline and returns once the
  /// reconciler waits for it.
  GivesUp,
};

struct Injection
{
  Failure kind = Failure::None;
  /// Before which executed step the failure strikes.
  size_t at = 0;
  /// Which controller or hardware fails, by position in the model.
  size_t which = 0;
};

class Explorer
{
public:
  /// Runs a request on the model and checks every bound. Returns the number of executed steps.
  size_t run(Model model, const SwitchRequest & request, const Injection & injection)
  {
    Desired desired = keep(model);
    Names failed_seeds;
    if (
      injection.kind == Failure::FaultPending &&
      !pend_fault(model, desired, injection.which, failed_seeds))
    {
      return 0;
    }
    const Model before = model;
    auto planned = plan_switch(model, desired.hardware, request);
    check_planner(before, request, planned);
    if (std::holds_alternative<Rejection>(planned))
    {
      return 0;
    }
    const auto & plan = std::get<SwitchPlan>(planned);
    desired = plan.desired;
    std::map<Activation, Activations> writers;
    const size_t bound = 12 * (model.controllers.size() + model.hardware.size()) + 12;
    size_t executed = 0;
    std::map<std::string, size_t> performs;
    // Claimers of the live interfaces of each component when it was taken
    std::map<std::string, Activations> taken;
    Names faulted;
    // A controller call given up on, and the taken components its controller claimed then
    std::optional<std::string> late;
    Names blocked_by_late;
    for (;; ++executed)
    {
      if (executed > bound)
      {
        fail("no termination within " + std::to_string(bound) + " steps");
        return executed;
      }
      if (injection.kind == Failure::ControllerFails && injection.at == executed)
      {
        auto it = std::next(model.controllers.begin(), static_cast<long>(injection.which));
        if (it->second.state == Lifecycle::Active && it->second.running)
        {
          failed_seeds.insert(it->first);
          apply_failures(model, desired, {it->first}, {});
          // RT stops the scope in the same cycle
          for (const auto & name : stop_closure(running_graph(model), {it->first}))
          {
            model.controllers.at(name).failed = true;
          }
        }
      }
      if (injection.kind == Failure::HardwareFails && injection.at == executed)
      {
        auto it = std::next(model.hardware.begin(), static_cast<long>(injection.which));
        if (it->second.state == Lifecycle::Active)
        {
          failed_seeds.insert(it->first);
          apply_failures(model, desired, {}, {{it->first, HardwareFault::Error}});
        }
      }
      for (const auto & [name, hw] : model.hardware)
      {
        if (hw.fault != HardwareFault::None)
        {
          faulted.insert(name);
        }
      }
      const Step step = next_step(model, desired);
      if (late && (std::holds_alternative<Done>(step) || std::holds_alternative<Blocked>(step)))
      {
        check_late(model, blocked_by_late);
        // A late activation is ended at once, a late deactivation ends inactive
        apply(model, returned::ControllerCallback{*late, Lifecycle::Inactive});
        late.reset();
        continue;
      }
      if (std::holds_alternative<Done>(step))
      {
        break;
      }
      // Only a hardware state request may bring a faulted component back up
      if (
        const auto * hw = std::get_if<step::Hardware>(&step);
        hw && faulted.count(hw->hardware) &&
        (hw->transition == step::HardwareTransition::Configure ||
         hw->transition == step::HardwareTransition::Activate))
      {
        fail(hw->hardware + " rises above its fault target: " + describe(step));
      }
      if (const auto * blocked = std::get_if<Blocked>(&step))
      {
        fail("blocked: " + blocked->reason);
        return executed;
      }
      const bool strike = injection.kind == Failure::EveryStepFails ||
                          (injection.kind == Failure::StepFails && injection.at == executed);
      for (const auto & hardware : testing::performed_on(step))
      {
        ++performs[hardware];
      }
      if (const auto * take = std::get_if<step::Take>(&step))
      {
        for (const auto & hardware : take->hardware)
        {
          auto & claimers = taken[hardware];
          for (const auto & interface : model.hardware.at(hardware).live)
          {
            for (const auto & [name, controller] : model.controllers)
            {
              const auto & claimed = controller.claims.command;
              if (std::find(claimed.begin(), claimed.end(), interface) != claimed.end())
              {
                claimers.insert({name, controller.activation});
              }
            }
          }
        }
      }
      Outcome outcome = outcome_of(step, strike);
      if (injection.kind == Failure::GivesUp && injection.at == executed && !late)
      {
        const auto * activate = std::get_if<step::Activate>(&step);
        const auto * deactivate = std::get_if<step::Deactivate>(&step);
        if (activate || deactivate)
        {
          late = activate ? activate->controller : deactivate->controller;
          outcome = Outcome{CallResult::TimedOut, Lifecycle::Unconfigured, {}, {}, false, {}, {}};
          // A hung activation is no updated claimer, SwitchModes gives its component back held
          const Claims claims = deactivate ? model.controllers.at(*late).claims : Claims{};
          for (const auto & [name, hw] : model.hardware)
          {
            for (const auto & interface : hw.command_interfaces)
            {
              const auto & claimed = claims.command;
              if (
                hw.taken &&
                std::find(claimed.begin(), claimed.end(), interface.name) != claimed.end())
              {
                blocked_by_late.insert(name);
              }
            }
          }
        }
      }
      const Failures failures = apply(model, desired, step, outcome);
      failed_seeds.merge(Names(failures.unusable));
      failed_seeds.merge(Names(failures.refused));
      for (const auto & violation : check_invariants(model))
      {
        fail("after " + describe(step) + ": " + violation);
      }
      for (const auto & name : late ? blocked_by_late : Names{})
      {
        if (!model.hardware.at(name).taken)
        {
          fail(
            "after " + describe(step) + ": " + name + " given back while " + *late +
            "'s call still runs");
        }
      }
      check_departed_writers(model, writers);
    }
    check_settled(model, desired);
    check_takes(model, taken);
    if (injection.kind == Failure::None)
    {
      check_mode_changes(before, model, performs, taken);
    }
    check_outcome(before, model, plan, request, failed_seeds, injection.kind != Failure::None);
    return executed;
  }

  /// The first failures, and how many there were in total.
  const std::vector<std::string> & failures() const { return failures_; }
  size_t failure_count() const { return failure_count_; }
  void set_context(std::string context) { context_ = std::move(context); }

private:
  void fail(const std::string & what)
  {
    ++failure_count_;
    if (failures_.size() < 20)
    {
      failures_.push_back(context_ + ": " + what);
    }
  }

  /// Faults an active component and runs every step before its fault handling, which a late call
  /// would hold up. Returns false if the component is not active.
  bool pend_fault(Model & model, Desired & desired, size_t which, Names & failed_seeds)
  {
    const auto it = std::next(model.hardware.begin(), static_cast<long>(which));
    if (it->second.state != Lifecycle::Active)
    {
      return false;
    }
    const std::string name = it->first;
    failed_seeds.insert(name);
    apply_failures(model, desired, {}, {{name, HardwareFault::Error}});
    for (size_t i = 0; i < 100; ++i)
    {
      const Step step = next_step(model, desired);
      const auto * hw = std::get_if<step::Hardware>(&step);
      if (std::holds_alternative<Done>(step) || (hw && hw->hardware == name))
      {
        return true;
      }
      apply(model, desired, step, outcome_of(step, false));
    }
    fail("the fault of " + name + " did not settle");
    return false;
  }

  static ChainGraph running_graph(const Model & model)
  {
    std::map<std::string, Claims> claims;
    for (const auto & [name, controller] : model.controllers)
    {
      if (controller.running)
      {
        claims.emplace(name, controller.claims);
      }
    }
    return ChainGraph(InterfaceIndex(model), claims);
  }

  static Outcome outcome_of(const Step & step, bool strike)
  {
    Outcome outcome = testing::outcome_for(step);
    if (const auto * hold = std::get_if<step::Hold>(&step); hold && strike)
    {
      outcome.faulted_hardware.insert(hold->interfaces.begin()->first);
    }
    if (const auto * take = std::get_if<step::Take>(&step); take && strike)
    {
      outcome.faulted_hardware.insert(*take->hardware.begin());
    }
    if (const auto * switch_modes = std::get_if<step::SwitchModes>(&step); switch_modes && strike)
    {
      outcome.faulted_hardware.insert(switch_modes->modes.begin()->first);
    }
    if (std::holds_alternative<step::Activate>(step) && strike)
    {
      outcome = call_outcome(CallResult::Failure, Lifecycle::Inactive);
    }
    if (std::holds_alternative<step::Deactivate>(step) && strike)
    {
      outcome = call_outcome(CallResult::Failure, Lifecycle::Finalized);
    }
    if (std::holds_alternative<step::SetChained>(step) && strike)
    {
      outcome.result = CallResult::Failure;
    }
    return outcome;
  }

  void check_planner(
    const Model & before, const SwitchRequest & request,
    const std::variant<SwitchPlan, Rejection> & planned)
  {
    const Names active = testing::active_names(before);
    if (request.strictness == Strictness::Strict)
    {
      const Names target = merged(without(active, request.deactivate), request.activate);
      bool startable = true;
      for (const auto & name : request.activate)
      {
        const auto state = before.controllers.at(name).state;
        startable = startable && (state == Lifecycle::Inactive || state == Lifecycle::Active);
      }
      const bool valid = startable && strict_target_valid(before, target);
      if (valid != std::holds_alternative<SwitchPlan>(planned))
      {
        fail(
          std::string("STRICT ") +
          (valid ? "valid target rejected: " : "invalid target accepted: ") +
          (valid ? std::get<Rejection>(planned).reason : text(target)));
      }
    }
    if (const auto * plan = std::get_if<SwitchPlan>(&planned))
    {
      // The same request must be rejected once what it stops implicitly is protected
      if (const Names implicit = without(plan->stopped, request.deactivate); !implicit.empty())
      {
        Model guarded = before;
        for (const auto & name : implicit)
        {
          guarded.controllers.at(name).is_protected = true;
        }
        if (
          std::holds_alternative<SwitchPlan>(plan_switch(guarded, plan->desired.hardware, request)))
        {
          fail("stopped protected controllers implicitly: " + text(implicit));
        }
      }
      const Names target = testing::names_of(plan->desired.active);
      if (!strict_target_valid(before, target))
      {
        fail("planned target is invalid: " + text(target));
      }
      if (request.strictness != Strictness::ForceAuto)
      {
        const Names lost = without(without(active, target), request.deactivate);
        if (!lost.empty())
        {
          fail("deactivated beyond the request: " + text(lost));
        }
      }
    }
  }

  void check_departed_writers(const Model & model, std::map<Activation, Activations> & writers)
  {
    const auto graph = running_graph(model);
    for (const auto & [name, provider] : model.controllers)
    {
      if (!provider.running || provider.failed)
      {
        continue;
      }
      auto & log = writers[{name, provider.activation}];
      for (const auto & [writer, activation] : log)
      {
        const auto & current = model.controllers.at(writer);
        const bool same = current.running && current.activation == activation;
        if (!same && !current.failed)
        {
          fail(name + " runs on references of " + writer + ", which stopped");
        }
      }
      for (const auto & writer : graph.writers_of(name))
      {
        log.insert({writer, model.controllers.at(writer).activation});
      }
    }
  }

  /// A taken component gets no writes, so no claimer of it may keep running in the same activation.
  /// While a deactivation hangs, its claimer may still write, so the joints it held on a taken
  /// component are held, and the component stays taken.
  void check_late(const Model & model, const Names & blocked)
  {
    for (const auto & name : blocked)
    {
      const auto & hw = model.hardware.at(name);
      for (const auto & interface : hw.live)
      {
        for (const auto & [controller, state] : model.controllers)
        {
          const auto & claimed = state.claims.command;
          if (
            state.state == Lifecycle::Active && state.quarantined && !state.running &&
            std::find(claimed.begin(), claimed.end(), interface) != claimed.end())
          {
            fail(name + " keeps " + interface + " live under the hung " + controller);
          }
        }
      }
    }
  }

  void check_takes(const Model & after, const std::map<std::string, Activations> & taken)
  {
    for (const auto & [hardware, claimers] : taken)
    {
      for (const auto & [name, activation] : claimers)
      {
        const auto & controller = after.controllers.at(name);
        if (controller.state == Lifecycle::Active && controller.activation == activation)
        {
          fail(hardware + " was taken while " + name + " kept running");
        }
      }
    }
  }

  /// Without failures a taken component performs once if its live interfaces change and never
  /// otherwise. One that holds and starts performs at most twice.
  void check_mode_changes(
    const Model & before, const Model & after, const std::map<std::string, size_t> & performs,
    const std::map<std::string, Activations> & taken)
  {
    for (const auto & [name, hw] : after.hardware)
    {
      const auto it = performs.find(name);
      const size_t calls = it == performs.end() ? 0 : it->second;
      const bool changed = hw.live != before.hardware.at(name).live;
      const bool wrong = taken.count(name) ? calls != (changed ? 1u : 0u) : calls > 2;
      if (wrong || (changed && calls == 0))
      {
        fail(
          name + " performed " + std::to_string(calls) + " times" +
          (taken.count(name) ? " after a take" : ""));
      }
    }
  }

  void check_settled(const Model & after, const Desired & desired)
  {
    const InterfaceIndex index(after);
    for (const auto & [name, controller] : after.controllers)
    {
      if (controller.state != Lifecycle::Active)
      {
        continue;
      }
      if (!desired.active.count(name) || !controller.running || controller.failed)
      {
        fail(name + " is active but not a settled member");
      }
      bool written = false;
      for (const auto & [other, writer] : after.controllers)
      {
        if (writer.state != Lifecycle::Active)
        {
          continue;
        }
        for (const auto & interface : writer.claims.command)
        {
          const auto * owner = index.command(interface);
          written = written || (owner && owner->kind == InterfaceOwner::Kind::Reference &&
                                owner->owner == name);
        }
      }
      if (controller.chained != written)
      {
        fail(
          name + " has chained mode " + std::to_string(controller.chained) + " but writers " +
          std::to_string(written));
      }
    }
  }

  void check_outcome(
    const Model & before, const Model & after, const SwitchPlan & plan,
    const SwitchRequest & request, const Names & failed, bool injected)
  {
    const Names active_before = testing::active_names(before);
    const Names active_after = testing::active_names(after);
    const Names target = testing::names_of(plan.desired.active);
    if (!injected && active_after != target)
    {
      fail("settled on " + text(active_after) + " instead of " + text(target));
    }
    if (!is_subset(without(active_after, active_before), plan.started))
    {
      fail("activated beyond the plan: " + text(without(active_after, active_before)));
    }
    // A failure may stop at most what is connected to it in the union of before and target
    std::map<std::string, Claims> claims;
    for (const auto & name : active_before)
    {
      claims[name] = before.controllers.at(name).claims;
    }
    for (const auto & [name, want] : plan.desired.active)
    {
      claims[name] = want.claims;
    }
    Names seeds = failed;
    const InterfaceIndex index(before);
    for (const auto & name : failed)
    {
      if (before.hardware.count(name))
      {
        seeds.merge(claimers_of_hardware(after, index, name));
        for (const auto & [controller, claim] : claims)
        {
          for (const auto & interface : claim.command)
          {
            const auto * owner = index.command(interface);
            if (owner && owner->owner == name)
            {
              seeds.insert(controller);
            }
          }
          for (const auto & interface : claim.state)
          {
            const auto * owner = index.state(interface);
            if (owner && owner->owner == name)
            {
              seeds.insert(controller);
            }
          }
        }
      }
    }
    const Names allowed = merged(
      merged(request.deactivate, without(active_before, target)),
      stop_closure(ChainGraph(index, claims), seeds));
    const Names lost = without(without(active_before, active_after), allowed);
    if (!lost.empty())
    {
      fail("lost beyond the failure scope: " + text(lost));
    }
  }

  std::string context_;
  std::vector<std::string> failures_;
  size_t failure_count_ = 0;
};

std::vector<Names> subsets(const Names & names)
{
  std::vector<std::string> list(names.begin(), names.end());
  std::vector<Names> result;
  for (size_t mask = 0; mask < (size_t{1} << list.size()); ++mask)
  {
    Names subset;
    for (size_t i = 0; i < list.size(); ++i)
    {
      if (mask & (size_t{1} << i))
      {
        subset.insert(list[i]);
      }
    }
    result.push_back(subset);
  }
  return result;
}

/// Every valid active set of the topology, reached from all inactive.
std::vector<Model> start_states(const Topology & topology)
{
  const Model base = build(topology);
  const Names all = testing::names_of(base.controllers);
  std::vector<Model> states;
  for (const auto & active : subsets(all))
  {
    Model model = base;
    const auto planned = plan_switch(model, keep(model).hardware, {active, {}, Strictness::Strict});
    if (const auto * plan = std::get_if<SwitchPlan>(&planned))
    {
      testing::settle(model, plan->desired);
      states.push_back(model);
    }
  }
  return states;
}

std::string text_of(const Topology & topology)
{
  std::string result;
  for (size_t i = 0; i < topology.size(); ++i)
  {
    result += " c" + std::to_string(i) + "{hw" + std::to_string(topology[i].hardware);
    for (size_t j = 0; j < topology[i].edges.size(); ++j)
    {
      if (topology[i].edges[j] != NONE)
      {
        result += (topology[i].edges[j] == WRITE ? " w" : " r") + std::to_string(j);
      }
    }
    result += topology[i].best_effort ? " be}" : "}";
  }
  return result;
}

std::vector<SwitchRequest> requests_for(const Names & all)
{
  std::vector<SwitchRequest> requests;
  for (const auto strictness : {Strictness::Strict, Strictness::Auto, Strictness::ForceAuto})
  {
    for (const auto & name : all)
    {
      requests.push_back({{name}, {}, strictness});
      requests.push_back({{}, {name}, strictness});
      requests.push_back({{name}, {name}, strictness});
    }
    requests.push_back({all, {}, strictness});
    requests.push_back({{}, all, strictness});
  }
  return requests;
}

void explore(
  const Topology & topology, Explorer & explorer, bool with_failures, bool with_give_ups = false)
{
  for (const auto & start : start_states(topology))
  {
    const Names all = testing::names_of(start.controllers);
    for (const auto & request : requests_for(all))
    {
      explorer.set_context(
        text_of(topology) + " active " + text(testing::active_names(start)) + " request +" +
        text(request.activate) + " -" + text(request.deactivate) + " strictness " +
        std::to_string(static_cast<int>(request.strictness)));
      const size_t steps = explorer.run(start, request, {});
      if (!with_failures)
      {
        continue;
      }
      explorer.run(start, request, {Failure::EveryStepFails, 0, 0});
      for (size_t at = 0; at < steps; ++at)
      {
        explorer.run(start, request, {Failure::StepFails, at, 0});
        for (size_t which = 0; which < start.controllers.size(); ++which)
        {
          explorer.run(start, request, {Failure::ControllerFails, at, which});
        }
        for (size_t which = 0; which < start.hardware.size(); ++which)
        {
          explorer.run(start, request, {Failure::HardwareFails, at, which});
        }
        if (with_give_ups)
        {
          explorer.run(start, request, {Failure::GivesUp, at, 0});
        }
      }
      for (size_t which = 0; which < start.hardware.size(); ++which)
      {
        explorer.run(start, request, {Failure::FaultPending, 0, which});
      }
    }
  }
}

void report(const Explorer & explorer)
{
  for (const auto & failure : explorer.failures())
  {
    ADD_FAILURE() << failure;
  }
  if (explorer.failure_count() > explorer.failures().size())
  {
    ADD_FAILURE() << explorer.failure_count() << " failures in total, the first "
                  << explorer.failures().size() << " are listed";
  }
}

size_t env_or(const char * name, size_t fallback)
{
  const char * value = std::getenv(name);
  return value ? static_cast<size_t>(std::stoul(value)) : fallback;
}

TEST(Explorer, every_topology_up_to_three_controllers_with_every_single_failure)
{
  Explorer explorer;
  size_t topologies = 0;
  for (size_t size = 1; size <= 3; ++size)
  {
    for_each_topology(
      size, ONE_JOINT_CHOICES,
      [&](const Topology & topology)
      {
        ++topologies;
        explore(topology, explorer, true);
      });
  }
  RecordProperty("topologies", static_cast<int>(topologies));
  report(explorer);
}

TEST(Explorer, every_topology_up_to_three_controllers_on_two_joints_of_one_component)
{
  Explorer explorer;
  for (size_t size = 1; size <= 3; ++size)
  {
    for_each_topology(
      size, TWO_JOINT_CHOICES,
      [&](const Topology & topology) { explore(topology, explorer, true, true); });
  }
  report(explorer);
}

TEST(Explorer, random_topologies_of_four_controllers)
{
  std::mt19937 random(static_cast<unsigned>(env_or("EXPLORER_SEED", 1)));
  Explorer explorer;
  const size_t count = env_or("EXPLORER_FOUR", 300);
  for (size_t i = 0; i < count; ++i)
  {
    explore(random_topology(random, 4), explorer, true);
  }
  report(explorer);
}

}  // namespace
}  // namespace hector_controller_manager::core
