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

#include "hector_controller_manager/step_executor.hpp"

#include <limits>
#include <set>
#include <string>

#include "hector_controller_manager/leak.hpp"
#include "hector_controller_manager/runtime/plan_builder.hpp"

namespace hector_controller_manager
{
using core::CallResult;
using core::Lifecycle;
using runtime::ControllerRuntime;

namespace
{
using Handles = std::vector<hardware_interface::CommandInterface::SharedPtr>;

/// The hardware command handles of the claims that hold a double.
Handles releasable(
  const core::Model & model, const Entities & entities, const core::Claims & claims)
{
  const core::InterfaceIndex index(model);
  Handles handles;
  for (const auto & name : claims.command)
  {
    if (index.hardware_command(name))
    {
      const auto & handle = entities.command_handles.at(name);
      if (handle->get_data_type() == hardware_interface::HandleDataType::DOUBLE)
      {
        handles.push_back(handle);
      }
    }
  }
  return handles;
}

/// A released handle holds NaN, so no later write applies a command of the ended activation.
void neutralize(const Handles & handles)
{
  for (const auto & handle : handles)
  {
    // A blocking set of a double handle cannot fail
    (void)handle->set_value(std::numeric_limits<double>::quiet_NaN(), true);
  }
}
}  // namespace

core::Outcome StepExecutor::lifecycle_call(
  runtime::OwnerThread & owner, std::function<core::Outcome()> call, Clock::time_point deadline,
  std::function<void(Lifecycle)> late, std::function<void(core::Outcome &)> undo,
  const runtime::WaitInterrupt & interrupt)
{
  supervisor_.expect();
  auto result = std::make_shared<core::Outcome>();
  auto job = owner.submit([call = std::move(call), result] { *result = call(); });
  auto inbox = inbox_;
  const bool returned = owner.wait(
    job, deadline,
    [inbox, result, late = std::move(late), undo = std::move(undo)]
    {
      if (undo)
      {
        undo(*result);
      }
      inbox->post([result, late] { late(result->state); });
    },
    interrupt);
  if (!returned)
  {
    return core::call_outcome(
      CallResult::TimedOut, Lifecycle::Unconfigured, "The call outlived its deadline");
  }
  return *result;
}

void StepExecutor::write_released(
  const std::string & stopped, core::Outcome & outcome, Clock::time_point deadline)
{
  for (const auto & [name, hw] : model_.hardware)
  {
    if (!hw.taken)
    {
      continue;
    }
    bool released = false;
    bool written = false;
    for (const auto & interface : hw.live)
    {
      const auto * claimer = core::command_claimer(model_, interface);
      if (claimer && claimer->second.state == Lifecycle::Active)
      {
        (claimer->first == stopped ? released : written) = true;
      }
    }
    if (!released || written)
    {
      continue;
    }
    auto * hardware = entities_.hardware.at(name).get();
    auto job = hardware->owner().submit([hardware] { hardware->write_through_block(); });
    if (!hardware->owner().wait(job, deadline, late_mode_switch(name)))
    {
      // It may still read handles a successor writes, so nothing starts on it until it returned
      outcome.faulted_hardware.insert(name);
      outcome.timed_out_hardware.insert(name);
      core::append_reason(outcome.reason, name + ": the NaN write outlived its deadline");
    }
  }
}

core::Outcome StepExecutor::switch_hardware(
  const std::map<std::string, core::ModeSwitch> & modes,
  const std::function<bool(const std::string &)> & give_back, Clock::time_point deadline)
{
  struct Result
  {
    std::string failure;
    std::vector<std::string> skipped;
  };
  struct Pending
  {
    std::string name;
    std::shared_ptr<Result> result;
    runtime::OwnerThread::JobPtr job;
  };
  std::vector<Pending> pending;
  for (const auto & [name, mode] : modes)
  {
    if (give_back(name) && mode.start.empty() && mode.stop.empty())
    {
      entities_.hardware.at(name)->unblock_writes();
      continue;
    }
    // A given up job runs late, maybe after an unload, so a runtime may die on this owner
    std::vector<std::pair<std::shared_ptr<ControllerRuntime>, uint64_t>> claimers;
    for (const auto & interface : mode.start)
    {
      const auto * claimer = core::command_claimer(model_, interface);
      claimers.emplace_back(
        claimer ? entities_.controllers.at(claimer->first) : nullptr,
        claimer ? claimer->second.activation : 0);
    }
    auto * hardware = entities_.hardware.at(name).get();
    hardware->block_writes();
    auto result = std::make_shared<Result>();
    // Without a mode change the job of a take only proves that no write which began earlier still
    // runs
    auto job = hardware->owner().submit(
      [hardware, mode = mode, claimers, result]
      {
        std::vector<std::string> start;
        for (size_t i = 0; i < mode.start.size(); ++i)
        {
          const auto & [claimer, activation] = claimers[i];
          const bool failed =
            claimer && claimer->failed_activation.load(std::memory_order_acquire) == activation;
          (failed ? result->skipped : start).push_back(mode.start[i]);
        }
        if (!start.empty() || !mode.stop.empty())
        {
          result->failure = hardware->switch_modes(start, mode.stop);
        }
      });
    pending.push_back({name, result, job});
  }
  core::Outcome outcome;
  for (auto & [name, result, job] : pending)
  {
    const bool returned =
      entities_.hardware.at(name)->owner().wait(job, deadline, late_mode_switch(name));
    if (!returned)
    {
      outcome.faulted_hardware.insert(name);
      outcome.timed_out_hardware.insert(name);
      core::append_reason(outcome.reason, name + ": the mode switch outlived its deadline");
      continue;
    }
    if (give_back(name))
    {
      entities_.hardware.at(name)->unblock_writes();
    }
    if (!result->failure.empty())
    {
      outcome.faulted_hardware.insert(name);
      core::append_reason(outcome.reason, name + ": " + result->failure);
    }
    outcome.skipped.insert(result->skipped.begin(), result->skipped.end());
  }
  return outcome;
}

void StepExecutor::detach_controllers(const core::step::Detach & detach, Clock::time_point deadline)
{
  core::Model next = model_;
  for (const auto & name : detach.controllers)
  {
    next.controllers.at(name).running = false;
  }
  // A write with claims of the old plan would limit, and so overwrite, the NaN of a released handle
  if (!publish_plan(next, deadline) || !slot_.wait_claims_current(generation_, deadline))
  {
    // RT is stuck in a call that may still run these controllers
    // Quiescence for the newest plan covers every older one, so earlier waiters keep waiting
    auto & [generation, waiting] = awaiting_quiescence_;
    generation = generation_;
    core::GivenUp given_up;
    for (const auto & name : detach.controllers)
    {
      if (waiting.insert(name).second)
      {
        given_up.controllers.insert(name);
      }
    }
    core::apply(model_, given_up);
  }
}

core::Outcome StepExecutor::deactivate_controller(
  const core::step::Deactivate & deactivate, Clock::time_point deadline)
{
  auto * controller = entities_.controllers.at(deactivate.controller).get();
  const std::string name = deactivate.controller;
  auto released = releasable(model_, entities_, model_.controllers.at(name).claims);
  auto outcome = lifecycle_call(
    controller->owner(),
    [controller, released]
    {
      auto outcome = controller->deactivate();
      // A controller may write its handles in on_deactivate, the NaN goes in after it
      neutralize(released);
      return outcome;
    },
    deadline, [&model = model_, name](Lifecycle state)
    { core::apply(model, core::returned::ControllerCallback{name, state}); });
  if (outcome.result != CallResult::TimedOut)
  {
    write_released(name, outcome, deadline);
  }
  return outcome;
}

std::string StepExecutor::await_fresh_states(
  const core::Claims & claims, Clock::time_point deadline)
{
  const core::InterfaceIndex index(model_);
  std::set<std::string> providers;
  for (const auto & interface : claims.state)
  {
    const auto * owner = index.state(interface);
    if (owner && owner->kind == core::InterfaceOwner::Kind::HardwareState)
    {
      providers.insert(owner->owner);
    }
  }
  const auto stale = [&]
  {
    std::string names;
    for (const auto & name : providers)
    {
      if (!entities_.hardware.at(name)->states_fresh())
      {
        names += (names.empty() ? "'" : ", '") + name + "'";
      }
    }
    return names;
  };
  (void)slot_.wait_until([&] { return stale().empty(); }, deadline);
  return stale();
}

core::Outcome StepExecutor::activate_controller(
  const core::step::Activate & activate, Clock::time_point deadline)
{
  auto * controller = entities_.controllers.at(activate.controller).get();
  const std::string name = activate.controller;
  // Until its first read a reactivated component still shows the states of before, and a
  // controller seeded from them drives the joints back to where they were
  if (const auto stale = await_fresh_states(activate.claims, deadline); !stale.empty())
  {
    return core::call_outcome(
      CallResult::Failure, Lifecycle::Inactive,
      "Hardware " + stale + " delivered no read since its last transition");
  }
  auto handles = entities_.handles_of(activate.claims);
  auto released = releasable(model_, entities_, activate.claims);
  const bool chained = model_.controllers.at(name).chained;
  return lifecycle_call(
    controller->owner(),
    [controller, handles, released, chained]
    {
      auto outcome = controller->activate(handles, chained);
      if (outcome.result != CallResult::Ok)
      {
        neutralize(released);
      }
      return outcome;
    },
    deadline, [&model = model_, name](Lifecycle state)
    { core::apply(model, core::returned::ControllerCallback{name, state}); },
    // The step failed at its deadline, so its request and maybe its providers are gone
    [controller, released](core::Outcome & late)
    {
      if (late.result == CallResult::Ok)
      {
        late = controller->deactivate();
        neutralize(released);
      }
    });
}

core::Outcome StepExecutor::set_chained(
  const core::step::SetChained & chained, Clock::time_point deadline)
{
  auto * controller = entities_.controllers.at(chained.controller).get();
  const std::string name = chained.controller;
  const bool flag = chained.chained;
  auto applied = std::make_shared<bool>(false);
  return lifecycle_call(
    controller->owner(),
    [controller, flag, applied]
    {
      auto refused = controller->set_chained(flag);
      *applied = refused.empty();
      return core::call_outcome(
        *applied ? CallResult::Ok : CallResult::Failure, Lifecycle::Inactive, std::move(refused));
    },
    deadline,
    [&model = model_, name, flag, applied](Lifecycle)
    {
      // The plugin keeps the mode it reached late, the model must match it
      core::apply(
        model, core::returned::ChainedMode{name, *applied ? std::optional(flag) : std::nullopt});
    });
}

core::Outcome StepExecutor::execute(
  const core::Step & step, Clock::time_point deadline, const runtime::WaitInterrupt & interrupt)
{
  supervisor_.expect();
  namespace step_ = core::step;
  if (const auto * hold = std::get_if<step_::Hold>(&step))
  {
    std::map<std::string, core::ModeSwitch> modes;
    for (const auto & [name, interfaces] : hold->interfaces)
    {
      modes[name].stop = interfaces;
    }
    // A taken component stays blocked, its hung claimer may still write the held handles
    return switch_hardware(
      modes, [this](const std::string & name) { return !model_.hardware.at(name).taken; },
      deadline);
  }
  if (const auto * take = std::get_if<step_::Take>(&step))
  {
    std::map<std::string, core::ModeSwitch> modes;
    for (const auto & name : take->hardware)
    {
      modes[name];
    }
    return switch_hardware(modes, [](const std::string &) { return false; }, deadline);
  }
  if (const auto * switch_modes = std::get_if<step_::SwitchModes>(&step))
  {
    return switch_hardware(switch_modes->modes, [](const std::string &) { return true; }, deadline);
  }
  if (const auto * detach = std::get_if<step_::Detach>(&step))
  {
    detach_controllers(*detach, deadline);
    return {};
  }
  if (const auto * attach = std::get_if<step_::Attach>(&step))
  {
    core::Model next = model_;
    for (const auto & name : attach->controllers)
    {
      next.controllers.at(name).running = true;
    }
    publish_plan(next, deadline);
    return {};
  }
  if (std::get_if<step_::AwaitUpdate>(&step))
  {
    core::Outcome outcome;
    outcome.not_updated = !slot_.wait_updated(generation_, deadline);
    return outcome;
  }
  if (const auto * deactivate = std::get_if<step_::Deactivate>(&step))
  {
    return deactivate_controller(*deactivate, deadline);
  }
  if (const auto * activate = std::get_if<step_::Activate>(&step))
  {
    return activate_controller(*activate, deadline);
  }
  if (const auto * chained = std::get_if<step_::SetChained>(&step))
  {
    return set_chained(*chained, deadline);
  }
  const auto & transition = std::get<step_::Hardware>(step);
  auto * hardware = entities_.hardware.at(transition.hardware).get();
  const std::string name = transition.hardware;
  const auto kind = transition.transition;
  return lifecycle_call(
    hardware->owner(), [hardware, kind] { return hardware->transition(kind); }, deadline,
    [&model = model_, name](Lifecycle state)
    { core::apply(model, core::returned::HardwareCallback{name, state}); }, {}, interrupt);
}

bool StepExecutor::publish_plan(const core::Model & model, Clock::time_point deadline)
{
  supervisor_.expect();
  auto plan = runtime::build_plan(model, entities_.controllers, entities_.hardware, ++generation_);
  slot_.publish(plan.get());
  plans_.push_back(plan);
  const bool reached = slot_.wait_quiescent(plan->generation, deadline);
  release_old_plans(Clock::now());
  return reached;
}

bool StepExecutor::release_old_plans(Clock::time_point deadline)
{
  supervisor_.expect();
  if (plans_.empty())
  {
    return true;
  }
  if (!slot_.wait_quiescent(plans_.back()->generation, deadline))
  {
    return false;
  }
  plans_.erase(plans_.begin(), plans_.end() - 1);
  return true;
}

bool StepExecutor::release_quiescent()
{
  supervisor_.expect();
  auto & [generation, waiting] = awaiting_quiescence_;
  if (waiting.empty() || !slot_.wait_claims_current(generation, Clock::now()))
  {
    return false;
  }
  core::apply(model_, core::returned::Released{std::exchange(waiting, {}), {}});
  return true;
}

void StepExecutor::wait_quiescent_or_ring(uint32_t seen)
{
  supervisor_.expect();
  slot_.wait_claims_current_or_ring(awaiting_quiescence_.first, seen, Clock::time_point::max());
}

std::function<void()> StepExecutor::late_mode_switch(const std::string & hardware) const
{
  return [&model = model_, inbox = inbox_, hardware]
  {
    inbox->post([&model, hardware]
                { core::apply(model, core::returned::Released{{}, {hardware}}); });
  };
}

bool StepExecutor::retire(Clock::time_point deadline)
{
  supervisor_.expect();
  slot_.publish(nullptr);
  if (!slot_.wait_quiescent(generation_ + 1, deadline))
  {
    // RT returns into the plan it is in
    for (auto & plan : plans_)
    {
      leak(std::move(plan));
    }
    return false;
  }
  return true;
}

}  // namespace hector_controller_manager
