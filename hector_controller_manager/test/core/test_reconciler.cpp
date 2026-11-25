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

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "fixtures.hpp"
#include "hector_controller_manager/core/names.hpp"
#include "hector_controller_manager/core/reconciler.hpp"

namespace hector_controller_manager::core
{
namespace
{
using testing::active_names;
using testing::before;
using testing::keep;
using testing::plan_of;
using testing::rig;
using testing::settle;
using testing::started_rig;
using testing::startup_active;
using testing::switch_to;
using testing::until_switch_modes;

/// Index of the first step of the given kind touching the name.
template <typename T>
std::optional<size_t> first(const std::vector<Step> & steps, const std::string & name)
{
  for (size_t i = 0; i < steps.size(); ++i)
  {
    if (const auto * found = std::get_if<T>(&steps[i]))
    {
      if constexpr (std::is_same_v<T, step::Activate> || std::is_same_v<T, step::Deactivate>)
      {
        if (found->controller == name)
        {
          return i;
        }
      }
      else if constexpr (
        std::is_same_v<T, step::Attach> || std::is_same_v<T, step::Detach> ||
        std::is_same_v<T, step::AwaitUpdate>)
      {
        if (found->controllers.count(name))
        {
          return i;
        }
      }
      else if constexpr (std::is_same_v<T, step::SwitchModes>)
      {
        if (found->modes.count(name))
        {
          return i;
        }
      }
      else if constexpr (std::is_same_v<T, step::Take>)
      {
        if (found->hardware.count(name))
        {
          return i;
        }
      }
      else
      {
        if (found->interfaces.count(name))
        {
          return i;
        }
      }
    }
  }
  return std::nullopt;
}

TEST(Reconciler, a_settled_model_is_done)
{
  const Model model = started_rig();
  EXPECT_TRUE(std::holds_alternative<Done>(next_step(model, keep(model))));
}

TEST(Reconciler, startup_makes_every_claimed_command_interface_live)
{
  const Model model = started_rig();
  EXPECT_EQ(model.hardware.at("arm").live, (Names{"a1/velocity", "a2/velocity", "a3/velocity"}));
  EXPECT_EQ(model.hardware.at("flipper").live, (Names{"f1/position"}));
  EXPECT_TRUE(model.hardware.at("imu").live.empty());
}

TEST(Reconciler, providers_activate_before_users_and_start_after_attach)
{
  Model model = rig();
  const auto plan = plan_of(model, {startup_active(), {}, Strictness::Strict});
  const auto steps = settle(model, plan.desired);
  EXPECT_TRUE(
    before(first<step::Activate>(steps, "arm_vel"), first<step::Activate>(steps, "arm_ik")));
  EXPECT_TRUE(
    before(first<step::Activate>(steps, "diff_drive"), first<step::Activate>(steps, "odom_pub")));
  EXPECT_TRUE(
    before(first<step::Attach>(steps, "arm_vel"), first<step::SwitchModes>(steps, "arm")));
}

TEST(Reconciler, a_switch_takes_before_detach_and_deactivates_users_first)
{
  Model model = started_rig();
  const auto plan = plan_of(model, {{}, {"wrist"}, Strictness::Strict});
  const auto steps = settle(model, plan.desired);
  EXPECT_TRUE(before(first<step::Take>(steps, "arm"), first<step::Detach>(steps, "arm_vel")));
  EXPECT_TRUE(
    before(first<step::AwaitUpdate>(steps, "arm_vel"), first<step::SwitchModes>(steps, "arm")));
  EXPECT_FALSE(model.hardware.at("arm").taken);
  EXPECT_TRUE(
    before(first<step::Deactivate>(steps, "arm_ik"), first<step::Deactivate>(steps, "arm_vel")));
  EXPECT_TRUE(
    before(first<step::Deactivate>(steps, "wrist"), first<step::Deactivate>(steps, "arm_vel")));
  EXPECT_EQ(active_names(model), without(startup_active(), {"wrist"}));
  EXPECT_EQ(model.controllers.at("arm_vel").activation, 2u);
}

TEST(Reconciler, a_taken_component_stays_taken_while_a_detached_claimer_may_still_write)
{
  Model model = started_rig();
  const Desired desired = plan_of(model, {{}, {"wrist"}, Strictness::Strict}).desired;
  Desired working = desired;
  Step step = next_step(model, working);
  ASSERT_TRUE(std::holds_alternative<step::Take>(step));
  apply(model, working, step, {});
  step = next_step(model, working);
  ASSERT_TRUE(std::holds_alternative<step::Detach>(step));
  // RT is stuck in a call of the detached controllers
  const Names detached = std::get<step::Detach>(step).controllers;
  apply(model, working, step, {});
  apply(model, GivenUp{detached, {}});
  for (step = next_step(model, working);
       !std::holds_alternative<Done>(step) && !std::holds_alternative<Blocked>(step);
       step = next_step(model, working))
  {
    ASSERT_FALSE(std::holds_alternative<step::SwitchModes>(step)) << describe(step);
    apply(model, working, step, {});
  }
  EXPECT_TRUE(model.hardware.at("arm").taken);
  apply(model, returned::Released{detached, {}});
  settle(model, desired);
  EXPECT_FALSE(model.hardware.at("arm").taken);
}

TEST(Reconciler, an_interface_that_cannot_take_nan_is_held_before_detach)
{
  Model model = started_rig();
  for (auto & interface : model.hardware.at("arm").command_interfaces)
  {
    interface.takes_nan = false;
  }
  const auto plan = plan_of(model, {{}, {"wrist"}, Strictness::Strict});
  const auto steps = settle(model, plan.desired);
  EXPECT_TRUE(before(first<step::Hold>(steps, "arm"), first<step::Detach>(steps, "arm_vel")));
  EXPECT_FALSE(first<step::Take>(steps, "arm").has_value());
}

TEST(Reconciler, a_stop_without_a_successor_holds_before_detach)
{
  Model model = started_rig();
  const auto plan = plan_of(model, {{}, {"flipper_pos"}, Strictness::Strict});
  const auto steps = settle(model, plan.desired);
  EXPECT_TRUE(
    before(first<step::Hold>(steps, "flipper"), first<step::Detach>(steps, "flipper_pos")));
  EXPECT_FALSE(first<step::Take>(steps, "flipper").has_value());
}

TEST(Reconciler, shutdown_holds_every_interface_before_detach)
{
  Model model = started_rig();
  Desired desired = keep(model);
  desired.active.clear();
  const auto steps = settle(model, desired);
  ASSERT_FALSE(steps.empty());
  EXPECT_TRUE(std::holds_alternative<step::Hold>(steps.front()));
  for (const auto & step : steps)
  {
    EXPECT_FALSE(std::holds_alternative<step::Take>(step));
  }
}

TEST(Reconciler, chained_mode_is_set_before_the_user_activates)
{
  Model model = started_rig();
  const auto plan = plan_of(model, {{"whole_body"}, {}, Strictness::Strict});
  const auto steps = settle(model, plan.desired);
  std::optional<size_t> chained;
  for (size_t i = 0; i < steps.size(); ++i)
  {
    if (const auto * set = std::get_if<step::SetChained>(&steps[i]);
        set && set->controller == "flipper_pos" && set->chained)
    {
      chained = i;
    }
  }
  EXPECT_TRUE(before(chained, first<step::Activate>(steps, "whole_body")));
  EXPECT_TRUE(model.controllers.at("flipper_pos").chained);
}

TEST(Reconciler, failures_shrink_desired_and_the_closure_stops)
{
  Model model = started_rig();
  Desired desired = keep(model);
  apply_failures(model, desired, {"diff_drive"}, {});
  EXPECT_EQ(
    Names(without(startup_active(), {"diff_drive", "odom_pub", "base_vel"})),
    testing::names_of(desired.active));
  settle(model, desired);
  EXPECT_EQ(active_names(model), without(startup_active(), {"diff_drive", "odom_pub", "base_vel"}));
  EXPECT_TRUE(model.hardware.at("base").live.empty());
}

TEST(Reconciler, a_hardware_fault_ends_unconfigured_and_stops_its_claimers)
{
  Model model = started_rig();
  Desired desired = keep(model);
  apply_failures(model, desired, {}, {{"flipper", HardwareFault::Error}});
  const auto steps = settle(model, desired);
  EXPECT_EQ(model.hardware.at("flipper").state, Lifecycle::Unconfigured);
  EXPECT_EQ(active_names(model), without(startup_active(), {"flipper_pos", "jsb"}));
  EXPECT_TRUE(first<step::Deactivate>(steps, "flipper_pos").has_value());
}

// Inactive hardware whose read asks to deactivate goes through on_error, as does active hardware
// whose on_deactivate fails. Nothing configures it again without a request.
TEST(Reconciler, fault_handling_that_ends_below_its_target_stays_there)
{
  for (const bool fails_to_deactivate : {false, true})
  {
    SCOPED_TRACE(fails_to_deactivate ? "on_deactivate fails" : "inactive");
    Model model = rig();
    model.hardware.at("flipper").state =
      fails_to_deactivate ? Lifecycle::Active : Lifecycle::Inactive;
    Desired desired = keep(model);
    apply_failures(model, desired, {}, {{"flipper", HardwareFault::Deactivate}});
    if (fails_to_deactivate)
    {
      const Step step = next_step(model, desired);
      ASSERT_TRUE(std::holds_alternative<step::Hardware>(step));
      apply(model, desired, step, call_outcome(CallResult::Error, Lifecycle::Unconfigured));
    }
    for (const auto & step : settle(model, desired))
    {
      const auto * hw = std::get_if<step::Hardware>(&step);
      EXPECT_FALSE(hw && hw->transition == step::HardwareTransition::Configure) << describe(step);
    }
    EXPECT_EQ(model.hardware.at("flipper").state, Lifecycle::Unconfigured);
  }
}

// A reader may stay on an inactive component, its cleanup and shutdown wait until the reader
// stopped
TEST(Reconciler, a_reader_blocks_a_cleanup_or_shutdown_of_its_hardware_but_not_a_deactivate)
{
  Model model = rig();
  switch_to(model, {"jsb"}, {});
  Desired desired = keep(model);
  desired.hardware["flipper"] = Lifecycle::Inactive;
  const Step deactivate = next_step(model, desired);
  const auto * hw = std::get_if<step::Hardware>(&deactivate);
  ASSERT_TRUE(hw) << describe(deactivate);
  EXPECT_EQ(hw->hardware, "flipper");
  EXPECT_EQ(hw->transition, step::HardwareTransition::Deactivate);
  model.hardware.at("flipper").state = Lifecycle::Inactive;
  for (const Lifecycle target : {Lifecycle::Unconfigured, Lifecycle::Finalized})
  {
    desired.hardware["flipper"] = target;
    const Step step = next_step(model, desired);
    const auto * down = std::get_if<step::Hardware>(&step);
    EXPECT_FALSE(down && down->hardware == "flipper") << describe(step);
  }
}

TEST(Reconciler, a_failed_hardware_transition_stops_the_claimers_it_strands)
{
  Model model = started_rig();
  model.hardware.at("imu").state = Lifecycle::Inactive;
  Desired desired = keep(model);
  desired.hardware["imu"] = Lifecycle::Active;
  apply(
    model, desired, step::Hardware{"imu", step::HardwareTransition::Activate},
    call_outcome(CallResult::Error, Lifecycle::Unconfigured));
  EXPECT_FALSE(desired.active.count("diff_drive"));
  EXPECT_TRUE(desired.active.count("arm_vel"));
  settle(model, desired);
  EXPECT_EQ(model.controllers.at("diff_drive").state, Lifecycle::Inactive);
  EXPECT_TRUE(check_invariants(model).empty());
}

TEST(Reconciler, a_quarantine_lasts_until_every_abandoned_call_returned)
{
  Model model = started_rig();
  model.controllers.at("wrist").quarantined = 2;
  model.hardware.at("arm").quarantined = 2;
  apply(model, returned::ControllerCallback{"wrist", Lifecycle::Active});
  apply(model, returned::HardwareCallback{"arm", Lifecycle::Active});
  EXPECT_TRUE(model.controllers.at("wrist").quarantined);
  EXPECT_TRUE(model.hardware.at("arm").quarantined);
  apply(model, returned::ControllerCallback{"wrist", Lifecycle::Active});
  apply(model, returned::HardwareCallback{"arm", Lifecycle::Active});
  EXPECT_FALSE(model.controllers.at("wrist").quarantined);
  EXPECT_FALSE(model.hardware.at("arm").quarantined);
}

TEST(Reconciler, every_given_up_call_quarantines_until_it_returned)
{
  Model model = started_rig();
  apply(model, GivenUp{{"wrist", "nav"}, {"arm"}});
  apply(model, GivenUp{{"wrist"}, {"arm"}});
  EXPECT_EQ(model.controllers.at("wrist").quarantined, 2u);
  EXPECT_EQ(model.controllers.at("nav").quarantined, 1u);
  EXPECT_EQ(model.hardware.at("arm").quarantined, 2u);
  apply(model, returned::Released{{"wrist", "nav", "unloaded"}, {}});
  apply(model, returned::ChainedMode{"wrist", std::nullopt});
  apply(model, returned::Released{{}, {"arm"}});
  EXPECT_EQ(model.controllers.at("wrist").quarantined, 0u);
  EXPECT_EQ(model.controllers.at("nav").quarantined, 0u);
  EXPECT_EQ(model.hardware.at("arm").quarantined, 1u);
  apply(model, returned::Released{{}, {"arm"}});
  EXPECT_EQ(model.hardware.at("arm").quarantined, 0u);
}

TEST(Reconciler, a_late_chained_mode_is_recorded_only_if_the_plugin_applied_it)
{
  Model model = started_rig();
  ASSERT_FALSE(model.controllers.at("diff_drive").chained);
  apply(model, GivenUp{{"diff_drive"}, {}});
  apply(model, returned::ChainedMode{"diff_drive", std::nullopt});
  EXPECT_FALSE(model.controllers.at("diff_drive").chained);
  apply(model, GivenUp{{"diff_drive"}, {}});
  apply(model, returned::ChainedMode{"diff_drive", true});
  EXPECT_TRUE(model.controllers.at("diff_drive").chained);
  EXPECT_FALSE(model.controllers.at("diff_drive").quarantined);
}

TEST(Reconciler, a_mode_switch_that_timed_out_faults_and_quarantines_its_hardware)
{
  Model model = started_rig();
  Desired desired = keep(model);
  Outcome outcome;
  outcome.faulted_hardware = {"arm"};
  outcome.timed_out_hardware = {"arm"};
  apply(model, desired, step::Hold{{{"arm", {"a1/velocity"}}}}, outcome);
  EXPECT_EQ(model.hardware.at("arm").quarantined, 1u);
  EXPECT_EQ(model.hardware.at("arm").fault, HardwareFault::Error);
  EXPECT_FALSE(model.hardware.at("flipper").quarantined);
}

TEST(Reconciler, a_refused_stop_ends_finalized_and_releases_claims)
{
  Model model = started_rig();
  const auto plan = plan_of(model, {{}, {"odom_pub"}, Strictness::Strict});
  Desired desired = plan.desired;
  Step step = next_step(model, desired);
  apply(model, desired, step, {});
  step = next_step(model, desired);
  ASSERT_TRUE(std::holds_alternative<step::Deactivate>(step));
  apply(model, desired, step, call_outcome(CallResult::Failure, Lifecycle::Finalized));
  EXPECT_EQ(model.controllers.at("odom_pub").state, Lifecycle::Finalized);
  EXPECT_TRUE(model.controllers.at("odom_pub").claims.state.empty());
  EXPECT_TRUE(std::holds_alternative<Done>(next_step(model, desired)));
}

TEST(Reconciler, a_quarantined_user_does_not_keep_its_provider_running)
{
  Model model = started_rig();
  const auto plan = std::get<SwitchPlan>(plan_switch(
    model, keep(model).hardware, {{}, {"arm_ik", "wrist", "arm_vel"}, Strictness::Strict}));
  Desired desired = plan.desired;
  for (Step step = next_step(model, desired); !std::holds_alternative<step::Deactivate>(step);
       step = next_step(model, desired))
  {
    apply(model, desired, step, {});
  }
  const Step step = next_step(model, desired);
  const auto & name = std::get<step::Deactivate>(step).controller;
  ASSERT_NE(name, "arm_vel");
  apply(model, desired, step, call_outcome(CallResult::TimedOut, Lifecycle::Active));
  for (Step next = next_step(model, desired); !std::holds_alternative<Blocked>(next);
       next = next_step(model, desired))
  {
    ASSERT_FALSE(std::holds_alternative<Done>(next));
    apply(model, desired, next, call_outcome(CallResult::Ok, Lifecycle::Inactive));
    ASSERT_TRUE(check_invariants(model).empty());
  }
  EXPECT_EQ(model.controllers.at("arm_vel").state, Lifecycle::Inactive);
  EXPECT_TRUE(model.hardware.at("arm").live.empty());
  apply(model, returned::ControllerCallback{name, Lifecycle::Inactive});
  settle(model, desired);
}

TEST(Reconciler, a_quarantine_alone_leaves_desired_reached_apart_from_it)
{
  Model model = started_rig();
  Desired desired = keep(model);
  // odom_pub hangs in on_deactivate after a failure
  auto & hung = model.controllers.at("odom_pub");
  hung.quarantined = 1;
  hung.running = false;
  desired.active.erase("odom_pub");
  EXPECT_TRUE(std::holds_alternative<Blocked>(next_step(model, desired)));
  EXPECT_TRUE(reached_apart_from_quarantine(model, desired));
  // A step left to do, here the start of an interface, is not reached
  Model unstarted = model;
  unstarted.hardware.at("flipper").live.clear();
  EXPECT_FALSE(reached_apart_from_quarantine(unstarted, desired));
  desired.active.at("jsb").min_activation += 1;
  EXPECT_FALSE(reached_apart_from_quarantine(model, desired));
}

TEST(Reconciler, a_late_deactivation_clears_the_failure)
{
  Model model = started_rig();
  auto & controller = model.controllers.at("odom_pub");
  controller.failed = true;
  controller.running = false;
  controller.quarantined = 1;
  apply(model, returned::ControllerCallback{"odom_pub", Lifecycle::Inactive});
  EXPECT_FALSE(controller.failed);
  EXPECT_EQ(controller.quarantined, 0u);
}

TEST(Reconciler, a_provider_left_chained_without_a_writer_restarts_unchained)
{
  Model model = started_rig();
  switch_to(model, {"nav"}, {});
  // nav hangs, it is gone from desired and diff_drive stays chained for it
  model.controllers.at("nav").quarantined = 1;
  model.controllers.at("nav").state = Lifecycle::Inactive;
  model.controllers.at("nav").running = false;
  Desired desired = keep(model);
  const uint64_t before = model.controllers.at("diff_drive").activation;
  for (Step step = next_step(model, desired); !std::holds_alternative<Blocked>(step);
       step = next_step(model, desired))
  {
    ASSERT_FALSE(std::holds_alternative<Done>(step));
    apply(model, desired, step, call_outcome(CallResult::Ok, Lifecycle::Active));
    ASSERT_TRUE(check_invariants(model).empty());
  }
  EXPECT_FALSE(model.controllers.at("diff_drive").chained);
  EXPECT_EQ(model.controllers.at("diff_drive").state, Lifecycle::Active);
  EXPECT_GT(model.controllers.at("diff_drive").activation, before);
}

TEST(Reconciler, start_skips_interfaces_of_a_claimer_that_failed)
{
  Model model = rig();
  const auto plan = plan_of(model, {{"flipper_vel"}, {}, Strictness::Strict});
  Desired desired = plan.desired;
  const Step step = until_switch_modes(model, desired);
  Outcome skipped = call_outcome(CallResult::Ok, Lifecycle::Active);
  skipped.skipped = {"f1/velocity"};
  apply(model, desired, step, skipped);
  EXPECT_TRUE(model.hardware.at("flipper").live.empty());
}

TEST(Reconciler, a_failed_activation_is_not_retried)
{
  Model model = rig();
  const auto plan = plan_of(model, {{"arm_vel", "arm_ik"}, {}, Strictness::Strict});
  Desired desired = plan.desired;
  size_t activations = 0;
  for (size_t i = 0; i < 50; ++i)
  {
    const Step step = next_step(model, desired);
    if (std::holds_alternative<Done>(step) || std::holds_alternative<Blocked>(step))
    {
      break;
    }
    Outcome outcome;
    if (const auto * activate = std::get_if<step::Activate>(&step))
    {
      ++activations;
      outcome = activate->controller == "arm_vel"
                  ? call_outcome(CallResult::Failure, Lifecycle::Inactive)
                  : call_outcome(CallResult::Ok, Lifecycle::Active);
    }
    apply(model, desired, step, outcome);
  }
  EXPECT_EQ(activations, 1u);
  EXPECT_TRUE(desired.active.empty());
  EXPECT_TRUE(std::holds_alternative<Done>(next_step(model, desired)));
}

TEST(Reconciler, terminates_when_every_chained_mode_change_is_refused)
{
  Model model = started_rig();
  const auto plan = plan_of(model, {{"whole_body"}, {}, Strictness::Strict});
  Desired desired = plan.desired;
  size_t steps = 0;
  for (Step step = next_step(model, desired);
       !std::holds_alternative<Done>(step) && !std::holds_alternative<Blocked>(step) && steps < 200;
       step = next_step(model, desired), ++steps)
  {
    Outcome outcome;
    outcome.state = Lifecycle::Active;
    if (std::holds_alternative<step::SetChained>(step))
    {
      outcome.result = CallResult::Failure;
    }
    apply(model, desired, step, outcome);
    ASSERT_TRUE(check_invariants(model).empty());
  }
  EXPECT_LT(steps, 200u);
  EXPECT_TRUE(std::holds_alternative<Done>(next_step(model, desired)));
}

TEST(Reconciler, a_failed_hardware_activation_settles_in_the_state_it_reached)
{
  Model model = rig();
  model.hardware.at("flipper").state = Lifecycle::Inactive;
  Desired desired = keep(model);
  desired.hardware["flipper"] = Lifecycle::Active;
  const Step step = next_step(model, desired);
  ASSERT_TRUE(std::holds_alternative<step::Hardware>(step));
  EXPECT_EQ(
    apply(model, desired, step, call_outcome(CallResult::Failure, Lifecycle::Inactive)).unusable,
    (Names{"flipper"}));
  EXPECT_TRUE(std::holds_alternative<Done>(next_step(model, desired)));
}

TEST(Reconciler, invariants_catch_two_live_modes_of_one_joint)
{
  Model model = started_rig();
  auto & velocity = model.controllers.at("flipper_vel");
  velocity.state = Lifecycle::Active;
  velocity.running = velocity.updated = true;
  velocity.claims = {{"f1/velocity"}, {}};
  model.hardware.at("flipper").live.insert("f1/velocity");
  const auto violations = check_invariants(model);
  ASSERT_EQ(violations.size(), 1u);
  EXPECT_NE(violations.front().find("joint f1"), std::string::npos);
}

/// Mode changes per command interface, every time a perform starts or stops it.
std::map<std::string, size_t> mode_changes(Model & model, const SwitchRequest & request)
{
  const auto plan = plan_of(model, request);
  std::map<std::string, size_t> changes;
  for (const auto & step : settle(model, plan.desired))
  {
    if (const auto * hold = std::get_if<step::Hold>(&step))
    {
      for (const auto & [hw, interfaces] : hold->interfaces)
      {
        for (const auto & interface : interfaces)
        {
          ++changes[interface];
        }
      }
    }
    if (const auto * switch_modes = std::get_if<step::SwitchModes>(&step))
    {
      for (const auto & [hw, modes] : switch_modes->modes)
      {
        for (const auto * list : {&modes.start, &modes.stop})
        {
          for (const auto & interface : *list)
          {
            ++changes[interface];
          }
        }
      }
    }
  }
  return changes;
}

/// Starts and stops on and off chains of the started rig, one after another.
std::vector<SwitchRequest> switch_sequence()
{
  return {
    {{"nav"}, {}, Strictness::Strict},        {{}, {"nav"}, Strictness::Strict},
    {{}, {"wrist"}, Strictness::Strict},      {{"wrist"}, {}, Strictness::Strict},
    {{"whole_body"}, {}, Strictness::Strict}, {{"teleop"}, {}, Strictness::Strict},
  };
}

TEST(Reconciler, a_switch_changes_each_command_interface_at_most_twice)
{
  Model model = started_rig();
  auto requests = switch_sequence();
  requests.push_back({{"flipper_vel"}, {}, Strictness::ForceAuto});
  size_t changed = 0;
  for (const auto & request : requests)
  {
    const auto changes = mode_changes(model, request);
    changed += changes.size();
    for (const auto & [interface, count] : changes)
    {
      EXPECT_LE(count, 2u) << interface << " in +" << text(request.activate) << " -"
                           << text(request.deactivate);
    }
  }
  EXPECT_GT(changed, 0u) << "precondition: a request changed a mode";
}

/// perform calls per hardware component during a switch.
std::map<std::string, size_t> performs(Model & model, const SwitchRequest & request)
{
  const auto plan = plan_of(model, request);
  std::map<std::string, size_t> calls;
  for (const auto & step : settle(model, plan.desired))
  {
    for (const auto & hardware : testing::performed_on(step))
    {
      ++calls[hardware];
    }
  }
  return calls;
}

TEST(Reconciler, a_switch_performs_at_most_once_per_component)
{
  Model model = started_rig();
  auto requests = switch_sequence();
  requests.push_back({{"flipper_vel"}, {}, Strictness::ForceAuto});
  requests.push_back({{"arm_pos"}, {}, Strictness::ForceAuto});
  size_t performed = 0;
  for (const auto & request : requests)
  {
    const auto calls = performs(model, request);
    performed += calls.size();
    for (const auto & [hardware, count] : calls)
    {
      EXPECT_LE(count, 1u) << hardware << " in +" << text(request.activate) << " -"
                           << text(request.deactivate);
    }
  }
  EXPECT_GT(performed, 0u) << "precondition: a request performed";
  // A hand over on one joint while the claimer of another joint stops without a successor
  Model bus;
  testing::add_hardware(bus, "bus", {"j1", "j2"}, {"position", "velocity"}, {});
  testing::add_controller(bus, {"vel", false, {"j1/velocity"}, {}, {}, {}});
  testing::add_controller(bus, {"pos", false, {"j1/position"}, {}, {}, {}});
  testing::add_controller(bus, {"j2", false, {"j2/velocity"}, {}, {}, {}});
  switch_to(bus, {"vel", "j2"}, {});
  EXPECT_EQ(
    performs(bus, {{"pos"}, {"vel", "j2"}, Strictness::Strict}),
    (std::map<std::string, size_t>{{"bus", 1}}));
  EXPECT_EQ(bus.hardware.at("bus").live, (Names{"j1/position"}));
}

TEST(Reconciler, a_restart_on_the_same_interfaces_performs_nothing)
{
  Model model = started_rig();
  auto restarts = switch_sequence();
  restarts.insert(restarts.begin(), {{"arm_vel"}, {"arm_vel"}, Strictness::Strict});
  for (const auto & request : restarts)
  {
    EXPECT_EQ(performs(model, request), (std::map<std::string, size_t>{}))
      << "+" << text(request.activate) << " -" << text(request.deactivate);
  }
}

TEST(Reconciler, a_hand_over_to_another_mode_of_a_joint_performs_once)
{
  Model model = started_rig();
  const auto plan = plan_of(model, {{"flipper_vel"}, {}, Strictness::ForceAuto});
  size_t calls = 0;
  for (const auto & step : settle(model, plan.desired))
  {
    calls += testing::performed_on(step).count("flipper");
  }
  EXPECT_EQ(calls, 1u);
  EXPECT_EQ(model.hardware.at("flipper").live, (Names{"f1/velocity"}));
}

TEST(Reconciler, a_restart_next_to_a_claimer_that_keeps_running_holds_and_starts)
{
  Model model;
  testing::add_hardware(model, "drive", {"j1", "j2"}, {"velocity"}, {});
  testing::add_controller(model, {"c1", false, {"j1/velocity"}, {}, {}, {}});
  testing::add_controller(model, {"c2", false, {"j2/velocity"}, {}, {}, {}});
  switch_to(model, {"c1", "c2"}, {});
  // A take would block c2's writes for every lifecycle callback of the restart
  EXPECT_EQ(
    performs(model, {{"c1"}, {"c1"}, Strictness::Strict}),
    (std::map<std::string, size_t>{{"drive", 2}}));
}

TEST(Reconciler, a_start_failing_on_one_component_still_starts_the_others)
{
  Model model = rig();
  const auto plan = plan_of(model, {startup_active(), {}, Strictness::Strict});
  Desired desired = plan.desired;
  const Step step = until_switch_modes(model, desired);
  Outcome outcome;
  outcome.faulted_hardware = {"base"};
  apply(model, desired, step, outcome);
  settle(model, desired);
  EXPECT_FALSE(model.hardware.at("arm").live.empty());
  EXPECT_FALSE(model.hardware.at("flipper").live.empty());
  EXPECT_TRUE(model.hardware.at("base").live.empty());
  EXPECT_EQ(model.controllers.at("base_vel").state, Lifecycle::Inactive);
  EXPECT_EQ(model.controllers.at("arm_vel").state, Lifecycle::Active);
}

TEST(Reconciler, invariants_catch_a_live_interface_without_claimer)
{
  Model model = started_rig();
  model.hardware.at("flipper").live.insert("f1/velocity");
  EXPECT_EQ(
    check_invariants(model),
    std::vector<std::string>{"f1/velocity of flipper is live without an updated claimer"});
}

TEST(Reconciler, invariants_catch_a_controller_that_runs_while_not_active)
{
  Model model = started_rig();
  model.controllers.at("monitor").running = true;
  EXPECT_EQ(check_invariants(model), std::vector<std::string>{"monitor runs while not active"});
}

TEST(Reconciler, invariants_catch_a_command_interface_with_two_claimers)
{
  Model model = started_rig();
  auto & rival = model.controllers.at("flipper_vel");
  rival.state = Lifecycle::Active;
  rival.claims = {{"f1/position"}, {}};
  EXPECT_EQ(
    check_invariants(model),
    std::vector<std::string>{"f1/position is claimed by flipper_pos and flipper_vel"});
}

TEST(Reconciler, invariants_catch_an_active_controller_over_an_inactive_provider)
{
  Model model = started_rig();
  auto & provider = model.controllers.at("diff_drive");
  provider.state = Lifecycle::Inactive;
  provider.running = false;
  provider.claims = {};
  EXPECT_EQ(
    check_invariants(model),
    std::vector<std::string>{"odom_pub is active over inactive provider diff_drive"});
}

// The deadline of an AwaitUpdate passed before RT updated the new claimer, so its interface waits
TEST(Reconciler, an_interface_starts_only_once_rt_updated_its_claimer)
{
  Model model = started_rig();
  Desired desired = plan_of(model, {{"flipper_vel"}, {}, Strictness::ForceAuto}).desired;
  Step step = next_step(model, desired);
  for (int steps = 0; !std::holds_alternative<step::AwaitUpdate>(step); ++steps)
  {
    ASSERT_LT(steps, 50) << "no AwaitUpdate, the last step is " << describe(step);
    apply(model, desired, step, testing::outcome_for(step));
    step = next_step(model, desired);
  }
  Outcome not_updated;
  not_updated.not_updated = true;
  apply(model, desired, step, not_updated);
  EXPECT_FALSE(model.controllers.at("flipper_vel").updated);
  step = next_step(model, desired);
  ASSERT_TRUE(std::holds_alternative<step::AwaitUpdate>(step)) << describe(step);
  EXPECT_FALSE(model.hardware.at("flipper").live.count("f1/velocity"));
  apply(model, desired, step, {});
  EXPECT_TRUE(std::holds_alternative<step::SwitchModes>(next_step(model, desired)));
  settle(model, desired);
  EXPECT_EQ(model.hardware.at("flipper").live, (Names{"f1/velocity"}));
}

// The late success of a requested activation is undone toward the state it started from
TEST(Reconciler, a_hardware_transition_past_its_deadline_quarantines_until_it_returned)
{
  Model model = rig();
  model.hardware.at("imu").state = Lifecycle::Inactive;
  Desired desired = keep(model);
  desired.hardware["imu"] = Lifecycle::Active;
  const Step step = next_step(model, desired);
  ASSERT_TRUE(std::holds_alternative<step::Hardware>(step)) << describe(step);
  const auto failures =
    apply(model, desired, step, call_outcome(CallResult::TimedOut, Lifecycle::Unconfigured));
  EXPECT_EQ(failures.unusable, (Names{"imu"}));
  EXPECT_EQ(model.hardware.at("imu").quarantined, 1u);
  EXPECT_EQ(model.hardware.at("imu").state, Lifecycle::Inactive);
  EXPECT_TRUE(reached_apart_from_quarantine(model, desired));
  apply(model, returned::HardwareCallback{"imu", Lifecycle::Active});
  EXPECT_EQ(model.hardware.at("imu").quarantined, 0u);
  const auto steps = settle(model, desired);
  ASSERT_EQ(steps.size(), 1u);
  const auto * undo = std::get_if<step::Hardware>(&steps.front());
  ASSERT_TRUE(undo) << describe(steps.front());
  EXPECT_EQ(undo->transition, step::HardwareTransition::Deactivate);
  EXPECT_EQ(model.hardware.at("imu").state, Lifecycle::Inactive);
}

/// The hardware transitions that bring imu, which nothing claims, from one state to another.
std::vector<step::HardwareTransition> imu_transitions(Lifecycle from, Lifecycle to)
{
  Model model = rig();
  model.hardware.at("imu").state = from;
  Desired desired = keep(model);
  desired.hardware["imu"] = to;
  std::vector<step::HardwareTransition> transitions;
  for (const auto & step : settle(model, desired))
  {
    const auto * hw = std::get_if<step::Hardware>(&step);
    EXPECT_TRUE(hw && hw->hardware == "imu") << describe(step);
    if (hw)
    {
      transitions.push_back(hw->transition);
    }
  }
  EXPECT_EQ(model.hardware.at("imu").state, to);
  return transitions;
}

TEST(Reconciler, a_hardware_target_is_reached_through_configure_cleanup_or_shutdown)
{
  using T = step::HardwareTransition;
  EXPECT_EQ(
    imu_transitions(Lifecycle::Unconfigured, Lifecycle::Active),
    (std::vector<T>{T::Configure, T::Activate}));
  EXPECT_EQ(
    imu_transitions(Lifecycle::Active, Lifecycle::Unconfigured),
    (std::vector<T>{T::Deactivate, T::Cleanup}));
  EXPECT_EQ(
    imu_transitions(Lifecycle::Active, Lifecycle::Finalized),
    (std::vector<T>{T::Deactivate, T::Shutdown}));
  EXPECT_EQ(
    imu_transitions(Lifecycle::Unconfigured, Lifecycle::Finalized), (std::vector<T>{T::Shutdown}));
}

}  // namespace
}  // namespace hector_controller_manager::core
