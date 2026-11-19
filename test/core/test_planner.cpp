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

#include "fixtures.hpp"
#include "hector_controller_manager/core/names.hpp"
#include "hector_controller_manager/core/planner.hpp"

namespace hector_controller_manager::core
{
namespace
{
using testing::active_names;
using testing::plan_of;
using testing::rig;
using testing::started_rig;
using testing::startup_active;
using testing::switch_to;

std::string rejection_of(const Model & model, const SwitchRequest & request)
{
  const auto result = plan_switch(model, keep(model).hardware, request);
  const auto * rejection = std::get_if<Rejection>(&result);
  return rejection ? rejection->reason : std::string();
}

TEST(Planner, startup_set_activates_in_one_switch)
{
  const Model model = started_rig();
  EXPECT_EQ(active_names(model), startup_active());
  EXPECT_TRUE(model.controllers.at("arm_vel").chained);
  EXPECT_FALSE(model.controllers.at("whole_body").chained);
}

TEST(Planner, auto_adds_inactive_providers)
{
  const Model model = rig();
  const auto plan = plan_of(model, {{"teleop"}, {}, Strictness::Auto});
  EXPECT_EQ(
    plan.started, (Names{
                    "teleop", "whole_body", "arm_ik", "wrist", "arm_vel", "diff_drive", "base_vel",
                    "flipper_pos"}));
}

// The autostart skips a controller whose expansion needs a skipped provider
TEST(Planner, the_auto_start_set_holds_every_provider_that_is_not_running)
{
  Model model = rig();
  EXPECT_EQ(
    std::get<Names>(auto_start_set(model, "whole_body")),
    (Names{"whole_body", "arm_ik", "arm_vel", "wrist", "diff_drive", "base_vel", "flipper_pos"}));
  switch_to(model, {"arm_vel"}, {});
  EXPECT_EQ(std::get<Names>(auto_start_set(model, "arm_ik")), (Names{"arm_ik"}));
  model.controllers.at("wrist").state = Lifecycle::Unconfigured;
  EXPECT_TRUE(std::holds_alternative<Rejection>(auto_start_set(model, "whole_body")));
  EXPECT_TRUE(std::holds_alternative<Rejection>(auto_start_set(model, "wrist")));
}

TEST(Planner, strict_rejects_a_missing_provider)
{
  const Model model = rig();
  const auto reason = rejection_of(model, {{"teleop"}, {}, Strictness::Strict});
  EXPECT_NE(reason.find("whole_body"), std::string::npos) << reason;
}

TEST(Planner, auto_rejects_stopping_a_provider_that_is_still_used)
{
  const Model model = started_rig();
  const auto reason = rejection_of(model, {{}, {"diff_drive"}, Strictness::Auto});
  EXPECT_NE(reason.find("odom_pub"), std::string::npos) << reason;
  EXPECT_NE(
    rejection_of(model, {{}, {"arm_vel"}, Strictness::Auto}).find("arm_ik"), std::string::npos);
}

TEST(Planner, auto_rejects_a_conflict_and_force_auto_takes_over)
{
  Model model = started_rig();
  switch_to(model, {"nav"}, {});
  const auto reason = rejection_of(model, {{"whole_body"}, {}, Strictness::Auto});
  EXPECT_NE(reason.find("nav"), std::string::npos) << reason;
  const auto plan = plan_of(model, {{"whole_body"}, {}, Strictness::ForceAuto});
  EXPECT_TRUE(plan.stopped.count("nav"));
  EXPECT_TRUE(plan.desired.active.count("whole_body"));
  EXPECT_FALSE(plan.desired.active.count("nav"));
}

TEST(Planner, force_auto_stops_another_mode_of_the_same_joint)
{
  const Model model = started_rig();
  const auto plan = plan_of(model, {{"flipper_vel"}, {}, Strictness::ForceAuto});
  EXPECT_TRUE(plan.stopped.count("flipper_pos"));
  EXPECT_FALSE(plan.desired.active.count("flipper_pos"));
}

TEST(Planner, force_auto_stops_the_users_of_what_it_stops)
{
  const Model model = started_rig();
  const auto plan = plan_of(model, {{"arm_pos"}, {}, Strictness::ForceAuto});
  EXPECT_EQ(
    intersection(plan.stopped, {"arm_vel", "arm_ik", "wrist"}),
    (Names{"arm_vel", "arm_ik", "wrist"}));
}

TEST(Planner, force_auto_does_not_stop_a_protected_controller)
{
  Model model = started_rig();
  model.controllers.at("flipper_pos").is_protected = true;
  const auto reason = rejection_of(model, {{"flipper_vel"}, {}, Strictness::ForceAuto});
  EXPECT_NE(reason.find("protected 'flipper_pos'"), std::string::npos) << reason;
  const auto plan = plan_of(model, {{"flipper_vel"}, {"flipper_pos"}, Strictness::ForceAuto});
  EXPECT_TRUE(plan.stopped.count("flipper_pos"));
}

TEST(Planner, a_chain_does_not_restart_a_protected_provider)
{
  Model model = started_rig();
  model.controllers.at("base_vel").is_protected = true;
  const auto reason = rejection_of(model, {{"nav"}, {}, Strictness::Strict});
  EXPECT_NE(reason.find("protected 'base_vel'"), std::string::npos) << reason;
}

TEST(Planner, chained_mode_change_restarts_the_provider_and_its_dependents)
{
  const Model model = started_rig();
  const auto plan = plan_of(model, {{"nav"}, {}, Strictness::Strict});
  EXPECT_EQ(plan.started, (Names{"nav", "diff_drive", "odom_pub", "base_vel"}));
  EXPECT_EQ(plan.stopped, (Names{"diff_drive", "odom_pub", "base_vel"}));
}

TEST(Planner, departed_writer_restarts_the_provider_that_stays_chained)
{
  const Model model = started_rig();
  const auto plan = plan_of(model, {{}, {"wrist"}, Strictness::Strict});
  EXPECT_EQ(plan.stopped, (Names{"wrist", "arm_vel", "arm_ik"}));
  EXPECT_EQ(plan.started, (Names{"arm_vel", "arm_ik"}));
}

TEST(Planner, joining_writer_restarts_nothing)
{
  Model model = started_rig();
  switch_to(model, {}, {"wrist"});
  const auto plan = plan_of(model, {{"wrist"}, {}, Strictness::Strict});
  EXPECT_EQ(plan.started, (Names{"wrist"}));
  EXPECT_EQ(plan.stopped, Names{});
}

TEST(Planner, attaching_a_reader_restarts_nothing)
{
  Model model = started_rig();
  switch_to(model, {}, {"odom_pub"});
  const auto plan = plan_of(model, {{"odom_pub"}, {}, Strictness::Strict});
  EXPECT_EQ(plan.started, (Names{"odom_pub"}));
  EXPECT_EQ(plan.stopped, Names{});
}

TEST(Planner, explicit_restart_restarts_the_controller_and_its_dependents)
{
  const Model model = started_rig();
  const auto plan = plan_of(model, {{"base_vel"}, {"base_vel"}, Strictness::Strict});
  EXPECT_EQ(plan.started, (Names{"base_vel", "diff_drive", "odom_pub"}));
  EXPECT_GT(
    plan.desired.active.at("base_vel").min_activation, model.controllers.at("base_vel").activation);
}

TEST(Planner, unchaining_a_provider_restarts_it)
{
  Model model = started_rig();
  switch_to(model, {"whole_body"}, {});
  const auto plan = plan_of(model, {{}, {"whole_body"}, Strictness::Strict});
  EXPECT_TRUE(plan.started.count("diff_drive"));
  EXPECT_TRUE(plan.desired.active.count("diff_drive"));
  EXPECT_FALSE(plan.desired.active.count("whole_body"));
}

TEST(Planner, rejection_names_the_unknown_controller)
{
  const Model model = rig();
  EXPECT_NE(
    rejection_of(model, {{"ghost"}, {}, Strictness::Strict}).find("ghost"), std::string::npos);
}

TEST(Planner, rejects_hardware_that_is_not_active)
{
  Model model = rig();
  model.hardware.at("arm").state = Lifecycle::Inactive;
  EXPECT_NE(
    rejection_of(model, {{"arm_pos"}, {}, Strictness::Strict}).find("arm"), std::string::npos);
}

TEST(Planner, a_switch_keeps_the_target_of_a_pending_hardware_fault)
{
  Model model = rig();
  model.hardware.at("arm").fault = HardwareFault::Error;
  Desired desired = keep(model);
  desired.hardware.at("arm") = Lifecycle::Unconfigured;
  const auto planned =
    plan_switch(model, desired.hardware, {{"flipper_pos"}, {}, Strictness::Strict});
  ASSERT_TRUE(std::holds_alternative<SwitchPlan>(planned));
  EXPECT_EQ(std::get<SwitchPlan>(planned).desired.hardware.at("arm"), Lifecycle::Unconfigured);
}

TEST(Planner, rejects_a_component_taken_for_a_quarantined_controller)
{
  Model model;
  testing::add_hardware(model, "bus", {"j1", "j2"}, {"velocity"}, {});
  testing::add_controller(model, {"c1", false, {"j1/velocity"}, {}, {}, {}});
  testing::add_controller(model, {"c2", false, {"j2/velocity"}, {}, {}, {}});
  switch_to(model, {"c1"}, {});
  // A hand over took the bus, and RT is stuck in an update of the detached c1
  auto & hung = model.controllers.at("c1");
  hung.running = false;
  hung.quarantined = 1;
  model.hardware.at("bus").taken = true;
  EXPECT_NE(
    rejection_of(model, {{"c2"}, {}, Strictness::Strict}).find("quarantined controller 'c1'"),
    std::string::npos);
}

TEST(Planner, best_effort_skips_unavailable_states)
{
  Model model = rig();
  model.hardware.at("flipper").state = Lifecycle::Unconfigured;
  const auto plan = plan_of(model, {{"jsb"}, {}, Strictness::Strict});
  const auto & state = plan.desired.active.at("jsb").claims.state;
  EXPECT_EQ(
    state, (std::vector<std::string>{
             "a1/position", "a2/position", "a3/position", "wl/position", "wr/position"}));
}

TEST(Planner, a_reader_that_starts_again_reads_the_states_available_now)
{
  Model model = rig();
  model.hardware.at("flipper").state = Lifecycle::Unconfigured;
  Desired desired = plan_of(model, {{"jsb"}, {}, Strictness::Strict}).desired;
  model.hardware.at("flipper").state = Lifecycle::Inactive;
  EXPECT_TRUE(resolve_starting(model, desired).empty());
  EXPECT_EQ(
    desired.active.at("jsb").claims.state,
    (std::vector<std::string>{
      "a1/position", "a2/position", "a3/position", "wl/position", "wr/position", "f1/position"}));
}

TEST(Planner, rejects_a_shared_command_interface)
{
  const Model model = started_rig();
  const auto reason = rejection_of(model, {{"nav", "whole_body"}, {}, Strictness::Strict});
  EXPECT_NE(reason.find("diff_drive/vx"), std::string::npos) << reason;
}

TEST(Planner, validation_lists_the_mode_switch_of_each_hardware)
{
  const Model model = started_rig();
  const auto plan = plan_of(model, {{"flipper_vel"}, {"flipper_pos"}, Strictness::Strict});
  EXPECT_EQ(plan.validation.at("flipper").start, (std::vector<std::string>{"f1/velocity"}));
  EXPECT_EQ(plan.validation.at("flipper").stop, (std::vector<std::string>{"f1/position"}));
}

TEST(Planner, rejects_two_modes_on_one_joint)
{
  const Model model = started_rig();
  EXPECT_NE(
    rejection_of(model, {{"flipper_vel"}, {}, Strictness::Strict}).find("f1"), std::string::npos);
  EXPECT_NE(rejection_of(model, {{"arm_pos"}, {}, Strictness::Auto}).find("a1"), std::string::npos);
}

TEST(Planner, one_controller_may_command_several_interfaces_of_a_joint)
{
  // Like GripperPositionEffortController, position plus effort of one joint
  Model model;
  testing::add_hardware(model, "gripper", {"g"}, {"position", "effort"}, {"g/position"});
  testing::add_controller(
    model, {"grip", false, {"g/position", "g/effort"}, {"g/position"}, {}, {}});
  const auto plan = plan_of(model, {{"grip"}, {}, Strictness::Strict});
  EXPECT_EQ(plan.started, (Names{"grip"}));
}

TEST(Planner, spawner_switches_one_controller_per_call)
{
  Model model = started_rig();
  switch_to(model, {}, {"odom_pub", "diff_drive"});
  switch_to(model, {}, {"base_vel"});
  switch_to(model, {"base_vel"}, {});
  switch_to(model, {"diff_drive"}, {});
  switch_to(model, {"whole_body"}, {});
  switch_to(model, {"teleop"}, {});
  switch_to(model, {}, {"teleop"});
  switch_to(model, {}, {"whole_body"});
  EXPECT_EQ(active_names(model), without(startup_active(), {"odom_pub"}));
}

TEST(Planner, auto_rejects_a_needed_provider_named_to_stop_even_if_inactive)
{
  Model model = started_rig();
  switch_to(model, {"arm_pos"}, {"arm_ik", "wrist", "arm_vel"});
  const auto needed = rejection_of(model, {{"wrist"}, {"arm_vel", "arm_pos"}, Strictness::Auto});
  EXPECT_NE(needed.find("arm_vel"), std::string::npos) << needed;
}

TEST(Planner, force_auto_rejects_stopping_what_the_request_activates)
{
  const Model model = started_rig();
  const auto reason =
    rejection_of(model, {{"diff_drive"}, {"arm_vel", "base_vel"}, Strictness::ForceAuto});
  EXPECT_NE(reason.find("which the request activates"), std::string::npos) << reason;
}

TEST(Planner, rejects_interfaces_a_quarantined_controller_holds)
{
  Model model = started_rig();
  switch_to(model, {}, {"flipper_pos"});
  auto & quarantined = model.controllers.at("flipper_vel");
  quarantined.quarantined = 1;
  quarantined.claims = {{"f1/velocity"}, {}};
  EXPECT_NE(
    rejection_of(model, {{"flipper_pos"}, {}, Strictness::Strict}).find("quarantined"),
    std::string::npos);
}

TEST(Planner, rejects_starting_a_provider_a_quarantined_controller_writes)
{
  Model model = rig();
  auto & hung = model.controllers.at("nav");
  hung.quarantined = 1;
  hung.claims = {{"diff_drive/vx", "diff_drive/wz"}, {}};
  const auto reason = rejection_of(model, {{"diff_drive", "base_vel"}, {}, Strictness::Strict});
  EXPECT_NE(reason.find("Quarantined 'nav'"), std::string::npos) << reason;
}

TEST(Planner, a_best_effort_reader_of_a_leaving_provider_restarts_without_its_state)
{
  Model model;
  testing::add_hardware(model, "hw", {"j"}, {"position"}, {"j/position"});
  testing::add_controller(model, {"provider", true, {"j/position"}, {}, {}, {"s"}});
  testing::add_controller(model, {"reader", false, {}, {"provider/s", "j/position"}, {}, {}, true});
  switch_to(model, {"provider", "reader"}, {});
  const auto plan = plan_of(model, {{}, {"provider"}, Strictness::Strict});
  EXPECT_EQ(plan.started, (Names{"reader"}));
  EXPECT_EQ(
    plan.desired.active.at("reader").claims.state, (std::vector<std::string>{"j/position"}));
}

TEST(Planner, rejects_a_cycle_through_a_write_and_a_read)
{
  Model model;
  testing::add_controller(model, {"a", true, {"b/r"}, {}, {}, {"s"}});
  testing::add_controller(model, {"b", true, {}, {"a/s"}, {"r"}, {}});
  const auto reason = rejection_of(model, {{"a", "b"}, {}, Strictness::Strict});
  EXPECT_NE(reason.find("cycle"), std::string::npos) << reason;
}

TEST(Planner, a_writer_may_read_the_provider_it_writes)
{
  Model model;
  testing::add_controller(model, {"a", false, {"b/r"}, {"b/s"}, {}, {}});
  testing::add_controller(model, {"b", true, {}, {}, {"r"}, {"s"}});
  const auto plan = plan_of(model, {{"a", "b"}, {}, Strictness::Strict});
  EXPECT_EQ(plan.started, (Names{"a", "b"}));
}

TEST(Planner, deactivating_an_inactive_controller_is_a_no_operation)
{
  const Model model = started_rig();
  const auto plan = plan_of(model, {{}, {"nav"}, Strictness::Strict});
  EXPECT_TRUE(plan.stopped.empty());
  EXPECT_TRUE(plan.started.empty());
}

}  // namespace
}  // namespace hector_controller_manager::core
