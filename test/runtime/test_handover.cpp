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

// The handover of command interfaces in a switch, on sync and async hardware. How many mode
// switches it performs, and that no command of an ended activation ever reaches the hardware.

#include <gtest/gtest.h>

#include <future>
#include <string>
#include <utility>
#include <vector>

#include "token_rig.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace testing::tokens;
using testing::eventually;

class Handover : public TokenRig
{
protected:
  void restart_writing_nothing_at_first(bool command_left_behind);
  void rejected_while_vel_b_waits_for_rt(core::Names activate, core::Names deactivate);
  void nan_write_during_a_hand_over(const std::string & successor, size_t interface);
  /// The single mode switch of a handover fails, the switch rolls back.
  void fail_the_mode_switch_of_a_hand_over();
  /// A later activation that writes nothing at first gets NaN, not the failed one's command.
  void start_after_a_failed_activation(CallbackReturn failure);
};

TEST_P(Handover, a_restart_on_the_same_interface_performs_no_mode_switch)
{
  ASSERT_TRUE(running({"vel"}));
  ASSERT_EQ(observed_.performs.load(), 1);
  const auto response = switch_controllers({"vel"}, {"vel"});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
  EXPECT_EQ(observed_.performs.load(), 1);
}

TEST_P(Handover, a_hand_over_to_another_mode_of_the_joint_performs_once)
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  const auto response = switch_controllers({"pos"}, {"vel"});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_POSITION));
  const auto performed = calls();
  ASSERT_EQ(performed.size(), 2u);
  EXPECT_EQ(performed.back().start, (std::vector<std::string>{"j1/position"}));
  EXPECT_EQ(performed.back().stop, (std::vector<std::string>{"j1/velocity"}));
}

// vel hands j1 over to pos while j2 stops without a successor, both in the single mode switch
TEST_P(Handover, stopping_a_second_joint_in_a_hand_over_performs_once)
{
  add("pos");
  ASSERT_TRUE(running({"vel", "j2"}));
  const int before = observed_.performs.load();
  const auto response = switch_controllers({"pos"}, {"vel", "j2"});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_POSITION));
  EXPECT_EQ(observed_.performs.load(), before + 1);
}

/// The new activation writes nothing at first, the ended one may leave a command behind.
void Handover::restart_writing_nothing_at_first(bool command_left_behind)
{
  ASSERT_TRUE(running({"vel"}));
  observed_.skip_first_updates = 20;
  observed_.write_on_deactivate = command_left_behind;
  const auto response = switch_controllers({"vel"}, {"vel"});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

TEST_P(Handover, a_restart_applies_no_command_of_the_ended_activation)
{
  restart_writing_nothing_at_first(true);
}

TEST_P(Handover, a_restart_without_a_command_left_behind_applies_nothing_old)
{
  restart_writing_nothing_at_first(false);
}

/// vel_b took over from vel while RT pauses before vel_b's first update. The request waits for that
/// update and is rejected as busy.
void Handover::rejected_while_vel_b_waits_for_rt(core::Names activate, core::Names deactivate)
{
  add("vel_b");
  ASSERT_TRUE(running({"vel"}));
  observed_.activate_gate.close();
  auto first =
    std::async(std::launch::async, [&] { return switch_controllers({"vel_b"}, {"vel"}, 400ms); });
  ASSERT_TRUE(observed_.activate_gate.holds());
  pause_rt_ = true;
  observed_.activate_gate.open();
  first.get();
  EXPECT_EQ(
    switch_controllers(std::move(activate), std::move(deactivate), 300ms).message,
    RequestQueue::busy().message);
  pause_rt_ = false;
  EXPECT_TRUE(eventually([&] { return !hardware().taken && !hardware().quarantined; }));
  EXPECT_EQ(hardware().fault, core::HardwareFault::None);
}

TEST_P(Handover, a_request_without_time_left_for_validation_is_rejected_as_busy)
{
  rejected_while_vel_b_waits_for_rt({"vel"}, {"vel_b"});
}

// Without a mode switch to validate it still waits for RT
TEST_P(Handover, a_request_whose_leftover_does_not_settle_is_rejected_as_busy)
{
  rejected_while_vel_b_waits_for_rt({}, {});
}

/// While the successor's activation is held, the released interface gets exactly one NaN write.
void Handover::nan_write_during_a_hand_over(const std::string & successor, size_t interface)
{
  if (successor != "vel")
  {
    add(successor);
  }
  ASSERT_TRUE(running({"vel"}));
  ASSERT_EQ(observed_.nan_writes[J1_VELOCITY].load(), 0);
  observed_.activate_gate.close();
  auto response =
    std::async(std::launch::async, [&] { return switch_controllers({successor}, {"vel"}); });
  ASSERT_TRUE(observed_.activate_gate.holds());
  EXPECT_TRUE(eventually([&] { return observed_.nan_writes[J1_VELOCITY].load() > 0; }));
  observed_.activate_gate.open();
  EXPECT_TRUE(response.get().ok);
  EXPECT_TRUE(applies_newest(interface));
  EXPECT_EQ(observed_.nan_writes[J1_VELOCITY].load(), 1);
}

TEST_P(Handover, a_restart_writes_nan_once_instead_of_the_last_command)
{
  nan_write_during_a_hand_over("vel", J1_VELOCITY);
}

TEST_P(Handover, a_hand_over_to_another_mode_writes_nan_once_instead_of_the_last_command)
{
  nan_write_during_a_hand_over("pos", J1_POSITION);
}

// RT is stuck in vel's update when vel and j2 hand the component over. The NaN write after the
// first deactivation must wait for the second, vel's late command is never applied.
TEST_P(Handover, the_nan_write_waits_for_every_claimer_that_hands_over)
{
  add("pos");
  ASSERT_TRUE(running({"vel", "j2"}));
  observed_.stall_j1_update = true;
  ASSERT_TRUE(stalled());
  auto response =
    std::async(std::launch::async, [&] { return switch_controllers({"pos"}, {"vel", "j2"}); });
  ASSERT_TRUE(eventually([&] { return hardware().taken; }));
  observed_.stall_j1_update = false;
  EXPECT_TRUE(response.get().ok);
  EXPECT_TRUE(applies_newest(J1_POSITION));
  EXPECT_EQ(observed_.late_applied.load(), 0);
}

// vel's on_deactivate hangs in a handover to pos, so the joint is held and the component stays
// blocked
TEST_P(Handover, a_hung_deactivation_in_a_hand_over_holds_the_joint)
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  observed_.deactivate_gate.close();
  observed_.write_on_deactivate = true;
  EXPECT_FALSE(switch_controllers({"pos"}, {"vel"}, 300ms).ok);
  ASSERT_TRUE(controller("vel")->quarantined) << "precondition: on_deactivate still hangs";
  EXPECT_TRUE(eventually([&] { return !observed_.live[J1_VELOCITY].load(); }));
  EXPECT_TRUE(hardware().taken);
  EXPECT_FALSE(writes_go_on()) << "the hung callback may still write its handles";
  const int writes = observed_.writes.load();
  observed_.deactivate_gate.open();
  ASSERT_TRUE(eventually([&] { return !controller("vel")->quarantined; }));
  EXPECT_TRUE(switch_controllers({"pos"}, {}).ok);
  EXPECT_TRUE(applies_newest(J1_POSITION));
  EXPECT_GT(observed_.writes.load(), writes);
}

TEST_P(Handover, a_hand_over_between_controllers_applies_nothing_of_the_old_one)
{
  add("vel_b");
  ASSERT_TRUE(running({"vel"}));
  observed_.skip_first_updates = 20;
  observed_.write_on_deactivate = true;
  const auto response = switch_controllers({"vel_b"}, {"vel"});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
  EXPECT_EQ(observed_.performs.load(), 1);
}

TEST_P(Handover, rt_stuck_in_update_during_a_restart_applies_no_old_command)
{
  ASSERT_TRUE(running({"vel"}));
  observed_.stall_j1_update = true;
  ASSERT_TRUE(stalled());
  auto response =
    std::async(std::launch::async, [&] { return switch_controllers({"vel"}, {"vel"}, 1s); });
  std::this_thread::sleep_for(300ms);
  observed_.stall_j1_update = false;
  response.get();
  ASSERT_TRUE(eventually(
    [&]
    {
      const auto model = *controller("vel");
      return !model.quarantined && (model.state != core::Lifecycle::Active || model.updated);
    }));
  EXPECT_TRUE(writes_go_on());
}

TEST_P(Handover, rt_stuck_in_a_detached_update_keeps_the_writes_blocked)
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  observed_.stall_j1_update = true;
  ASSERT_TRUE(stalled());
  EXPECT_FALSE(switch_controllers({"pos"}, {"vel"}, 300ms).ok);
  const int before = observed_.stopped_commands.load();
  // The update RT was stuck in writes a fresh command of vel
  observed_.stall_j1_update = false;
  ASSERT_TRUE(eventually(
    [&]
    {
      const auto model = *controller("vel");
      return !model.quarantined && model.state != core::Lifecycle::Active;
    }));
  EXPECT_TRUE(eventually([&] { return !hardware().taken; }));
  EXPECT_TRUE(writes_go_on());
  EXPECT_EQ(observed_.stopped_commands.load(), before);
}

// The activation leaves less than 100 ms, and only the give back without a mode switch is left
TEST_P(Handover, a_hand_over_close_to_its_deadline_is_not_rolled_back)
{
  add("vel_b");
  ASSERT_TRUE(running({"vel"}));
  observed_.activate_delay_once_ms = 440;
  const auto response = switch_controllers({"vel_b"}, {"vel"}, 500ms);
  EXPECT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(eventually([&] { return controller("vel_b")->state == core::Lifecycle::Active; }));
}

void Handover::fail_the_mode_switch_of_a_hand_over()
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  observed_.perform_failures = 1;
  EXPECT_FALSE(switch_controllers({"pos"}, {"vel"}).ok);
  EXPECT_TRUE(eventually(
    [&]
    {
      const auto model = hardware();
      return !model.taken && !model.quarantined && model.fault == core::HardwareFault::None;
    }));
  // The rollback keeps the fault's target, so on_error leaves the component down
  ASSERT_TRUE(settled());
  EXPECT_NE(hardware().state, core::Lifecycle::Active);
}

TEST_P(Handover, a_failing_perform_in_a_hand_over_rolls_back_with_the_component_down)
{
  fail_the_mode_switch_of_a_hand_over();
}

TEST_P(Handover, a_throwing_perform_in_a_hand_over_rolls_back_with_the_component_down)
{
  observed_.perform_throws = true;
  fail_the_mode_switch_of_a_hand_over();
}

TEST_P(Handover, a_mode_switch_that_outlives_the_deadline_applies_nothing)
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  observed_.perform_delay_ms = 600;
  EXPECT_FALSE(switch_controllers({"pos"}, {"vel"}, 400ms).ok);
  ASSERT_TRUE(eventually(
    [&]
    {
      const auto model = hardware();
      return model.state != core::Lifecycle::Active && !model.quarantined;
    }));
  EXPECT_NE(controller("vel")->state, core::Lifecycle::Active);
  EXPECT_NE(controller("pos")->state, core::Lifecycle::Active);
}

TEST_P(Handover, a_claimer_of_another_joint_keeps_its_writes_during_a_slow_restart)
{
  ASSERT_TRUE(running({"vel", "j2"}));
  observed_.activate_gate.close();
  auto response =
    std::async(std::launch::async, [&] { return switch_controllers({"vel"}, {"vel"}); });
  ASSERT_TRUE(observed_.activate_gate.holds());
  // Per write of the component, not per time, so load cannot break the rate. A write that has
  // begun may count on one side of the window only.
  const int writes = observed_.writes.load();
  const int applied = observed_.applied_j2.load();
  EXPECT_TRUE(eventually([&] { return observed_.writes.load() >= writes + 50; }));
  EXPECT_GE(observed_.applied_j2.load() - applied, 48);
  observed_.activate_gate.open();
  EXPECT_TRUE(response.get().ok);
}

TEST_P(Handover, a_new_activation_that_fails_at_once_leaves_the_interface_held)
{
  ASSERT_TRUE(running({"vel"}));
  observed_.failing_epoch = observed_.next_epoch.load();
  switch_controllers({"vel"}, {"vel"});
  ASSERT_TRUE(eventually([&] { return controller("vel")->state != core::Lifecycle::Active; }));
  EXPECT_TRUE(eventually([&] { return !observed_.live[J1_VELOCITY].load(); }));
  EXPECT_TRUE(eventually([&] { return hardware().live.empty(); }));
  EXPECT_TRUE(writes_go_on());
  const auto response = switch_controllers({"vel"}, {});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

TEST_P(Handover, a_hung_activation_hands_the_component_back_held)
{
  ASSERT_TRUE(running({"vel"}));
  observed_.activate_gate.close();
  EXPECT_FALSE(switch_controllers({"vel"}, {"vel"}, 300ms).ok);
  EXPECT_TRUE(eventually([&] { return !observed_.live[J1_VELOCITY].load(); }));
  EXPECT_TRUE(writes_go_on());
  ASSERT_TRUE(controller("vel")->quarantined) << "precondition: the activation still hangs";
  observed_.activate_gate.open();
  ASSERT_TRUE(eventually(
    [&]
    {
      const auto model = *controller("vel");
      return !model.quarantined && model.state != core::Lifecycle::Active;
    }));
}

// Every release path leaves NaN for a next claimer that writes nothing in its first updates, which
// is legitimate for a controller without a command yet

TEST_P(Handover, a_restart_that_holds_and_starts_applies_nothing_old)
{
  ASSERT_TRUE(running({"vel", "j2"}));
  observed_.skip_first_updates = 20;
  const int performs = observed_.performs.load();
  const auto response = switch_controllers({"vel"}, {"vel"});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
  // j2 keeps running, so the component holds and starts instead of being taken
  EXPECT_EQ(observed_.performs.load(), performs + 2);
}

TEST_P(Handover, a_start_after_a_stop_applies_nothing_of_the_stopped_activation)
{
  ASSERT_TRUE(running({"vel"}));
  ASSERT_TRUE(switch_controllers({}, {"vel"}).ok);
  observed_.skip_first_updates = 20;
  const auto response = switch_controllers({"vel"}, {});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

TEST_P(Handover, a_start_after_a_fault_stop_applies_nothing_of_the_failed_activation)
{
  ASSERT_TRUE(running({"vel"}));
  observed_.failing_epoch = observed_.next_epoch.load() - 1;
  ASSERT_TRUE(eventually(
    [&]
    {
      const auto model = *controller("vel");
      return model.state == core::Lifecycle::Inactive && !model.quarantined;
    }));
  observed_.skip_first_updates = 20;
  const auto response = switch_controllers({"vel"}, {});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

void Handover::start_after_a_failed_activation(CallbackReturn failure)
{
  add("vel");
  add("vel_b");
  observed_.failing_activation = failure;
  EXPECT_FALSE(switch_controllers({"vel"}, {}).ok);
  observed_.skip_first_updates = 20;
  const auto response = switch_controllers({"vel_b"}, {});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

TEST_P(Handover, a_start_after_an_activation_that_fails_applies_nothing_of_it)
{
  start_after_a_failed_activation(CallbackReturn::FAILURE);
}

TEST_P(Handover, a_start_after_an_activation_with_an_error_applies_nothing_of_it)
{
  start_after_a_failed_activation(CallbackReturn::ERROR);
}

TEST_P(Handover, a_start_after_a_refused_stop_applies_nothing_of_the_refusing_activation)
{
  add("vel_b");
  ASSERT_TRUE(running({"vel"}));
  observed_.deactivation = CallbackReturn::FAILURE;
  observed_.write_on_deactivate = true;
  switch_controllers({}, {"vel"});
  ASSERT_NE(controller("vel")->state, core::Lifecycle::Active);
  observed_.skip_first_updates = 20;
  const auto response = switch_controllers({"vel_b"}, {});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

TEST_P(Handover, a_start_after_a_late_deactivation_applies_nothing_of_it)
{
  add("vel_b");
  ASSERT_TRUE(running({"vel"}));
  observed_.deactivate_gate.close();
  observed_.write_on_deactivate = true;
  EXPECT_FALSE(switch_controllers({}, {"vel"}, 300ms).ok);
  ASSERT_TRUE(controller("vel")->quarantined) << "precondition: the deactivation still hangs";
  observed_.deactivate_gate.open();
  ASSERT_TRUE(eventually([&] { return !controller("vel")->quarantined; }));
  observed_.skip_first_updates = 20;
  const auto response = switch_controllers({"vel_b"}, {});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

/// The hang check does not fault a component before a request gives up on its perform.
class HandoverSlowHang : public Handover
{
protected:
  void SetUp() override
  {
    hang_timeout_ = 5s;
    Handover::SetUp();
  }
};

// Sanitizers slow RT, the wait for it to leave a read scales with the hang timeout
TEST_P(HandoverSlowHang, a_mode_switch_waits_for_rt_in_a_long_read_without_a_fault)
{
  ASSERT_TRUE(running({"vel"}));
  observed_.read_gate.close();
  ASSERT_TRUE(observed_.read_gate.holds());
  auto stop = std::async(std::launch::async, [&] { return switch_controllers({}, {"vel"}, 4s); });
  // Past the owner's one second wait for RT to leave the read
  std::this_thread::sleep_for(1500ms);
  observed_.read_gate.open();
  const auto response = stop.get();
  EXPECT_TRUE(response.ok) << response.message;
  EXPECT_EQ(hardware().fault, core::HardwareFault::None);
}

INSTANTIATE_TEST_SUITE_P(
  Hardware, HandoverSlowHang, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "async" : "sync"; });

INSTANTIATE_TEST_SUITE_P(
  Hardware, Handover, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "async" : "sync"; });

}  // namespace
}  // namespace hector_controller_manager
