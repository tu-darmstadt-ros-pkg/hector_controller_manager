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

#include <atomic>
#include <chrono>
#include <future>
#include <string>
#include <thread>
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
  void fail_pos_with_a_late_stop();
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
  loop_.paused = true;
  observed_.activate_gate.open();
  first.get();
  EXPECT_EQ(
    switch_controllers(std::move(activate), std::move(deactivate), 300ms).message,
    RequestQueue::busy().message);
  loop_.paused = false;
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
  EXPECT_EQ(writes_over_cycles(20), 0) << "the hung callback may still write its handles";
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
  ASSERT_TRUE(eventually([&] { return hardware().taken; }))
    << "precondition: the restart took the hardware while RT is stuck";
  observed_.stall_j1_update = false;
  const auto answered = response.get();
  EXPECT_TRUE(answered.ok) << answered.message;
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
  constexpr auto timeout = 500ms;
  std::atomic<std::chrono::steady_clock::time_point> begin;
  observed_.activate_gate.close();
  auto response = std::async(
    std::launch::async,
    [&]
    {
      begin = std::chrono::steady_clock::now();
      return switch_controllers({"vel_b"}, {"vel"}, timeout);
    });
  ASSERT_TRUE(observed_.activate_gate.holds());
  std::this_thread::sleep_until(begin.load() + timeout - 60ms);
  observed_.activate_gate.open();
  const auto answered = response.get();
  EXPECT_TRUE(answered.ok) << answered.message;
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
  observed_.perform_gate.close();
  EXPECT_FALSE(switch_controllers({"pos"}, {"vel"}, 400ms).ok);
  ASSERT_TRUE(observed_.perform_gate.holds()) << "precondition: the mode switch started";
  observed_.perform_gate.open();
  ASSERT_TRUE(eventually(
    [&]
    {
      const auto model = hardware();
      return model.state != core::Lifecycle::Active && !model.quarantined;
    }));
  EXPECT_NE(controller("vel")->state, core::Lifecycle::Active);
  EXPECT_NE(controller("pos")->state, core::Lifecycle::Active);
}

// A Dynamixel perform with bus retries takes longer than the hang timeout of 100 ms
TEST_P(Handover, a_mode_switch_longer_than_the_hang_timeout_but_within_its_deadline_succeeds)
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  observed_.perform_gate.close();
  auto response =
    std::async(std::launch::async, [&] { return switch_controllers({"pos"}, {"vel"}, 2s); });
  ASSERT_TRUE(observed_.perform_gate.holds());
  std::this_thread::sleep_for(3 * hang_timeout_);
  observed_.perform_gate.open();
  const auto answered = response.get();
  EXPECT_TRUE(answered.ok) << answered.message;
  EXPECT_EQ(hardware().fault, core::HardwareFault::None);
  EXPECT_TRUE(applies_newest(J1_POSITION));
}

// Only a mode switch stuck past mode_switch_timeout_ms is given up, whatever the request's deadline
TEST_P(Handover, a_mode_switch_stuck_past_the_mode_switch_timeout_is_given_up)
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  observed_.perform_gate.close();
  const auto begin = std::chrono::steady_clock::now();
  const auto answered = switch_controllers({"pos"}, {"vel"}, 400ms);
  EXPECT_FALSE(answered.ok);
  EXPECT_GE(testing::elapsed_ms(begin), milliseconds(mode_switch_timeout_))
    << "given up before the mode switch timeout";
  EXPECT_LE(testing::elapsed_ms(begin), answer_bound_ms(400ms));
  EXPECT_NE(answered.message.find("the mode switch outlived its deadline"), std::string::npos)
    << answered.message;
  EXPECT_EQ(answered.message.find("no read or write ran"), std::string::npos) << answered.message;
  ASSERT_TRUE(eventually([&] { return hardware().quarantined; }))
    << "precondition: the perform still runs";
  observed_.perform_gate.open();
  ASSERT_TRUE(eventually(
    [&]
    {
      const auto model = hardware();
      return model.state != core::Lifecycle::Active && !model.quarantined;
    }));
}

// The failing perform of a hand over runs past the grace, so the answer does not wait for a
// rollback, idle work takes the rest over like any leftover
TEST_P(Handover, a_mode_switch_failing_past_the_grace_leaves_the_rollback_to_idle_work)
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  constexpr auto timeout = 300ms;
  observed_.perform_failures = 1;
  observed_.perform_gate.close();
  const auto begin = std::chrono::steady_clock::now();
  auto response =
    std::async(std::launch::async, [&] { return switch_controllers({"pos"}, {"vel"}, timeout); });
  ASSERT_TRUE(observed_.perform_gate.holds()) << "precondition: the hand over began";
  std::this_thread::sleep_until(begin + timeout + SupervisorConfig{}.grace + 100ms);
  observed_.perform_gate.open();
  const auto answered = response.get();
  EXPECT_FALSE(answered.ok);
  EXPECT_NE(answered.message.find("rollback incomplete"), std::string::npos) << answered.message;
  EXPECT_LE(testing::elapsed_ms(begin), answer_bound_ms(timeout));
  EXPECT_TRUE(eventually([&] { return !quarantined(); }));
}

// A Dynamixel start that begins shortly before the deadline outlasts it and still finishes
TEST_P(Handover, a_mode_switch_that_starts_shortly_before_the_deadline_finishes)
{
  add("vel");
  constexpr auto timeout = 300ms;
  observed_.activate_gate.close();
  observed_.perform_gate.close();
  const auto begin = std::chrono::steady_clock::now();
  auto response =
    std::async(std::launch::async, [&] { return switch_controllers({"vel"}, {}, timeout); });
  ASSERT_TRUE(observed_.activate_gate.holds());
  std::this_thread::sleep_until(begin + timeout - 50ms);
  observed_.activate_gate.open();
  ASSERT_TRUE(observed_.perform_gate.holds()) << "precondition: the start began";
  std::this_thread::sleep_until(begin + timeout + 300ms);
  observed_.perform_gate.open();
  const auto answered = response.get();
  EXPECT_TRUE(answered.ok) << answered.message;
  EXPECT_LE(testing::elapsed_ms(begin), answer_bound_ms(timeout));
  EXPECT_FALSE(quarantined());
  const auto model = hardware();
  EXPECT_EQ(model.state, core::Lifecycle::Active);
  EXPECT_EQ(model.fault, core::HardwareFault::None);
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

// pos fails in its first update, the rollback's mode switch outlasts the grace and still finishes
TEST_P(Handover, a_rollback_mode_switch_past_the_grace_finishes)
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  constexpr auto timeout = 300ms;
  observed_.failing_epoch = observed_.next_epoch.load();
  observed_.perform_gate.close();
  const auto begin = std::chrono::steady_clock::now();
  auto response =
    std::async(std::launch::async, [&] { return switch_controllers({"pos"}, {"vel"}, timeout); });
  ASSERT_TRUE(observed_.perform_gate.holds()) << "precondition: the restore of vel began";
  std::this_thread::sleep_until(begin + timeout + SupervisorConfig{}.grace + 200ms);
  observed_.perform_gate.open();
  const auto answered = response.get();
  EXPECT_FALSE(answered.ok) << answered.message;
  EXPECT_LE(testing::elapsed_ms(begin), answer_bound_ms(timeout));
  EXPECT_FALSE(quarantined());
  const auto model = hardware();
  EXPECT_EQ(model.state, core::Lifecycle::Active);
  EXPECT_EQ(model.fault, core::HardwareFault::None);
  // What the rollback did not reach before the grace is left to idle work, as any leftover
  EXPECT_NE(answered.message.find("rollback incomplete"), std::string::npos) << answered.message;
}

/// pos fails in its first update and its stop outlives the grace of the rollback, which answers
/// incomplete while the stop of pos hangs.
void Handover::fail_pos_with_a_late_stop()
{
  add("pos");
  ASSERT_TRUE(running({"vel", "j2"}));
  observed_.activate_gate.close();
  auto response = std::async(
    std::launch::async, [&] { return switch_controllers({"pos"}, {"vel", "j2"}, 400ms); });
  ASSERT_TRUE(observed_.activate_gate.holds());
  observed_.failing_epoch = observed_.next_epoch.load();
  observed_.deactivate_gate.close();
  observed_.activate_gate.open();
  const auto answered = response.get();
  EXPECT_FALSE(answered.ok) << answered.message;
  EXPECT_NE(answered.message.find("; rollback incomplete: "), std::string::npos)
    << answered.message;
  EXPECT_NE(answered.message.find("; not restored: 'j2', 'vel'"), std::string::npos)
    << answered.message;
  ASSERT_TRUE(observed_.deactivate_gate.holds()) << "precondition: the stop of pos is late";
}

// The component stays taken for the hung pos, so the rollback restores neither vel nor j2
TEST_P(Handover, an_incomplete_rollback_restores_nothing_on_a_component_taken_for_a_hung_claimer)
{
  fail_pos_with_a_late_stop();
  const auto applied = observed_.applied_epoch[J1_VELOCITY].load();
  observed_.deactivate_gate.open();
  ASSERT_TRUE(eventually([&] { return !quarantined(); }));
  ASSERT_TRUE(settled());
  EXPECT_NE(controller("pos")->state, core::Lifecycle::Active);
  EXPECT_NE(controller("vel")->state, core::Lifecycle::Active);
  EXPECT_NE(controller("j2")->state, core::Lifecycle::Active);
  EXPECT_TRUE(cycles(10));
  EXPECT_FALSE(observed_.live[J1_VELOCITY].load());
  EXPECT_EQ(observed_.applied_epoch[J1_VELOCITY].load(), applied);
}

TEST_P(Handover, a_component_taken_for_a_hung_claimer_is_claimed_again_once_it_returns)
{
  fail_pos_with_a_late_stop();
  // j2 commands another joint than the hung pos
  const auto waiting = switch_controllers({"j2"}, {});
  EXPECT_FALSE(waiting.ok);
  EXPECT_NE(waiting.message.find("quarantined controller 'pos'"), std::string::npos)
    << waiting.message;
  observed_.deactivate_gate.open();
  ASSERT_TRUE(eventually([&] { return !quarantined(); }));
  const auto response = switch_controllers({"vel", "j2"}, {});
  ASSERT_TRUE(response.ok) << response.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

// pos fails its activation and the restore of vel and j2 outlives the grace of the rollback. The
// fallback keeps only what runs, so neither starts once its late activation returned.
TEST_P(Handover, an_incomplete_rollback_does_not_restore_what_stopped_later)
{
  add("pos");
  ASSERT_TRUE(running({"vel", "j2"}));
  // pos returns before the gate, which then holds the restore
  observed_.failing_activation = CallbackReturn::ERROR;
  observed_.activate_gate.close();
  const auto answered = switch_controllers({"pos"}, {"vel", "j2"}, 400ms);
  EXPECT_FALSE(answered.ok) << answered.message;
  EXPECT_NE(answered.message.find("; rollback incomplete: "), std::string::npos)
    << answered.message;
  EXPECT_NE(answered.message.find("; not restored: 'j2', 'vel'"), std::string::npos)
    << answered.message;
  ASSERT_TRUE(observed_.activate_gate.holds()) << "precondition: the restore is late";
  observed_.activate_gate.open();
  ASSERT_TRUE(eventually([&] { return !quarantined(); }));
  ASSERT_TRUE(settled());
  EXPECT_NE(controller("pos")->state, core::Lifecycle::Active);
  EXPECT_NE(controller("vel")->state, core::Lifecycle::Active);
  EXPECT_NE(controller("j2")->state, core::Lifecycle::Active);
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

// The late success of the activation is undone by a deactivation that leaves a command behind
TEST_P(Handover, a_start_after_a_late_activation_applies_nothing_of_it)
{
  add("vel");
  add("vel_b");
  observed_.activate_gate.close();
  observed_.write_on_deactivate = true;
  EXPECT_FALSE(switch_controllers({"vel"}, {}, 300ms).ok);
  ASSERT_TRUE(controller("vel")->quarantined) << "precondition: the activation still hangs";
  observed_.activate_gate.open();
  ASSERT_TRUE(eventually([&] { return !controller("vel")->quarantined; }));
  ASSERT_EQ(controller("vel")->state, core::Lifecycle::Inactive);
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

// The stop of vel hangs past the deadline with the component taken. The rollback's hold would wait
// for RT in a long read, so it is left to idle work and the answer keeps its bound.
TEST_P(HandoverSlowHang, a_rollback_past_the_deadline_leaves_its_hold_to_idle_work)
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  constexpr auto timeout = 300ms;
  const int before = observed_.stopped_commands.load();
  observed_.deactivate_gate.close();
  const auto begin = std::chrono::steady_clock::now();
  auto response = std::async(
    std::launch::async,
    [&]
    {
      auto answered = switch_controllers({"pos"}, {"vel"}, timeout);
      return std::make_pair(answered, testing::elapsed_ms(begin));
    });
  ASSERT_TRUE(observed_.deactivate_gate.holds()) << "precondition: vel is taken and detached";
  observed_.read_gate.close();
  ASSERT_TRUE(observed_.read_gate.holds());
  std::this_thread::sleep_until(begin + timeout + SupervisorConfig{}.grace + 200ms);
  EXPECT_TRUE(observed_.live[J1_VELOCITY].load()) << "held before RT left the read";
  observed_.read_gate.open();
  const auto [answered, took] = response.get();
  EXPECT_FALSE(answered.ok);
  EXPECT_LE(took, testing::answer_bound_ms(timeout)) << answered.message;
  EXPECT_NE(answered.message.find("rollback incomplete"), std::string::npos) << answered.message;
  EXPECT_TRUE(eventually([&] { return !observed_.live[J1_VELOCITY].load(); }));
  EXPECT_EQ(hardware().fault, core::HardwareFault::None);
  EXPECT_EQ(observed_.stopped_commands.load(), before);
}

INSTANTIATE_TEST_SUITE_P(
  Hardware, HandoverSlowHang, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "async" : "sync"; });

INSTANTIATE_TEST_SUITE_P(
  Hardware, Handover, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "async" : "sync"; });

}  // namespace
}  // namespace hector_controller_manager
