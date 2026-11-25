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

// Fault handling and shutdown on a component whose writers mark each command with its activation,
// on sync and async hardware.

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include "log_capture.hpp"
#include "token_rig.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace testing::tokens;
using testing::eventually;

class FaultHandling : public TokenRig
{
protected:
  void stop_vel(bool estop);
  void slow_hold_keeps_the_component(bool estop);
};

// A request arrives while the stop of a failed controller hangs in on_deactivate as idle work. It
// touches nothing of the quarantined controller.
TEST_P(FaultHandling, a_hung_callback_in_fault_handling_does_not_hold_up_or_fail_a_request)
{
  ASSERT_TRUE(running({"vel", "j2"}));
  observed_.deactivate_gate.close();
  // j2's activation, the one before vel's
  observed_.failing_epoch = observed_.next_epoch.load() - 2;
  ASSERT_TRUE(observed_.deactivate_gate.holds());
  const auto response = switch_controllers({}, {}, testing::FINISH_TIMEOUT);
  EXPECT_NE(response.message, RequestQueue::busy().message);
  EXPECT_TRUE(response.ok) << response.message;
}

// The started controller fails in its first update, inside the switch
TEST_P(FaultHandling, a_start_that_fails_in_its_first_update_is_not_reported_activated)
{
  add("vel");
  observed_.failing_epoch = observed_.next_epoch.load();
  const auto response = switch_controllers({"vel"}, {});
  EXPECT_NE(controller("vel")->state, core::Lifecycle::Active);
  EXPECT_FALSE(response.ok) << response.message;
}

// Shutdown skips a component whose perform is late. Every request after the shutdown is rejected
// before it acts, so none can hold the supervisor until the late perform returned.
TEST_P(FaultHandling, a_hardware_state_request_after_shutdown_is_rejected)
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  observed_.perform_gate.close();
  EXPECT_FALSE(switch_controllers({"pos"}, {"vel"}, 400ms).ok);
  supervisor_->shutdown(1s);
  ASSERT_TRUE(hardware().quarantined);
  observed_.perform_gate.open();
  ASSERT_TRUE(eventually([&] { return !hardware().quarantined; }))
    << "precondition: the late perform returned";
  const auto response = supervisor_->set_hardware_state("system", core::Lifecycle::Active, 4s);
  EXPECT_EQ(response.message, "The controller manager is not running");
  // The idle work handles the fault and finalizes the component, before or after the request
  EXPECT_TRUE(eventually([&] { return hardware().state == core::Lifecycle::Finalized; }))
    << response.message;
}

// RT hangs in an update of vel, so the shutdown cannot stop vel. It answers within its timeout and
// grace, and the component goes down once RT returned.
TEST_P(FaultHandling, a_shutdown_while_rt_is_stuck_in_an_update_answers_in_time)
{
  ASSERT_TRUE(running({"vel"}));
  observed_.stall_j1_update = true;
  ASSERT_TRUE(stalled());
  constexpr auto timeout = 1000ms;
  const auto begin = std::chrono::steady_clock::now();
  const auto response = supervisor_->shutdown(timeout);
  const auto took = testing::elapsed_ms(begin);
  EXPECT_FALSE(response.ok) << response.message;
  EXPECT_LE(took, testing::answer_bound_ms(timeout)) << "shutdown took " << took << " ms";
  observed_.stall_j1_update = false;
  EXPECT_TRUE(eventually([&] { return hardware().state == core::Lifecycle::Finalized; }))
    << response.message;
}

// A switch accepted while the fault of the component waits for a late perform
TEST_P(FaultHandling, a_switch_during_fault_handling_does_not_reactivate_the_component)
{
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  observed_.perform_gate.close();
  EXPECT_FALSE(switch_controllers({"pos"}, {"vel"}, 400ms).ok);
  ASSERT_TRUE(hardware().quarantined);
  ASSERT_EQ(hardware().fault, core::HardwareFault::Error);
  const auto unrelated = switch_controllers({}, {});
  ASSERT_TRUE(hardware().quarantined) << "precondition: the switch ran while the call was late";
  // The late perform is followed by an urgent hold, then on_error runs
  observed_.perform_gate.open();
  ASSERT_TRUE(eventually(
    [&]
    {
      const auto model = hardware();
      return !model.quarantined && model.fault == core::HardwareFault::None;
    }));
  ASSERT_TRUE(settled());
  EXPECT_NE(hardware().state, core::Lifecycle::Active)
    << "the faulted component came back up, the switch answered: " << unrelated.message;
}

// The fault path waits for the hung stop of the claimer, the fault blocks the writes meanwhile
TEST_P(FaultHandling, a_faulted_component_applies_nothing_until_recovered)
{
  ASSERT_TRUE(running({"vel"}));
  observed_.deactivate_gate.close();
  observed_.fail_read = true;
  ASSERT_TRUE(observed_.deactivate_gate.holds());
  observed_.fail_read = false;
  EXPECT_EQ(writes_over_cycles(20), 0);
  observed_.deactivate_gate.open();
  ASSERT_TRUE(eventually(
    [&] { return hardware().state == core::Lifecycle::Unconfigured && !quarantined(); }));
  ASSERT_TRUE(settled());
  // Recovered, the joint starts only after the new activation updated
  ASSERT_TRUE(
    supervisor_->set_hardware_state("system", core::Lifecycle::Active, testing::FINISH_TIMEOUT).ok);
  observed_.skip_first_updates = 20;
  ASSERT_TRUE(switch_controllers({"vel"}, {}).ok);
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

/// Engages the e-stop or fails vel in an update, either stops vel and holds its joint in idle work.
void FaultHandling::stop_vel(bool estop)
{
  if (estop)
  {
    supervisor_->set_estop(true, "'/estop', it says engaged");
  }
  else
  {
    observed_.failing_epoch = observed_.next_epoch.load() - 1;
  }
}

// A Dynamixel perform takes up to 0.8 s, so the hold must not be given up after the hang timeout
void FaultHandling::slow_hold_keeps_the_component(bool estop)
{
  ASSERT_TRUE(running({"vel"}));
  observed_.perform_gate.close();
  stop_vel(estop);
  ASSERT_TRUE(observed_.perform_gate.holds()) << "precondition: the hold started";
  std::this_thread::sleep_for(3 * hang_timeout_);
  observed_.perform_gate.open();
  ASSERT_TRUE(eventually([&] { return !quarantined() && settled(); }));
  const auto model = hardware();
  EXPECT_EQ(model.state, core::Lifecycle::Active);
  EXPECT_EQ(model.fault, core::HardwareFault::None);
  supervisor_->set_estop(false);
  const auto restarted = switch_controllers({"vel"}, {"vel"});
  EXPECT_TRUE(restarted.ok) << restarted.message;
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

TEST_P(FaultHandling, a_hold_after_the_estop_longer_than_the_hang_timeout_keeps_the_component)
{
  slow_hold_keeps_the_component(true);
}

TEST_P(FaultHandling, a_hold_after_a_fault_longer_than_the_hang_timeout_keeps_the_component)
{
  slow_hold_keeps_the_component(false);
}

// Giving the hold up for a waiting request would fault a healthy component and not help the request
TEST_P(FaultHandling, a_request_during_a_slow_hold_after_the_estop_keeps_the_component)
{
  ASSERT_TRUE(running({"vel"}));
  observed_.perform_gate.close();
  stop_vel(true);
  ASSERT_TRUE(observed_.perform_gate.holds()) << "precondition: the hold started";
  auto request = std::async(std::launch::async, [&] { return switch_controllers({}, {}); });
  std::this_thread::sleep_for(3 * hang_timeout_);
  observed_.perform_gate.open();
  const auto answered = request.get();
  EXPECT_TRUE(answered.ok) << answered.message;
  ASSERT_TRUE(eventually([&] { return !quarantined() && settled(); }));
  const auto model = hardware();
  EXPECT_EQ(model.state, core::Lifecycle::Active);
  EXPECT_EQ(model.fault, core::HardwareFault::None);
  const auto performed = calls();
  ASSERT_FALSE(performed.empty());
  EXPECT_EQ(performed.back().stop, std::vector<std::string>{INTERFACES[J1_VELOCITY]});
  EXPECT_FALSE(observed_.live[J1_VELOCITY].load()) << "the joint is not held";
}

TEST_P(FaultHandling, a_hold_after_the_estop_past_the_mode_switch_timeout_is_given_up)
{
  ASSERT_TRUE(running({"vel"}));
  observed_.perform_gate.close();
  const auto begin = std::chrono::steady_clock::now();
  stop_vel(true);
  ASSERT_TRUE(observed_.perform_gate.holds()) << "precondition: the hold started";
  ASSERT_TRUE(eventually([&] { return hardware().quarantined > 0; }));
  const auto mode_switch_timeout =
    std::chrono::duration_cast<std::chrono::milliseconds>(SupervisorConfig{}.mode_switch_timeout);
  EXPECT_GE(testing::elapsed_ms(begin), mode_switch_timeout.count());
  observed_.perform_gate.open();
  EXPECT_TRUE(eventually(
    [&]
    {
      const auto model = hardware();
      return model.state != core::Lifecycle::Active && !model.quarantined;
    }));
}

// The shutdown's hold outlasts the shutdown's deadline, it finishes and the hardware still goes
// down
TEST_P(FaultHandling, a_shutdown_hold_past_its_deadline_finishes_and_the_hardware_goes_down)
{
  ASSERT_TRUE(running({"vel"}));
  constexpr auto timeout = 300ms;
  observed_.perform_gate.close();
  const auto begin = std::chrono::steady_clock::now();
  auto response = std::async(std::launch::async, [&] { return supervisor_->shutdown(timeout); });
  ASSERT_TRUE(observed_.perform_gate.holds()) << "precondition: the hold began";
  std::this_thread::sleep_until(begin + timeout + SupervisorConfig{}.grace + 200ms);
  observed_.perform_gate.open();
  const auto answered = response.get();
  // The hold used up the deadline, the controller's stop is left out as a late one would be
  EXPECT_NE(answered.message.find("deadline passed"), std::string::npos) << answered.message;
  EXPECT_LE(testing::elapsed_ms(begin), answer_bound_ms(timeout));
  EXPECT_FALSE(quarantined());
  EXPECT_EQ(hardware().state, core::Lifecycle::Finalized);
  EXPECT_EQ(hardware().fault, core::HardwareFault::None);
}

// The hold of vel's joint outlives the mode switch timeout, which faults the component and stops
// j2 without a hold while the component is quarantined. j2's joint stays live until the hold
// returned and on_error ran.
TEST_P(FaultHandling, a_hold_given_up_while_another_joint_runs_stops_both_and_keeps_working)
{
  const testing::LogCapture log;
  ASSERT_TRUE(running({"vel", "j2"}));
  observed_.perform_gate.close();
  stop_vel(false);
  ASSERT_TRUE(observed_.perform_gate.holds()) << "precondition: the hold started";
  ASSERT_TRUE(eventually([&] { return controller("j2")->state != core::Lifecycle::Active; }))
    << "precondition: j2 stopped while the hold ran";
  observed_.perform_gate.open();
  ASSERT_TRUE(eventually([&] { return !quarantined() && settled(); }));
  EXPECT_FALSE(log.contains("Broken invariant")) << log.text();
}

/// Only an async owner runs the reads.
class AsyncFaultHandling : public FaultHandling
{
};

// The owner hangs in a read while a switch waits for its validation. The hang faults the component
// and stops vel while it is quarantined, so vel's joint is held only after the validation returned.
TEST_P(AsyncFaultHandling, an_owner_hung_in_a_read_during_a_switch_faults_only_its_component)
{
  const testing::LogCapture log;
  add("pos");
  ASSERT_TRUE(running({"vel"}));
  observed_.read_gate.close();
  ASSERT_TRUE(observed_.read_gate.holds());
  EXPECT_FALSE(switch_controllers({"pos"}, {"vel"}, 400ms).ok);
  ASSERT_TRUE(eventually([&] { return controller("vel")->state == core::Lifecycle::Inactive; }));
  ASSERT_TRUE(hardware().quarantined) << "precondition: the validation was given up";
  // The hold lands after the validation returned
  observed_.perform_gate.close();
  observed_.read_gate.open();
  ASSERT_TRUE(observed_.perform_gate.holds()) << "precondition: the hold RT asked for runs";
  ASSERT_TRUE(eventually([&] { return !hardware().quarantined; }));
  observed_.perform_gate.open();
  ASSERT_TRUE(eventually([&] { return !quarantined() && settled(); }));
  EXPECT_FALSE(log.contains("Broken invariant")) << log.text();
  ASSERT_TRUE(
    supervisor_->set_hardware_state("system", core::Lifecycle::Active, testing::FINISH_TIMEOUT).ok);
  ASSERT_TRUE(switch_controllers({"vel"}, {}).ok);
  EXPECT_TRUE(applies_newest(J1_VELOCITY));
}

// The owner hangs in a read after j2's start was validated, so the mode switch that starts j2 waits
// behind the read and is given up. It runs only after j2 was unloaded and checks j2's failure then.
// Async only, since on sync hardware RT reads and no owner call holds a switch back on cue.
TEST_P(AsyncFaultHandling, a_mode_switch_given_up_behind_a_hung_read_runs_after_its_claimer_unloads)
{
  const testing::LogCapture log;
  ASSERT_TRUE(running({"vel"}));
  add("j2");
  observed_.activate_gate.close();
  auto started = std::async(std::launch::async, [&] { return switch_controllers({"j2"}, {}); });
  ASSERT_TRUE(observed_.activate_gate.holds()) << "precondition: the switch was validated";
  observed_.read_gate.close();
  ASSERT_TRUE(observed_.read_gate.holds());
  observed_.activate_gate.open();
  EXPECT_FALSE(started.get().ok);
  ASSERT_TRUE(log.contains(
    "start system[j2/velocity], stop system[] failed for 'system': system: "
    "the mode switch outlived its deadline"))
    << "precondition: the mode switch was given up behind the read, " << log.text();
  ASSERT_TRUE(eventually([&] { return controller("j2")->state == core::Lifecycle::Inactive; }));
  const auto unloaded = supervisor_->unload("j2");
  ASSERT_TRUE(unloaded.ok) << unloaded.message;
  observed_.read_gate.open();
  ASSERT_TRUE(eventually([&] { return !quarantined() && settled(); }));
  EXPECT_FALSE(log.contains("Broken invariant")) << log.text();
}

/// Engages and releases the e-stop before a cycle on request, so no update sees it engaged.
class EStopPulse : public FaultHandling
{
protected:
  void cycle(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    if (pulse_.exchange(false))
    {
      supervisor_->set_estop(true, "'/estop', it says engaged");
      supervisor_->set_estop(false);
    }
    FaultHandling::cycle(time, period);
  }

  std::atomic<bool> pulse_{false};
};

TEST_P(EStopPulse, an_engage_shorter_than_a_cycle_stops_the_command_claimers)
{
  ASSERT_TRUE(running({"vel"}));
  pulse_ = true;
  EXPECT_TRUE(eventually([&] { return controller("vel")->state == core::Lifecycle::Inactive; }));
}

INSTANTIATE_TEST_SUITE_P(
  Hardware, EStopPulse, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "async" : "sync"; });

INSTANTIATE_TEST_SUITE_P(
  Hardware, AsyncFaultHandling, ::testing::Values(true),
  [](const ::testing::TestParamInfo<bool> &) { return "async"; });

INSTANTIATE_TEST_SUITE_P(
  Hardware, FaultHandling, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "async" : "sync"; });

}  // namespace
}  // namespace hector_controller_manager
