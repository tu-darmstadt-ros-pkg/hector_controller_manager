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

#include <future>

#include "token_rig.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace testing::tokens;
using runtime::Clock;
using testing::eventually;

class FaultHandling : public TokenRig
{
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
  const auto start = Clock::now();
  const auto response = switch_controllers({}, {}, 500ms);
  const auto took = Clock::now() - start;
  EXPECT_NE(response.message, RequestQueue::busy().message);
  EXPECT_LT(took, 500ms);
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

// Shutdown skips a component whose perform is late, and the request comes before the idle work
// that would finalize it once the perform returned. Its own leftover reconcile finalizes the
// component before it acts, so the rejection shows in the answer.
TEST_P(FaultHandling, a_hardware_state_request_after_shutdown_is_rejected)
{
  add("pos");
  ASSERT_TRUE(supervisor_->load("slow", "test/Writer").ok);
  ASSERT_TRUE(running({"vel"}));
  observed_.perform_gate.close();
  EXPECT_FALSE(switch_controllers({"pos"}, {"vel"}, 400ms).ok);
  supervisor_->shutdown(1s);
  ASSERT_TRUE(hardware().quarantined);
  // A configure holds the supervisor while the late perform returns and the request queues
  observed_.configure_delay_ms = 2500;
  const int configures = observed_.configures.load();
  auto configure = std::async(std::launch::async, [&] { return supervisor_->configure("slow"); });
  ASSERT_TRUE(eventually([&] { return observed_.configures.load() > configures; }));
  const int performs = observed_.performs.load();
  observed_.perform_gate.open();
  ASSERT_TRUE(eventually([&] { return observed_.performs.load() > performs; }));
  const auto response = supervisor_->set_hardware_state("system", core::Lifecycle::Active, 4s);
  configure.get();
  ASSERT_FALSE(hardware().quarantined) << "precondition: the request ran after the late perform";
  EXPECT_EQ(response.message, "The controller manager is not running");
  // The idle work then handles the fault and finalizes the component
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
  const int writes = observed_.writes.load();
  const int cycles = cycles_.load();
  ASSERT_TRUE(eventually([&] { return cycles_.load() > cycles + 20; }));
  EXPECT_EQ(observed_.writes.load(), writes);
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

INSTANTIATE_TEST_SUITE_P(
  Hardware, FaultHandling, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "async" : "sync"; });

}  // namespace
}  // namespace hector_controller_manager
