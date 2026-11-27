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

// The startup set of a controller manager with the suite's plugins, as it comes up, after an e-stop
// release and after a hardware recovery.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "../runtime/log_capture.hpp"
#include "cm_test_suite/trace.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "services_rig.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace testing::services;
using testing::elapsed_ms;
using testing::eventually;

/// The startup set loads jsb, base_vel and arm_pos, the protected arm_vel only loads.
class Autostart : public EStop
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    auto parameters = EStop::extra_parameters();
    parameters.emplace_back(
      "startup.controllers_loaded",
      std::vector<std::string>{"jsb", "base_vel", "arm_pos", "arm_vel"});
    parameters.emplace_back("startup.controllers_active", startup_active());
    parameters.emplace_back("autostart_cooldown_ms", cooldown_ms());
    return parameters;
  }

  virtual std::vector<std::string> startup_active() const { return {"jsb", "base_vel", "arm_pos"}; }
  virtual int64_t cooldown_ms() const { return 300; }

  void release()
  {
    publish("a", false);
    publish("b", false);
  }

  bool active(const std::string & name) { return state_of(*cm_, name) == "active"; }

  static cm_test_suite::Target hardware_target(const std::string & name)
  {
    return {cm_test_suite::TargetKind::Hardware, *cm_test_suite::rig::hardware_index(name)};
  }

  bool activate_hardware(const std::string & name)
  {
    return set_hardware(name, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  bool set_hardware(const std::string & name, uint8_t state)
  {
    auto request = std::make_shared<ControllerManager::SetHardwareComponentState::Request>();
    request->name = name;
    request->target_state.id = state;
    auto response = std::make_shared<ControllerManager::SetHardwareComponentState::Response>();
    cm_->set_hardware_component_state_srv_cb(request, response);
    return response->ok;
  }

  /// Faults the component and waits until it ended unconfigured.
  bool fault_hardware(const std::string & name)
  {
    cm_test_suite::faults().arm(
      hardware_target(name), cm_test_suite::Hook::Read, cm_test_suite::Fault::Error);
    return eventually([&] { return hardware_state_of(*cm_, name) == "unconfigured"; });
  }

  /// Faults base, which stops base_vel and jsb.
  bool fault_base() { return fault_hardware("base") && !active("base_vel") && !active("jsb"); }

  /// Releases and waits until the startup set runs.
  bool started_and_released()
  {
    if (!started())
    {
      return false;
    }
    release();
    return eventually([&] { return active("base_vel") && active("arm_pos"); });
  }

  /// Releases, then engages while `fault` hits the stop of arm_pos.
  void engage_while_arm_pos_fails_to_stop(cm_test_suite::Fault fault)
  {
    {
      // The status shows the activations before the autostart's switch answered, an engage
      // before the answer would roll that switch back instead
      const testing::LogCapture log;
      release();
      ASSERT_TRUE(eventually([&] { return log.contains("Startup activated"); })) << log.text();
    }
    ASSERT_TRUE(active("base_vel") && active("arm_pos"));
    cm_test_suite::faults().arm(
      {cm_test_suite::TargetKind::Controller, *cm_test_suite::rig::controller_index("arm_pos")},
      cm_test_suite::Hook::Deactivate, fault);
    publish("a", true);
    ASSERT_TRUE(eventually([&] { return !active("base_vel") && !active("arm_pos"); }));
  }

  /// Releases and expects the startup set without arm_pos.
  void release_without_arm_pos(const testing::LogCapture & log)
  {
    publish("a", false);
    ASSERT_TRUE(eventually([&] { return log.contains("Startup activated"); })) << log.text();
    EXPECT_TRUE(active("base_vel"));
    EXPECT_FALSE(active("arm_pos"));
    EXPECT_FALSE(log.contains("Startup activates none")) << log.text();
  }

  /// The initial startup switch has answered. jsb counts as active before its first update, and
  /// a fault armed before the answer hits the startup switch instead of idle work.
  /// The initial startup answered and logged, which may come well after its status shows jsb.
  bool started()
  {
    return eventually([&]
                      { return active("jsb") && startup_log_.contains("Startup activated"); }) &&
           switch_controllers(*cm_, {}, {})->ok;
  }

  /// Exists before the manager, so the initial startup's line is never missed. A capture of a test
  /// starts empty again.
  const testing::LogCapture startup_log_;
};

// The initial startup leaves out what the engaged e-stop rejects, the release brings it, a hard
// e-stop that faulted the hardware is recovered, and an engage stops it again
TEST_P(Autostart, a_release_restores_the_startup_set_and_faulted_hardware)
{
  ASSERT_TRUE(started());
  EXPECT_FALSE(active("base_vel"));
  release();
  EXPECT_TRUE(eventually([&] { return active("base_vel") && active("arm_pos"); }));
  publish("a", true);
  EXPECT_TRUE(eventually([&] { return !active("base_vel") && !active("arm_pos"); }));
  // A power cut faults the hardware, which ends unconfigured
  EXPECT_TRUE(fault_hardware("base"));
  publish("a", false);
  // The status shows the startup switch's steps, base_vel may be active before jsb
  EXPECT_TRUE(eventually([&] { return active("base_vel") && active("jsb"); }));
  EXPECT_EQ(hardware_state_of(*cm_, "base"), "active");
}

// Motors are not up yet after the power returns, so the first attempt fails and the next one works
TEST_P(Autostart, a_failed_hardware_attempt_is_retried_after_the_cooldown)
{
  ASSERT_TRUE(started());
  ASSERT_TRUE(fault_hardware("base"));
  const auto base = hardware_target("base");
  cm_test_suite::faults().arm(base, cm_test_suite::Hook::Activate, cm_test_suite::Fault::Error);
  const testing::LogCapture log;
  release();
  EXPECT_TRUE(eventually([&] { return active("base_vel"); }));
  EXPECT_TRUE(log.contains("Autostart attempt 1 failed"));
}

// base faults in RT while a slow switch holds the supervisor past the cooldown, so the status still
// shows base active when the autostart reads it
TEST_P(Autostart, a_release_recovers_hardware_that_faulted_behind_a_slow_request)
{
  ASSERT_TRUE(started());
  cm_test_suite::faults().arm(
    {cm_test_suite::TargetKind::Controller, *cm_test_suite::rig::controller_index("arm_vel")},
    cm_test_suite::Hook::Activate, cm_test_suite::Fault::Late);
  auto slow =
    std::async(std::launch::async, [&] { return switch_controllers(*cm_, {"arm_vel"}, {}); });
  ASSERT_TRUE(eventually([] { return cm_test_suite::trace().hangs_running() > 0; }))
    << "precondition: the slow activation holds the supervisor";
  const auto base = hardware_target("base");
  cm_test_suite::faults().arm(base, cm_test_suite::Hook::Read, cm_test_suite::Fault::Error);
  ASSERT_TRUE(
    eventually([&] { return !cm_test_suite::faults().pending(base, cm_test_suite::Hook::Read); }));
  release();
  EXPECT_TRUE(eventually([&] { return active("base_vel") && active("jsb"); }));
  EXPECT_EQ(hardware_state_of(*cm_, "base"), "active");
  slow.get();
}

// The engage comes while the autostart brings the hardware up, after its cooldown
TEST_P(Autostart, an_engage_during_the_hardware_bring_up_aborts_before_the_switch)
{
  ASSERT_TRUE(started());
  ASSERT_TRUE(fault_hardware("base"));
  const auto base = hardware_target("base");
  // The activation of the recovered base takes almost a second
  cm_test_suite::faults().arm(base, cm_test_suite::Hook::Activate, cm_test_suite::Fault::Late);
  const testing::LogCapture log;
  release();
  ASSERT_TRUE(eventually([&] { return hardware_state_of(*cm_, "base") == "inactive"; }))
    << "precondition: the autostart configured base and activates it";
  publish("a", true);
  EXPECT_TRUE(eventually([&] { return log.contains("Autostart aborted"); }));
  EXPECT_FALSE(log.contains("Startup skips"));
  EXPECT_FALSE(active("base_vel"));
}

// An engage and a release while the first autostart activates the recovered base. The first one
// ends before its switch, only the second one switches.
TEST_P(Autostart, a_newer_release_supersedes_an_older_autostart)
{
  ASSERT_TRUE(started());
  ASSERT_TRUE(fault_hardware("base"));
  const auto base = hardware_target("base");
  cm_test_suite::faults().arm(base, cm_test_suite::Hook::Activate, cm_test_suite::Fault::Late);
  const testing::LogCapture log;
  release();
  ASSERT_TRUE(eventually([&] { return hardware_state_of(*cm_, "base") == "inactive"; }))
    << "precondition: the first autostart activates base";
  publish("a", true);
  ASSERT_TRUE(eventually([&] { return log.contains("E-stop engaged"); }));
  publish("a", false);
  // The status shows base_vel active before the switch answered and logged
  EXPECT_TRUE(eventually([&] { return log.contains("Startup activated"); })) << log.text();
  EXPECT_TRUE(active("base_vel"));
  EXPECT_TRUE(log.contains("Autostart aborted, the e-stop engaged or a newer release won"))
    << log.text();
  const std::string text = log.text();
  const std::string switched = "Startup activated";
  const auto first = text.find(switched);
  EXPECT_NE(first, std::string::npos);
  EXPECT_EQ(text.find(switched, first + 1), std::string::npos) << text;
}

// A soft e-stop leaves the protected arm_vel running, the autostart skips arm_pos on its joints
TEST_P(Autostart, a_release_skips_what_a_running_protected_controller_holds)
{
  ASSERT_TRUE(started());
  ASSERT_TRUE(switch_controllers(*cm_, {"arm_vel"}, {})->ok) << "a protected controller may start";
  const testing::LogCapture log;
  release();
  EXPECT_TRUE(eventually([&] { return active("base_vel"); }));
  EXPECT_FALSE(active("arm_pos"));
  EXPECT_TRUE(active("arm_vel"));
  EXPECT_TRUE(
    log.contains("Startup skips 'arm_pos' (a running controller claims its joint or reference)"));
}

// arm_pos refuses the stop of an engage and ends finalized
TEST_P(Autostart, a_release_skips_a_finalized_controller_and_starts_the_rest)
{
  ASSERT_TRUE(started());
  engage_while_arm_pos_fails_to_stop(cm_test_suite::Fault::Failure);
  const testing::LogCapture log;
  release_without_arm_pos(log);
  EXPECT_TRUE(log.contains(
    "Startup skips 'arm_pos' (Controller 'arm_pos' cannot be activated, it is finalized)"))
    << log.text();
}

// arm_pos hangs in the stop of an engage and is quarantined until it returns
TEST_P(Autostart, a_release_skips_a_quarantined_controller_and_starts_the_rest)
{
  ASSERT_TRUE(started());
  engage_while_arm_pos_fails_to_stop(cm_test_suite::Fault::Hang);
  const testing::LogCapture log;
  release_without_arm_pos(log);
  // It still counts as running, so the reason may be that it holds its joints
  EXPECT_TRUE(log.contains("Startup skips 'arm_pos' (")) << log.text();
}

// A stopped startup controller comes back with the rest, as after a release
TEST_P(Autostart, a_hardware_reactivation_restores_the_startup_set)
{
  ASSERT_TRUE(started_and_released());
  ASSERT_TRUE(switch_controllers(*cm_, {}, {"arm_pos"})->ok);
  ASSERT_TRUE(fault_base());
  ASSERT_TRUE(activate_hardware("base"));
  EXPECT_TRUE(eventually([&] { return active("base_vel") && active("jsb") && active("arm_pos"); }));
}

// Neither an activation of active hardware nor a failed one restores what the operator stopped
TEST_P(Autostart, only_a_hardware_activation_that_brings_it_up_restores_the_startup_set)
{
  ASSERT_TRUE(started_and_released());
  ASSERT_TRUE(switch_controllers(*cm_, {}, {"arm_pos"})->ok);
  ASSERT_TRUE(activate_hardware("arm"));
  ASSERT_TRUE(fault_base());
  cm_test_suite::faults().arm(
    hardware_target("base"), cm_test_suite::Hook::Activate, cm_test_suite::Fault::Error);
  ASSERT_FALSE(activate_hardware("base"));
  EXPECT_FALSE(active("arm_pos"));
}

// Inactive hardware is not up, its activation restores the set like one after a fault
TEST_P(Autostart, an_activation_from_inactive_restores_the_startup_set)
{
  ASSERT_TRUE(started_and_released());
  // Nothing may use base while it goes inactive
  ASSERT_TRUE(switch_controllers(*cm_, {}, {"arm_pos", "base_vel", "jsb"})->ok);
  ASSERT_TRUE(set_hardware("base", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE));
  ASSERT_TRUE(activate_hardware("base"));
  EXPECT_TRUE(eventually([&] { return active("base_vel") && active("jsb") && active("arm_pos"); }));
}

// arm is still down, its arm_pos must not hold back the rest of the startup set
TEST_P(Autostart, a_partial_hardware_recovery_restores_what_its_hardware_allows)
{
  ASSERT_TRUE(started_and_released());
  ASSERT_TRUE(fault_hardware("arm"));
  ASSERT_TRUE(fault_base());
  const testing::LogCapture log;
  ASSERT_TRUE(activate_hardware("base"));
  EXPECT_TRUE(eventually([&] { return active("base_vel") && active("jsb"); })) << log.text();
  EXPECT_FALSE(active("arm_pos"));
  EXPECT_TRUE(log.contains(
    "Startup skips 'arm_pos' ('arm_pos' claims 'a1/position' of hardware 'arm', which is not "
    "active or has a fault)"))
    << log.text();
}

// The latch still keeps the command claimers off, the broadcaster comes back
TEST_P(Autostart, a_hardware_reactivation_while_engaged_restores_only_the_broadcaster)
{
  ASSERT_TRUE(started());
  ASSERT_TRUE(fault_base());
  const testing::LogCapture log;
  ASSERT_TRUE(activate_hardware("base"));
  EXPECT_TRUE(eventually([&] { return active("jsb"); }));
  EXPECT_FALSE(active("base_vel"));
  EXPECT_TRUE(log.contains(
    "Startup skips 'base_vel' (e-stop engaged by '" + topic("a") + "', it sent no message yet)"))
    << log.text();
}

INSTANTIATE_TEST_SUITE_P(ExecutorThreads, Autostart, ::testing::Values(2));

/// base_vel is protected, so the initial startup starts it also while engaged.
class AutostartProtected : public Autostart
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    auto parameters = Autostart::extra_parameters();
    parameters.emplace_back("base_vel.protected", true);
    return parameters;
  }
};

// Like a torque off controller after a hard e-stop, only an operator brings it back
TEST_P(AutostartProtected, a_release_does_not_restore_a_protected_controller)
{
  ASSERT_TRUE(started());
  ASSERT_TRUE(active("base_vel")) << "precondition: the latch allows a protected start";
  ASSERT_TRUE(fault_base());
  const testing::LogCapture log;
  release();
  ASSERT_TRUE(eventually([&] { return log.contains("Startup activated"); })) << log.text();
  EXPECT_TRUE(active("jsb"));
  EXPECT_TRUE(active("arm_pos"));
  EXPECT_FALSE(active("base_vel"));
  EXPECT_TRUE(log.contains("Startup skips 'base_vel' (protected)")) << log.text();
}

INSTANTIATE_TEST_SUITE_P(ExecutorThreads, AutostartProtected, ::testing::Values(2));

/// imu starts inactive and the startup names base active.
class AutostartHardwareTargets : public Autostart
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    auto parameters = Autostart::extra_parameters();
    parameters.emplace_back(
      "hardware_components_initial_state.inactive", std::vector<std::string>{"imu"});
    parameters.emplace_back("startup.hardware_active", std::vector<std::string>{"base"});
    return parameters;
  }
};

TEST_P(AutostartHardwareTargets, a_release_recovers_hardware_named_active_and_leaves_inactive_alone)
{
  ASSERT_TRUE(started());
  ASSERT_EQ(hardware_state_of(*cm_, "imu"), "inactive");
  ASSERT_TRUE(fault_base());
  const testing::LogCapture log;
  release();
  ASSERT_TRUE(eventually([&] { return log.contains("Startup activated"); })) << log.text();
  EXPECT_EQ(hardware_state_of(*cm_, "base"), "active");
  EXPECT_EQ(hardware_state_of(*cm_, "imu"), "inactive");
}

INSTANTIATE_TEST_SUITE_P(ExecutorThreads, AutostartHardwareTargets, ::testing::Values(2));

/// The startup set names a controller that is not loaded, the log is captured from the start.
class AutostartUnloaded : public Autostart
{
protected:
  std::vector<std::string> startup_active() const override { return {"jsb", "ghost"}; }

  const testing::LogCapture log_;
};

// Documented as all or nothing, so the log says that none starts and which controller is missing
TEST_P(AutostartUnloaded, an_unloaded_startup_controller_activates_nothing_and_is_named)
{
  const std::string rejected =
    "Startup activates none of 'ghost', 'jsb', the switch was rejected: Controller 'ghost' is not "
    "loaded";
  EXPECT_TRUE(eventually([&] { return log_.contains(rejected); })) << log_.text();
  EXPECT_FALSE(active("jsb"));
}

INSTANTIATE_TEST_SUITE_P(ExecutorThreads, AutostartUnloaded, ::testing::Values(2));

/// A cooldown no engage of a test misses.
class AutostartLongCooldown : public Autostart
{
protected:
  int64_t cooldown_ms() const override { return 30000; }
};

TEST_P(AutostartLongCooldown, an_engage_during_the_autostart_cooldown_aborts_it)
{
  ASSERT_TRUE(started());
  const testing::LogCapture log;
  release();
  ASSERT_TRUE(eventually([&] { return log.contains("E-stop released"); }));
  publish("b", true);
  EXPECT_TRUE(eventually([&] { return log.contains("Autostart aborted"); }));
  EXPECT_FALSE(active("base_vel"));
}

TEST_P(AutostartLongCooldown, a_shutdown_during_the_autostart_cooldown_ends_it_without_a_warning)
{
  ASSERT_TRUE(started());
  const testing::LogCapture log;
  release();
  ASSERT_TRUE(eventually([&] { return log.contains("E-stop released"); }));
  cm_->shutdown();
  EXPECT_FALSE(
    eventually([&] { return log.contains("Autostart aborted"); }, std::chrono::milliseconds(500)))
    << log.text();
}

INSTANTIATE_TEST_SUITE_P(ExecutorThreads, AutostartLongCooldown, ::testing::Values(2));

/// One autostart attempt, so only a real failure may end it.
class AutostartOnce : public Autostart
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    auto parameters = Autostart::extra_parameters();
    parameters.emplace_back("autostart_attempts", 1);
    return parameters;
  }
};

// The autostart's recovery waits for a slow on_error and gives way to every request meanwhile,
// each "try again" is asked again instead of using up the attempt
TEST_P(AutostartOnce, requests_during_a_slow_on_error_do_not_use_up_the_autostart)
{
  ASSERT_TRUE(started());
  const auto base = hardware_target("base");
  // on_error takes 2.5 s, the autostart's recovery waits for it well past its cooldown
  cm_test_suite::faults().arm(base, cm_test_suite::Hook::Error, cm_test_suite::Fault::Hang);
  const testing::LogCapture log;
  cm_test_suite::faults().arm(base, cm_test_suite::Hook::Read, cm_test_suite::Fault::Error);
  ASSERT_TRUE(eventually([&] { return log.contains("Hardware 'base' failed"); }));
  // A request queued before the idle work reached on_error would run it to its end itself
  ASSERT_TRUE(eventually([] { return cm_test_suite::trace().hangs_running() > 0; }))
    << "precondition: on_error hangs in idle work";
  release();
  std::atomic<bool> stop{false};
  std::thread traffic(
    [&]
    {
      while (!stop)
      {
        switch_controllers(*cm_, {}, {});
        std::this_thread::sleep_for(20ms);
      }
    });
  EXPECT_TRUE(eventually([&] { return active("base_vel"); }));
  stop = true;
  traffic.join();
  EXPECT_TRUE(log.contains("Autostart asks again for 'base'"))
    << "precondition: it gave way, log:\n"
    << log.text();
  EXPECT_FALSE(log.contains("Autostart gave up"));
}

INSTANTIATE_TEST_SUITE_P(ExecutorThreads, AutostartOnce, ::testing::Values(2));

}  // namespace
}  // namespace hector_controller_manager
