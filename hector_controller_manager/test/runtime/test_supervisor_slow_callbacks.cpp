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

// Supervisor fault handling while plugin callbacks are slow or hang.

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <memory>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "gate.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/supervisor.hpp"
#include "log_capture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "supervisor_rig.hpp"
#include "urdf.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace std::chrono_literals;
using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::return_type;
using SteadyClock = std::chrono::steady_clock;
using testing::eventually;

using testing::answer_bound_ms;
using testing::SLACK;

const std::string URDF = testing::urdf({{"j1", {"velocity"}}, {"j2", {"velocity"}}});

void sleep_ms(const std::atomic<int> & ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms.load()));
}

/// Plugin callbacks that run right now, every one that uses the fixture's state.
std::atomic<int> g_busy{0};

struct Observed
{
  std::atomic<int> activations{0};
  testing::Gate activate_gate;
  std::atomic<int> errors{0};
  testing::Gate error_gate;
  std::atomic<int> start_delay_ms{0};
  std::atomic<int> starts{0};
  std::atomic<bool> fail_write_once{false};
  std::atomic<bool> j1_live{false};
  std::atomic<bool> j2_live{false};
  std::atomic<int> live_writes_while_deactivating{0};
  std::atomic<int> write_delay_ms{0};
  std::atomic<int> delayed_writes{0};
  testing::Gate configure_gate;
  testing::Gate cleanup_gate;
  std::atomic<int> activate_delay_ms{0};
  testing::Gate controller_activate_gate;
  std::atomic<int> deactivate_delay_ms{0};
  testing::Gate deactivate_gate;
  testing::Gate shutdown_gate;
  testing::Gate prepare_gate;
  /// The controllers on j1 fail their update.
  std::atomic<bool> fail_update{false};
  std::atomic<bool> deactivating{false};
};

class System : public hardware_interface::SystemInterface
{
public:
  explicit System(Observed & observed) : observed_(observed) {}
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    const testing::InFlight busy(g_busy);
    observed_.activate_gate.pass();
    observed_.activations.fetch_add(1);
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    const testing::InFlight busy(g_busy);
    observed_.error_gate.pass();
    observed_.j1_live = false;
    observed_.j2_live = false;
    observed_.errors.fetch_add(1);
    return CallbackReturn::SUCCESS;
  }
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return return_type::OK;
  }
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    const testing::InFlight busy(g_busy);
    if (observed_.fail_write_once.exchange(false))
    {
      return return_type::ERROR;
    }
    if (observed_.j1_live && observed_.deactivating)
    {
      observed_.live_writes_while_deactivating.fetch_add(1);
    }
    if (observed_.write_delay_ms.load() > 0)
    {
      sleep_ms(observed_.write_delay_ms);
      observed_.delayed_writes.fetch_add(1);
    }
    return return_type::OK;
  }
  return_type prepare_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override
  {
    const testing::InFlight busy(g_busy);
    observed_.prepare_gate.pass();
    return return_type::OK;
  }
  return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {
    const testing::InFlight busy(g_busy);
    if (!start.empty())
    {
      observed_.starts.fetch_add(1);
      sleep_ms(observed_.start_delay_ms);
    }
    for (const auto & name : start)
    {
      observed_.j1_live = observed_.j1_live || name == "j1/velocity";
      observed_.j2_live = observed_.j2_live || name == "j2/velocity";
    }
    for (const auto & name : stop)
    {
      observed_.j1_live = observed_.j1_live && name != "j1/velocity";
      observed_.j2_live = observed_.j2_live && name != "j2/velocity";
    }
    return return_type::OK;
  }

private:
  Observed & observed_;
};

/// Named "a..." commands j1/velocity, named "b..." commands j2/velocity.
class Controller : public testing::NoInterfaces<>
{
public:
  Controller(Observed & observed, std::string joint) : observed_(observed), joint_(std::move(joint))
  {
  }
  InterfaceConfiguration command_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, {joint_ + "/velocity"}};
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const testing::InFlight busy(g_busy);
    observed_.configure_gate.pass();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    const testing::InFlight busy(g_busy);
    observed_.cleanup_gate.pass();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    const testing::InFlight busy(g_busy);
    sleep_ms(observed_.activate_delay_ms);
    observed_.controller_activate_gate.pass();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    const testing::InFlight busy(g_busy);
    observed_.deactivating = true;
    sleep_ms(observed_.deactivate_delay_ms);
    observed_.deactivate_gate.pass();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    const testing::InFlight busy(g_busy);
    observed_.shutdown_gate.pass();
    return CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    const testing::InFlight busy(g_busy);
    if (observed_.fail_update && joint_ == "j1")
    {
      return controller_interface::return_type::ERROR;
    }
    (void)command_interfaces_[0].set_value(1.5);
    return controller_interface::return_type::OK;
  }

private:
  Observed & observed_;
  std::string joint_;
};

std::atomic<bool> g_host_destroyed{false};
std::atomic<int> g_forget_after_destroy{0};

/// Notes a forget after its destruction.
class Host : public testing::FakeHost
{
public:
  explicit Host(Observed & observed)
  : FakeHost(
      [&observed](const std::string & name, const std::string &)
      { return std::make_shared<Controller>(observed, name[0] == 'b' ? "j2" : "j1"); },
      [&observed] { return std::make_unique<System>(observed); })
  {
  }
  ~Host() override { g_host_destroyed = true; }
  bool forget_controller(const runtime::ControllerRuntime::Plugin &) override
  {
    if (g_host_destroyed)
    {
      g_forget_after_destroy.fetch_add(1);
    }
    return true;
  }
};

class SupervisorSlowCallbacks : public testing::SupervisorRig<>
{
protected:
  void start(
    std::chrono::milliseconds default_timeout, PluginHost * host = nullptr,
    const std::string & urdf = URDF)
  {
    SupervisorConfig config;
    config.default_timeout = default_timeout;
    config.hang_timeout = hang_timeout_;
    config.hardware_fault_timeout = hardware_fault_timeout_;
    config.mode_switch_timeout = mode_switch_timeout_;
    SupervisorRig::start(config, host ? *host : host_, urdf);
  }

  void TearDown() override
  {
    if (!supervisor_)
    {
      return;
    }
    // Shutdown must not start a callback that outlives it, the plugin would be leaked with it
    for (auto * gate :
         {&observed_.activate_gate, &observed_.error_gate, &observed_.configure_gate,
          &observed_.cleanup_gate, &observed_.controller_activate_gate, &observed_.deactivate_gate,
          &observed_.shutdown_gate, &observed_.prepare_gate})
    {
      gate->open();
    }
    for (auto * delay :
         {&observed_.start_delay_ms, &observed_.activate_delay_ms, &observed_.deactivate_delay_ms,
          &observed_.write_delay_ms})
    {
      *delay = 0;
    }
    // A call given up on still runs, its return ends the quarantine
    EXPECT_TRUE(eventually([&] { return !quarantined(); }, 10s));
    stop();
    EXPECT_TRUE(eventually([] { return g_busy.load() == 0; }));
  }

  std::chrono::milliseconds hang_timeout_{100};
  std::chrono::milliseconds hardware_fault_timeout_{2000};
  std::chrono::milliseconds mode_switch_timeout_ =
    std::chrono::duration_cast<std::chrono::milliseconds>(SupervisorConfig{}.mode_switch_timeout);
  Observed observed_;
  Host host_{observed_};
};

// A hardware fault whose on_error outlives its deadline. Nothing is reactivated because of a
// fault, so the late on_error leaves the component unconfigured until a request asks for more.
TEST_F(SupervisorSlowCallbacks, a_slow_on_error_after_a_fault_does_not_reactivate_the_hardware)
{
  hardware_fault_timeout_ = 300ms;
  start(300ms);
  ASSERT_EQ(observed_.activations.load(), 1);
  observed_.error_gate.close();
  observed_.fail_write_once = true;
  ASSERT_TRUE(observed_.error_gate.holds());
  ASSERT_TRUE(eventually([&] { return hardware().quarantined > 0; }))
    << "precondition: on_error outlived its step";
  observed_.error_gate.open();
  ASSERT_TRUE(eventually([&] { return observed_.errors.load() == 1 && !quarantined(); }));
  ASSERT_TRUE(settled());
  EXPECT_EQ(observed_.activations.load(), 1)
    << "the faulted hardware was configured and activated again";
  EXPECT_NE(hardware().state, core::Lifecycle::Active);
}

// on_error on a dead bus spends bus timeouts. Longer than the hang timeout is not hung yet.
TEST_F(SupervisorSlowCallbacks, a_slow_on_error_is_not_given_up_at_the_hang_timeout)
{
  start(2000ms);
  observed_.error_gate.close();
  observed_.fail_write_once = true;
  ASSERT_TRUE(observed_.error_gate.holds());
  std::this_thread::sleep_for(3 * hang_timeout_);
  EXPECT_EQ(hardware().quarantined, 0u);
  observed_.error_gate.open();
  EXPECT_TRUE(eventually([&] { return observed_.errors.load() == 1 && !quarantined(); }));
}

// A waiting request gives the slow on_error up within the hang timeout
TEST_F(SupervisorSlowCallbacks, a_request_during_a_slow_on_error_starts_at_once)
{
  start(2000ms);
  observed_.error_gate.close();
  observed_.fail_write_once = true;
  ASSERT_TRUE(observed_.error_gate.holds());
  const auto begin = SteadyClock::now();
  const auto response = switch_controllers({}, {}, 1s);
  const auto took = testing::elapsed_ms(begin);
  EXPECT_NE(response.message, RequestQueue::busy().message);
  // One interrupt period
  EXPECT_LT(took, (hang_timeout_ + SLACK).count()) << "the request waited " << took << " ms";
  EXPECT_GT(hardware().quarantined, 0u);
}

// A recovery request waits for the slow on_error instead of being rejected while it runs
TEST_F(SupervisorSlowCallbacks, a_hardware_state_request_waits_for_a_slow_on_error)
{
  start(2000ms);
  observed_.error_gate.close();
  observed_.fail_write_once = true;
  ASSERT_TRUE(observed_.error_gate.holds());
  auto recover = std::async(
    std::launch::async,
    [&] { return supervisor_->set_hardware_state("system", core::Lifecycle::Active, 3s); });
  ASSERT_TRUE(eventually([&] { return hardware().quarantined > 0; }))
    << "precondition: the request gave the slow on_error up";
  observed_.error_gate.open();
  const auto response = recover.get();
  EXPECT_TRUE(response.ok) << response.message;
  EXPECT_EQ(hardware().state, core::Lifecycle::Active);
  EXPECT_EQ(observed_.activations.load(), 2);
}

// The recovery request waits for on_error only as long as no other request waits behind it
TEST_F(
  SupervisorSlowCallbacks, a_hardware_state_request_waiting_for_on_error_gives_way_to_a_request)
{
  start(2000ms);
  observed_.error_gate.close();
  observed_.fail_write_once = true;
  ASSERT_TRUE(observed_.error_gate.holds());
  auto recover = std::async(
    std::launch::async,
    [&] { return supervisor_->set_hardware_state("system", core::Lifecycle::Active, 3s); });
  ASSERT_TRUE(eventually([&] { return hardware().quarantined > 0; }))
    << "precondition: the request gave the slow on_error up";
  const auto begin = SteadyClock::now();
  const auto timeout = 300ms;
  const auto response = switch_controllers({}, {}, timeout);
  const auto took = testing::elapsed_ms(begin);
  EXPECT_NE(response.message, RequestQueue::busy().message);
  EXPECT_LT(took, timeout.count()) << "the switch waited " << took << " ms";
  const auto recovered = recover.get();
  EXPECT_FALSE(recovered.ok);
  EXPECT_NE(recovered.message.find("try again"), std::string::npos) << recovered.message;
  observed_.error_gate.open();
}

// Setting hardware to active fails while a hung claimer delays the fault handling. The caller
// heard it failed, so the component must not reach active later on its own.
TEST_F(
  SupervisorSlowCallbacks, a_failed_hardware_activation_is_not_applied_after_the_fault_is_handled)
{
  // Fault handling gives each step the hang timeout, so this one only bounds load and configure
  start(testing::FINISH_TIMEOUT);
  load_and_configure("a", "t");
  ASSERT_TRUE(switch_controllers({"a"}, {}).ok);
  ASSERT_EQ(observed_.activations.load(), 1);
  observed_.deactivate_gate.close();
  observed_.fail_write_once = true;
  ASSERT_TRUE(observed_.deactivate_gate.holds());
  ASSERT_TRUE(eventually([&] { return quarantined(); }))
    << "precondition: the stop of the claimer outlived its step";
  EXPECT_FALSE(supervisor_->set_hardware_state("system", core::Lifecycle::Active, 300ms).ok);
  observed_.deactivate_gate.open();
  ASSERT_TRUE(eventually([&] { return observed_.errors.load() == 1 && !quarantined(); }));
  ASSERT_TRUE(settled());
  EXPECT_EQ(observed_.activations.load(), 1)
    << "hardware came back active after a failed hardware state request";
}

// A switch must answer within its timeout plus 500 ms, even while the supervisor handles a fault
// whose on_deactivate hangs. A request that cannot start in time is rejected as busy.
TEST_F(SupervisorSlowCallbacks, a_switch_answers_in_time_while_fault_handling_hangs)
{
  start(testing::FINISH_TIMEOUT);
  load_and_configure("a", "t");
  load_and_configure("b", "t");
  ASSERT_TRUE(switch_controllers({"a"}, {}).ok);
  observed_.deactivate_gate.close();
  observed_.fail_update = true;
  ASSERT_TRUE(observed_.deactivate_gate.holds());
  observed_.fail_update = false;
  const auto begin = SteadyClock::now();
  const auto timeout = 300ms;
  const auto response = switch_controllers({"b"}, {}, timeout);
  const auto took = testing::elapsed_ms(begin);
  EXPECT_LE(took, answer_bound_ms(timeout)) << "switch took " << took << " ms";
  EXPECT_TRUE(response.ok) << response.message;
}

// Fault handling holds the supervisor in a slow on_deactivate for almost the whole shutdown
// timeout, below a long hang timeout. The shutdown that waited still stops every controller and
// holds its interfaces within its bound.
TEST_F(SupervisorSlowCallbacks, a_shutdown_behind_slow_fault_handling_stops_every_controller)
{
  hang_timeout_ = 1000ms;
  start(testing::FINISH_TIMEOUT);
  load_and_configure("a", "t");
  load_and_configure("b", "t");
  ASSERT_TRUE(switch_controllers({"a", "b"}, {}).ok);
  observed_.deactivate_delay_ms = 950;
  observed_.fail_update = true;
  ASSERT_TRUE(eventually([&] { return observed_.deactivating.load(); }));
  observed_.fail_update = false;
  observed_.deactivate_delay_ms = 0;
  const auto begin = SteadyClock::now();
  const auto timeout = 1000ms;
  const auto response = supervisor_->shutdown(timeout);
  const auto took = testing::elapsed_ms(begin);
  EXPECT_LE(took, answer_bound_ms(timeout)) << "shutdown took " << took << " ms";
  const auto status = supervisor_->status();
  for (const auto & controller : status->controllers)
  {
    EXPECT_NE(controller.model.state, core::Lifecycle::Active)
      << controller.name << " is active after shutdown: " << response.message;
  }
  EXPECT_FALSE(observed_.j1_live.load()) << "the hardware still applies j1/velocity";
  EXPECT_FALSE(observed_.j2_live.load()) << "the hardware still applies j2/velocity";
  EXPECT_EQ(hardware().state, core::Lifecycle::Finalized);
}

// The held prepare of a restart hangs the component, so the fault is absorbed while the restart
// asks for the next activation of the controller that ran
TEST_F(SupervisorSlowCallbacks, a_fault_during_a_restart_names_the_cause_of_the_stop)
{
  start(testing::FINISH_TIMEOUT);
  load_and_configure("a", "t");
  ASSERT_TRUE(switch_controllers({"a"}, {}).ok);
  const testing::LogCapture log;
  observed_.prepare_gate.close();
  auto restart = std::async(std::launch::async, [&] { return switch_controllers({"a"}, {"a"}); });
  ASSERT_TRUE(observed_.prepare_gate.holds());
  const auto system = supervisor_->status()->hardware.front().runtime;
  ASSERT_TRUE(eventually([&] { return system->fault() != core::HardwareFault::None; }))
    << "precondition: the held prepare hangs the component";
  ASSERT_TRUE(cycles(5));
  observed_.prepare_gate.open();
  restart.get();
  EXPECT_TRUE(log.contains("Controller 'a' stopped with hardware 'system' (writes 'j1/velocity')"))
    << log.text();
}

// Ctrl-C while a hardware state request activates a component on a slow bus. The shutdown waits
// for that request instead of being dropped at its own timeout, so the hardware still goes down.
TEST_F(
  SupervisorSlowCallbacks, a_shutdown_behind_a_long_hardware_request_still_shuts_the_hardware_down)
{
  start(300ms);
  ASSERT_TRUE(
    supervisor_->set_hardware_state("system", core::Lifecycle::Inactive, testing::FINISH_TIMEOUT)
      .ok);
  observed_.activate_gate.close();
  auto activation = std::async(
    std::launch::async,
    [&]
    {
      return supervisor_->set_hardware_state(
        "system", core::Lifecycle::Active, testing::FINISH_TIMEOUT);
    });
  ASSERT_TRUE(observed_.activate_gate.holds());
  auto shutdown =
    std::async(std::launch::async, [&] { return supervisor_->shutdown(300ms); }).share();
  // Past the shutdown's timeout plus its grace
  EXPECT_EQ(shutdown.wait_for(1s), std::future_status::timeout) << shutdown.get().message;
  observed_.activate_gate.open();
  EXPECT_TRUE(activation.get().ok);
  const auto response = shutdown.get();
  EXPECT_TRUE(response.ok) << response.message;
  EXPECT_EQ(hardware().state, core::Lifecycle::Finalized);
}

// Protection keeps a controller from implicit stops only, a failure stops it like any other
TEST_F(
  SupervisorSlowCallbacks, a_protected_controller_stops_on_its_own_failure_and_on_a_hardware_fault)
{
  start(testing::FINISH_TIMEOUT);
  load_and_configure("a", "t", true);
  load_and_configure("b", "t", true);
  ASSERT_TRUE(switch_controllers({"a", "b"}, {}).ok);
  const auto active = [&](const std::string & name)
  { return controller(name)->state == core::Lifecycle::Active; };
  observed_.fail_update = true;
  EXPECT_TRUE(eventually([&] { return !active("a"); })) << "its own failure";
  observed_.fail_update = false;
  ASSERT_TRUE(settled());
  EXPECT_FALSE(active("a")) << "restarted after its failure";
  EXPECT_TRUE(active("b"));
  observed_.fail_write_once = true;
  EXPECT_TRUE(eventually([&] { return !active("b"); })) << "a fault of its hardware";
  ASSERT_TRUE(settled());
  EXPECT_FALSE(active("b")) << "restarted after the fault";
}

// The same bound while a configure that timed out returns during the switch and its cleanup hangs
TEST_F(SupervisorSlowCallbacks, a_switch_answers_in_time_while_a_late_configure_is_cleaned_up)
{
  start(testing::FINISH_TIMEOUT);
  load_and_configure("b", "t");
  ASSERT_TRUE(supervisor_->load("a", "t").ok);
  observed_.configure_gate.close();
  observed_.cleanup_gate.close();
  EXPECT_FALSE(supervisor_->configure("a").ok);
  observed_.controller_activate_gate.close();
  const auto timeout = 300ms;
  const auto begin = SteadyClock::now();
  auto pending =
    std::async(std::launch::async, [&] { return switch_controllers({"b"}, {}, timeout); });
  ASSERT_TRUE(observed_.controller_activate_gate.holds());
  observed_.configure_gate.open();
  // Only the held activation is left in a callback
  ASSERT_TRUE(eventually([] { return g_busy.load() == 1; })) << "precondition: configure returned";
  observed_.controller_activate_gate.open();
  const auto response = pending.get();
  const auto took = testing::elapsed_ms(begin);
  EXPECT_LE(took, answer_bound_ms(timeout)) << "switch took " << took << " ms";
  EXPECT_NE(response.message, RequestQueue::busy().message);
}

/// Delays of on_activate, of the hardware start and of on_deactivate in ms.
class SupervisorLateStart : public SupervisorSlowCallbacks,
                            public ::testing::WithParamInterface<std::array<int, 3>>
{
};

// perform(start) outlives the mode switch timeout and is given up. The switch rolls back and
// deactivates the controller. The late start must not make the hardware apply the stopped
// controller's last command, neither during nor after its on_deactivate.
TEST_P(SupervisorLateStart, a_late_start_does_not_apply_a_stopped_controllers_command)
{
  hang_timeout_ = 300ms;
  mode_switch_timeout_ = 100ms;
  start(testing::FINISH_TIMEOUT);
  load_and_configure("a", "t");
  observed_.activate_delay_ms = GetParam()[0];
  observed_.start_delay_ms = GetParam()[1];
  observed_.deactivate_delay_ms = GetParam()[2];
  EXPECT_FALSE(switch_controllers({"a"}, {}, 300ms).ok);
  ASSERT_TRUE(eventually([&] { return observed_.starts.load() > 0; }))
    << "precondition: the slow start was submitted";
  ASSERT_TRUE(eventually([&] { return !quarantined(); }));
  ASSERT_TRUE(settled());
  EXPECT_FALSE(observed_.j1_live.load());
  EXPECT_EQ(observed_.live_writes_while_deactivating.load(), 0)
    << "hardware applied j1/velocity of a detached, deactivating controller";
}

INSTANTIATE_TEST_SUITE_P(
  Delays, SupervisorLateStart,
  ::testing::Values(std::array<int, 3>{150, 200, 200}, std::array<int, 3>{150, 200, 0}));

// An unload whose on_shutdown hangs returns, then the CM is torn down. The late job of the
// abandoned shutdown must not call into the destroyed supervisor and host.
TEST_F(SupervisorSlowCallbacks, a_hung_unload_does_not_call_a_destroyed_host)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      alignas(Host) static unsigned char storage[sizeof(Host)];
      auto * host = new (storage) Host(observed_);
      // The leaked plugin's node still runs, so static destructors must not run under it
      const auto fail = [](const char * reason)
      {
        std::fputs(reason, stderr);
        std::quick_exit(1);
      };
      // Load and configure in the fresh child must not miss the deadline on a loaded machine
      start(testing::FINISH_TIMEOUT, host);
      if (!supervisor_->load("a", "t").ok || !supervisor_->configure("a").ok)
      {
        fail("precondition: load and configure failed");
      }
      observed_.shutdown_gate.close();
      if (const auto unloaded = supervisor_->unload("a");
          unloaded.ok || unloaded.message.find("quarantined") == std::string::npos)
      {
        fail("the unload was not given up");
      }
      loop_.stop();
      supervisor_.reset();
      host->~Host();
      observed_.shutdown_gate.open();
      if (!eventually([] { return g_busy.load() == 0; }))
      {
        fail("the abandoned on_shutdown did not return");
      }
      // Nothing shows that the rest of the abandoned job ran, a short wait can only miss a call
      std::this_thread::sleep_for(200ms);
      if (g_forget_after_destroy.load() > 0)
      {
        fail("the abandoned job called the destroyed host");
      }
      std::quick_exit(0);
    },
    ::testing::ExitedWithCode(0), "");
}

// A hang stops the claimers of a component, so an owner that stalls while no controller uses the
// component, for example on a slow bus, is no fault
TEST_F(SupervisorSlowCallbacks, a_stalled_async_owner_without_running_claimers_is_not_hung)
{
  start(
    2000ms, nullptr,
    testing::urdf({{"j1", {"velocity"}}, {"j2", {"velocity"}}}, R"(is_async="true")"));
  ASSERT_TRUE(supervisor_->status()->hardware.front().runtime->is_async())
    << "precondition: the owner writes";
  // Three hang timeouts
  observed_.write_delay_ms = 300;
  ASSERT_TRUE(eventually([&] { return observed_.delayed_writes.load() >= 1; }));
  observed_.write_delay_ms = 0;
  // A request first absorbs what RT reported
  ASSERT_TRUE(switch_controllers({}, {}).ok);
  EXPECT_EQ(hardware().state, core::Lifecycle::Active);
  EXPECT_EQ(hardware().fault, core::HardwareFault::None);
}

// The counterpart with a running claimer. RT faults the component and stops the claimer.
TEST_F(SupervisorSlowCallbacks, a_stalled_async_owner_with_a_running_claimer_faults_and_stops_it)
{
  start(
    2000ms, nullptr,
    testing::urdf({{"j1", {"velocity"}}, {"j2", {"velocity"}}}, R"(is_async="true")"));
  load_and_configure("a", "t");
  ASSERT_TRUE(switch_controllers({"a"}, {}).ok);
  const testing::LogCapture log;
  // Three hang timeouts
  observed_.write_delay_ms = 300;
  EXPECT_TRUE(eventually([&] { return controller("a")->state != core::Lifecycle::Active; }));
  observed_.write_delay_ms = 0;
  EXPECT_TRUE(
    log.contains("Hardware 'system' failed, its controllers stopped: no read or write ran"))
    << log.text();
  EXPECT_TRUE(log.contains("Controller 'a' stopped with hardware 'system' (writes 'j1/velocity')"))
    << log.text();
}

}  // namespace
}  // namespace hector_controller_manager
