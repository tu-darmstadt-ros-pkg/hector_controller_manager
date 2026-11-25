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
#include <sched.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "allocation_counter.hpp"
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
using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::return_type;
using testing::eventually;

const std::string URDF = testing::urdf({{"j1", {"position", "velocity"}}});

struct Observed
{
  std::atomic<int> stops{0};
  std::atomic<bool> velocity_live{false};
  std::atomic<double> applied{0.0};
  std::atomic<bool> fail_update{false};
  std::atomic<bool> fail_broadcaster_update{false};
  testing::Gate configure_gate;
  std::atomic<int> cleanups{0};
  testing::Gate hardware_deactivate_gate;
  std::atomic<int> hardware_activations{0};
  std::atomic<bool> throw_on_perform{false};
  std::atomic<bool> throw_on_hardware_activate{false};
  std::atomic<bool> refuse_activate{false};
  std::atomic<bool> throw_on_activate{false};
  std::atomic<bool> refuse_deactivate{false};
  std::atomic<bool> throw_on_deactivate{false};
  std::atomic<unsigned int> hardware_rw_rate{0};
  /// The controller's activation makes every later hardware read fail and returns after the first.
  std::atomic<bool> fail_read_on_activate{false};
  std::atomic<bool> fail_read{false};
  std::atomic<int> failed_reads{0};
  /// The fault of `fail_read_on_activate` did not come before the activation returned.
  std::atomic<bool> read_fault_missed{false};
  testing::Gate destroy_gate;
};

class System : public hardware_interface::SystemInterface
{
public:
  explicit System(Observed & observed) : observed_(observed) {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    observed_.hardware_rw_rate = get_hardware_info().rw_rate;
    observed_.hardware_activations.fetch_add(1);
    if (observed_.throw_on_hardware_activate)
    {
      throw std::runtime_error("4712");
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    observed_.hardware_deactivate_gate.pass();
    return CallbackReturn::SUCCESS;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (observed_.fail_read)
    {
      observed_.failed_reads.fetch_add(1);
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (observed_.velocity_live)
    {
      double value = 0.0;
      if (get_command(get_command_interface_handle("j1/velocity"), value, false))
      {
        observed_.applied = value;
      }
    }
    return return_type::OK;
  }

  return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {
    if (observed_.throw_on_perform)
    {
      throw std::runtime_error("4711");
    }
    for (const auto & name : start)
    {
      observed_.velocity_live = observed_.velocity_live || name == "j1/velocity";
    }
    for (const auto & name : stop)
    {
      if (name == "j1/velocity")
      {
        observed_.velocity_live = false;
        observed_.stops.fetch_add(1);
      }
    }
    return return_type::OK;
  }

private:
  Observed & observed_;
};

class Controller : public controller_interface::ControllerInterface
{
public:
  explicit Controller(Observed & observed) : observed_(observed) {}
  ~Controller() override { observed_.destroy_gate.pass(); }

  InterfaceConfiguration command_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, {"j1/velocity"}};
  }
  InterfaceConfiguration state_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, {"j1/position"}};
  }
  CallbackReturn on_init() override { return CallbackReturn::SUCCESS; }
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    observed_.configure_gate.pass();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    observed_.cleanups.fetch_add(1);
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (observed_.fail_read_on_activate)
    {
      // So the fault is there before the switch goes on, whenever RT reads
      observed_.fail_read = true;
      observed_.read_fault_missed =
        !testing::eventually([&] { return observed_.failed_reads > 0; }, std::chrono::seconds(1));
    }
    if (observed_.throw_on_activate)
    {
      throw std::runtime_error("4711");
    }
    return observed_.refuse_activate ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (observed_.throw_on_deactivate)
    {
      throw std::runtime_error("4713");
    }
    return observed_.refuse_deactivate ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (observed_.fail_update)
    {
      return controller_interface::return_type::ERROR;
    }
    (void)command_interfaces_[0].set_value(1.5);
    return controller_interface::return_type::OK;
  }

private:
  Observed & observed_;
};

/// Reads the joint position and commands nothing, like a joint state broadcaster.
class Broadcaster : public testing::NoInterfaces<>
{
public:
  explicit Broadcaster(Observed & observed) : observed_(observed) {}

  InterfaceConfiguration state_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, {"j1/position"}};
  }
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return observed_.fail_broadcaster_update ? controller_interface::return_type::ERROR
                                             : controller_interface::return_type::OK;
  }

private:
  Observed & observed_;
};

/// The system of URDF as an async component with these async properties.
std::string async_urdf(const std::string & properties)
{
  return testing::urdf({{"j1", {"velocity"}}}, R"(is_async="true")", "<async " + properties + "/>");
}

class SupervisorTest : public testing::SupervisorRig<>
{
protected:
  void SetUp() override
  {
    SupervisorConfig config;
    config.default_timeout = testing::FINISH_TIMEOUT;
    // Callbacks held here are released later, fault handling must not give a slow one up
    config.hang_timeout = testing::FINISH_TIMEOUT;
    start(config, host_, urdf(), status_listener());
    load_and_configure("controller", "test/Controller");
  }

  void cycle(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    testing::CountAllocations scope(counting_.load());
    SupervisorRig::cycle(time, period);
  }

  void TearDown() override
  {
    observed_.configure_gate.open();
    observed_.hardware_deactivate_gate.open();
    observed_.destroy_gate.open();
    stop();
    EXPECT_FALSE(observed_.read_fault_missed) << "precondition: the read fault came in on_activate";
  }

  virtual std::string urdf() const { return URDF; }
  virtual Supervisor::StatusListener status_listener() { return nullptr; }

  bool activate() { return switch_controllers({"controller"}, {}).ok; }

  core::Lifecycle state_of_controller() { return controller("controller")->state; }

  void deactivates_under_an_active_reader();
  void does_not_deactivate_under_an_active_commander();

  Observed observed_;
  testing::FakeHost host_{
    [this](const std::string &, const std::string & type) -> runtime::ControllerRuntime::Plugin
    {
      if (type == "test/Broadcaster")
      {
        return std::make_shared<Broadcaster>(observed_);
      }
      return std::make_shared<Controller>(observed_);
    },
    [this] { return std::make_unique<System>(observed_); }};
  std::atomic<bool> counting_{false};
};

TEST_F(SupervisorTest, an_activated_controller_reaches_the_hardware_after_its_first_update)
{
  ASSERT_TRUE(activate());
  EXPECT_TRUE(eventually([&] { return observed_.applied.load() == 1.5; }));
}

// The snapshot an unload replaces holds the last reference of the controller, whose plugin may take
// long to destroy. Readers of the status must not wait for that.
TEST_F(SupervisorTest, a_slow_destruction_after_an_unload_does_not_block_the_status)
{
  observed_.destroy_gate.close();
  auto unloaded = std::async(std::launch::async, [&] { return supervisor_->unload("controller"); });
  // Opens the gate before the unload's future waits for the unload
  const testing::OnExit open([&] { observed_.destroy_gate.open(); });
  ASSERT_TRUE(observed_.destroy_gate.holds()) << "precondition: the plugin is being destroyed";
  auto read = std::async(std::launch::async, [&] { return supervisor_->status() != nullptr; });
  const bool answered = read.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
  observed_.destroy_gate.open();
  EXPECT_TRUE(answered) << "the status waited for the destruction of the old snapshot";
  EXPECT_TRUE(unloaded.get().ok);
}

// The description sets no rw_rate, the legacy controller manager then used its update rate
TEST_F(SupervisorTest, hardware_without_an_rw_rate_gets_the_update_rate)
{
  EXPECT_EQ(observed_.hardware_rw_rate.load(), SupervisorConfig{}.update_rate);
}

TEST_F(SupervisorTest, an_update_error_stops_the_controller_and_holds_its_hardware)
{
  ASSERT_TRUE(activate());
  ASSERT_TRUE(eventually([&] { return observed_.velocity_live.load(); }));
  observed_.fail_update = true;
  EXPECT_TRUE(eventually([&] { return state_of_controller() == core::Lifecycle::Inactive; }));
  EXPECT_FALSE(observed_.velocity_live.load());
  EXPECT_EQ(observed_.stops.load(), 1);
}

TEST_F(SupervisorTest, a_failed_mode_switch_answers_with_the_plugin_reason)
{
  observed_.throw_on_perform = true;
  const auto response = switch_controllers({"controller"}, {});
  EXPECT_FALSE(response.ok);
  EXPECT_NE(response.message.find("system: A command mode switch threw: 4711"), std::string::npos)
    << response.message;
}

TEST_F(SupervisorTest, a_refused_activation_answers_with_the_callback_and_state)
{
  observed_.refuse_activate = true;
  const auto response = switch_controllers({"controller"}, {});
  EXPECT_FALSE(response.ok);
  EXPECT_NE(response.message.find("on_activate returned FAILURE"), std::string::npos)
    << response.message;
}

TEST_F(SupervisorTest, a_throwing_activation_answers_with_the_callback_and_state)
{
  observed_.throw_on_activate = true;
  const auto response = switch_controllers({"controller"}, {});
  EXPECT_FALSE(response.ok);
  EXPECT_NE(
    response.message.find("on_activate returned ERROR or threw, the controller is unconfigured"),
    std::string::npos)
    << response.message;
}

TEST_F(SupervisorTest, a_refused_stop_answers_that_the_controller_refused)
{
  ASSERT_TRUE(activate());
  observed_.refuse_deactivate = true;
  const auto response = switch_controllers({}, {"controller"});
  EXPECT_NE(response.message.find("refused to stop"), std::string::npos) << response.message;
}

// The refused activation fails the switch, whose answer then also names the fault it absorbed
TEST_F(SupervisorTest, a_hardware_fault_names_its_reason_in_the_log_and_the_answer)
{
  observed_.fail_read_on_activate = true;
  observed_.refuse_activate = true;
  const testing::LogCapture log;
  const auto response = switch_controllers({"controller"}, {});
  EXPECT_FALSE(response.ok);
  EXPECT_NE(
    response.message.find("Hardware 'system' failed: read returned ERROR"), std::string::npos)
    << response.message;
  EXPECT_TRUE(
    log.contains("Hardware 'system' failed, its controllers stopped: read returned ERROR"))
    << log.text();
}

TEST_F(SupervisorTest, a_hardware_fault_names_the_claim_of_each_controller_it_stopped)
{
  ASSERT_TRUE(activate());
  const testing::LogCapture log;
  observed_.fail_read = true;
  EXPECT_TRUE(eventually(
    [&]
    {
      return log.contains(
        "Controller 'controller' stopped with hardware 'system' (writes 'j1/velocity')");
    }))
    << log.text();
}

// The controller activates, the fault then stops it before the switch answers
TEST_F(SupervisorTest, a_hardware_fault_that_stops_a_started_controller_fails_the_switch)
{
  observed_.fail_read_on_activate = true;
  const auto response = switch_controllers({"controller"}, {});
  EXPECT_FALSE(response.ok);
  EXPECT_NE(
    response.message.find("Hardware 'system' failed: read returned ERROR"), std::string::npos)
    << response.message;
  EXPECT_NE(state_of_controller(), core::Lifecycle::Active);
}

TEST_F(SupervisorTest, a_throwing_stop_answers_with_the_callback_and_state)
{
  ASSERT_TRUE(activate());
  observed_.throw_on_deactivate = true;
  const auto response = switch_controllers({}, {"controller"});
  EXPECT_NE(
    response.message.find("on_deactivate returned ERROR or threw, the controller is unconfigured"),
    std::string::npos)
    << response.message;
}

TEST_F(SupervisorTest, a_failed_hardware_transition_answers_with_the_plugin_reason)
{
  ASSERT_TRUE(
    supervisor_->set_hardware_state("system", core::Lifecycle::Inactive, testing::FINISH_TIMEOUT)
      .ok);
  observed_.throw_on_hardware_activate = true;
  const auto response =
    supervisor_->set_hardware_state("system", core::Lifecycle::Active, testing::FINISH_TIMEOUT);
  EXPECT_FALSE(response.ok);
  EXPECT_NE(response.message.find("on_activate threw: 4712"), std::string::npos)
    << response.message;
}

void SupervisorTest::deactivates_under_an_active_reader()
{
  load_and_configure("broadcaster", "test/Broadcaster");
  ASSERT_TRUE(switch_controllers({"broadcaster"}, {}).ok);
  const auto response =
    supervisor_->set_hardware_state("system", core::Lifecycle::Inactive, testing::FINISH_TIMEOUT);
  EXPECT_TRUE(response.ok) << response.message;
  EXPECT_EQ(hardware().state, core::Lifecycle::Inactive);
  EXPECT_EQ(controller("broadcaster")->state, core::Lifecycle::Active);
}

void SupervisorTest::does_not_deactivate_under_an_active_commander()
{
  ASSERT_TRUE(activate());
  const auto response =
    supervisor_->set_hardware_state("system", core::Lifecycle::Inactive, testing::FINISH_TIMEOUT);
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.message, "Hardware 'system' is commanded by 'controller'");
  EXPECT_EQ(hardware().state, core::Lifecycle::Active);
}

/// The fixture with the system as an async component.
class AsyncSupervisorTest : public SupervisorTest
{
protected:
  std::string urdf() const override { return async_urdf(""); }
};

// A reader stays active on inactive hardware, as upstream leaves it
TEST_F(SupervisorTest, hardware_deactivates_under_an_active_reader)
{
  deactivates_under_an_active_reader();
}

TEST_F(AsyncSupervisorTest, hardware_deactivates_under_an_active_reader)
{
  deactivates_under_an_active_reader();
}

TEST_F(SupervisorTest, hardware_does_not_deactivate_under_an_active_commander)
{
  does_not_deactivate_under_an_active_commander();
}

TEST_F(AsyncSupervisorTest, hardware_does_not_deactivate_under_an_active_commander)
{
  does_not_deactivate_under_an_active_commander();
}

constexpr char STATUS_ERROR[] = "4714";

/// The statuses the supervisor publishes throw while `throw_on_status_` says so.
class SupervisorThrowTest : public SupervisorTest
{
protected:
  enum class StatusThrow
  {
    Never,
    Always,
    /// Every status that shows the broadcaster not active.
    BroadcasterStopped,
    /// The first status that shows the broadcaster not active, after it waited for the close of
    /// `closing_`.
    BroadcasterStoppedAfterClose
  };

  Supervisor::StatusListener status_listener() override
  {
    return [this](const std::shared_ptr<const Status> & status)
    {
      const auto when = throw_on_status_.load();
      const bool stopped = std::any_of(
        status->controllers.begin(), status->controllers.end(), [](const ControllerStatus & entry)
        { return entry.name == "broadcaster" && entry.model.state != core::Lifecycle::Active; });
      if (when == StatusThrow::BroadcasterStoppedAfterClose && stopped)
      {
        throw_on_status_ = StatusThrow::Never;
        waits_for_close_ = true;
        EXPECT_TRUE(eventually([&] { return closing_.load()->requests_waiting(); }));
        throw std::runtime_error(STATUS_ERROR);
      }
      if (when == StatusThrow::Always || (when == StatusThrow::BroadcasterStopped && stopped))
      {
        throw std::runtime_error(STATUS_ERROR);
      }
    };
  }

  void TearDown() override
  {
    throw_on_status_ = StatusThrow::Never;
    SupervisorTest::TearDown();
  }

  /// Fails the broadcaster in RT while the controller commands, every status from `when` on
  /// throws. Returns once RT held the controller's command and the supervisor stayed quiet
  /// without a wake.
  void hold_under_repeated_throws(StatusThrow when, const testing::LogCapture & log)
  {
    load_and_configure("broadcaster", "test/Broadcaster");
    ASSERT_TRUE(switch_controllers({"controller", "broadcaster"}, {}).ok);
    ASSERT_TRUE(eventually([&] { return observed_.velocity_live.load(); }));
    ASSERT_TRUE(settled());
    throw_on_status_ = when;
    observed_.fail_broadcaster_update = true;
    EXPECT_TRUE(eventually([&] { return !observed_.velocity_live.load(); }))
      << "the command claimer kept commanding, " << log.text();
    // The stop's own events come within a few cycles, a spinning supervisor keeps throwing
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const size_t thrown = log.count(std::string("threw: ") + STATUS_ERROR);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    EXPECT_EQ(log.count(std::string("threw: ") + STATUS_ERROR), thrown) << log.text();
  }

  /// The wake of the request retries the work that threw, so it comes after the log checks.
  void rejects_switches(const testing::LogCapture & log)
  {
    const size_t thrown = log.count(std::string("threw: ") + STATUS_ERROR);
    EXPECT_EQ(
      switch_controllers({"broadcaster"}, {}).message,
      "A broken invariant stopped the controller manager, restart it");
    EXPECT_TRUE(
      eventually([&] { return log.count(std::string("threw: ") + STATUS_ERROR) > thrown; }))
      << "precondition: a wake throws again, so the quiet log before was no missed throw";
  }

  std::atomic<StatusThrow> throw_on_status_{StatusThrow::Never};
  /// The supervisor under destruction, `supervisor_` is null meanwhile.
  std::atomic<Supervisor *> closing_{nullptr};
  std::atomic<bool> waits_for_close_{false};
};

// Every status throws from the one published when the supervisor absorbs the failure on
TEST_F(SupervisorThrowTest, a_throw_in_fault_handling_holds_everything)
{
  if (runtime::CHECKED_BUILD)
  {
    GTEST_SKIP() << "A checked build aborts instead";
  }
  const testing::LogCapture log;
  hold_under_repeated_throws(StatusThrow::Always, log);
  EXPECT_TRUE(log.contains(std::string("Fault handling threw: ") + STATUS_ERROR)) << log.text();
  EXPECT_FALSE(log.contains("Idle work threw")) << log.text();
  rejects_switches(log);
}

// Every status throws from the one published after the idle work stopped the broadcaster on
TEST_F(SupervisorThrowTest, a_throw_in_idle_work_holds_everything)
{
  if (runtime::CHECKED_BUILD)
  {
    GTEST_SKIP() << "A checked build aborts instead";
  }
  const testing::LogCapture log;
  hold_under_repeated_throws(StatusThrow::BroadcasterStopped, log);
  // The hold rings from RT and from the hardware owner, and a wake that finds nothing retries
  EXPECT_NE(
    log.first(std::string("threw: ") + STATUS_ERROR).find("Idle work threw"), std::string::npos)
    << log.text();
  EXPECT_TRUE(log.contains("Controller 'controller' stopped by holding everything")) << log.text();
  EXPECT_FALSE(log.contains("stopped by the e-stop")) << log.text();
  rejects_switches(log);
}

// The close rings while the work that throws runs, so the wait after the throw must not miss it
TEST_F(SupervisorThrowTest, a_close_during_work_that_throws_ends_the_supervisor)
{
  if (runtime::CHECKED_BUILD)
  {
    GTEST_SKIP() << "A checked build aborts instead";
  }
  load_and_configure("broadcaster", "test/Broadcaster");
  ASSERT_TRUE(switch_controllers({"broadcaster"}, {}).ok);
  ASSERT_TRUE(settled());
  closing_ = supervisor_.get();
  throw_on_status_ = StatusThrow::BroadcasterStoppedAfterClose;
  observed_.fail_broadcaster_update = true;
  ASSERT_TRUE(eventually([&] { return waits_for_close_.load(); }));
  loop_.stop();
  // A hanging destructor must fail the test, not block it
  auto destroyed = std::make_shared<std::promise<void>>();
  std::thread(
    [this, destroyed]
    {
      supervisor_.reset();
      destroyed->set_value();
    })
    .detach();
  ASSERT_EQ(destroyed->get_future().wait_for(testing::FINISH_TIMEOUT), std::future_status::ready)
    << "the supervisor thread missed the close";
}

TEST_F(SupervisorTest, a_configure_that_succeeds_after_its_timeout_is_cleaned_up)
{
  ASSERT_TRUE(supervisor_->load("slow", "test/Controller").ok);
  observed_.configure_gate.close();
  EXPECT_FALSE(supervisor_->configure("slow").ok);
  const auto slow = [&] { return supervisor_->status()->controllers.back(); };
  ASSERT_EQ(slow().name, "slow");
  ASSERT_TRUE(slow().model.quarantined) << "precondition: the configure still runs";
  observed_.configure_gate.open();
  EXPECT_TRUE(eventually([&] { return observed_.cleanups.load() == 1; }));
  EXPECT_TRUE(eventually(
    [&]
    { return slow().model.state == core::Lifecycle::Unconfigured && !slow().model.quarantined; }));
}

TEST_F(SupervisorTest, every_controller_lifecycle_request_after_the_shutdown_is_rejected)
{
  ASSERT_TRUE(supervisor_->load("unconfigured", "test/Controller").ok);
  ASSERT_TRUE(supervisor_->shutdown(testing::FINISH_TIMEOUT).ok);
  for (const auto & response :
       {supervisor_->load("late", "test/Controller"), supervisor_->configure("unconfigured"),
        supervisor_->cleanup("controller"), supervisor_->unload("controller")})
  {
    EXPECT_FALSE(response.ok);
    EXPECT_EQ(response.message, "The controller manager is shut down");
  }
  EXPECT_EQ(observed_.cleanups.load(), 0);
}

TEST_F(SupervisorTest, hardware_that_stops_after_the_shutdown_deadline_still_goes_down)
{
  observed_.hardware_deactivate_gate.close();
  EXPECT_FALSE(supervisor_->shutdown(std::chrono::milliseconds(200)).ok);
  const auto system = [&] { return supervisor_->status()->hardware.front().model; };
  ASSERT_TRUE(system().quarantined) << "precondition: the deactivation still runs";
  observed_.hardware_deactivate_gate.open();
  EXPECT_TRUE(eventually(
    [&] { return system().state == core::Lifecycle::Finalized && !system().quarantined; }));
  EXPECT_EQ(observed_.hardware_activations.load(), 1);
}

TEST_F(SupervisorTest, the_control_loop_does_not_allocate)
{
  if (!testing::allocation_counting_available())
  {
    GTEST_SKIP() << "Sanitizers replace operator new";
  }
  ASSERT_TRUE(activate());
  ASSERT_TRUE(eventually([&] { return observed_.velocity_live.load(); }));
  const uint64_t before = testing::counted_allocations();
  counting_ = true;
  EXPECT_TRUE(cycles(100));
  observed_.fail_update = true;
  EXPECT_TRUE(eventually([&] { return state_of_controller() == core::Lifecycle::Inactive; }));
  EXPECT_FALSE(activate()) << "the new activation fails in its first update";
  EXPECT_TRUE(cycles(100));
  counting_ = false;
  EXPECT_EQ(testing::counted_allocations() - before, 0u);
}

// RT records the reason of a hardware fault without allocating
TEST_F(SupervisorTest, a_hardware_fault_does_not_allocate_in_the_control_loop)
{
  if (!testing::allocation_counting_available())
  {
    GTEST_SKIP() << "Sanitizers replace operator new";
  }
  ASSERT_TRUE(activate());
  ASSERT_TRUE(eventually([&] { return observed_.velocity_live.load(); }));
  const uint64_t before = testing::counted_allocations();
  counting_ = true;
  observed_.fail_read = true;
  EXPECT_TRUE(eventually([&] { return state_of_controller() == core::Lifecycle::Inactive; }));
  counting_ = false;
  EXPECT_EQ(testing::counted_allocations() - before, 0u);
}

TEST_F(SupervisorTest, an_asynchronous_component_is_created_on_an_owner_with_its_cores)
{
  cpu_set_t allowed;
  ASSERT_EQ(sched_getaffinity(0, sizeof(allowed), &allowed), 0);
  int core = CPU_SETSIZE - 1;
  while (!CPU_ISSET(core, &allowed))
  {
    --core;
  }
  Observed observed;
  cpu_set_t created;
  CPU_ZERO(&created);
  testing::FakeHost host{
    nullptr, [&]
    {
      sched_getaffinity(0, sizeof(created), &created);
      return std::make_unique<System>(observed);
    }};
  Supervisor supervisor(SupervisorConfig{}, host, rclcpp::get_logger("test"), nullptr);
  const auto response =
    supervisor.initialize(async_urdf("affinity=\"[" + std::to_string(core) + "]\""), {});
  EXPECT_TRUE(response.ok) << response.message;
  EXPECT_EQ(CPU_COUNT(&created), 1);
  EXPECT_TRUE(CPU_ISSET(core, &created));
  supervisor.shutdown(testing::FINISH_TIMEOUT);
}

TEST_F(SupervisorTest, a_detached_asynchronous_component_is_rejected)
{
  Observed observed;
  testing::FakeHost host{nullptr, [&] { return std::make_unique<System>(observed); }};
  Supervisor supervisor(SupervisorConfig{}, host, rclcpp::get_logger("test"), nullptr);
  const auto response = supervisor.initialize(async_urdf("scheduling_policy=\"detached\""), {});
  EXPECT_FALSE(response.ok);
  EXPECT_NE(
    response.message.find("Hardware 'system' sets the scheduling_policy 'detached'"),
    std::string::npos)
    << response.message;
  supervisor.shutdown(testing::FINISH_TIMEOUT);
}

const std::string DUPLICATE_URDF =
  testing::urdf_of({{"first", {{"j1", {"velocity"}}}}, {"second", {{"j1", {"velocity"}}}}});

/// 0 if a fresh supervisor rejects the description, which it destroys again before returning.
int reject_duplicate_interfaces()
{
  Observed observed;
  testing::FakeHost host{nullptr, [&observed] { return std::make_unique<System>(observed); }};
  Supervisor supervisor(SupervisorConfig{}, host, rclcpp::get_logger("test"), nullptr);
  return supervisor.initialize(DUPLICATE_URDF, {}).ok ? 1 : 0;
}

// Two components export the same interface. The description must be rejected, not crash the CM.
// Without a fixture, whose controller nodes would still run DDS threads under the child's exit.
TEST(SupervisorDescription, a_description_with_duplicate_interfaces_is_rejected_without_a_crash)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(std::exit(reject_duplicate_interfaces()), ::testing::ExitedWithCode(0), "");
}

}  // namespace
}  // namespace hector_controller_manager
