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

// Supervisor tests for quarantine, late returns, teardown under stuck calls, failed switches and
// the e-stop, with chained controllers and a controller that can block RT.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
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
using namespace std::chrono_literals;
using testing::eventually;

const std::string URDF =
  testing::urdf({{"j1", {"velocity"}}, {"j2", {"velocity"}}, {"j3", {"velocity"}}});

/// What the user's update throws.
enum class Throw
{
  Nothing,
  Exception,
  Int,
  LongText
};

struct Observed
{
  std::atomic<bool> block_update{false};
  std::atomic<bool> in_blocked_update{false};
  testing::Gate cleanup_gate;
  testing::Gate shutdown_gate;
  /// Callbacks at a gate. A quarantined one outlives its request, not the fixture.
  std::atomic<int> held{0};
  std::atomic<int> controllers_destroyed{0};
  testing::Gate load_gate;
  testing::Gate hardware_gate;
  testing::Gate chain_gate;
  testing::Gate user_activate_gate;
  std::atomic<int> user_deactivations{0};
  std::atomic<bool> fail_user_update{false};
  std::atomic<bool> reject_prepare{false};
  std::atomic<bool> fail_exporter_update{false};
  std::atomic<Throw> throw_from_user_update{Throw::Nothing};
  /// The provider's reference in its first update after an activation.
  std::atomic<double> first_reference{0.0};
};

void hold(Observed & observed, testing::Gate & gate)
{
  const testing::InFlight held(observed.held);
  gate.pass();
}

class System : public hardware_interface::SystemInterface
{
public:
  explicit System(Observed & observed) : observed_(observed) {}
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return return_type::OK;
  }
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return return_type::OK;
  }
  return_type prepare_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override
  {
    return observed_.reject_prepare ? return_type::ERROR : return_type::OK;
  }
  return_type perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override
  {
    return return_type::OK;
  }

private:
  Observed & observed_;
};

/// Commands `<joint>/velocity`. The one on j3 can block RT inside its update.
class Controller : public testing::NoInterfaces<>
{
public:
  Controller(Observed & observed, std::string joint, bool may_block)
  : observed_(observed), joint_(std::move(joint)), may_block_(may_block)
  {
  }
  ~Controller() override { observed_.controllers_destroyed.fetch_add(1); }
  InterfaceConfiguration command_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, {joint_ + "/velocity"}};
  }
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    hold(observed_, observed_.shutdown_gate);
    return CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    while (may_block_ && observed_.block_update.load())
    {
      observed_.in_blocked_update = true;
      std::this_thread::sleep_for(1ms);
    }
    observed_.in_blocked_update = false;
    return controller_interface::return_type::OK;
  }

private:
  Observed & observed_;
  std::string joint_;
  bool may_block_;
};

/// Exports the state interface "<name>/position". Named "j1", it collides with the hardware state
/// j1/position, so registering its exports fails after a successful configure.
class Exporter : public testing::NoInterfacesChainable
{
public:
  explicit Exporter(Observed & observed) : observed_(observed) {}
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    hold(observed_, observed_.cleanup_gate);
    return CallbackReturn::SUCCESS;
  }

protected:
  std::vector<hardware_interface::StateInterface::SharedPtr> on_export_state_interfaces_list()
    override
  {
    hardware_interface::InterfaceInfo info{};
    info.name = "position";
    return {std::make_shared<hardware_interface::StateInterface>(
      hardware_interface::InterfaceDescription(get_node()->get_name(), info))};
  }
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return observed_.fail_exporter_update ? controller_interface::return_type::ERROR
                                          : controller_interface::return_type::OK;
  }

private:
  Observed & observed_;
};

/// Reads the state "e/position" of the exporter named "e", its activation can be held.
class Reader : public testing::NoInterfaces<>
{
public:
  explicit Reader(Observed & observed) : observed_(observed) {}
  InterfaceConfiguration state_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, {"e/position"}};
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    hold(observed_, observed_.user_activate_gate);
    return CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return controller_interface::return_type::OK;
  }

private:
  Observed & observed_;
};

/// Exports the reference "<name>/x", its chained mode change can be held.
class Provider : public testing::NoInterfacesChainable
{
public:
  explicit Provider(Observed & observed) : observed_(observed) {}
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    first_update_ = true;
    return CallbackReturn::SUCCESS;
  }

protected:
  bool on_set_chained_mode(bool) override
  {
    hold(observed_, observed_.chain_gate);
    return true;
  }
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list()
    override
  {
    hardware_interface::InterfaceInfo info{};
    info.name = "x";
    reference_ = std::make_shared<hardware_interface::CommandInterface>(
      hardware_interface::InterfaceDescription(get_node()->get_name(), info));
    return {reference_};
  }
  /// As from a topic while unchained
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    (void)reference_->set_value(7.0);
    return controller_interface::return_type::OK;
  }
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (first_update_)
    {
      first_update_ = false;
      observed_.first_reference = reference_->get_optional<double>().value_or(0.0);
    }
    return controller_interface::return_type::OK;
  }

private:
  Observed & observed_;
  hardware_interface::CommandInterface::SharedPtr reference_;
  bool first_update_ = false;
};

/// Writes the reference "p/x" of the provider named "p", its activation can be held.
class User : public testing::NoInterfaces<>
{
public:
  explicit User(Observed & observed) : observed_(observed) {}
  InterfaceConfiguration command_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, {"p/x"}};
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    hold(observed_, observed_.user_activate_gate);
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    observed_.user_deactivations.fetch_add(1);
    return CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    switch (observed_.throw_from_user_update.load())
    {
      case Throw::Nothing:
        break;
      case Throw::Exception:
        throw std::runtime_error("clock 4711");
      case Throw::Int:
        throw 4711;
      case Throw::LongText:
        throw std::runtime_error(std::string(300, 'x'));
    }
    return observed_.fail_user_update ? controller_interface::return_type::ERROR
                                      : controller_interface::return_type::OK;
  }

private:
  Observed & observed_;
};

/// Creates the fake of a type, "test/Controller_<joint>" commands that joint and the j3 one may
/// block RT.
runtime::ControllerRuntime::Plugin create(Observed & observed, const std::string & type)
{
  hold(observed, observed.load_gate);
  if (type == "test/Exporter")
  {
    return std::make_shared<Exporter>(observed);
  }
  if (type == "test/Provider")
  {
    return std::make_shared<Provider>(observed);
  }
  if (type == "test/User")
  {
    return std::make_shared<User>(observed);
  }
  if (type == "test/Reader")
  {
    return std::make_shared<Reader>(observed);
  }
  const std::string joint = type.substr(type.size() - 2);
  return std::make_shared<Controller>(observed, joint, joint == "j3");
}

class SupervisorFailures : public testing::SupervisorRig<>
{
protected:
  void SetUp() override
  {
    SupervisorConfig config;
    config.default_timeout = testing::FINISH_TIMEOUT;
    start(config, host_, URDF);
  }

  void cycle(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    if (estop_seen_ < 0 && supervisor_->rt().estop())
    {
      estop_seen_ = loop_.cycles();
    }
    SupervisorRig::cycle(time, period);
    if (const auto * watched = watched_.load();
        watched && stopped_at_ < 0 &&
        watched->failed_activation.load() == watched_activation_.load())
    {
      stopped_at_ = loop_.cycles() + 1;
    }
  }

  void TearDown() override
  {
    observed_.block_update = false;
    for (auto * gate :
         {&observed_.cleanup_gate, &observed_.shutdown_gate, &observed_.load_gate,
          &observed_.hardware_gate, &observed_.chain_gate, &observed_.user_activate_gate})
    {
      gate->open();
    }
    EXPECT_TRUE(eventually([&] { return observed_.held.load() == 0; }, 10s));
    stop();
  }

  /// Blocks RT inside the update of the controller on j3.
  void block_rt()
  {
    observed_.block_update = true;
    ASSERT_TRUE(eventually([&] { return observed_.in_blocked_update.load(); }));
  }

  void engage() { supervisor_->set_estop(true, "'/estop', it says engaged"); }

  Observed observed_;
  testing::FakeHost host_{
    [this](const std::string &, const std::string & type) { return create(observed_, type); },
    [this]
    {
      hold(observed_, observed_.hardware_gate);
      return std::make_unique<System>(observed_);
    }};
  /// The cycles before the first that saw the e-stop and before the watched activation stopped.
  /// RT counts them itself, so a slow test thread does not stretch them.
  std::atomic<int> estop_seen_{-1};
  std::atomic<const runtime::ControllerRuntime *> watched_{nullptr};
  std::atomic<uint64_t> watched_activation_{0};
  std::atomic<int> stopped_at_{-1};
};

// Two deactivations whose Detach both miss quiescence while RT is stuck in an update. The
// controller leaves quarantine once its blocker ended.
TEST_F(SupervisorFailures, a_second_unquiesced_detach_must_not_strand_the_first)
{
  load_and_configure("a", "test/Controller_j1");
  load_and_configure("b", "test/Controller_j2");
  load_and_configure("stuck", "test/Controller_j3");
  ASSERT_TRUE(switch_controllers({"a", "b", "stuck"}, {}).ok);

  block_rt();
  switch_controllers({}, {"a"}, 300ms);
  ASSERT_TRUE(controller("a")->quarantined) << "precondition: a's Detach missed quiescence";
  switch_controllers({}, {"b"}, 300ms);
  ASSERT_TRUE(controller("b")->quarantined) << "precondition: b's Detach missed quiescence";

  observed_.block_update = false;
  EXPECT_TRUE(eventually([&] { return !controller("b")->quarantined; }));
  EXPECT_TRUE(eventually([&] { return !controller("a")->quarantined; }))
    << "a stays quarantined forever";
  EXPECT_TRUE(switch_controllers({}, {"a"}).ok) << "every request naming a is rejected";
}

// A plan retained while RT was stuck must not keep an unloaded plugin alive after unload returned
TEST_F(SupervisorFailures, unload_destroys_the_plugin_after_rt_was_stuck)
{
  load_and_configure("a", "test/Controller_j1");
  load_and_configure("stuck", "test/Controller_j3");
  ASSERT_TRUE(switch_controllers({"a", "stuck"}, {}).ok);
  block_rt();
  switch_controllers({}, {"a"}, 300ms);
  ASSERT_TRUE(controller("a")->quarantined) << "precondition: a's Detach missed quiescence";
  observed_.block_update = false;
  ASSERT_TRUE(eventually([&] { return !controller("a")->quarantined; }))
    << "RT resuming ends the quarantine without another request";
  switch_controllers({}, {"a"});
  ASSERT_EQ(controller("a")->state, core::Lifecycle::Inactive);
  ASSERT_TRUE(supervisor_->unload("a").ok);
  EXPECT_EQ(observed_.controllers_destroyed.load(), 1);
}

// Controllers can be loaded before the robot description arrived, when no plan exists yet
TEST_F(SupervisorFailures, unload_before_any_plan_was_published_is_accepted)
{
  SupervisorConfig config;
  config.default_timeout = testing::FINISH_TIMEOUT;
  Supervisor fresh(config, host_, rclcpp::get_logger("test"), nullptr);
  ASSERT_TRUE(fresh.load("early", "test/Controller_j1").ok);
  EXPECT_TRUE(fresh.unload("early").ok);
  fresh.shutdown(testing::FINISH_TIMEOUT);
}

// A load that returns after its deadline created a plugin nobody owns, the host forgets it
TEST_F(SupervisorFailures, a_load_that_returns_late_is_forgotten)
{
  SupervisorConfig config;
  config.default_timeout = 300ms;
  Supervisor fresh(config, host_, rclcpp::get_logger("test"), nullptr);
  observed_.load_gate.close();
  EXPECT_FALSE(fresh.load("late", "test/Controller_j1").ok);
  observed_.load_gate.open();
  EXPECT_TRUE(eventually([&] { return host_.forgotten.load() > 0; }));
  EXPECT_EQ(host_.forgotten.load(), 1);
  const auto status = fresh.status();
  EXPECT_TRUE(status->controllers.empty());
  fresh.shutdown(testing::FINISH_TIMEOUT);
}

// A load that waited in the queue behind a slow cleanup still gets its whole timeout
TEST_F(SupervisorFailures, a_load_that_waited_in_the_queue_gets_its_whole_timeout)
{
  SupervisorConfig config;
  config.default_timeout = 1s;
  Supervisor fresh(config, host_, rclcpp::get_logger("test"), nullptr);
  ASSERT_TRUE(fresh.load("e", "test/Exporter").ok);
  ASSERT_TRUE(fresh.configure("e").ok);
  observed_.cleanup_gate.close();
  auto cleanup = std::async(std::launch::async, [&] { return fresh.cleanup("e"); });
  ASSERT_TRUE(observed_.cleanup_gate.holds());
  observed_.load_gate.close();
  const auto queued = std::chrono::steady_clock::now();
  auto load = std::async(std::launch::async, [&] { return fresh.load("b", "test/Controller_j2"); });
  std::this_thread::sleep_until(queued + 700ms);
  observed_.cleanup_gate.open();
  ASSERT_TRUE(cleanup.get().ok) << "precondition: the cleanup held the supervisor";
  ASSERT_TRUE(observed_.load_gate.holds()) << "precondition: the load runs";
  // Past the deadline the load had from its push, within the timeout from its start
  std::this_thread::sleep_until(queued + 1200ms);
  observed_.load_gate.open();
  const auto loaded = load.get();
  EXPECT_TRUE(loaded.ok) << loaded.message;
  EXPECT_EQ(host_.forgotten.load(), 0);
  fresh.shutdown(testing::FINISH_TIMEOUT);
}

// An abandoned hardware initialization still runs in the host, an unrelated load must not wait
TEST_F(SupervisorFailures, a_hung_hardware_initialization_does_not_hold_up_a_load)
{
  SupervisorConfig config;
  config.default_timeout = testing::FINISH_TIMEOUT;
  config.hardware_init_timeout = 300ms;
  Supervisor fresh(config, host_, rclcpp::get_logger("test"), nullptr);
  observed_.hardware_gate.close();
  const auto initialized = fresh.initialize(URDF, {});
  EXPECT_FALSE(initialized.ok);
  EXPECT_NE(initialized.message.find("did not finish in time"), std::string::npos)
    << initialized.message;
  // A load waiting for the held initialization would miss its deadline
  EXPECT_TRUE(fresh.load("other", "test/Controller_j1").ok);
  observed_.hardware_gate.open();
  // The supervisor may go only once the host call returned, see the test below
  EXPECT_TRUE(eventually([&] { return observed_.held.load() == 0; }));
  fresh.shutdown(testing::FINISH_TIMEOUT);
}

// The host goes with the supervisor, a call into it that outlives the teardown must not return
TEST_F(SupervisorFailures, teardown_behind_a_running_host_call_aborts)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_DEATH(
    {
      SupervisorConfig config;
      config.default_timeout = 100ms;
      config.hardware_init_timeout = 50ms;
      observed_.hardware_gate.close();
      auto fresh = std::make_unique<Supervisor>(config, host_, rclcpp::get_logger("test"), nullptr);
      (void)fresh->initialize(URDF, {});
      fresh.reset();
    },
    "still runs while the supervisor goes");
}

// A supervisor torn down while RT is stuck in a call leaves what RT uses alive, without a crash
TEST_F(SupervisorFailures, teardown_while_rt_is_in_a_call_leaves_rt_safe)
{
  SupervisorConfig config;
  config.default_timeout = testing::FINISH_TIMEOUT;
  auto fresh = std::make_unique<Supervisor>(config, host_, rclcpp::get_logger("test"), nullptr);
  auto * rt = &fresh->rt();
  std::atomic<bool> stop{false};
  testing::RtThread loop(
    [&](const rclcpp::Time & time, const rclcpp::Duration & period)
    {
      if (!stop)
      {
        testing::read_update_write(*rt, time, period);
      }
    });
  ASSERT_TRUE(fresh->initialize(URDF, {}).ok);
  ASSERT_TRUE(fresh->load("stuck", "test/Controller_j3").ok);
  ASSERT_TRUE(fresh->configure("stuck").ok);
  ASSERT_TRUE(
    fresh->switch_controllers({{"stuck"}, {}, core::Strictness::Strict}, testing::FINISH_TIMEOUT)
      .ok);
  block_rt();
  stop = true;
  fresh.reset();
  // RT returns from the stuck update into the abandoned plan and slot, and leaves its loop
  observed_.block_update = false;
  loop.stop();
}

// A chained mode change that returns after its deadline still changed the plugin's mode
TEST_F(SupervisorFailures, a_late_chained_mode_change_is_recorded)
{
  load_and_configure("p", "test/Provider");
  load_and_configure("u", "test/User");
  observed_.chain_gate.close();
  EXPECT_FALSE(switch_controllers({"p", "u"}, {}, 300ms).ok);
  ASSERT_TRUE(controller("p")->quarantined) << "precondition: the chained mode change is late";
  observed_.chain_gate.open();
  ASSERT_TRUE(eventually([&] { return !controller("p")->quarantined; }));
  EXPECT_TRUE(controller("p")->chained) << "the plugin is chained, the model has to say so";
}

// The user's activation succeeds after the deadline, when the rollback already stopped its
// provider. The late activation is ended at once, so the user never counts as active over a stopped
// provider.
TEST_F(SupervisorFailures, a_late_activation_over_a_stopped_provider_is_deactivated_at_once)
{
  load_and_configure("p", "test/Provider");
  load_and_configure("u", "test/User");
  observed_.user_activate_gate.close();
  EXPECT_FALSE(switch_controllers({"p", "u"}, {}, 300ms).ok);
  ASSERT_EQ(controller("p")->state, core::Lifecycle::Inactive) << "precondition: p rolled back";
  ASSERT_TRUE(controller("u")->quarantined) << "precondition: u's activation is late";
  observed_.user_activate_gate.open();
  ASSERT_TRUE(eventually([&] { return !controller("u")->quarantined; }));
  EXPECT_EQ(controller("u")->state, core::Lifecycle::Inactive);
  EXPECT_EQ(observed_.user_deactivations.load(), 1);
  EXPECT_TRUE(switch_controllers({"p", "u"}, {}).ok)
    << "the late activation broke the controller manager";
}

// p ran unchained and set its reference from a topic. Chained to u, which writes nothing in its
// first update, p must not take that value as u's command.
TEST_F(SupervisorFailures, a_reference_without_a_write_of_its_writer_is_nan)
{
  load_and_configure("p", "test/Provider");
  load_and_configure("u", "test/User");
  ASSERT_TRUE(switch_controllers({"p"}, {}).ok);
  ASSERT_TRUE(eventually([&] { return observed_.first_reference.load() == 7.0; }))
    << "precondition: p set its reference unchained";
  ASSERT_TRUE(switch_controllers({}, {"p"}).ok);
  observed_.first_reference = 0.0;
  ASSERT_TRUE(switch_controllers({"p", "u"}, {}).ok);
  ASSERT_TRUE(eventually([&] { return observed_.first_reference.load() != 0.0; }));
  EXPECT_TRUE(std::isnan(observed_.first_reference.load())) << observed_.first_reference.load();
}

// The user fails in its first update and stops the provider it writes, which did not fail itself
TEST_F(SupervisorFailures, a_failed_switch_tells_a_failed_start_from_one_stopped_with_it)
{
  load_and_configure("p", "test/Provider");
  load_and_configure("u", "test/User");
  observed_.fail_user_update = true;
  const testing::LogCapture log;
  const auto response = switch_controllers({"p", "u"}, {});
  EXPECT_FALSE(response.ok);
  EXPECT_NE(
    response.message.find("'u' failed in an update after its activation"), std::string::npos)
    << response.message;
  EXPECT_NE(
    response.message.find(
      "'p' stopped after its activation with controller 'u', which writes 'p/x'"),
    std::string::npos)
    << response.message;
  EXPECT_TRUE(log.contains("Controller 'u' failed in an update and stopped with its dependents"));
  EXPECT_TRUE(log.contains("Controller 'p' stopped with controller 'u', which writes 'p/x'"))
    << log.text();
}

// e fails while r, which reads it, is still in on_activate, so RT never ran r
TEST_F(SupervisorFailures, a_switch_names_a_started_controller_that_another_failure_stopped)
{
  load_and_configure("e", "test/Exporter");
  load_and_configure("r", "test/Reader");
  ASSERT_TRUE(switch_controllers({"e"}, {}).ok);
  observed_.user_activate_gate.close();
  auto pending = std::async(std::launch::async, [&] { return switch_controllers({"r"}, {}); });
  ASSERT_TRUE(eventually([&] { return observed_.held.load() > 0; }));
  observed_.fail_exporter_update = true;
  ASSERT_TRUE(cycles(20));
  observed_.user_activate_gate.open();
  const auto response = pending.get();
  EXPECT_FALSE(response.ok);
  EXPECT_NE(
    response.message.find(
      "'r' stopped after its activation with controller 'e' (reads 'e/position')"),
    std::string::npos)
    << response.message;
}

// e fails while u is in on_activate, then u fails its first update and the switch rolls back
TEST_F(SupervisorFailures, a_failed_switch_does_not_restart_a_controller_that_failed_during_it)
{
  load_and_configure("e", "test/Exporter");
  load_and_configure("p", "test/Provider");
  load_and_configure("u", "test/User");
  ASSERT_TRUE(switch_controllers({"e"}, {}).ok);
  const auto activation = controller("e")->activation;
  observed_.user_activate_gate.close();
  observed_.fail_user_update = true;
  auto pending = std::async(std::launch::async, [&] { return switch_controllers({"p", "u"}, {}); });
  ASSERT_TRUE(eventually([&] { return observed_.held.load() > 0; }));
  observed_.fail_exporter_update = true;
  ASSERT_TRUE(cycles(20));
  observed_.fail_exporter_update = false;
  observed_.user_activate_gate.open();
  const auto response = pending.get();
  EXPECT_FALSE(response.ok);
  ASSERT_TRUE(cycles(20));
  EXPECT_EQ(controller("e")->activation, activation) << response.message;
  EXPECT_EQ(controller("e")->state, core::Lifecycle::Inactive);
}

// The e-stop engages while u is in on_activate, so the switch rolls back during the e-stop
TEST_F(SupervisorFailures, a_failed_switch_does_not_restart_what_the_estop_stopped_during_it)
{
  load_and_configure("a", "test/Controller_j1");
  load_and_configure("guard", "test/Controller_j2", true);
  load_and_configure("p", "test/Provider", true);
  load_and_configure("u", "test/User", true);
  ASSERT_TRUE(switch_controllers({"a", "guard"}, {}).ok);
  const auto activation = controller("a")->activation;
  const auto guard_activation = controller("guard")->activation;
  observed_.user_activate_gate.close();
  observed_.fail_user_update = true;
  auto pending = std::async(std::launch::async, [&] { return switch_controllers({"p", "u"}, {}); });
  ASSERT_TRUE(eventually([&] { return observed_.held.load() > 0; }));
  engage();
  ASSERT_TRUE(cycles(20));
  observed_.user_activate_gate.open();
  const auto response = pending.get();
  EXPECT_FALSE(response.ok);
  ASSERT_TRUE(cycles(20));
  EXPECT_EQ(controller("a")->activation, activation) << response.message;
  EXPECT_EQ(controller("a")->state, core::Lifecycle::Inactive);
  EXPECT_EQ(controller("guard")->activation, guard_activation) << "protected";
  EXPECT_EQ(controller("guard")->state, core::Lifecycle::Active);
}

// The latch passed the switch, then the e-stop engages while u is in on_activate
TEST_F(SupervisorFailures, a_switch_accepted_just_before_an_engage_rolls_back)
{
  load_and_configure("guard", "test/Controller_j2", true);
  load_and_configure("p", "test/Provider");
  load_and_configure("u", "test/User");
  ASSERT_TRUE(switch_controllers({"guard"}, {}).ok);
  const auto activation = controller("guard")->activation;
  observed_.user_activate_gate.close();
  auto pending =
    std::async(std::launch::async, [&] { return switch_controllers({"p", "u"}, {"guard"}); });
  ASSERT_TRUE(observed_.user_activate_gate.holds());
  engage();
  observed_.user_activate_gate.open();
  const auto response = pending.get();
  EXPECT_FALSE(response.ok);
  EXPECT_NE(
    response.message.find(
      "'u' stopped after its activation by the e-stop, engaged by '/estop', it says engaged"),
    std::string::npos)
    << response.message;
  EXPECT_EQ(controller("u")->state, core::Lifecycle::Inactive);
  EXPECT_EQ(controller("p")->state, core::Lifecycle::Inactive);
  EXPECT_EQ(controller("guard")->state, core::Lifecycle::Active) << "rolled back";
  EXPECT_EQ(controller("guard")->activation, activation + 1);
}

TEST_F(SupervisorFailures, a_failed_switch_restarts_the_controller_it_deactivated)
{
  load_and_configure("a", "test/Controller_j1");
  load_and_configure("p", "test/Provider");
  load_and_configure("u", "test/User");
  ASSERT_TRUE(switch_controllers({"a"}, {}).ok);
  const auto activation = controller("a")->activation;
  observed_.fail_user_update = true;
  const auto response = switch_controllers({"p", "u"}, {"a"});
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(controller("a")->activation, activation + 1) << response.message;
  EXPECT_EQ(controller("a")->state, core::Lifecycle::Active);
}

TEST_F(SupervisorFailures, an_update_that_throws_names_the_exception_in_the_log_and_the_answer)
{
  load_and_configure("p", "test/Provider");
  load_and_configure("u", "test/User");
  observed_.throw_from_user_update = Throw::Exception;
  {
    const testing::LogCapture log;
    const auto response = switch_controllers({"p", "u"}, {});
    EXPECT_FALSE(response.ok);
    EXPECT_NE(
      response.message.find("'u' failed in an update after its activation, it threw: clock 4711"),
      std::string::npos)
      << response.message;
    EXPECT_TRUE(log.contains(
      "Controller 'u' failed in an update and stopped with its dependents, it threw: clock 4711"))
      << log.text();
  }
  observed_.throw_from_user_update = Throw::Int;
  const auto response = switch_controllers({"p", "u"}, {});
  EXPECT_NE(response.message.find("it threw: an exception of unknown type"), std::string::npos)
    << response.message;
  // The fixed buffer keeps the first 255 characters
  observed_.throw_from_user_update = Throw::LongText;
  const auto long_text = switch_controllers({"p", "u"}, {});
  EXPECT_NE(long_text.message.find("it threw: " + std::string(255, 'x')), std::string::npos)
    << long_text.message;
  EXPECT_EQ(long_text.message.find(std::string(256, 'x')), std::string::npos) << long_text.message;
  // An update that returns ERROR threw nothing
  observed_.throw_from_user_update = Throw::Nothing;
  observed_.fail_user_update = true;
  const auto plain = switch_controllers({"p", "u"}, {});
  EXPECT_NE(plain.message.find("'u' failed in an update"), std::string::npos) << plain.message;
  EXPECT_EQ(plain.message.find("it threw"), std::string::npos) << plain.message;
}

// The e-stop stops the command claimers in RT while a request holds the supervisor
TEST_F(SupervisorFailures, the_estop_stops_command_claimers_within_a_cycle_during_a_request)
{
  load_and_configure("a", "test/Controller_j1");
  load_and_configure("guard", "test/Controller_j2", true);
  load_and_configure("x", "test/Exporter");
  ASSERT_TRUE(switch_controllers({"a", "guard", "x"}, {}).ok);
  observed_.load_gate.close();
  auto load =
    std::async(std::launch::async, [&] { return supervisor_->load("late", "test/Controller_j3"); });
  ASSERT_TRUE(observed_.load_gate.holds());
  const auto a = entry("a");
  watched_activation_ = a->model.activation;
  watched_ = a->runtime.get();
  engage();
  ASSERT_TRUE(eventually([&] { return stopped_at_ >= 0; }));
  // In the update of the first cycle that saw it, or of the cycle the flag came in
  EXPECT_LE(stopped_at_ - estop_seen_, 1);
  observed_.load_gate.open();
  load.get();
  EXPECT_TRUE(eventually([&] { return controller("a")->state == core::Lifecycle::Inactive; }));
  EXPECT_EQ(controller("guard")->state, core::Lifecycle::Active) << "protected";
  EXPECT_EQ(controller("x")->state, core::Lifecycle::Active) << "claims no command interface";
  const auto guard = entry("guard");
  EXPECT_NE(guard->runtime->failed_activation.load(), guard->model.activation);
}

// While engaged, a failure that the e-stop did not cause is not logged as an e-stop
TEST_F(SupervisorFailures, a_failure_while_engaged_is_not_blamed_on_the_estop)
{
  load_and_configure("p", "test/Provider", true);
  load_and_configure("u", "test/User", true);
  engage();
  ASSERT_TRUE(switch_controllers({"p", "u"}, {}).ok);
  const testing::LogCapture log;
  observed_.fail_user_update = true;
  ASSERT_TRUE(eventually([&] { return controller("p")->state == core::Lifecycle::Inactive; }));
  EXPECT_TRUE(log.contains("Controller 'p' stopped with controller 'u', which writes 'p/x'"))
    << log.text();
  EXPECT_FALSE(log.contains("stopped by the e-stop"));
}

// The e-stop stops the protected provider through its writer, the log names that writer
TEST_F(SupervisorFailures, the_estop_names_the_writer_through_which_it_stopped_a_protected_provider)
{
  load_and_configure("p", "test/Provider", true);
  load_and_configure("u", "test/User");
  ASSERT_TRUE(switch_controllers({"p", "u"}, {}).ok);
  const testing::LogCapture log;
  engage();
  ASSERT_TRUE(eventually([&] { return controller("p")->state == core::Lifecycle::Inactive; }));
  EXPECT_TRUE(log.contains("Controller 'u' stopped by the e-stop")) << log.text();
  EXPECT_TRUE(log.contains("Controller 'p' stopped with controller 'u', which writes 'p/x'"))
    << log.text();
}

// A short engage stops a in RT while a slow load holds the supervisor past the autostart's
// cooldown, so the status still shows a active when the startup switch is asked for
TEST_F(SupervisorFailures, a_startup_switch_behind_a_slow_request_restarts_what_an_engage_stopped)
{
  load_and_configure("a", "test/Controller_j1");
  ASSERT_TRUE(switch_controllers({"a"}, {}).ok);
  observed_.load_gate.close();
  auto load =
    std::async(std::launch::async, [&] { return supervisor_->load("b", "test/Controller_j2"); });
  ASSERT_TRUE(observed_.load_gate.holds()) << "precondition: the load holds the supervisor";
  engage();
  supervisor_->set_estop(false);
  const auto a = *entry("a");
  ASSERT_TRUE(eventually([&] { return a.runtime->failed_activation.load() == a.model.activation; }))
    << "precondition: RT stopped a";
  ASSERT_EQ(controller("a")->state, core::Lifecycle::Active)
    << "precondition: the status still shows a active";
  auto startup = std::async(
    std::launch::async,
    [&] { return supervisor_->startup_switch({"a"}, true, testing::FINISH_TIMEOUT); });
  ASSERT_TRUE(eventually([&] { return supervisor_->requests_waiting(); }))
    << "precondition: the startup switch is asked for while the load still runs";
  observed_.load_gate.open();
  ASSERT_TRUE(load.get().ok);
  const auto started = startup.get();
  EXPECT_TRUE(started.response.ok) << started.response.message;
  EXPECT_EQ(started.selected, core::Names{"a"});
  EXPECT_EQ(controller("a")->state, core::Lifecycle::Active);
}

// Protection does not hold against the e-stop in a chain with an unprotected writer, said once
TEST_F(SupervisorFailures, a_protected_provider_of_an_unprotected_writer_is_warned_about)
{
  load_and_configure("p", "test/Provider", true);
  load_and_configure("u", "test/User");
  const testing::LogCapture log;
  ASSERT_TRUE(switch_controllers({"p", "u"}, {}).ok);
  EXPECT_TRUE(log.contains("Protected 'p' is chained to a controller the e-stop stops"));
}

TEST_F(SupervisorFailures, a_switch_the_hardware_rejects_in_prepare_is_rejected)
{
  load_and_configure("a", "test/Controller_j1");
  observed_.reject_prepare = true;
  const auto rejected = switch_controllers({"a"}, {});
  EXPECT_FALSE(rejected.ok);
  EXPECT_NE(
    rejected.message.find(
      "Hardware rejected the mode switch: 'system' rejected starting [j1/velocity]"),
    std::string::npos)
    << rejected.message;
  EXPECT_EQ(controller("a")->state, core::Lifecycle::Inactive);
  observed_.reject_prepare = false;
  EXPECT_TRUE(switch_controllers({"a"}, {}).ok);
}

// While engaged only protected command claimers and controllers without commands may start
TEST_F(SupervisorFailures, the_engaged_estop_rejects_starting_an_unprotected_command_claimer)
{
  load_and_configure("a", "test/Controller_j1");
  load_and_configure("guard", "test/Controller_j2", true);
  engage();
  const auto rejected = switch_controllers({"a"}, {});
  EXPECT_FALSE(rejected.ok);
  EXPECT_NE(
    rejected.message.find("e-stop is engaged by '/estop', it says engaged, 'a'"), std::string::npos)
    << rejected.message;
  EXPECT_TRUE(switch_controllers({"guard"}, {}).ok);
  supervisor_->set_estop(false);
  EXPECT_TRUE(switch_controllers({"a"}, {}).ok);
}

// The exports of a configured controller collide, and its on_cleanup hangs. The cleanup must
// run on the owner thread with a deadline, so configure returns within its timeout plus grace.
TEST_F(SupervisorFailures, cleanup_after_colliding_exports_is_bounded)
{
  ASSERT_TRUE(supervisor_->load("j1", "test/Exporter").ok);
  observed_.cleanup_gate.close();
  const auto start = std::chrono::steady_clock::now();
  const auto response = supervisor_->configure("j1");
  const auto took = std::chrono::steady_clock::now() - start;
  EXPECT_FALSE(response.ok);
  EXPECT_LT(took, testing::FINISH_TIMEOUT + 1s)
    << "configure blocked the supervisor for "
    << std::chrono::duration_cast<std::chrono::milliseconds>(took).count()
    << " ms in an unbounded on_cleanup";
}

// An unload whose on_shutdown outlives the deadline abandons the plugin. The supervisor rejects
// every request touching a controller whose callback still runs, so loading the same name must
// fail.
TEST_F(SupervisorFailures, reload_while_the_abandoned_plugin_still_shuts_down_is_rejected)
{
  load_and_configure("x", "test/Controller_j1");
  observed_.shutdown_gate.close();
  const auto unloaded = supervisor_->unload("x");
  EXPECT_FALSE(unloaded.ok);
  EXPECT_EQ(
    unloaded.message, "Shutting down 'x' did not finish, it is quarantined until it returns");
  EXPECT_FALSE(supervisor_->load("x", "test/Controller_j1").ok)
    << "a new 'x' coexists with the abandoned one";
  observed_.shutdown_gate.open();
  EXPECT_TRUE(eventually([&] { return !controller("x"); }))
    << "the unload completes once on_shutdown returned";
  EXPECT_TRUE(supervisor_->load("x", "test/Controller_j1").ok);
}

}  // namespace
}  // namespace hector_controller_manager
