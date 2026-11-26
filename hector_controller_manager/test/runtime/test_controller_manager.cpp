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
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include "fakes.hpp"
#include "gate.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/controller_manager.hpp"
#include "hector_controller_manager/introspection.hpp"
#include "hector_controller_manager/leak.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "log_capture.hpp"
#include "pal_statistics/pal_statistics.hpp"
#include "pal_statistics_msgs/msg/statistics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_environment.hpp"
#include "rt_thread.hpp"
#include "std_msgs/msg/string.hpp"
#include "urdf.hpp"

namespace hector_controller_manager
{
namespace
{
using hardware_interface::return_type;
using namespace std::chrono_literals;
using testing::eventually;

const std::string URDF = testing::urdf({{"j1", {"velocity"}}});

/// Busy waits, so all of the time counts as execution time.
void spin_for(std::chrono::microseconds time)
{
  const auto end = std::chrono::steady_clock::now() + time;
  while (std::chrono::steady_clock::now() < end)
  {
  }
}

/// The namespace of this test run, so concurrent runs in one domain do not cross-talk.
const std::string RUN_NAMESPACE = "/hcm_cm_" + std::to_string(::getpid());

/// How long the next configure and activate of System take.
std::atomic<int> lifecycle_delay_ms{0};
/// Holds the next configure of System.
testing::Gate configure_gate;
/// Holds the updates of SlowController.
testing::Gate update_gate;
std::atomic<bool> activation_fails{false};

class System : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(lifecycle_delay_ms.load()));
    configure_gate.pass();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(lifecycle_delay_ms.load()));
    return activation_fails ? hardware_interface::CallbackReturn::FAILURE
                            : hardware_interface::CallbackReturn::SUCCESS;
  }
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // Long enough to tell microseconds from seconds in the statistics
    spin_for(1ms);
    return return_type::OK;
  }
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return return_type::OK;
  }
};

class SlowController : public testing::NoInterfaces<>
{
public:
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    update_gate.pass();
    spin_for(1ms);
    return controller_interface::return_type::OK;
  }
};

/// Creates every controller as a `Controller` instead of through pluginlib.
template <typename Controller>
class FakeControllerManager : public ControllerManager
{
public:
  using ControllerManager::ControllerManager;

private:
  runtime::ControllerRuntime::Plugin create_controller(
    const std::string & name, const std::string &) override
  {
    auto plugin = testing::initialized(std::make_shared<Controller>(), name);
    this->executor()->add_node(plugin->get_node()->get_node_base_interface());
    return plugin;
  }
};
using SlowControllerManager = FakeControllerManager<SlowController>;

class ControllerManagerTest : public ::testing::Test
{
protected:
  // Also after a failed assertion, the next test would get a failing or held System
  void TearDown() override
  {
    lifecycle_delay_ms = 0;
    activation_fails = false;
    configure_gate.open();
    update_gate.open();
  }
};

/// Spins the controller manager on its executor and shuts it down at the end of the scope.
class Spinning
{
public:
  Spinning(std::shared_ptr<rclcpp::Executor> executor, std::shared_ptr<ControllerManager> cm)
  : executor_(std::move(executor)), cm_(std::move(cm))
  {
    executor_->add_node(cm_);
    spinner_ = std::thread([this] { executor_->spin(); });
  }
  ~Spinning()
  {
    cm_->shutdown();
    executor_->cancel();
    spinner_.join();
  }
  Spinning(const Spinning &) = delete;
  Spinning & operator=(const Spinning &) = delete;

private:
  std::shared_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<ControllerManager> cm_;
  std::thread spinner_;
};

/// Empty if a controller manager with these parameter overrides can be created, else why not.
std::string construction_error(const std::vector<rclcpp::Parameter> & overrides)
{
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides(overrides);
  try
  {
    std::make_shared<ControllerManager>(
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>(), "cm_parameters", "", options);
    return "";
  }
  catch (const std::runtime_error & error)
  {
    return error.what();
  }
}

// The gz plugin's factory uses the controller manager it is constructing
TEST_F(ControllerManagerTest, the_hardware_factory_runs_only_after_construction_returned)
{
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::shared_ptr<ControllerManager> cm;
  std::mutex cm_mutex;
  std::atomic<bool> factory_saw_cm{false};
  std::atomic<bool> factory_ran{false};
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides({{"robot_description", URDF}});
  auto created = std::make_shared<ControllerManager>(
    executor, "cm_construction", "", options,
    [&](const hardware_interface::HardwareInfo &)
    {
      std::lock_guard<std::mutex> lock(cm_mutex);
      factory_saw_cm = cm != nullptr;
      factory_ran = true;
      return std::make_unique<System>();
    });
  // Like an owner that publishes the pointer a while after make_shared returned
  std::this_thread::sleep_for(300ms);
  {
    std::lock_guard<std::mutex> lock(cm_mutex);
    cm = created;
  }
  const Spinning spinning(executor, cm);
  EXPECT_TRUE(eventually([&] { return cm->is_initialized(); }));
  EXPECT_TRUE(factory_ran.load());
  EXPECT_TRUE(factory_saw_cm.load());
}

size_t threads_of_own_executor(const std::vector<rclcpp::Parameter> & overrides)
{
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides(overrides);
  const auto cm = std::make_shared<ControllerManager>(nullptr, "cm_threads", "", options);
  return std::dynamic_pointer_cast<rclcpp::executors::MultiThreadedExecutor>(cm->executor())
    ->get_number_of_threads();
}

TEST_F(ControllerManagerTest, its_own_executor_has_the_configured_threads)
{
  EXPECT_EQ(threads_of_own_executor({}), 2u);
  EXPECT_EQ(threads_of_own_executor({{"executor_threads", 3}}), 3u);
  EXPECT_THROW(threads_of_own_executor({{"executor_threads", 0}}), std::runtime_error);
}

TEST_F(ControllerManagerTest, a_value_below_the_minimum_of_its_parameter_is_rejected)
{
  const std::vector<rclcpp::Parameter> rejected = {
    {"update_rate", 0},
    {"update_rate", 65536},
    {"update_rate", int64_t{1} << 32},
    {"hardware_hang_timeout_ms", 0},
    {"hardware_hang_timeout_ms", -5},
    {"default_timeout_ms", 0},
    {"default_timeout_ms", -5},
    {"hardware_state_timeout_ms", 0},
    {"hardware_fault_timeout_ms", 0},
    {"mode_switch_timeout_ms", 0},
    {"mode_switch_timeout_ms", -5},
    {"estop_publisher_grace_ms", 0},
    {"estop_publisher_grace_ms", -5},
    {"autostart_cooldown_ms", -1},
    {"autostart_attempts", 0},
  };
  for (const auto & parameter : rejected)
  {
    const std::string error = construction_error({parameter});
    EXPECT_NE(error.find(parameter.get_name()), std::string::npos)
      << parameter.value_to_string() << ": " << error;
  }
  EXPECT_EQ("", construction_error({{"update_rate", 65535}}));
  EXPECT_EQ(
    "", construction_error(
          {{"update_rate", 1},
           {"hardware_hang_timeout_ms", 1},
           {"default_timeout_ms", 1},
           {"hardware_state_timeout_ms", 1},
           {"hardware_fault_timeout_ms", 1},
           {"mode_switch_timeout_ms", 1},
           {"estop_publisher_grace_ms", 1},
           {"autostart_cooldown_ms", 0},
           {"autostart_attempts", 1}}));
}

TEST_F(ControllerManagerTest, the_cpu_affinity_is_one_core_or_a_list_of_them)
{
  EXPECT_EQ("", construction_error({{"cpu_affinity", 1}}));
  EXPECT_EQ("", construction_error({{"cpu_affinity", std::vector<int64_t>{1, 2}}}));
  const std::string error = construction_error({{"cpu_affinity", std::string("1")}});
  EXPECT_NE(error.find("cpu_affinity"), std::string::npos) << error;
}

TEST_F(ControllerManagerTest, lock_memory_is_a_bool)
{
  const std::string error = construction_error({{"lock_memory", std::string("yes")}});
  EXPECT_NE(error.find("lock_memory"), std::string::npos) << error;
}

// The startup thread and the autostart read them while hardware is up, a type error must come now
TEST_F(ControllerManagerTest, a_wrongly_typed_startup_parameter_stops_the_node_at_construction)
{
  for (const std::string name :
       {"startup.hardware_active", "startup.controllers_loaded", "startup.controllers_active",
        "hardware_components_initial_state.unconfigured",
        "hardware_components_initial_state.inactive"})
  {
    const std::string error = construction_error({{name, std::string("jsb")}});
    EXPECT_NE(error.find(name), std::string::npos) << error;
    EXPECT_EQ("", construction_error({{name, std::vector<std::string>{"jsb"}}})) << name;
  }
}

TEST_F(ControllerManagerTest, a_wrongly_typed_controller_parameter_fails_the_load)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides(
    {{"robot_description", URDF},
     {"typed.type", 5},
     {"guarded.type", std::string("test/Slow")},
     {"guarded.protected", std::string("yes")}});
  auto cm = std::make_shared<SlowControllerManager>(
    executor, "cm_typed", "", options,
    [](const hardware_interface::HardwareInfo &) { return std::make_unique<System>(); });
  const Spinning spinning(executor, cm);
  ASSERT_TRUE(eventually([&] { return cm->is_initialized(); }, 10s));
  EXPECT_EQ(
    cm->load_controller("typed").message,
    "Controller 'typed' cannot be loaded, parameter 'typed.type' has invalid type: expected "
    "[string] got [integer]");
  EXPECT_EQ(
    cm->load_controller("guarded").message,
    "Controller 'guarded' cannot be loaded, parameter 'guarded.protected' has invalid type: "
    "expected [bool] got [string]");
}

TEST_F(ControllerManagerTest, an_estop_deadline_below_100_ms_is_rejected)
{
  for (const int64_t deadline : {1, 99, -100})
  {
    const std::string error = construction_error({{"estop_deadline_ms", deadline}});
    EXPECT_NE(error.find("estop_deadline_ms"), std::string::npos) << deadline << ": " << error;
  }
  for (const int64_t deadline : {0, 100})
  {
    EXPECT_EQ("", construction_error({{"estop_deadline_ms", deadline}})) << deadline;
  }
}

TEST_F(ControllerManagerTest, the_read_only_capabilities_offer_force_auto)
{
  rclcpp::NodeOptions overridden = get_cm_node_options();
  overridden.parameter_overrides({{"capabilities", std::vector<std::string>{"none"}}});
  for (const auto & options : {get_cm_node_options(), overridden})
  {
    const auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    const auto cm = std::make_shared<ControllerManager>(executor, "cm_capabilities", "", options);
    EXPECT_EQ(
      cm->get_parameter("capabilities").as_string_array(), std::vector<std::string>{"force_auto"});
    EXPECT_FALSE(
      cm->set_parameter(rclcpp::Parameter("capabilities", std::vector<std::string>{})).successful);
  }
}

TEST_F(ControllerManagerTest, a_given_executor_ignores_executor_threads_and_says_so)
{
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides({{"executor_threads", 3}});
  const testing::LogCapture log;
  const auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  const auto cm = std::make_shared<ControllerManager>(executor, "cm_given", "", options);
  EXPECT_EQ(cm->executor(), executor);
  EXPECT_TRUE(log.contains("executor_threads is ignored"));
}

/// An async system whose write takes longer than a control cycle.
class SlowAsyncSystem : public System
{
public:
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    std::this_thread::sleep_for(3ms);
    return return_type::OK;
  }
};

/// Whether the diagnostics warn about the cycles an async owner slower than RT skips.
bool warns_about_skipped_cycles(bool print_warnings)
{
  const std::string urdf = testing::urdf(
    {{"j1", {"velocity"}}}, R"(is_async="true")",
    std::string(R"(<async print_warnings=")") + (print_warnings ? "true" : "false") + R"("/>)");
  const testing::LogCapture log;
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides({{"robot_description", urdf}});
  auto cm = std::make_shared<ControllerManager>(
    executor, "cm_overruns", "", options,
    [](const hardware_interface::HardwareInfo &) { return std::make_unique<SlowAsyncSystem>(); });
  const Spinning spinning(executor, cm);
  const testing::RtThread loop([&](const rclcpp::Time &, const rclcpp::Duration & period)
                               { testing::read_update_write(*cm, cm->now(), period); });
  EXPECT_TRUE(eventually([&] { return cm->is_initialized(); }, 10s));
  // Two diagnostics periods of one second
  std::this_thread::sleep_for(2500ms);
  return log.contains("'system' skipped");
}

TEST_F(ControllerManagerTest, an_async_owner_that_skips_cycles_is_reported_unless_quiet)
{
  EXPECT_TRUE(warns_about_skipped_cycles(true));
  EXPECT_FALSE(warns_about_skipped_cycles(false));
}

// Hardware slower than the default timeout comes up at startup and through a state request
TEST_F(ControllerManagerTest, hardware_state_changes_get_the_hardware_state_timeout)
{
  lifecycle_delay_ms = 400;
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides(
    {{"robot_description", URDF},
     {"default_timeout_ms", 200},
     {"hardware_state_timeout_ms", 3000}});
  auto cm = std::make_shared<ControllerManager>(
    executor, "cm_hardware_state", "", options,
    [](const hardware_interface::HardwareInfo &) { return std::make_unique<System>(); });
  const Spinning spinning(executor, cm);
  const auto state = [&]
  {
    auto response = std::make_shared<ControllerManager::ListHardwareComponents::Response>();
    cm->list_hardware_components_srv_cb(
      std::make_shared<ControllerManager::ListHardwareComponents::Request>(), response);
    return response->component.empty() ? std::string() : response->component.front().state.label;
  };
  // Initialized is reported before the bring up of the hardware finished
  EXPECT_TRUE(eventually([&] { return state() == "active"; }, 10s));
  const auto request = [&](uint8_t id)
  {
    auto message = std::make_shared<ControllerManager::SetHardwareComponentState::Request>();
    message->name = "system";
    message->target_state.id = id;
    auto response = std::make_shared<ControllerManager::SetHardwareComponentState::Response>();
    cm->set_hardware_component_state_srv_cb(message, response);
    return response->ok;
  };
  EXPECT_TRUE(request(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE));
  EXPECT_TRUE(request(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
  EXPECT_EQ(state(), "active");
}

// Both update and read spin 1 ms, so a wrong scale lands far outside the bounds
TEST_F(ControllerManagerTest, the_statistics_give_execution_times_in_microseconds)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides(
    {{"robot_description", URDF}, {"slow.type", std::string("test/Slow")}});
  auto cm = std::make_shared<SlowControllerManager>(
    executor, "cm_statistics", "", options,
    [](const hardware_interface::HardwareInfo &) { return std::make_unique<System>(); });
  const Spinning spinning(executor, cm);
  // Joined before the spinning ends, also after a failed assertion
  const testing::RtThread loop(
    [&](const rclcpp::Time &, const rclcpp::Duration & period)
    { testing::read_update_write(*cm, cm->now(), period); }, 0.001, 0ms);
  const std::string key =
    cm->get_node_topics_interface()->resolve_topic_name(hardware_interface::CM_STATISTICS_TOPIC);
  const auto value_of = [&](const std::string & name)
  {
    const auto message = pal_statistics::getRegistry(key)->createMsg();
    for (const auto & statistic : message.statistics)
    {
      if (statistic.name == name)
      {
        return statistic.value;
      }
    }
    return 0.0;
  };
  const std::string read = "system.stats/read_cycle/execution_time/current_value";
  const std::string update = "slow.stats/execution_time/current_value";
  EXPECT_TRUE(eventually([&] { return cm->is_initialized(); }, 10s));
  ASSERT_TRUE(cm->load_controller("slow").ok);
  auto configure = std::make_shared<ControllerManager::ConfigureController::Request>();
  configure->name = "slow";
  auto configured = std::make_shared<ControllerManager::ConfigureController::Response>();
  cm->configure_controller_service_cb(configure, configured);
  ASSERT_TRUE(configured->ok);
  auto activate = std::make_shared<ControllerManager::SwitchController::Request>();
  activate->activate_controllers = {"slow"};
  activate->strictness = ControllerManager::SwitchController::Request::STRICT;
  auto activated = std::make_shared<ControllerManager::SwitchController::Response>();
  EXPECT_TRUE(cm->active_controllers().empty());
  cm->switch_controller_service_cb(activate, activated);
  ASSERT_TRUE(activated->ok) << activated->message;
  EXPECT_EQ(cm->active_controllers(), std::vector<std::string>{"slow"});
  EXPECT_TRUE(eventually([&] { return value_of(read) != 0.0 && value_of(update) != 0.0; }, 10s));
  // Generous above, a preempted cycle may take a few ms
  for (const auto & name : {read, update})
  {
    EXPECT_GE(value_of(name), 1000.0) << name;
    EXPECT_LT(value_of(name), 50'000.0) << name;
  }
}

/// The names in the statistics registry of `cm`, none if it has none.
std::set<std::string> statistics_of(ControllerManager & cm)
{
  const auto registry = pal_statistics::getRegistry(
    cm.get_node_topics_interface()->resolve_topic_name(hardware_interface::CM_STATISTICS_TOPIC));
  std::set<std::string> names;
  if (registry)
  {
    for (const auto & statistic : registry->createMsg().statistics)
    {
      names.insert(statistic.name);
    }
  }
  return names;
}

// A registry holds its manager's publisher and clock, which a successor of the same name must not
// inherit, and one manager's shutdown must not unregister another's statistics
// Each switch publishes a new status and frees the old one, which a reader must still hold. A
// sanitizer build catches a read of the freed snapshot.
TEST_F(ControllerManagerTest, active_controllers_during_switches_reads_a_snapshot_it_holds)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides(
    {{"robot_description", URDF}, {"slow.type", std::string("test/Slow")}});
  auto cm = std::make_shared<SlowControllerManager>(
    executor, "cm_active_reader", "", options,
    [](const hardware_interface::HardwareInfo &) { return std::make_unique<System>(); });
  const Spinning spinning(executor, cm);
  const testing::RtThread loop(
    [&](const rclcpp::Time &, const rclcpp::Duration & period)
    { testing::read_update_write(*cm, cm->now(), period); }, 0.001, 0ms);
  ASSERT_TRUE(eventually([&] { return cm->is_initialized(); }, 10s));
  ASSERT_TRUE(cm->load_controller("slow").ok);
  auto configure = std::make_shared<ControllerManager::ConfigureController::Request>();
  configure->name = "slow";
  auto configured = std::make_shared<ControllerManager::ConfigureController::Response>();
  cm->configure_controller_service_cb(configure, configured);
  ASSERT_TRUE(configured->ok);

  std::atomic<bool> done{false};
  std::atomic<int> unexpected{0};
  std::thread reader(
    [&]
    {
      while (!done)
      {
        const auto active = cm->active_controllers();
        if (!active.empty() && active != std::vector<std::string>{"slow"})
        {
          ++unexpected;
        }
      }
    });
  const testing::OnExit join(
    [&]
    {
      done = true;
      reader.join();
    });
  // About 3 ms a switch, enough rounds that a sanitizer build sees a freed snapshot nearly always
  for (int i = 0; i < 1000; ++i)
  {
    auto request = std::make_shared<ControllerManager::SwitchController::Request>();
    (i % 2 == 0 ? request->activate_controllers : request->deactivate_controllers) = {"slow"};
    request->strictness = ControllerManager::SwitchController::Request::STRICT;
    auto response = std::make_shared<ControllerManager::SwitchController::Response>();
    cm->switch_controller_service_cb(request, response);
    ASSERT_TRUE(response->ok) << response->message;
  }
  EXPECT_EQ(unexpected, 0);
}

TEST_F(ControllerManagerTest, each_manager_and_each_successor_of_a_name_has_statistics_of_its_own)
{
  const auto make = [](const std::string & name)
  {
    rclcpp::NodeOptions options = get_cm_node_options();
    options.parameter_overrides({{"robot_description", URDF}});
    return std::make_shared<ControllerManager>(
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>(), name, RUN_NAMESPACE, options,
      [](const hardware_interface::HardwareInfo &) { return std::make_unique<System>(); });
  };
  auto first = make("cm_first");
  auto second = make("cm_second");
  EXPECT_TRUE(statistics_of(*first).contains("cm_first.total_time"));
  EXPECT_FALSE(statistics_of(*first).contains("cm_second.total_time"));
  const std::set<std::string> kept = statistics_of(*second);
  ASSERT_TRUE(kept.contains("cm_second.total_time"));
  first->shutdown();
  EXPECT_EQ(statistics_of(*second), kept);

  const std::string key =
    first->get_node_topics_interface()->resolve_topic_name(hardware_interface::CM_STATISTICS_TOPIC);
  const std::weak_ptr left_behind = pal_statistics::getRegistry(key);
  first.reset();
  EXPECT_EQ(statistics_of(*second), kept);
  auto successor = make("cm_first");
  EXPECT_TRUE(left_behind.expired());
  EXPECT_TRUE(statistics_of(*successor).contains("cm_first.total_time"));
  successor->shutdown();
  second->shutdown();
}

/// Counts its reads in a value registered for introspection, as upstream plugins do.
class IntrospectedSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    REGISTER_ROS2_CONTROL_INTROSPECTION("reads", &reads_);
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    ++reads_;
    // An async owner is still in its read when RT samples next
    std::this_thread::sleep_for(1ms);
    return return_type::OK;
  }
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return return_type::OK;
  }
  // Off RT while the value is still enabled, so a sample outside the gate races
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    ++reads_;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

private:
  // Not a double, so the registry reads it through a lambda of this file, which TSan instruments
  uint64_t reads_ = 0;
};

/// Counts its activations in a value registered for introspection. Also registers a value enabled
/// right away, as upstream allows, on the heap, so a sample after its destruction reads freed
/// memory.
class IntrospectedController : public testing::NoInterfaces<>
{
public:
  ~IntrospectedController() override
  {
    loads_.reset();
    // The base unregisters the value only after this
    std::this_thread::sleep_for(20ms);
  }
  controller_interface::CallbackReturn on_init() override
  {
    REGISTER_ROS2_CONTROL_INTROSPECTION("activations", &activations_);
    REGISTER_ENTITY(
      hardware_interface::DEFAULT_REGISTRY_KEY, get_name() + ".loads", loads_.get(),
      &stats_registrations_, true);
    std::this_thread::sleep_for(20ms);
    ++*loads_;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // Introspection is enabled already, so RT would sample while this runs
    std::this_thread::sleep_for(20ms);
    ++activations_;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return controller_interface::return_type::OK;
  }

private:
  uint64_t activations_ = 0;
  std::unique_ptr<uint64_t> loads_ = std::make_unique<uint64_t>(0);
};

class IntrospectionTest : public ControllerManagerTest, public ::testing::WithParamInterface<bool>
{
};

// The cases of a sample racing with a plugin call bite only under TSan
TEST_P(IntrospectionTest, publishes_the_values_of_active_plugins_and_controllers)
{
  const std::string urdf =
    testing::urdf({{"j1", {"velocity"}}}, GetParam() ? R"(is_async="true")" : "");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides(
    {{"robot_description", urdf},
     {"probe.type", std::string("test/Introspected")},
     {"temp.type", std::string("test/Introspected")}});
  const std::string name = GetParam() ? "cm_introspection_async" : "cm_introspection_sync";
  auto cm = std::make_shared<FakeControllerManager<IntrospectedController>>(
    executor, name, RUN_NAMESPACE, options, [](const hardware_interface::HardwareInfo &)
    { return std::make_unique<IntrospectedSystem>(); });
  auto listener = std::make_shared<rclcpp::Node>("introspection_listener", RUN_NAMESPACE);
  std::mutex mutex;
  std::map<std::string, double> latest;
  auto subscription = listener->create_subscription<pal_statistics_msgs::msg::Statistics>(
    // pal_statistics publishes best effort
    Introspection::get()->topic() + "/full", rclcpp::SensorDataQoS(),
    [&](const pal_statistics_msgs::msg::Statistics & message)
    {
      const std::lock_guard lock(mutex);
      latest.clear();
      for (const auto & statistic : message.statistics)
      {
        latest[statistic.name] = statistic.value;
      }
    });
  executor->add_node(listener);
  const Spinning spinning(executor, cm);
  const testing::RtThread loop([&](const rclcpp::Time &, const rclcpp::Duration & period)
                               { testing::read_update_write(*cm, cm->now(), period); });
  const auto published = [&](const std::string & name) -> std::optional<double>
  {
    const std::lock_guard lock(mutex);
    const auto it = latest.find(name);
    return it == latest.end() ? std::nullopt : std::optional(it->second);
  };
  const auto seen = [&]
  {
    const std::lock_guard lock(mutex);
    std::string text;
    for (const auto & [name, value] : latest)
    {
      text += name + "=" + std::to_string(value) + " ";
    }
    return text;
  };
  ASSERT_TRUE(eventually([&] { return cm->is_initialized(); }, 10s));
  // Creation and destruction write and free a value RT samples
  for (int i = 0; i < 3; ++i)
  {
    ASSERT_TRUE(cm->load_controller("temp").ok);
    EXPECT_TRUE(eventually([&] { return published("temp.loads") == 1; }, 10s)) << seen();
    auto unload = std::make_shared<ControllerManager::UnloadController::Request>();
    unload->name = "temp";
    auto unloaded = std::make_shared<ControllerManager::UnloadController::Response>();
    cm->unload_controller_service_cb(unload, unloaded);
    ASSERT_TRUE(unloaded->ok);
  }
  ASSERT_TRUE(cm->load_controller("probe").ok);
  auto configure = std::make_shared<ControllerManager::ConfigureController::Request>();
  configure->name = "probe";
  auto configured = std::make_shared<ControllerManager::ConfigureController::Response>();
  cm->configure_controller_service_cb(configure, configured);
  ASSERT_TRUE(configured->ok);
  const auto switch_probe = [&](bool activate)
  {
    auto request = std::make_shared<ControllerManager::SwitchController::Request>();
    (activate ? request->activate_controllers : request->deactivate_controllers) = {"probe"};
    request->strictness = ControllerManager::SwitchController::Request::STRICT;
    auto response = std::make_shared<ControllerManager::SwitchController::Response>();
    cm->switch_controller_service_cb(request, response);
    return response->ok;
  };
  constexpr int ACTIVATIONS = 5;
  for (int i = 0; i < ACTIVATIONS; ++i)
  {
    ASSERT_TRUE(i == 0 || switch_probe(false));
    ASSERT_TRUE(switch_probe(true));
    // A sample between the switches, the registry queues only as many enables as values
    ASSERT_TRUE(eventually([&] { return published("probe.activations") == i + 1; }, 10s)) << seen();
  }
  EXPECT_TRUE(eventually(
    [&]
    {
      return published("probe.activations") == ACTIVATIONS &&
             published("probe.total_triggers").value_or(0.0) > 0.0 &&
             published("system.reads").value_or(0.0) > 0.0;
    },
    10s))
    << seen();
  auto inactive = std::make_shared<ControllerManager::SetHardwareComponentState::Request>();
  inactive->name = "system";
  inactive->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  auto deactivated = std::make_shared<ControllerManager::SetHardwareComponentState::Response>();
  cm->set_hardware_component_state_srv_cb(inactive, deactivated);
  ASSERT_TRUE(deactivated->ok);
  // An inactive component still reads, its values leave the topic like upstream
  EXPECT_TRUE(eventually(
    [&] { return published("probe.total_triggers") && !published("system.reads"); }, 10s))
    << seen();
}

INSTANTIATE_TEST_SUITE_P(
  SyncAndAsync, IntrospectionTest, ::testing::Bool(),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "async" : "sync"; });

/// A controller manager with an IntrospectedSystem whose control loop runs.
class RunningManager
{
public:
  explicit RunningManager(const std::string & name)
  : executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
    cm_(
      std::make_shared<ControllerManager>(
        executor_, name, RUN_NAMESPACE, options(), [](const hardware_interface::HardwareInfo &)
        { return std::make_unique<IntrospectedSystem>(); })),
    spinning_(executor_, cm_),
    loop_([this](const rclcpp::Time &, const rclcpp::Duration & period)
          { testing::read_update_write(*cm_, cm_->now(), period); })
  {
  }

  ControllerManager & cm() { return *cm_; }

private:
  static rclcpp::NodeOptions options()
  {
    rclcpp::NodeOptions options = get_cm_node_options();
    options.parameter_overrides({{"robot_description", URDF}});
    return options;
  }

  std::shared_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<ControllerManager> cm_;
  Spinning spinning_;
  testing::RtThread loop_;
};

// The RT loop of one manager cannot hold off the writers of another, which TSan reports. The
// registry stays published after either manager ends.
TEST_F(ControllerManagerTest, introspection_is_sampled_only_while_one_manager_lives)
{
  auto first = std::make_unique<RunningManager>("cm_introspection_first");
  RunningManager second("cm_introspection_second");
  ASSERT_TRUE(
    eventually([&] { return first->cm().is_initialized() && second.cm().is_initialized(); }, 10s));

  auto listener = std::make_shared<rclcpp::Node>("introspection_listener", RUN_NAMESPACE);
  std::atomic<int> samples{0};
  const auto subscription = listener->create_subscription<pal_statistics_msgs::msg::Statistics>(
    Introspection::get()->topic() + "/full", rclcpp::SensorDataQoS(),
    [&](const pal_statistics_msgs::msg::Statistics & message)
    {
      for (const auto & statistic : message.statistics)
      {
        samples += statistic.name == "system.reads" ? 1 : 0;
      }
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  const auto samples_within = [&](std::chrono::milliseconds time)
  {
    samples = 0;
    const auto end = std::chrono::steady_clock::now() + time;
    while (std::chrono::steady_clock::now() < end)
    {
      executor.spin_some(10ms);
    }
    return samples.load();
  };
  // Samples taken before the second manager joined may still arrive
  samples_within(500ms);
  EXPECT_EQ(samples_within(1s), 0);
  first.reset();
  EXPECT_GT(samples_within(1s), 0);
}

/// Options with the controller `slow` that a SlowControllerManager creates.
rclcpp::NodeOptions slow_controller_options()
{
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides({{"slow.type", std::string("test/Slow")}});
  return options;
}

/// A controller manager that takes its description from the topic, on one executor thread, with a
/// probe timer that ticks only while that thread is free.
class TopicDescription
{
public:
  explicit TopicDescription(const std::string & suffix)
  : name_space_(RUN_NAMESPACE + "_" + suffix),
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
    cm_(
      std::make_shared<SlowControllerManager>(
        executor_, "cm_topic", name_space_, slow_controller_options(),
        [this](const hardware_interface::HardwareInfo &)
        {
          ++created;
          return std::make_unique<System>();
        })),
    probe_(cm_->create_wall_timer(1ms, [this] { ticked = true; })),
    client_(std::make_shared<rclcpp::Node>("cm_topic_client", name_space_)),
    publisher_(client_->create_publisher<std_msgs::msg::String>(
      name_space_ + "/robot_description", rclcpp::QoS(1).reliable().transient_local()))
  {
  }

  void publish(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    publisher_->publish(message);
  }
  bool executor_free()
  {
    ticked = false;
    return eventually([this] { return ticked.load(); });
  }
  /// Whether the manager dropped its subscription, so no later description can reach it.
  bool unsubscribed()
  {
    return eventually([this] { return publisher_->get_subscription_count() == 0; });
  }
  ControllerManager & cm() { return *cm_; }
  template <typename ServiceT>
  bool offers(const std::string & service, std::chrono::milliseconds timeout)
  {
    return client_
      ->create_client<ServiceT>(std::string(cm_->get_fully_qualified_name()) + "/" + service)
      ->wait_for_service(timeout);
  }

  std::atomic<int> created{0};
  std::atomic<bool> ticked{false};

private:
  std::string name_space_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<ControllerManager> cm_;
  rclcpp::TimerBase::SharedPtr probe_;
  rclcpp::Node::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  const Spinning spinning_{executor_, cm_};
};

// The bring up of a description from the topic holds no executor thread, also with only one, and
// a later description starts no second bring up
TEST_F(ControllerManagerTest, the_first_description_on_the_topic_initializes_off_the_executor)
{
  TopicDescription topic("first");
  configure_gate.close();
  topic.publish(URDF);
  ASSERT_TRUE(configure_gate.holds()) << "precondition: initialize() runs";
  EXPECT_TRUE(topic.executor_free());
  configure_gate.open();
  EXPECT_TRUE(eventually([&] { return topic.cm().is_initialized(); }));
  ASSERT_TRUE(topic.unsubscribed());
  EXPECT_EQ(topic.created.load(), 1);
}

// A failed initialization needs a restart, a later description is ignored
TEST_F(ControllerManagerTest, a_description_after_a_failed_initialization_is_ignored)
{
  const testing::LogCapture log;
  TopicDescription topic("failed");
  topic.publish("no robot description");
  ASSERT_TRUE(
    eventually([&] { return log.contains("Initialization failed, the node needs a restart"); }))
    << log.text();
  ASSERT_TRUE(topic.unsubscribed());
  EXPECT_FALSE(topic.cm().is_initialized());
  EXPECT_EQ(topic.created.load(), 0);
  EXPECT_EQ(
    topic.cm().load_controller("slow").message, "Initialization failed, the node needs a restart");
}

const std::string LIMITED_URDF = testing::urdf({{"j1", {"velocity"}, {"position"}, true}});

/// Empty once a manager with these overrides and LIMITED_URDF initialized, else what it logged.
std::string startup_error(std::vector<rclcpp::Parameter> overrides)
{
  const testing::LogCapture log;
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions options = get_cm_node_options();
  overrides.emplace_back("robot_description", LIMITED_URDF);
  options.parameter_overrides(overrides);
  auto cm = std::make_shared<ControllerManager>(
    executor, "cm_limits", "", options,
    [](const hardware_interface::HardwareInfo &) { return std::make_unique<System>(); });
  const Spinning spinning(executor, cm);
  eventually(
    [&]
    {
      return cm->is_initialized() || log.contains("Startup failed") ||
             log.contains("needs a restart");
    },
    10s);
  return cm->is_initialized() ? "" : log.text();
}

TEST_F(ControllerManagerTest, the_command_limit_parameters_are_checked_at_startup)
{
  const rclcpp::Parameter acceleration("joint_limits.j1.has_acceleration_limits", true);
  EXPECT_EQ(startup_error({}), "");
  EXPECT_EQ(startup_error({acceleration, {"joint_limits.j1.max_acceleration", 2.0}}), "");
  EXPECT_NE(
    startup_error({{"joint_limits.j2.has_acceleration_limits", true},
                   {"joint_limits.j2.max_acceleration", 2.0}})
      .find("joint_limits.j2 names no joint"),
    std::string::npos);
  EXPECT_EQ(
    startup_error(
      {{"enforce_command_limits", false},
       {"joint_limits.j2.has_acceleration_limits", true},
       {"joint_limits.j2.max_acceleration", 2.0}}),
    "");
  EXPECT_NE(
    startup_error({{"joint_limits.j1.has_deceleration_limits", true},
                   {"joint_limits.j1.max_deceleration", 2.0}})
      .find("deceleration"),
    std::string::npos);
  EXPECT_NE(
    startup_error({{"command_limits_excluded_hardware", std::vector<std::string>{"none"}}})
      .find("exclude unknown hardware 'none'"),
    std::string::npos);
}

// As upstream, so a client waits for the services instead of being refused
TEST_F(ControllerManagerTest, the_services_appear_only_after_initialization)
{
  TopicDescription topic("services");
  using Load = ControllerManager::LoadController;
  using List = ControllerManager::ListControllers;
  EXPECT_FALSE(topic.offers<Load>("load_controller", 1s));
  EXPECT_FALSE(topic.offers<List>("list_controllers", 0s));
  topic.publish(URDF);
  EXPECT_TRUE(topic.offers<Load>("load_controller", 10s));
  EXPECT_TRUE(topic.offers<List>("list_controllers", 1s));
}

// The controller would get an empty robot description
TEST_F(ControllerManagerTest, a_load_before_the_description_arrived_is_refused)
{
  TopicDescription topic("load");
  const Response early = topic.cm().load_controller("slow");
  EXPECT_FALSE(early.ok);
  EXPECT_EQ(early.message, "Not initialized yet, waiting for the robot description");
  topic.publish(URDF);
  ASSERT_TRUE(eventually([&] { return topic.cm().is_initialized(); }));
  const Response late = topic.cm().load_controller("slow");
  EXPECT_TRUE(late.ok) << late.message;
}

// The components loaded, so the node runs and a hardware state request can still recover them
TEST_F(ControllerManagerTest, a_failed_bring_up_after_loading_needs_no_restart)
{
  const testing::LogCapture log;
  activation_fails = true;
  TopicDescription topic("bring_up");
  topic.publish(URDF);
  ASSERT_TRUE(eventually([&] { return log.contains("Bring up incomplete"); })) << log.text();
  EXPECT_TRUE(topic.cm().is_initialized());
  EXPECT_FALSE(log.contains("needs a restart")) << log.text();
}

/// Sends list and request service calls without waiting for their answers, until destroyed.
class Hammer
{
public:
  explicit Hammer(const std::string & cm)
  : node_(std::make_shared<rclcpp::Node>("cm_hammer", RUN_NAMESPACE)),
    list_(node_->create_client<ControllerManager::ListControllers>(cm + "/list_controllers")),
    reload_(node_->create_client<ControllerManager::ReloadControllerLibraries>(
      cm + "/reload_controller_libraries"))
  {
    executor_.add_node(node_);
    thread_ = std::thread(
      [this]
      {
        while (!stop_)
        {
          (void)list_->async_send_request(
            std::make_shared<ControllerManager::ListControllers::Request>());
          (void)reload_->async_send_request(
            std::make_shared<ControllerManager::ReloadControllerLibraries::Request>());
          executor_.spin_some();
          list_->prune_pending_requests();
          reload_->prune_pending_requests();
          std::this_thread::sleep_for(1ms);
        }
      });
  }
  ~Hammer()
  {
    stop_ = true;
    thread_.join();
  }
  bool ready() { return list_->wait_for_service(10s) && reload_->wait_for_service(10s); }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ControllerManager::ListControllers>::SharedPtr list_;
  rclcpp::Client<ControllerManager::ReloadControllerLibraries>::SharedPtr reload_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

/// Destroys a controller manager whose RT is stuck in an update while its executor spins and a
/// client calls its services. Exits with 0 if that ends without a crash.
[[noreturn]] void destroy_while_called()
{
  // Exits without unwinding, RT still calls into the controller manager
  const auto fail = [](const std::string & reason)
  {
    std::fputs(reason.c_str(), stderr);
    std::quick_exit(2);
  };
  auto executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides(
    {{"robot_description", URDF},
     {"slow.type", std::string("test/Slow")},
     // Bounds the teardown behind the stuck RT, long enough for a load under a sanitizer
     {"default_timeout_ms", 2000}});
  auto cm = std::make_shared<SlowControllerManager>(
    executor, "cm_destroyed", RUN_NAMESPACE, options,
    [](const hardware_interface::HardwareInfo &) { return std::make_unique<System>(); });
  executor->add_node(cm);
  // Both run until the quick exit, RT never returns from the held update, so nothing joins them
  std::thread([executor] { executor->spin(); }).detach();
  new testing::RtThread([rt = cm.get()](const rclcpp::Time &, const rclcpp::Duration & period)
                        { testing::read_update_write(*rt, rt->now(), period); });
  // The services come last in the bring up, a load beside their creation hits a cyclone race
  const auto created = [&]
  {
    return cm->get_service_names_and_types_by_node(cm->get_name(), cm->get_namespace())
      .contains(std::string(cm->get_fully_qualified_name()) + "/set_hardware_component_state");
  };
  if (!eventually(created, 10s))
  {
    fail("precondition: the services are created");
  }
  const auto loaded = cm->load_controller("slow");
  if (!loaded.ok)
  {
    fail("precondition: slow is loaded, " + loaded.message);
  }
  auto configure = std::make_shared<ControllerManager::ConfigureController::Request>();
  configure->name = "slow";
  auto configured = std::make_shared<ControllerManager::ConfigureController::Response>();
  cm->configure_controller_service_cb(configure, configured);
  auto activate = std::make_shared<ControllerManager::SwitchController::Request>();
  activate->activate_controllers = {"slow"};
  auto activated = std::make_shared<ControllerManager::SwitchController::Response>();
  cm->switch_controller_service_cb(activate, activated);
  if (!configured->ok || !activated->ok)
  {
    fail("precondition: slow is active");
  }
  update_gate.close();
  if (!update_gate.holds())
  {
    fail("precondition: RT is stuck in the update");
  }
  Hammer hammer(cm->get_fully_qualified_name());
  if (!hammer.ready())
  {
    fail("precondition: the services are offered");
  }
  cm.reset();
  // Answers that were already on their way arrive now
  std::this_thread::sleep_for(200ms);
  std::quick_exit(0);
}

// Embedded, as by gz_ros2_control, the executor may still spin while the controller manager goes.
// The stuck RT holds up the teardown of the supervisor, so calls keep arriving all through it.
TEST_F(ControllerManagerTest, service_calls_during_the_destruction_do_not_crash)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(destroy_while_called(), ::testing::ExitedWithCode(0), "");
}

// Static destructors would destroy what the DDS threads of a leaked node still use
TEST_F(ControllerManagerTest, the_process_exits_without_static_destruction_after_a_leak)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      leak(std::make_shared<rclcpp::Node>("leaked"));
      rclcpp::shutdown();
      std::atexit([] { std::_Exit(3); });
      exit_process(0);
    },
    ::testing::ExitedWithCode(0), "");
}

// The introspection node lives to the end of the process
TEST_F(ControllerManagerTest, the_process_exits_without_static_destruction_after_a_manager)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      rclcpp::NodeOptions options = get_cm_node_options();
      options.parameter_overrides({{"robot_description", URDF}});
      auto cm = std::make_shared<ControllerManager>(
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>(), "cm_exit", RUN_NAMESPACE,
        options,
        [](const hardware_interface::HardwareInfo &) { return std::make_unique<System>(); });
      cm->shutdown();
      cm.reset();
      rclcpp::shutdown();
      std::atexit([] { std::_Exit(3); });
      exit_process(0);
    },
    ::testing::ExitedWithCode(0), "");
}

TEST_F(ControllerManagerTest, the_process_exits_normally_without_a_leak)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      rclcpp::shutdown();
      std::atexit([] { std::_Exit(3); });
      exit_process(0);
    },
    ::testing::ExitedWithCode(3), "");
}

}  // namespace
}  // namespace hector_controller_manager
