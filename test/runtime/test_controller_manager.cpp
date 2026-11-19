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
#include <cstdlib>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "fakes.hpp"
#include "gate.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/controller_manager.hpp"
#include "hector_controller_manager/entities.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "log_capture.hpp"
#include "pal_statistics/pal_statistics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace hector_controller_manager
{
namespace
{
using hardware_interface::return_type;
using namespace std::chrono_literals;
using testing::eventually;

constexpr char URDF[] = R"(<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <link name="arm"/>
  <joint name="j1" type="continuous"><parent link="base"/><child link="arm"/></joint>
  <ros2_control name="system" type="system">
    <hardware><plugin>test/System</plugin></hardware>
    <joint name="j1">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>)";

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
    spin_for(1ms);
    return controller_interface::return_type::OK;
  }
};

/// Creates every controller as a SlowController instead of through pluginlib.
class SlowControllerManager : public ControllerManager
{
public:
  using ControllerManager::ControllerManager;

private:
  runtime::ControllerRuntime::Plugin create_controller(
    const std::string & name, const std::string &) override
  {
    auto plugin = testing::initialized(std::make_shared<SlowController>(), name);
    executor()->add_node(plugin->get_node()->get_node_base_interface());
    return plugin;
  }
};

class ControllerManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  // Also after a failed assertion, the next test would get a failing System
  void TearDown() override
  {
    lifecycle_delay_ms = 0;
    activation_fails = false;
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

/// Whether a controller manager with these parameter overrides can be created.
bool constructs(const std::vector<rclcpp::Parameter> & overrides)
{
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides(overrides);
  try
  {
    std::make_shared<ControllerManager>(
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>(), "cm_parameters", "", options);
    return true;
  }
  catch (const std::runtime_error &)
  {
    return false;
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
  options.parameter_overrides({{"robot_description", std::string(URDF)}});
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
    {"hardware_hang_timeout_ms", 0},
    {"hardware_hang_timeout_ms", -5},
    {"default_timeout_ms", 0},
    {"default_timeout_ms", -5},
    {"hardware_state_timeout_ms", 0},
    {"hardware_fault_timeout_ms", 0},
    {"estop_publisher_grace_ms", 0},
    {"estop_publisher_grace_ms", -5},
    {"autostart_cooldown_ms", -1},
    {"autostart_attempts", 0},
  };
  for (const auto & parameter : rejected)
  {
    EXPECT_FALSE(constructs({parameter}))
      << parameter.get_name() << " " << parameter.value_to_string();
  }
  EXPECT_TRUE(constructs(
    {{"update_rate", 1},
     {"hardware_hang_timeout_ms", 1},
     {"default_timeout_ms", 1},
     {"hardware_state_timeout_ms", 1},
     {"hardware_fault_timeout_ms", 1},
     {"estop_publisher_grace_ms", 1},
     {"autostart_cooldown_ms", 0},
     {"autostart_attempts", 1}}));
}

TEST_F(ControllerManagerTest, an_estop_deadline_below_100_ms_is_rejected)
{
  for (const int64_t deadline : {1, 99, -100})
  {
    EXPECT_FALSE(constructs({{"estop_deadline_ms", deadline}})) << deadline;
  }
  for (const int64_t deadline : {0, 100})
  {
    EXPECT_TRUE(constructs({{"estop_deadline_ms", deadline}})) << deadline;
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

// Hardware slower than the default timeout comes up at startup and through a state request
TEST_F(ControllerManagerTest, hardware_state_changes_get_the_hardware_state_timeout)
{
  lifecycle_delay_ms = 400;
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::NodeOptions options = get_cm_node_options();
  options.parameter_overrides(
    {{"robot_description", std::string(URDF)},
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
    {{"robot_description", std::string(URDF)}, {"slow.type", std::string("test/Slow")}});
  auto cm = std::make_shared<SlowControllerManager>(
    executor, "cm_statistics", "", options,
    [](const hardware_interface::HardwareInfo &) { return std::make_unique<System>(); });
  const Spinning spinning(executor, cm);
  std::atomic<bool> stop{false};
  std::thread loop(
    [&]
    {
      const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.001);
      while (!stop)
      {
        cm->read(cm->now(), period);
        cm->update(cm->now(), period);
        cm->write(cm->now(), period);
      }
    });
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
  stop = true;
  loop.join();
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
TEST_F(ControllerManagerTest, each_manager_and_each_successor_of_a_name_has_statistics_of_its_own)
{
  const auto make = [](const std::string & name)
  {
    rclcpp::NodeOptions options = get_cm_node_options();
    options.parameter_overrides({{"robot_description", std::string(URDF)}});
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
  auto successor = make("cm_first");
  EXPECT_TRUE(left_behind.expired());
  EXPECT_TRUE(statistics_of(*successor).contains("cm_first.total_time"));
  successor->shutdown();
  second->shutdown();
}

// controller_interface registers each controller's introspection in this registry and warns
// of every entity it cannot register without it. It stays published while any manager lives,
// also once the manager that created it is gone.
TEST_F(ControllerManagerTest, the_controllers_introspection_outlives_the_manager_that_made_it)
{
  const auto manager = [](const std::string & name)
  {
    rclcpp::NodeOptions options = get_cm_node_options();
    options.parameter_overrides({{"robot_description", std::string(URDF)}});
    const auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto cm = std::make_shared<ControllerManager>(
      executor, name, RUN_NAMESPACE, options,
      [](const hardware_interface::HardwareInfo &) { return std::make_unique<System>(); });
    return std::make_unique<Spinning>(executor, cm);
  };
  auto first = manager("cm_introspection_first");
  const auto second = manager("cm_introspection_second");
  const auto registry = pal_statistics::getRegistry(hardware_interface::DEFAULT_REGISTRY_KEY);
  ASSERT_NE(registry, nullptr);
  double probe = 1.0;
  registry->registerVariable("introspection_probe", &probe);
  first.reset();

  auto listener = std::make_shared<rclcpp::Node>("introspection_listener", RUN_NAMESPACE);
  std::atomic<int> received{0};
  using Values = pal_statistics_msgs::msg::StatisticsValues;
  const auto subscription = listener->create_subscription<Values>(
    "/ros2_control/introspection_data/values", rclcpp::SensorDataQoS(),
    [&](const Values &) { ++received; });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received == 0 && std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some(10ms);
  }
  // Every 50 ms, 40 in 2 s, where a publish of every other tick gives 25
  received = 0;
  const auto end = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < end)
  {
    executor.spin_some(10ms);
  }
  registry->unregisterVariable("introspection_probe");
  EXPECT_GE(received, 32);
  EXPECT_LE(received, 60);
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
    executor_->add_node(cm_);
    spinner_ = std::thread([this] { executor_->spin(); });
  }
  ~TopicDescription()
  {
    cm_->shutdown();
    executor_->cancel();
    spinner_.join();
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
  std::thread spinner_;
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
  topic.publish(URDF);
  EXPECT_TRUE(topic.executor_free());
  std::this_thread::sleep_for(300ms);
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
  topic.publish(URDF);
  EXPECT_TRUE(topic.executor_free());
  std::this_thread::sleep_for(300ms);
  EXPECT_FALSE(topic.cm().is_initialized());
  EXPECT_EQ(topic.created.load(), 0);
  EXPECT_EQ(
    topic.cm().load_controller("slow").message, "Initialization failed, the node needs a restart");
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

// The controllers' introspection node lives to the end of the process
TEST_F(ControllerManagerTest, the_process_exits_without_static_destruction_after_a_manager)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      rclcpp::NodeOptions options = get_cm_node_options();
      options.parameter_overrides({{"robot_description", std::string(URDF)}});
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
