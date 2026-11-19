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

// The request services of a controller manager with the suite's plugins, called through DDS like
// a spawner calls them, and its e-stop topics.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "../runtime/log_capture.hpp"
#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "services_rig.hpp"
#include "std_msgs/msg/string.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace testing::services;
using testing::elapsed_ms;
using testing::eventually;

// An unload waits until the executor dropped the controller's node, which needs an executor thread
// that no request callback holds. Two unloads arrive together, so whichever runs second is in
// flight while the first one waits, and a slow on_shutdown gives the second one time to arrive.
TEST_P(Services, two_unloads_at_once_answer_in_time)
{
  const auto & controllers = cm_test_suite::rig::controllers();
  for (uint16_t index : {0, 1})
  {
    auto load = send(load_, controllers.at(index).name);
    ASSERT_TRUE(answered(load));
    cm_test_suite::faults().arm(
      {cm_test_suite::TargetKind::Controller, index}, cm_test_suite::Hook::Shutdown,
      cm_test_suite::Fault::Delay);
  }
  const testing::LogCapture log;
  const auto begin = std::chrono::steady_clock::now();
  auto first = send(unload_, controllers.at(0).name);
  auto second = send(unload_, controllers.at(1).name);
  EXPECT_TRUE(answered(first));
  EXPECT_TRUE(answered(second));
  const auto took = elapsed_ms(begin);
  EXPECT_LT(took, 500) << "the unloads took " << took << " ms";
  EXPECT_FALSE(log.contains("its plugin is abandoned"));
}

// The parameter reaches the planner, so FORCE_AUTO does not take the arm from the protected arm_vel
TEST_P(Services, force_auto_leaves_a_protected_controller_running)
{
  for (const std::string name : {"arm_vel", "arm_pos"})
  {
    auto load = send(load_, name);
    ASSERT_TRUE(answered(load));
    ASSERT_TRUE(configure(*cm_, name));
  }
  const auto switch_to = [&](std::vector<std::string> activate, std::vector<std::string> deactivate)
  {
    return switch_controllers(
      *cm_, std::move(activate), std::move(deactivate),
      ControllerManager::SwitchController::Request::FORCE_AUTO);
  };
  ASSERT_TRUE(switch_to({"arm_vel"}, {})->ok);
  const auto rejected = switch_to({"arm_pos"}, {});
  EXPECT_FALSE(rejected->ok);
  EXPECT_NE(rejected->message.find("protected 'arm_vel'"), std::string::npos) << rejected->message;
  EXPECT_TRUE(switch_to({"arm_pos"}, {"arm_vel"})->ok);
}

// A switch publishes many statuses, the activity topic carries only those that changed a state
TEST_P(Services, the_activity_is_published_only_on_a_change)
{
  using Activity = controller_manager_msgs::msg::ControllerManagerActivity;
  std::vector<std::string> jsb_states;
  const auto subscription = client_node_->create_subscription<Activity>(
    std::string(cm_->get_fully_qualified_name()) + "/activity",
    // Deep enough to keep every message until the test spins
    rclcpp::QoS(100).reliable().transient_local(),
    [&](const Activity & message)
    {
      for (const auto & controller : message.controllers)
      {
        if (controller.name == "jsb")
        {
          jsb_states.push_back(controller.state.label);
        }
      }
    });
  const auto received = [&](const std::string & label, size_t count)
  {
    return eventually(
      [&]
      {
        client_executor_.spin_some(10ms);
        return std::count(jsb_states.begin(), jsb_states.end(), label) >= static_cast<int>(count);
      });
  };
  auto load = send(load_, "jsb");
  ASSERT_TRUE(answered(load));
  ASSERT_TRUE(configure(*cm_, "jsb"));
  ASSERT_TRUE(received("inactive", 1));
  const size_t before = jsb_states.size();
  const auto switch_to = [&](std::vector<std::string> activate, std::vector<std::string> deactivate)
  { return switch_controllers(*cm_, std::move(activate), std::move(deactivate))->ok; };
  ASSERT_TRUE(switch_to({"jsb"}, {}));
  for (int i = 0; i < 3; ++i)
  {
    ASSERT_TRUE(switch_to({}, {}));
  }
  ASSERT_TRUE(switch_to({}, {"jsb"}));
  ASSERT_TRUE(received("inactive", 2));
  EXPECT_EQ(
    std::vector<std::string>(
      jsb_states.begin() + static_cast<std::ptrdiff_t>(before), jsb_states.end()),
    (std::vector<std::string>{"active", "inactive"}));
}

/// The robot_description_semantic a controller created now gets as a parameter override.
std::string semantic_of_a_new_controller(ControllerManager & cm)
{
  PluginHost & host = cm;
  const auto & monitor = cm_test_suite::rig::controller("monitor");
  const auto plugin =
    host.create_controller(monitor.name, cm_test_suite::rig::plugin_type(monitor));
  std::string semantic;
  for (const auto & parameter : plugin->get_node()->get_node_options().parameter_overrides())
  {
    if (parameter.get_name() == "robot_description_semantic")
    {
      semantic = parameter.as_string();
    }
  }
  host.forget_controller(plugin);
  return semantic;
}

constexpr char SRDF[] = "<robot name=\"test\"><group name=\"arm\"/></robot>";

// Without the parameter the first message on the topic is forwarded, a later one is ignored
TEST_P(Services, the_semantic_description_on_the_topic_reaches_new_controllers)
{
  EXPECT_EQ(semantic_of_a_new_controller(*cm_), "");
  auto publisher = client_node_->create_publisher<std_msgs::msg::String>(
    RUN_NAMESPACE + "/robot_description_semantic", rclcpp::QoS(1).transient_local());
  std_msgs::msg::String message;
  message.data = SRDF;
  publisher->publish(message);
  EXPECT_TRUE(eventually([&] { return semantic_of_a_new_controller(*cm_) == SRDF; }));
  message.data = "<robot name=\"other\"/>";
  publisher->publish(message);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(semantic_of_a_new_controller(*cm_), SRDF);
}

/// robot_description_semantic set as a parameter of the controller manager.
class Semantic : public Services
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    return {{"robot_description_semantic", std::string(SRDF)}};
  }
};

TEST_P(Semantic, the_semantic_description_parameter_reaches_new_controllers)
{
  EXPECT_EQ(semantic_of_a_new_controller(*cm_), SRDF);
}

INSTANTIATE_TEST_SUITE_P(ExecutorThreads, Semantic, ::testing::Values(2));

// A topic that delivered nothing counts as engaged, a switch is accepted once both released
TEST_P(EStop, an_unknown_estop_state_counts_as_engaged_and_an_engage_stops_claimers)
{
  for (const std::string name : {"jsb", "base_vel"})
  {
    auto load = send(load_, name);
    ASSERT_TRUE(answered(load));
    ASSERT_TRUE(configure(*cm_, name));
  }
  EXPECT_TRUE(switch_controllers(*cm_, {"jsb"}, {})->ok) << "a broadcaster claims no command";
  const testing::LogCapture log;
  const auto rejected = switch_controllers(*cm_, {"base_vel"}, {});
  EXPECT_FALSE(rejected->ok);
  EXPECT_NE(rejected->message.find("e-stop is engaged"), std::string::npos) << rejected->message;
  EXPECT_TRUE(log.contains("Switch rejected: " + rejected->message)) << log.text();
  publish("a", false);
  // Only one of two topics released, the other has said nothing yet
  const std::string cause = "e-stop is engaged by '" + topic("b") + "', it sent no message yet";
  EXPECT_TRUE(eventually(
    [&]
    {
      return switch_controllers(*cm_, {"base_vel"}, {})->message.find(cause) != std::string::npos;
    }));
  publish("b", false);
  EXPECT_TRUE(eventually([&] { return switch_controllers(*cm_, {"base_vel"}, {})->ok; }));
  publish("a", true);
  EXPECT_TRUE(eventually([&] { return state_of(*cm_, "base_vel") == "inactive"; }));
  EXPECT_EQ(state_of(*cm_, "jsb"), "active");
}

// A released topic whose publisher dies is engaged again, until a new publisher releases it
TEST_P(EStop, a_publisher_that_goes_away_engages_until_a_new_one_releases)
{
  {
    const testing::LogCapture log;
    publish("a", false);
    publish("b", false);
    ASSERT_TRUE(eventually([&] { return log.contains("E-stop released"); }));
  }
  {
    const testing::LogCapture log;
    const auto begin = std::chrono::steady_clock::now();
    publishers_.erase("a");
    EXPECT_TRUE(eventually(
      [&] { return log.contains("engaged by '" + topic("a") + "', it has no publisher"); }));
    // The grace of 1 s plus one check of 100 ms
    EXPECT_LT(std::chrono::steady_clock::now() - begin, 1500ms);
  }
  const testing::LogCapture log;
  publish("a", false);
  EXPECT_TRUE(eventually([&] { return log.contains("E-stop released"); }));
}

// A publisher the subscription does not match never delivers, the topic stays engaged
TEST_P(EStop, a_qos_incompatible_publisher_does_not_release)
{
  const testing::LogCapture log;
  publish("b", false);
  auto volatile_publisher =
    client_node_->create_publisher<std_msgs::msg::Bool>(topic("a"), rclcpp::QoS(1).best_effort());
  std_msgs::msg::Bool released;
  released.data = false;
  for (int i = 0; i < 10; ++i)
  {
    volatile_publisher->publish(released);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_FALSE(log.contains("E-stop released"));
}

INSTANTIATE_TEST_SUITE_P(ExecutorThreads, EStop, ::testing::Values(2));

/// The e-stop topics with a deadline, as the upstream e_stop_manager republishes its aggregates.
class EStopDeadline : public EStop
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    auto parameters = EStop::extra_parameters();
    parameters.emplace_back("estop_deadline_ms", int64_t{500});
    return parameters;
  }

  /// Publishes false on the topics every 100 ms until `done` holds or the time is up.
  template <typename Done>
  void release_periodically(
    const std::vector<std::string> & which, Done done, std::chrono::milliseconds time)
  {
    const auto end = std::chrono::steady_clock::now() + time;
    while (!done() && std::chrono::steady_clock::now() < end)
    {
      for (const auto & name : which)
      {
        publish(name, false);
      }
      std::this_thread::sleep_for(100ms);
    }
  }
};

TEST_P(EStopDeadline, a_topic_that_misses_its_deadline_engages_until_a_new_release)
{
  {
    const testing::LogCapture log;
    release_periodically({"a", "b"}, [&] { return log.contains("E-stop released"); }, 5s);
    ASSERT_TRUE(log.contains("E-stop released"));
  }
  {
    const testing::LogCapture log;
    release_periodically({"a", "b"}, [] { return false; }, 1500ms);
    EXPECT_FALSE(log.contains("E-stop engaged")) << log.text();
  }
  const std::string missed = "engaged by '" + topic("a") + "', it sent no message within";
  {
    const testing::LogCapture log;
    const auto begin = std::chrono::steady_clock::now();
    // a keeps its publisher, only its messages stop
    release_periodically({"b"}, [&] { return log.contains(missed); }, 3s);
    EXPECT_TRUE(log.contains(missed)) << log.text();
    // The deadline of 500 ms plus one check of 100 ms
    EXPECT_LT(std::chrono::steady_clock::now() - begin, 1500ms);
  }
  const testing::LogCapture log;
  publish("a", false);
  EXPECT_TRUE(eventually([&] { return log.contains("E-stop released"); }));
}

// An operator who reads why a stopped controller does not start again
TEST_P(EStopDeadline, the_cause_of_an_engage_reaches_a_refused_switch_and_the_diagnostics)
{
  auto load = send(load_, "base_vel");
  ASSERT_TRUE(answered(load));
  ASSERT_TRUE(configure(*cm_, "base_vel"));
  const std::string cause =
    "E-stop engaged by '" + topic("a") + "', it sent no message within estop_deadline_ms";
  {
    const testing::LogCapture log;
    release_periodically({"a", "b"}, [&] { return log.contains("E-stop released"); }, 5s);
    ASSERT_TRUE(log.contains("E-stop released"));
  }
  {
    const testing::LogCapture log;
    release_periodically({"b"}, [&] { return log.contains(cause); }, 3s);
    ASSERT_TRUE(log.contains(cause)) << log.text();
  }
  const auto rejected = switch_controllers(*cm_, {"base_vel"}, {});
  EXPECT_FALSE(rejected->ok);
  EXPECT_NE(
    rejected->message.find("e-stop is engaged by '" + topic("a") + "', it sent no message"),
    std::string::npos)
    << rejected->message;
  bool diagnosed = false;
  const auto subscription =
    client_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      [&](const diagnostic_msgs::msg::DiagnosticArray & array)
      {
        for (const auto & status : array.status)
        {
          diagnosed = diagnosed || (status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN &&
                                    status.message == cause);
        }
      });
  EXPECT_TRUE(eventually(
    [&]
    {
      client_executor_.spin_some(100ms);
      return diagnosed;
    }));
}

INSTANTIATE_TEST_SUITE_P(ExecutorThreads, EStopDeadline, ::testing::Values(3));

INSTANTIATE_TEST_SUITE_P(
  ExecutorThreads, Services, ::testing::Values(1, 2),
  [](const ::testing::TestParamInfo<size_t> & info) { return std::to_string(info.param); });

}  // namespace
}  // namespace hector_controller_manager
