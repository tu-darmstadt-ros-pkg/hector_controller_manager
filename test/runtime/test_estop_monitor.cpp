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

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gate.hpp"
#include "hector_controller_manager/estop_monitor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace std::chrono_literals;

/// The namespace of this test run, so concurrent runs in one domain do not cross-talk.
const std::string RUN_NAMESPACE = "/hcm_estop_" + std::to_string(::getpid());

class EStopMonitorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// Timers hold every executor thread of the node, an engage still reaches the listener
TEST_F(EStopMonitorTest, an_engage_arrives_while_the_executor_of_the_node_is_held)
{
  for (const size_t threads : {1u, 2u})
  {
    SCOPED_TRACE("executor threads " + std::to_string(threads));
    const std::string topic = RUN_NAMESPACE + "_" + std::to_string(threads);
    auto node = std::make_shared<rclcpp::Node>("estop_held");
    std::mutex mutex;
    std::vector<bool> changes;
    EStopMonitor monitor(
      *node, EStopTopics({topic}, 1s),
      [&](bool engaged, const std::string &, bool changed)
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (changed)
        {
          changes.push_back(engaged);
        }
      });
    const auto changed_to = [&](std::vector<bool> expected)
    {
      return testing::eventually(
        [&]
        {
          std::lock_guard<std::mutex> lock(mutex);
          return changes == expected;
        });
    };
    testing::Gate held;
    held.close();
    std::atomic<size_t> holding{0};
    // The node keeps its callback groups only weakly
    std::vector<rclcpp::CallbackGroup::SharedPtr> groups;
    std::vector<rclcpp::TimerBase::SharedPtr> holders;
    for (size_t i = 0; i < threads; ++i)
    {
      groups.push_back(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
      holders.push_back(node->create_wall_timer(
        1ms,
        [&]
        {
          ++holding;
          held.pass();
        },
        groups.back()));
    }
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), threads);
    executor.add_node(node);
    std::thread spinner([&] { executor.spin(); });
    monitor.start();
    auto client = std::make_shared<rclcpp::Node>("estop_held_client");
    auto estop = client->create_publisher<std_msgs::msg::Bool>(
      topic, rclcpp::QoS(1).reliable().transient_local());
    std_msgs::msg::Bool message;
    message.data = false;
    estop->publish(message);
    EXPECT_TRUE(changed_to({false}));
    ASSERT_TRUE(testing::eventually([&] { return holding.load() >= threads; }))
      << "precondition: every executor thread is held";
    message.data = true;
    estop->publish(message);
    EXPECT_TRUE(changed_to({false, true}));
    EXPECT_TRUE(monitor.engaged());
    for (auto & holder : holders)
    {
      holder->cancel();
    }
    held.open();
    monitor.stop();
    executor.cancel();
    spinner.join();
  }
}

// The thread is stopped once, also when two threads stop it at the same time
TEST_F(EStopMonitorTest, concurrent_stops_end_the_thread_once)
{
  auto node = std::make_shared<rclcpp::Node>("estop_twice");
  EStopMonitor monitor(
    *node, EStopTopics({RUN_NAMESPACE + "_twice"}, 1s), [](bool, const std::string &, bool) {});
  monitor.start();
  std::thread first([&] { monitor.stop(); });
  std::thread second([&] { monitor.stop(); });
  first.join();
  second.join();
}

}  // namespace
}  // namespace hector_controller_manager
