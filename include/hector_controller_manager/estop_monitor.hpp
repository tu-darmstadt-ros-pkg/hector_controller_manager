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

#ifndef HECTOR_CONTROLLER_MANAGER__ESTOP_MONITOR_HPP_
#define HECTOR_CONTROLLER_MANAGER__ESTOP_MONITOR_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "hector_controller_manager/estop_topics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace hector_controller_manager
{
/// Watches the e-stop topics of a node. Its subscriptions and the publisher check run in a callback
/// group that an executor of its own spins on a thread of its own, so no slow callback of the node
/// and no executor size delays an engage. Without topics it is released and does nothing.
class EStopMonitor
{
public:
  /// Called after every update with the monitor's lock held, `changed` when `engaged` flipped.
  using Listener = std::function<void(bool engaged, const std::string & cause, bool changed)>;

  /// How often it checks the publishers and deadlines, so a shorter deadline cannot be met.
  static constexpr std::chrono::milliseconds CHECK_PERIOD{100};

  /// Subscribes on `node` and reports the initial engaged state of topics that sent nothing yet,
  /// with `changed` false. Messages wait until start().
  EStopMonitor(rclcpp::Node & node, EStopTopics topics, Listener listener);
  ~EStopMonitor();

  EStopMonitor(const EStopMonitor &) = delete;
  EStopMonitor & operator=(const EStopMonitor &) = delete;

  /// Called once, before stop().
  void start();
  /// Ends the thread, from any thread and any number of times.
  void stop();
  /// Lock free, so a waiter on another lock may read it.
  bool engaged() const { return engaged_.load(std::memory_order_acquire); }

private:
  /// After a change of `topics_`, with `mutex_` held.
  void apply();

  Listener listener_;
  std::mutex mutex_;
  EStopTopics topics_;
  /// Written under `mutex_`, its exchange tells whether it flipped. Any thread acquires it.
  std::atomic<bool> engaged_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::map<std::string, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr liveness_timer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::promise<void> stop_;
  std::once_flag stopped_;
  std::thread thread_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__ESTOP_MONITOR_HPP_
