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

#include "hector_controller_manager/estop_monitor.hpp"

#include <utility>

namespace hector_controller_manager
{
EStopMonitor::EStopMonitor(rclcpp::Node & node, EStopTopics topics, Listener listener)
: listener_(std::move(listener)),
  topics_(std::move(topics)),
  engaged_(!topics_.empty()),
  group_(node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false))
{
  if (topics_.empty())
  {
    return;
  }
  listener_(true, topics_.cause(), false);
  rclcpp::SubscriptionOptions options;
  options.callback_group = group_;
  for (const auto & topic : topics_.names())
  {
    subscriptions_.emplace(
      topic, node.create_subscription<std_msgs::msg::Bool>(
               topic, rclcpp::QoS(1).reliable().transient_local(),
               [this, topic](const std_msgs::msg::Bool & message)
               {
                 std::lock_guard<std::mutex> lock(mutex_);
                 topics_.message(topic, message.data, Clock::now());
                 apply();
               },
               options));
  }
  // A publisher that is gone counts as engaged until a live one releases again. The publisher
  // count works on every RMW.
  liveness_timer_ = node.create_wall_timer(
    CHECK_PERIOD,
    [this]
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = Clock::now();
      for (const auto & [topic, subscription] : subscriptions_)
      {
        if (
          topics_.publishers(topic, subscription->get_publisher_count(), now) ||
          topics_.deadline_missed(topic, now))
        {
          apply();
        }
      }
    },
    group_);
  executor_.add_callback_group(group_, node.get_node_base_interface());
}

EStopMonitor::~EStopMonitor() { stop(); }

void EStopMonitor::start()
{
  if (!subscriptions_.empty())
  {
    thread_ = std::thread([this, stopped = stop_.get_future().share()]
                          { executor_.spin_until_future_complete(stopped); });
  }
}

void EStopMonitor::stop()
{
  std::call_once(
    stopped_,
    [this]
    {
      if (thread_.joinable())
      {
        stop_.set_value();
        thread_.join();
      }
    });
}

void EStopMonitor::apply()
{
  const bool engaged = topics_.engaged();
  const bool changed = engaged_.exchange(engaged, std::memory_order_acq_rel) != engaged;
  listener_(engaged, topics_.cause(), changed);
}

}  // namespace hector_controller_manager
