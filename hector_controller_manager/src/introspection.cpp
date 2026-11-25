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

#include "hector_controller_manager/introspection.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "hardware_interface/introspection.hpp"
#include "hector_controller_manager/leak.hpp"

namespace hector_controller_manager
{
namespace
{
std::atomic<Introspection *> instance{nullptr};
}  // namespace

Introspection::Introspection(rclcpp::Node & manager)
: node_(
    std::make_shared<rclcpp::Node>(
      hardware_interface::DEFAULT_REGISTRY_KEY, manager.get_namespace(),
      rclcpp::NodeOptions()
        .context(manager.get_node_base_interface()->get_context())
        // A remap of the manager's name must not rename this node too
        .use_global_arguments(false)
        .start_parameter_services(false)
        .parameter_overrides({manager.get_parameter("use_sim_time")}))),
  registry_(
    pal_statistics::getOrCreateRegistry(
      node_, hardware_interface::DEFAULT_INTROSPECTION_TOPIC,
      hardware_interface::DEFAULT_REGISTRY_KEY))
{
  registry_->startPublishThread();
  RCLCPP_INFO(manager.get_logger(), "Introspection is published on %s", topic().c_str());
}

std::unique_ptr<Introspection::Member> Introspection::join(rclcpp::Node & manager)
{
  static std::mutex creation;
  Introspection * introspection = nullptr;
  {
    const std::lock_guard lock(creation);
    introspection = instance.load(std::memory_order_acquire);
    if (!introspection)
    {
      introspection = new Introspection(manager);
      record_leak();
      instance.store(introspection, std::memory_order_release);
    }
  }
  const std::lock_guard lock(introspection->mutex_);
  if (++introspection->members_ == 2)
  {
    (void)introspection->gate_.owner_acquire(Clock::time_point::max());
  }
  return std::make_unique<Member>(*introspection);
}

std::string Introspection::topic() const
{
  return node_->get_node_topics_interface()->resolve_topic_name(
    hardware_interface::DEFAULT_INTROSPECTION_TOPIC);
}

Introspection * Introspection::get() noexcept { return instance.load(std::memory_order_acquire); }

void Introspection::leave()
{
  const std::lock_guard lock(mutex_);
  if (--members_ == 1)
  {
    gate_.owner_release();
  }
}

}  // namespace hector_controller_manager
