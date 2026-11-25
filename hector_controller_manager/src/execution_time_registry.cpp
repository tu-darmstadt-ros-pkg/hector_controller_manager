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

#include "hector_controller_manager/execution_time_registry.hpp"

#include <functional>
#include <memory>
#include <utility>

#include "hardware_interface/introspection.hpp"

namespace hector_controller_manager
{
namespace
{
std::shared_ptr<pal_statistics::StatisticsRegistry> own_registry(rclcpp::Node & node)
{
  const std::string key =
    node.get_node_topics_interface()->resolve_topic_name(hardware_interface::CM_STATISTICS_TOPIC);
  pal_statistics::deleteRegistry(key);
  auto registry = pal_statistics::getOrCreateRegistry(
    node.get_node_parameters_interface(), node.get_node_topics_interface(),
    node.get_node_logging_interface(), node.get_node_clock_interface(),
    hardware_interface::CM_STATISTICS_TOPIC, key);
  registry->startPublishThread();
  return registry;
}
}  // namespace

ExecutionTimeRegistry::ExecutionTimeRegistry(
  rclcpp::Node & node, std::string total, std::function<double()> cycle_seconds)
: total_(std::move(total)), cycle_seconds_(std::move(cycle_seconds)), registry_(own_registry(node))
{
}

void ExecutionTimeRegistry::add(const Status & status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Microseconds like upstream ros2_control, which the rqml statistics panel expects
  const auto add = [&](const std::string & name, std::function<double()> seconds)
  {
    if (registered_.insert(name).second)
    {
      registry_->registerFunction(name, [seconds = std::move(seconds)] { return seconds() * 1e6; });
    }
  };
  // The runtime may be unloaded before a sample
  const auto sampled = [](auto runtime, auto seconds) -> std::function<double()>
  {
    return [weak = std::weak_ptr(runtime), seconds]
    {
      const auto locked = weak.lock();
      return locked ? std::invoke(seconds, *locked) : 0.0;
    };
  };
  add(total_, cycle_seconds_);
  for (const auto & hardware : status.hardware)
  {
    add(
      hardware.name + ".stats/read_cycle/execution_time/current_value",
      sampled(
        hardware.runtime,
        [](const runtime::HardwareRuntime & runtime) { return runtime.stats().read_seconds(); }));
    add(
      hardware.name + ".stats/write_cycle/execution_time/current_value",
      sampled(
        hardware.runtime,
        [](const runtime::HardwareRuntime & runtime) { return runtime.stats().write_seconds(); }));
  }
  for (const auto & controller : status.controllers)
  {
    add(
      controller.name + ".stats/execution_time/current_value",
      sampled(controller.runtime, &runtime::ControllerRuntime::update_seconds));
  }
}

void ExecutionTimeRegistry::remove_all()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & name : registered_)
  {
    registry_->unregisterVariable(name);
  }
}

}  // namespace hector_controller_manager
