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

#ifndef HECTOR_CONTROLLER_MANAGER__PLUGIN_LOADER_HPP_
#define HECTOR_CONTROLLER_MANAGER__PLUGIN_LOADER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/runtime/controller_runtime.hpp"
#include "pluginlib/class_loader.hpp"

namespace hector_controller_manager
{
/// The pluginlib loaders of controllers and hardware, safe from any thread. Every plugin it creates
/// pins its library until the process exits.
class PluginLoader
{
public:
  PluginLoader();

  /// Null if no loader declares the type.
  runtime::ControllerRuntime::Plugin create_controller(const std::string & type);
  /// Throws for a hardware type other than system, actuator and sensor.
  std::unique_ptr<hardware_interface::HardwareComponentInterface> create_hardware(
    const hardware_interface::HardwareInfo & info);
  /// Every declared controller type with its base class.
  std::vector<std::pair<std::string, std::string>> controller_types();

private:
  std::mutex mutex_;
  pluginlib::ClassLoader<controller_interface::ControllerInterface> controllers_;
  pluginlib::ClassLoader<controller_interface::ChainableControllerInterface> chainable_;
  pluginlib::ClassLoader<hardware_interface::SystemInterface> systems_;
  pluginlib::ClassLoader<hardware_interface::ActuatorInterface> actuators_;
  pluginlib::ClassLoader<hardware_interface::SensorInterface> sensors_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__PLUGIN_LOADER_HPP_
