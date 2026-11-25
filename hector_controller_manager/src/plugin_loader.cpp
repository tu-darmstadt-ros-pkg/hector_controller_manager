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

#include "hector_controller_manager/plugin_loader.hpp"

#include <stdexcept>

#include "hector_controller_manager/leak.hpp"

namespace hector_controller_manager
{
namespace
{
/// Static destructors after main unload the plugin libraries before rclcpp's default context
/// destroys the callbacks their dependencies registered on it, as MoveIt's do.
void pin_plugin_library(const std::string & path)
{
  if (!pin_library(path))
  {
    throw std::runtime_error("The plugin library '" + path + "' is not loaded");
  }
}
}  // namespace

PluginLoader::PluginLoader()
: controllers_("controller_interface", "controller_interface::ControllerInterface"),
  chainable_("controller_interface", "controller_interface::ChainableControllerInterface"),
  systems_("hardware_interface", "hardware_interface::SystemInterface"),
  actuators_("hardware_interface", "hardware_interface::ActuatorInterface"),
  sensors_("hardware_interface", "hardware_interface::SensorInterface")
{
}

runtime::ControllerRuntime::Plugin PluginLoader::create_controller(const std::string & type)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (chainable_.isClassAvailable(type))
  {
    auto plugin = chainable_.createSharedInstance(type);
    pin_plugin_library(chainable_.getClassLibraryPath(type));
    return plugin;
  }
  if (controllers_.isClassAvailable(type))
  {
    auto plugin = controllers_.createSharedInstance(type);
    pin_plugin_library(controllers_.getClassLibraryPath(type));
    return plugin;
  }
  return nullptr;
}

std::unique_ptr<hardware_interface::HardwareComponentInterface> PluginLoader::create_hardware(
  const hardware_interface::HardwareInfo & info)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & plugin = info.hardware_plugin_name;
  std::unique_ptr<hardware_interface::HardwareComponentInterface> component;
  if (info.type == "system")
  {
    component.reset(systems_.createUnmanagedInstance(plugin));
    pin_plugin_library(systems_.getClassLibraryPath(plugin));
  }
  else if (info.type == "actuator")
  {
    component.reset(actuators_.createUnmanagedInstance(plugin));
    pin_plugin_library(actuators_.getClassLibraryPath(plugin));
  }
  else if (info.type == "sensor")
  {
    component.reset(sensors_.createUnmanagedInstance(plugin));
    pin_plugin_library(sensors_.getClassLibraryPath(plugin));
  }
  else
  {
    throw std::runtime_error("Unknown hardware type '" + info.type + "'");
  }
  return component;
}

std::vector<std::pair<std::string, std::string>> PluginLoader::controller_types()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::pair<std::string, std::string>> types;
  for (const auto & type : controllers_.getDeclaredClasses())
  {
    types.emplace_back(type, "controller_interface::ControllerInterface");
  }
  for (const auto & type : chainable_.getDeclaredClasses())
  {
    types.emplace_back(type, "controller_interface::ChainableControllerInterface");
  }
  return types;
}

}  // namespace hector_controller_manager
