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

#ifndef RUNTIME__FAKES_HPP_
#define RUNTIME__FAKES_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hector_controller_manager/plugin_host.hpp"
#include "hector_controller_manager/runtime/hardware_runtime.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hector_controller_manager::testing
{
/// Initializes a fake controller as the controller manager would, at 100 Hz.
inline runtime::ControllerRuntime::Plugin initialized(
  runtime::ControllerRuntime::Plugin controller, const std::string & name)
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = name;
  params.update_rate = 100;
  params.controller_manager_update_rate = 100;
  if (controller->init(params) != controller_interface::return_type::OK)
  {
    throw std::runtime_error("init failed");
  }
  return controller;
}

/// The parameters of a fake hardware component, on the steady clock.
inline hardware_interface::HardwareComponentParams steady_params(
  const hardware_interface::HardwareInfo & info)
{
  hardware_interface::HardwareComponentParams params;
  params.hardware_info = info;
  params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  params.logger = rclcpp::get_logger("test");
  return params;
}

/// A system, sync unless `async`, with the position command and state of joint j1.
inline hardware_interface::HardwareInfo hardware_info(const std::string & name, bool async = false)
{
  hardware_interface::HardwareInfo info;
  info.name = name;
  info.type = "system";
  info.hardware_plugin_name = "test/System";
  // HardwareInfo leaves these uninitialized
  info.is_async = async;
  info.rw_rate = 0;
  hardware_interface::ComponentInfo joint;
  joint.name = "j1";
  joint.type = "joint";
  hardware_interface::InterfaceInfo position{};
  position.name = "position";
  joint.command_interfaces.push_back(position);
  joint.state_interfaces.push_back(position);
  info.joints.push_back(joint);
  return info;
}

/// Creates the plugin of `factory`, configures and activates it on the owner thread, as the
/// supervisor brings a component up.
template <typename Factory>
bool bring_up(runtime::HardwareRuntime & hardware, Factory factory, bool activate = true)
{
  auto job = hardware.owner().submit(
    [&]
    {
      hardware.initialize([&](const auto &) { return factory(); }, steady_params(hardware.info()));
      hardware.transition(core::step::HardwareTransition::Configure);
      if (activate)
      {
        hardware.transition(core::step::HardwareTransition::Activate);
      }
    });
  return hardware.owner().wait(job, Clock::now() + std::chrono::seconds(5));
}

/// A controller that claims nothing, a fake overrides what it uses.
template <typename Base = controller_interface::ControllerInterface>
class NoInterfaces : public Base
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }
};

/// A chainable controller that claims and exports nothing.
class NoInterfacesChainable
: public NoInterfaces<controller_interface::ChainableControllerInterface>
{
protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list()
    override
  {
    return {};
  }
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return controller_interface::return_type::OK;
  }
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return controller_interface::return_type::OK;
  }
};

/// Creates fake plugins through the given factories and counts the controllers it forgot.
class FakeHost : public PluginHost
{
public:
  using Controllers = std::function<runtime::ControllerRuntime::Plugin(
    const std::string & name, const std::string & type)>;
  using Hardware = std::function<std::unique_ptr<hardware_interface::HardwareComponentInterface>()>;

  FakeHost(Controllers controllers, Hardware hardware)
  : controllers_(std::move(controllers)), hardware_(std::move(hardware))
  {
  }

  runtime::ControllerRuntime::Plugin create_controller(
    const std::string & name, const std::string & type) override
  {
    return initialized(controllers_(name, type), name);
  }
  bool forget_controller(const runtime::ControllerRuntime::Plugin &) override
  {
    forgotten.fetch_add(1);
    return true;
  }
  std::unique_ptr<hardware_interface::HardwareComponentInterface> create_hardware(
    const hardware_interface::HardwareInfo &) override
  {
    return hardware_();
  }
  hardware_interface::HardwareComponentParams hardware_params(
    const hardware_interface::HardwareInfo & info) override
  {
    return steady_params(info);
  }

  std::atomic<int> forgotten{0};

private:
  Controllers controllers_;
  Hardware hardware_;
};

}  // namespace hector_controller_manager::testing

#endif  // RUNTIME__FAKES_HPP_
