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

#ifndef HECTOR_CONTROLLER_MANAGER__ENTITIES_HPP_
#define HECTOR_CONTROLLER_MANAGER__ENTITIES_HPP_

#include <map>
#include <memory>
#include <string>

#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/runtime/controller_runtime.hpp"
#include "hector_controller_manager/runtime/hardware_runtime.hpp"

namespace hector_controller_manager
{
/// The runtimes of the model's entities and the interfaces they provide, by name. Supervisor
/// thread only.
struct Entities
{
  std::map<std::string, std::shared_ptr<runtime::ControllerRuntime>> controllers;
  std::map<std::string, std::shared_ptr<runtime::HardwareRuntime>> hardware;
  std::map<std::string, hardware_interface::CommandInterface::SharedPtr> command_handles;
  std::map<std::string, hardware_interface::StateInterface::ConstSharedPtr> state_handles;

  runtime::InterfaceHandles handles_of(const core::Claims & claims) const
  {
    runtime::InterfaceHandles handles;
    for (const auto & name : claims.command)
    {
      handles.command.push_back(command_handles.at(name));
    }
    for (const auto & name : claims.state)
    {
      handles.state.push_back(state_handles.at(name));
    }
    return handles;
  }
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__ENTITIES_HPP_
