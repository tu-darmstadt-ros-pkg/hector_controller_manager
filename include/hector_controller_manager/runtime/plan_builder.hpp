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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__PLAN_BUILDER_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__PLAN_BUILDER_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/runtime/controller_runtime.hpp"
#include "hector_controller_manager/runtime/hardware_runtime.hpp"
#include "hector_controller_manager/runtime/plan.hpp"

namespace hector_controller_manager::runtime
{
/// The plan RT runs for the controllers the model marks running, with their update order and, for
/// every controller and hardware component, the failure scope RT stops in the cycle it fails.
std::shared_ptr<Plan> build_plan(
  const core::Model & model,
  const std::map<std::string, std::shared_ptr<ControllerRuntime>> & controllers,
  const std::map<std::string, std::shared_ptr<HardwareRuntime>> & hardware, uint64_t generation);

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__PLAN_BUILDER_HPP_
