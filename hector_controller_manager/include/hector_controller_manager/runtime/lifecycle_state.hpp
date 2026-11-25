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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__LIFECYCLE_STATE_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__LIFECYCLE_STATE_HPP_

#include <cstdint>

#include "hector_controller_manager/core/model.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hector_controller_manager::runtime
{
/// The primary state of lifecycle_msgs, id and label, of a lifecycle state of the model.
rclcpp_lifecycle::State primary_state(core::Lifecycle state);

/// The lifecycle state of a primary state id. A transition state counts as unconfigured.
core::Lifecycle lifecycle_of_id(uint8_t id);

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__LIFECYCLE_STATE_HPP_
