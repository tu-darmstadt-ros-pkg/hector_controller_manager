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

#include "hector_controller_manager/runtime/lifecycle_state.hpp"

#include <stdexcept>

#include "lifecycle_msgs/msg/state.hpp"

namespace hector_controller_manager::runtime
{
using core::Lifecycle;
using StateMsg = lifecycle_msgs::msg::State;

rclcpp_lifecycle::State primary_state(Lifecycle state)
{
  switch (state)
  {
    case Lifecycle::Unconfigured:
      return rclcpp_lifecycle::State(StateMsg::PRIMARY_STATE_UNCONFIGURED, to_string(state));
    case Lifecycle::Inactive:
      return rclcpp_lifecycle::State(StateMsg::PRIMARY_STATE_INACTIVE, to_string(state));
    case Lifecycle::Active:
      return rclcpp_lifecycle::State(StateMsg::PRIMARY_STATE_ACTIVE, to_string(state));
    case Lifecycle::Finalized:
      return rclcpp_lifecycle::State(StateMsg::PRIMARY_STATE_FINALIZED, to_string(state));
  }
  throw std::logic_error("Unknown lifecycle state");
}

Lifecycle lifecycle_of_id(uint8_t id)
{
  switch (id)
  {
    case StateMsg::PRIMARY_STATE_INACTIVE:
      return Lifecycle::Inactive;
    case StateMsg::PRIMARY_STATE_ACTIVE:
      return Lifecycle::Active;
    case StateMsg::PRIMARY_STATE_FINALIZED:
      return Lifecycle::Finalized;
    default:
      return Lifecycle::Unconfigured;
  }
}

}  // namespace hector_controller_manager::runtime
