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

#include <gtest/gtest.h>

#include "hector_controller_manager/runtime/lifecycle_state.hpp"
#include "hector_controller_manager/status_messages.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using core::Lifecycle;
using StateMsg = lifecycle_msgs::msg::State;

TEST(LifecycleState, primary_states_have_the_ids_and_labels_of_lifecycle_msgs)
{
  EXPECT_EQ(primary_state(Lifecycle::Unconfigured).id(), StateMsg::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(primary_state(Lifecycle::Inactive).id(), StateMsg::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(primary_state(Lifecycle::Active).id(), StateMsg::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(primary_state(Lifecycle::Finalized).id(), StateMsg::PRIMARY_STATE_FINALIZED);
  EXPECT_EQ(primary_state(Lifecycle::Unconfigured).label(), "unconfigured");
  EXPECT_EQ(primary_state(Lifecycle::Inactive).label(), "inactive");
  EXPECT_EQ(primary_state(Lifecycle::Active).label(), "active");
  EXPECT_EQ(primary_state(Lifecycle::Finalized).label(), "finalized");
}

TEST(LifecycleState, ids_map_back_and_transition_states_count_as_unconfigured)
{
  for (const auto state :
       {Lifecycle::Unconfigured, Lifecycle::Inactive, Lifecycle::Active, Lifecycle::Finalized})
  {
    EXPECT_EQ(lifecycle_of_id(primary_state(state).id()), state);
  }
  EXPECT_EQ(lifecycle_of_id(StateMsg::TRANSITION_STATE_ACTIVATING), Lifecycle::Unconfigured);
}

TEST(LifecycleState, a_hardware_target_takes_the_id_and_the_label_only_without_an_id)
{
  StateMsg target;
  target.id = StateMsg::PRIMARY_STATE_INACTIVE;
  target.label = "active";
  EXPECT_EQ(lifecycle_of(target), Lifecycle::Inactive);
  target.id = StateMsg::PRIMARY_STATE_UNCONFIGURED;
  EXPECT_EQ(lifecycle_of(target), std::nullopt);
  target.id = StateMsg::PRIMARY_STATE_UNKNOWN;
  EXPECT_EQ(lifecycle_of(target), Lifecycle::Active);
}

}  // namespace
}  // namespace hector_controller_manager::runtime
