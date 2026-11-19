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

#include "fixtures.hpp"
#include "hector_controller_manager/core/names.hpp"
#include "hector_controller_manager/core/rollback.hpp"

namespace hector_controller_manager::core
{
namespace
{
using testing::rig;
using testing::started_rig;
using testing::startup_active;
using testing::switch_to;

TEST(Rollback, returns_to_the_state_before_the_switch)
{
  const Model model = started_rig();
  const Desired before = keep(model);
  Desired current = before;
  current.active.erase("wrist");
  EXPECT_EQ(plan_rollback(model, before, current, {}).active, before.active);
}

TEST(Rollback, drops_what_claims_hardware_that_became_unusable)
{
  const Model model = started_rig();
  const Desired before = keep(model);
  const Desired rollback = plan_rollback(model, before, before, {"flipper"});
  EXPECT_FALSE(rollback.active.count("flipper_pos"));
  EXPECT_FALSE(rollback.active.count("jsb"));
  EXPECT_TRUE(rollback.active.count("arm_vel"));
}

TEST(Rollback, drops_a_quarantined_controller_and_the_providers_it_writes)
{
  Model model = started_rig();
  const Desired before = keep(model);
  model.controllers.at("wrist").quarantined = 1;
  const Desired rollback = plan_rollback(model, before, before, {});
  EXPECT_FALSE(rollback.active.count("wrist"));
  EXPECT_FALSE(rollback.active.count("arm_vel"));
}

TEST(Rollback, drops_a_quarantined_controller_that_writes_no_provider)
{
  Model model = started_rig();
  const Desired before = keep(model);
  model.controllers.at("odom_pub").quarantined = 1;
  const Desired rollback = plan_rollback(model, before, before, {});
  EXPECT_FALSE(rollback.active.count("odom_pub"));
  EXPECT_TRUE(rollback.active.count("diff_drive"));
}

TEST(Rollback, drops_a_failed_controller)
{
  Model model = started_rig();
  const Desired before = keep(model);
  model.controllers.at("odom_pub").failed = true;
  EXPECT_FALSE(plan_rollback(model, before, before, {}).active.count("odom_pub"));
}

// The failed switch started wrist on arm_vel's references, its rollback stops wrist again
TEST(Rollback, restarts_a_provider_whose_writer_it_stops)
{
  Model model = rig();
  switch_to(model, without(startup_active(), {"wrist"}), {});
  const Desired before = keep(model);
  switch_to(model, {"wrist"}, {});
  const Desired rollback = plan_rollback(model, before, keep(model), {});
  EXPECT_FALSE(rollback.active.count("wrist"));
  EXPECT_EQ(
    rollback.active.at("arm_vel").min_activation, model.controllers.at("arm_vel").activation + 1);
}

TEST(Rollback, keeps_the_hardware_targets_of_the_current_desired_state)
{
  const Model model = started_rig();
  const Desired before = keep(model);
  Desired current = before;
  current.hardware["imu"] = Lifecycle::Unconfigured;
  EXPECT_EQ(plan_rollback(model, before, current, {}).hardware.at("imu"), Lifecycle::Unconfigured);
}

TEST(Rollback, keep_running_drops_what_is_not_running_in_the_asked_activation)
{
  Model model = started_rig();
  Desired desired = keep(model);
  desired.active.at("odom_pub").min_activation = model.controllers.at("odom_pub").activation + 1;
  model.controllers.at("jsb").running = false;
  const Desired kept = keep_running(model, desired);
  EXPECT_FALSE(kept.active.count("odom_pub"));
  EXPECT_FALSE(kept.active.count("jsb"));
  EXPECT_TRUE(kept.active.count("arm_vel"));
}

TEST(Rollback, a_rollback_without_a_writer_restarts_the_provider_and_its_scope)
{
  const Model model = started_rig();
  Desired desired = keep(model);
  desired.active.erase("wrist");
  restart_for_departed_writers(model, keep(model), desired);
  for (const auto & [name, want] : desired.active)
  {
    const bool restarts = name == "arm_vel" || name == "arm_ik";
    EXPECT_EQ(want.min_activation, model.controllers.at(name).activation + (restarts ? 1 : 0))
      << name;
  }
}

TEST(Rollback, a_rollback_counts_a_writer_the_failed_switch_already_stopped)
{
  Model model = started_rig();
  const Desired before = keep(model);
  model.controllers.at("wrist").state = Lifecycle::Inactive;
  model.controllers.at("wrist").claims = {};
  Desired desired = keep(model);
  restart_for_departed_writers(model, before, desired);
  EXPECT_EQ(
    desired.active.at("arm_vel").min_activation, model.controllers.at("arm_vel").activation + 1);
}

}  // namespace
}  // namespace hector_controller_manager::core
