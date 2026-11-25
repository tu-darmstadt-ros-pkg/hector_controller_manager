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

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "fixtures.hpp"
#include "hector_controller_manager/core/planner.hpp"

namespace hector_controller_manager::core
{
namespace
{
using testing::keep;
using testing::rig;
using testing::switch_to;

const std::optional<std::string> ENGAGED = "'/estop'";

/// Selects, and expects the planner to accept the AUTO switch of the selection.
StartupSelection select(
  const Model & model, const std::vector<std::string> & wanted, bool restore = false,
  const std::optional<std::string> & estop_cause = std::nullopt)
{
  auto selection = select_startup(model, wanted, restore, estop_cause);
  const auto plan =
    plan_switch(model, keep(model).hardware, {selection.activate, {}, Strictness::Auto});
  if (const auto * rejection = std::get_if<Rejection>(&plan))
  {
    ADD_FAILURE() << "the switch of the selection is rejected: " << rejection->reason;
  }
  return selection;
}

using Skipped = std::vector<std::string>;

TEST(Startup, a_released_startup_activates_every_wanted_controller)
{
  const auto selection = select(rig(), {"jsb", "base_vel", "arm_pos"});
  EXPECT_EQ(selection.activate, (Names{"jsb", "base_vel", "arm_pos"}));
  EXPECT_EQ(selection.skipped, Skipped{});
}

// Like a torque off controller after a hard e-stop, only an operator brings it back
TEST(Startup, a_restore_leaves_out_a_protected_controller)
{
  Model model = rig();
  model.controllers.at("base_vel").is_protected = true;
  const auto restored = select(model, {"jsb", "base_vel"}, true);
  EXPECT_EQ(restored.activate, Names{"jsb"});
  EXPECT_EQ(restored.skipped, Skipped{"'base_vel' (protected)"});
  EXPECT_EQ(select(model, {"jsb", "base_vel"}).activate, (Names{"jsb", "base_vel"}));
}

TEST(Startup, the_engaged_estop_leaves_out_only_unprotected_command_claimers)
{
  Model model = rig();
  model.controllers.at("arm_pos").is_protected = true;
  const auto selection = select(model, {"jsb", "base_vel", "arm_pos"}, false, ENGAGED);
  EXPECT_EQ(selection.activate, (Names{"jsb", "arm_pos"}));
  EXPECT_EQ(selection.skipped, Skipped{"'base_vel' (e-stop engaged by '/estop')"});
}

// Like a running gripper trajectory controller next to a startup gripper position controller
TEST(Startup, a_running_controller_on_its_joint_leaves_out_only_its_rival)
{
  Model model = rig();
  switch_to(model, {"arm_vel"}, {});
  const auto selection = select(model, {"arm_pos", "base_vel"});
  EXPECT_EQ(selection.activate, Names{"base_vel"});
  EXPECT_EQ(
    selection.skipped, Skipped{"'arm_pos' (a running controller claims its joint or reference)"});
}

// arm_ik and wrist write different references of arm_vel
TEST(Startup, a_running_writer_of_another_reference_leaves_a_writer_in)
{
  Model model = rig();
  switch_to(model, {"arm_vel", "arm_ik"}, {});
  const auto selection = select(model, {"arm_vel", "arm_ik", "wrist"});
  EXPECT_EQ(selection.activate, Names{"wrist"}) << "the running ones are left alone";
  EXPECT_EQ(selection.skipped, Skipped{});
}

// Like athena, arm_pos claims the arm joints and the startup set has arm_vel and its writer arm_ik
TEST(Startup, a_user_of_a_provider_on_a_running_joint_is_left_out)
{
  Model model = rig();
  switch_to(model, {"arm_pos"}, {});
  const auto selection = select(model, {"arm_vel", "arm_ik", "base_vel"});
  EXPECT_EQ(selection.activate, Names{"base_vel"});
  EXPECT_EQ(
    selection.skipped,
    (Skipped{
      "'arm_vel' (a running controller claims its joint or reference)",
      "'arm_ik' (it needs 'arm_vel', which is skipped: a running controller claims its joint or "
      "reference)"}));
}

// The latch would reject arm_vel, which AUTO adds for arm_ik, and with it the whole startup switch
TEST(Startup, a_protected_user_of_a_provider_the_estop_leaves_out_is_left_out)
{
  Model model = rig();
  model.controllers.at("arm_ik").is_protected = true;
  const auto selection = select(model, {"jsb", "arm_ik"}, false, ENGAGED);
  EXPECT_EQ(selection.activate, Names{"jsb"});
  EXPECT_EQ(
    selection.skipped,
    Skipped{"'arm_ik' (it needs 'arm_vel', which is skipped: e-stop engaged by '/estop')"});
}

// arm is still down, its arm_pos must not hold back the rest of the startup set
TEST(Startup, a_claimer_of_hardware_that_is_down_is_left_out)
{
  Model model = rig();
  model.hardware.at("arm").state = Lifecycle::Unconfigured;
  const auto selection = select(model, {"arm_pos", "base_vel"});
  EXPECT_EQ(selection.activate, Names{"base_vel"});
  EXPECT_EQ(
    selection.skipped,
    Skipped{
      "'arm_pos' ('arm_pos' claims 'a1/position' of hardware 'arm', which is not active or has a "
      "fault)"});
}

TEST(Startup, a_reader_of_a_state_interface_that_is_down_is_left_out)
{
  Model model = rig();
  model.hardware.at("imu").state = Lifecycle::Unconfigured;
  const auto selection = select(model, {"jsb", "base_vel", "diff_drive"});
  EXPECT_EQ(selection.activate, (Names{"jsb", "base_vel"}));
  EXPECT_EQ(
    selection.skipped,
    Skipped{"'diff_drive' ('diff_drive' claims the unavailable state interface 'imu/yaw')"});
}

TEST(Startup, a_controller_that_cannot_be_activated_is_left_out_with_the_reason)
{
  Model model = rig();
  model.controllers.at("arm_pos").state = Lifecycle::Finalized;
  model.controllers.at("flipper_vel").state = Lifecycle::Unconfigured;
  const auto selection = select(model, {"arm_pos", "flipper_vel", "base_vel"});
  EXPECT_EQ(selection.activate, Names{"base_vel"});
  ASSERT_EQ(selection.skipped.size(), 2u);
  EXPECT_EQ(
    selection.skipped[0], "'arm_pos' (Controller 'arm_pos' cannot be activated, it is finalized)");
  EXPECT_EQ(selection.skipped[1].rfind("'flipper_vel' (Controller 'flipper_vel' cannot be", 0), 0u)
    << selection.skipped[1];
}

// It may still hang in its stop, so it is not taken as running
TEST(Startup, a_quarantined_active_controller_is_left_out_with_the_reason)
{
  Model model = rig();
  switch_to(model, {"arm_pos"}, {});
  model.controllers.at("arm_pos").quarantined = 1;
  const auto selection = select_startup(model, {"arm_pos"}, false, std::nullopt);
  EXPECT_EQ(selection.activate, Names{});
  EXPECT_EQ(
    selection.skipped, Skipped{"'arm_pos' (a running controller claims its joint or reference)"});
}

// c1 hangs in its stop after a hold emptied live, the bus stays taken for it
TEST(Startup, a_claimer_of_another_joint_of_a_component_taken_for_a_hung_claimer_is_left_out)
{
  Model model;
  testing::add_hardware(model, "bus", {"j1", "j2"}, {"velocity"}, {});
  testing::add_controller(model, {"c1", false, {"j1/velocity"}, {}, {}, {}});
  testing::add_controller(model, {"c2", false, {"j2/velocity"}, {}, {}, {}});
  switch_to(model, {"c1"}, {});
  model.controllers.at("c1").running = false;
  model.controllers.at("c1").quarantined = 1;
  model.hardware.at("bus").taken = true;
  model.hardware.at("bus").live.clear();
  const auto selection = select_startup(model, {"c2"}, false, std::nullopt);
  EXPECT_EQ(selection.activate, Names{});
  EXPECT_EQ(
    selection.skipped,
    Skipped{"'c2' ('c2' claims 'j2/velocity' of hardware 'bus', which waits for the quarantined "
            "controller 'c1')"});
}

TEST(Startup, an_unknown_controller_is_kept_for_the_switch_to_reject)
{
  EXPECT_EQ(select_startup(rig(), {"unknown"}, false, std::nullopt).activate, Names{"unknown"});
}

}  // namespace
}  // namespace hector_controller_manager::core
