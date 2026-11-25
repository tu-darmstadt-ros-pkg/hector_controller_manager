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

#include "fixtures.hpp"
#include "hector_controller_manager/core/chain_rules.hpp"
#include "hector_controller_manager/core/names.hpp"

namespace hector_controller_manager::core
{
namespace
{
using testing::before;
using testing::rig;
using testing::startup_active;

ChainGraph graph_of(const Model & model, const Names & names)
{
  std::map<std::string, Claims> claims;
  for (const auto & name : names)
  {
    const auto & interfaces = model.controllers.at(name).interfaces;
    claims[name] = {interfaces.command, interfaces.state};
  }
  return ChainGraph(InterfaceIndex(model), claims);
}

TEST(ChainRules, failing_writer_stops_users_and_written_providers)
{
  const Model model = rig();
  const auto graph = graph_of(model, merged(startup_active(), {"whole_body", "teleop"}));
  EXPECT_EQ(
    stop_closure(graph, {"arm_ik"}), (Names{
                                       "arm_ik", "arm_vel", "wrist", "whole_body", "teleop",
                                       "diff_drive", "flipper_pos", "base_vel", "odom_pub"}));
}

TEST(ChainRules, failing_reader_stops_only_itself)
{
  const Model model = rig();
  const auto graph = graph_of(model, startup_active());
  EXPECT_EQ(stop_closure(graph, {"odom_pub"}), (Names{"odom_pub"}));
  EXPECT_EQ(stop_closure(graph, {"jsb"}), (Names{"jsb"}));
}

TEST(ChainRules, failing_provider_stops_its_readers_and_writers)
{
  const Model model = rig();
  const auto graph = graph_of(model, startup_active());
  EXPECT_EQ(stop_closure(graph, {"diff_drive"}), (Names{"diff_drive", "odom_pub", "base_vel"}));
  EXPECT_EQ(stop_closure(graph, {"arm_vel"}), (Names{"arm_vel", "arm_ik", "wrist"}));
}

TEST(ChainRules, claims_hardware_counts_command_and_state_interfaces_of_that_component)
{
  Model model = rig();
  model.controllers.at("diff_drive").claims = {{"base_vel/wl", "base_vel/wr"}, {"imu/yaw"}};
  const InterfaceIndex index(model);
  EXPECT_TRUE(claims_hardware(index, {{"a1/velocity"}, {}}, "arm"));
  EXPECT_TRUE(claims_hardware(index, {{}, {"imu/yaw"}}, "imu"));
  EXPECT_FALSE(claims_hardware(index, {{"a1/velocity"}, {}}, "base"));
  EXPECT_FALSE(claims_hardware(index, {{"arm_vel/a1"}, {"diff_drive/odom"}}, "arm"));
  EXPECT_EQ(claimers_of_hardware(model, index, "imu"), (Names{"diff_drive"}));
  EXPECT_EQ(claimers_of_hardware(model, index, "base"), (Names{}));
}

TEST(ChainRules, commands_hardware_counts_only_command_interfaces_of_that_component)
{
  const Model model = rig();
  const InterfaceIndex index(model);
  EXPECT_TRUE(commands_hardware(index, {{"a1/velocity"}, {}}, "arm"));
  EXPECT_FALSE(commands_hardware(index, {{}, {"a1/position"}}, "arm"));
  EXPECT_FALSE(commands_hardware(index, {{"a1/velocity"}, {}}, "base"));
  EXPECT_FALSE(commands_hardware(index, {{"arm_vel/a1"}, {}}, "arm"));
}

TEST(ChainRules, an_acyclic_chain_passes_the_cycle_check)
{
  const Model model = rig();
  EXPECT_NO_THROW(graph_of(model, merged(startup_active(), {"whole_body"})).check_acyclic());
}

TEST(ChainRules, update_order_runs_writers_before_providers_before_readers)
{
  const Model model = rig();
  const auto order = graph_of(model, merged(startup_active(), {"whole_body"})).update_order();
  const auto position = [&](const std::string & name) -> std::optional<size_t>
  {
    const auto it = std::find(order.begin(), order.end(), name);
    return it == order.end() ? std::nullopt : std::optional<size_t>(it - order.begin());
  };
  EXPECT_TRUE(before(position("whole_body"), position("arm_ik")));
  EXPECT_TRUE(before(position("arm_ik"), position("arm_vel")));
  EXPECT_TRUE(before(position("whole_body"), position("diff_drive")));
  EXPECT_TRUE(before(position("diff_drive"), position("base_vel")));
  EXPECT_TRUE(before(position("diff_drive"), position("odom_pub")));
}

TEST(ChainRules, a_cycle_is_rejected)
{
  Model model;
  testing::add_controller(model, {"a", true, {"b/r"}, {}, {"r"}, {}});
  testing::add_controller(model, {"b", true, {"a/r"}, {}, {"r"}, {}});
  EXPECT_THROW(graph_of(model, {"a", "b"}).check_acyclic(), std::invalid_argument);
}

}  // namespace
}  // namespace hector_controller_manager::core
