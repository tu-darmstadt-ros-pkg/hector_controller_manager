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

#include "hector_controller_manager/hardware_preparation.hpp"
#include "urdf.hpp"

namespace hector_controller_manager
{
namespace
{
/// Two components, `arm` with the limited joint j1 and `base` with j2. `attributes` go on `arm`.
std::string urdf(const std::string & attributes = "", const std::string & properties = "")
{
  return testing::urdf_of(
    {{"arm", {{"j1", {"velocity"}, {"position"}, true}}, attributes, properties},
     {"base", {{"j2", {"velocity"}}}}});
}

joint_limits::JointLimits velocity_limit(double max_velocity)
{
  joint_limits::JointLimits limits;
  limits.has_velocity_limits = true;
  limits.max_velocity = max_velocity;
  return limits;
}

void expect_error(const HardwarePreparation & prepared, const std::string & part)
{
  EXPECT_TRUE(prepared.hardware.empty());
  ASSERT_TRUE(prepared.error);
  EXPECT_NE(prepared.error->find(part), std::string::npos) << *prepared.error;
}

TEST(PrepareHardware, every_component_reads_and_writes_at_the_update_rate_and_is_limited)
{
  SupervisorConfig config;
  config.update_rate = 250;
  const auto prepared = prepare_hardware(urdf(), config, {});
  ASSERT_EQ(prepared.error, std::nullopt);
  ASSERT_EQ(prepared.hardware.size(), 2u);
  for (const auto & hardware : prepared.hardware)
  {
    EXPECT_EQ(hardware.info.rw_rate, 250u) << hardware.info.name;
    EXPECT_TRUE(hardware.limit_commands) << hardware.info.name;
  }
}

TEST(PrepareHardware, an_invalid_description_is_rejected)
{
  expect_error(prepare_hardware("<robot", SupervisorConfig{}, {}), "Invalid robot description");
}

TEST(PrepareHardware, an_rw_rate_other_than_the_update_rate_is_rejected)
{
  SupervisorConfig config;
  config.update_rate = 100;
  EXPECT_EQ(prepare_hardware(urdf("rw_rate=\"100\""), config, {}).error, std::nullopt);
  expect_error(prepare_hardware(urdf("rw_rate=\"50\""), config, {}), "sets rw_rate 50");
}

TEST(PrepareHardware, an_asynchronous_component_must_run_synchronized)
{
  const SupervisorConfig config;
  EXPECT_EQ(
    prepare_hardware(
      urdf("is_async=\"true\"", "<async scheduling_policy=\"synchronized\"/>"), config, {})
      .error,
    std::nullopt);
  expect_error(
    prepare_hardware(
      urdf("is_async=\"true\"", "<async scheduling_policy=\"detached\"/>"), config, {}),
    "sets the scheduling_policy 'detached'");
}

TEST(PrepareHardware, unknown_hardware_in_the_initial_states_or_the_exclusions_is_rejected)
{
  SupervisorConfig config;
  EXPECT_EQ(
    prepare_hardware(urdf(), config, {{"arm", core::Lifecycle::Inactive}}).error, std::nullopt);
  expect_error(
    prepare_hardware(urdf(), config, {{"none", core::Lifecycle::Inactive}}),
    "initial state names unknown hardware 'none'");
  config.command_limits_excluded = {"none"};
  expect_error(prepare_hardware(urdf(), config, {}), "exclude unknown hardware 'none'");
}

TEST(PrepareHardware, limit_parameters_tighten_the_limits_of_a_limited_joint)
{
  SupervisorConfig config;
  config.limit_parameters = {{"j1", velocity_limit(1.5)}};
  const auto prepared = prepare_hardware(urdf(), config, {});
  ASSERT_EQ(prepared.error, std::nullopt);
  EXPECT_DOUBLE_EQ(prepared.hardware.at(0).info.limits.at("j1").max_velocity, 1.5);
  config.limit_parameters.at("j1").has_jerk_limits = true;
  expect_error(prepare_hardware(urdf(), config, {}), "joint_limits.j1 sets a deceleration or jerk");
}

TEST(PrepareHardware, limit_parameters_must_name_a_limited_joint_while_limits_are_enforced)
{
  SupervisorConfig config;
  config.command_limits_excluded = {"arm"};
  config.limit_parameters = {{"j1", velocity_limit(1.5)}};
  expect_error(
    prepare_hardware(urdf(), config, {}), "joint_limits.j1 names no joint whose commands");
  config.enforce_command_limits = false;
  const auto prepared = prepare_hardware(urdf(), config, {});
  ASSERT_EQ(prepared.error, std::nullopt) << "a shared parameter file may keep its limits";
  EXPECT_DOUBLE_EQ(prepared.hardware.at(0).info.limits.at("j1").max_velocity, 2.0);
  EXPECT_FALSE(prepared.hardware.at(0).limit_commands);
}

TEST(PrepareHardware, an_excluded_component_is_not_limited)
{
  SupervisorConfig config;
  config.command_limits_excluded = {"base"};
  const auto prepared = prepare_hardware(urdf(), config, {});
  ASSERT_EQ(prepared.error, std::nullopt);
  EXPECT_TRUE(prepared.hardware.at(0).limit_commands);
  EXPECT_FALSE(prepared.hardware.at(1).limit_commands);
}

}  // namespace
}  // namespace hector_controller_manager
