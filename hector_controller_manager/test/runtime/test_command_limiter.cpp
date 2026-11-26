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

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "allocation_counter.hpp"
#include "gate.hpp"
#include "hector_controller_manager/runtime/command_limiter.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using joint_limits::JointLimits;

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double PERIOD = 0.01;
constexpr double NO_CAP = std::numeric_limits<double>::infinity();

JointLimits limits(double lower, double upper, double velocity, double acceleration = NaN)
{
  JointLimits result;
  result.has_position_limits = std::isfinite(lower);
  result.min_position = lower;
  result.max_position = upper;
  result.has_velocity_limits = std::isfinite(velocity);
  result.max_velocity = velocity;
  result.has_acceleration_limits = std::isfinite(acceleration);
  result.max_acceleration = acceleration;
  return result;
}

/// One component with its handles, joints get a position and a velocity state.
class Component
{
public:
  // HardwareInfo leaves it uninitialized
  Component() { info_.is_async = false; }

  void add(
    const std::string & joint, const JointLimits & joint_limits,
    const std::vector<std::string> & commands, bool limits_enabled = true)
  {
    hardware_interface::ComponentInfo component;
    component.name = joint;
    component.type = "joint";
    component.enable_limits = limits_enabled;
    for (const auto & name : commands)
    {
      hardware_interface::InterfaceInfo interface{};
      interface.name = name;
      interface.enable_limits = true;
      component.command_interfaces.push_back(interface);
      commands_.push_back(std::make_shared<CommandInterface>(joint, name));
    }
    for (const char * name : {"position", "velocity"})
    {
      states_.push_back(std::make_shared<StateInterface>(joint, name));
      state(joint + "/" + name, 0.0);
    }
    info_.joints.push_back(component);
    info_.limits[joint] = joint_limits;
  }

  /// Every command interface is claimed by activation 1.
  CommandLimiter limiter(double max_period = NO_CAP)
  {
    claims_ = std::vector<std::atomic<uint32_t>>(commands_.size());
    values_.assign(commands_.size(), NaN);
    claim(1);
    return CommandLimiter(info_, commands_, {states_.begin(), states_.end()}, max_period);
  }

  StateInterface & state(const std::string & name)
  {
    for (const auto & state : states_)
    {
      if (state->get_name() == name)
      {
        return *state;
      }
    }
    throw std::runtime_error("no state " + name);
  }
  void state(const std::string & name, double value) { ASSERT_TRUE(state(name).set_value(value)); }
  CommandInterface & command(const std::string & name)
  {
    for (const auto & command : commands_)
    {
      if (command->get_name() == name)
      {
        return *command;
      }
    }
    throw std::runtime_error("no command " + name);
  }
  /// Commands `goal` in a new update `period` after the last, limits it and returns what the
  /// hardware would see. Without `write` it never gets there.
  double limit(
    CommandLimiter & limiter, CommandInterface & handle, double goal, double period = PERIOD,
    bool write = true)
  {
    double & value = values_.at(index(handle));
    value = goal;
    result = enforce(limiter, period);
    if (write)
    {
      limiter.written();
    }
    return value;
  }
  double limit(
    CommandLimiter & limiter, const std::string & name, double goal, double period = PERIOD,
    bool write = true)
  {
    return limit(limiter, command(name), goal, period, write);
  }
  CommandLimiter::Result enforce(CommandLimiter & limiter, double period = PERIOD)
  {
    time_ns_ += static_cast<int64_t>(period * 1e9);
    return limiter.enforce(++update_, time_ns_, period, claims_.data(), values_.data());
  }
  /// The claim word of an activation, 0 is unclaimed.
  void claim(uint32_t interface, uint32_t activation)
  {
    claims_[interface] = activation == 0 ? 0 : (activation << 1) | CLAIMED;
  }
  void claim(uint32_t activation)
  {
    for (uint32_t i = 0; i < claims_.size(); ++i)
    {
      claim(i, activation);
    }
  }
  const std::atomic<uint32_t> * claims() const { return claims_.data(); }
  /// The commands of every interface by index, as the hardware would see them.
  double * values() { return values_.data(); }
  size_t index(const CommandInterface & handle) const
  {
    for (size_t i = 0; i < commands_.size(); ++i)
    {
      if (commands_[i].get() == &handle)
      {
        return i;
      }
    }
    throw std::runtime_error("no command " + handle.get_name());
  }

  hardware_interface::HardwareInfo info_;
  CommandLimiter::Result result;

private:
  std::vector<CommandInterface::SharedPtr> commands_;
  std::vector<StateInterface::SharedPtr> states_;
  std::vector<std::atomic<uint32_t>> claims_;
  std::vector<double> values_;
  uint64_t update_ = 0;
  int64_t time_ns_ = 0;
};

TEST(CommandLimiter, a_position_jump_moves_at_most_the_velocity_limit_per_period)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position"});
  auto limiter = component.limiter();
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.01);
  EXPECT_TRUE(component.result.clamped);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.02);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 0.025), 0.025);
  EXPECT_FALSE(component.result.clamped);
}

TEST(CommandLimiter, the_speed_of_a_position_command_grows_at_most_by_the_acceleration_limit)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0, 2.0), {"position"});
  auto limiter = component.limiter();
  // From rest the speed may reach 0.02 in the first period and 0.04 in the second
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.0002);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.0006);
  // Slowing down is not limited, so the goal is reached without overshoot
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 0.0007), 0.0007);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 0.0007), 0.0007);
}

TEST(CommandLimiter, the_acceleration_starts_from_the_commanded_and_not_the_actual_speed)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0, 2.0), {"position"});
  auto limiter = component.limiter();
  component.limit(limiter, "j1/position", 5.0);
  component.state("j1/velocity", 1.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.0006);
}

TEST(CommandLimiter, a_revolute_goal_is_clamped_to_its_range)
{
  Component component;
  component.add("j1", limits(-1, 1, NaN), {"position"});
  auto limiter = component.limiter();
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 1.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", -5.0), -1.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 0.5), 0.5);
}

TEST(CommandLimiter, a_continuous_joint_is_not_clamped_to_a_range)
{
  Component component;
  component.add("j1", limits(NaN, NaN, 1000.0), {"position"});
  auto limiter = component.limiter();
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 7.0), 7.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", -3.0), -3.0);
}

TEST(CommandLimiter, a_joint_outside_its_range_holds_or_moves_back_at_the_limited_speed)
{
  Component component;
  component.add("j1", limits(-1, 1, 1.0), {"position"});
  component.state("j1/position", 1.05);
  auto limiter = component.limiter();
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 2.0), 1.05);
  EXPECT_TRUE(component.result.out_of_range);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 0.0), 1.04);
  component.state("j1/position", 0.9);
  component.limit(limiter, "j1/position", 0.0);
  EXPECT_FALSE(component.result.out_of_range);

  Component below;
  below.add("j1", limits(-1, 1, 1.0), {"position"});
  below.state("j1/position", -1.05);
  auto lower = below.limiter();
  EXPECT_DOUBLE_EQ(below.limit(lower, "j1/position", -2.0), -1.05);
  EXPECT_DOUBLE_EQ(below.limit(lower, "j1/position", 0.0), -1.04);
}

TEST(CommandLimiter, nan_passes_and_the_next_command_starts_from_the_actual_position)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position"});
  auto limiter = component.limiter();
  component.limit(limiter, "j1/position", 5.0);
  EXPECT_TRUE(std::isnan(component.limit(limiter, "j1/position", NaN)));
  EXPECT_FALSE(component.result.clamped);
  component.state("j1/position", 0.5);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.51);
}

TEST(CommandLimiter, a_new_claim_starts_from_the_actual_position)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position"});
  auto limiter = component.limiter();
  component.claim(1);
  component.limit(limiter, "j1/position", 5.0);
  component.state("j1/position", 0.5);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.02) << "the same claim goes on";
  component.claim(2);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", -5.0), 0.49);
}

// Releasing a torque off interface hands the joint back to its position command
TEST(CommandLimiter, a_new_claim_of_another_interface_of_the_joint_starts_from_the_actual_position)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position", "torque_off"});
  auto limiter = component.limiter();
  component.claim(0, 1);
  component.claim(1, 1);
  component.limit(limiter, "j1/position", 5.0);
  component.state("j1/position", 0.5);
  component.claim(1, 0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.51);
}

TEST(CommandLimiter, a_joint_without_a_known_position_holds)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position"});
  component.state("j1/position", NaN);
  auto limiter = component.limiter();
  EXPECT_TRUE(std::isnan(component.limit(limiter, "j1/position", 5.0)));
  component.state("j1/position", 0.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.01);
}

TEST(CommandLimiter, a_velocity_command_is_clamped_and_its_acceleration_limited)
{
  Component component;
  component.add("j1", limits(NaN, NaN, 2.0, 10.0), {"velocity"});
  auto limiter = component.limiter();
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 5.0), 0.1);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 5.0), 0.2);
  // Stopping is not limited, reversing is
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", -5.0), -0.1);
  for (int i = 0; i < 100; ++i)
  {
    component.limit(limiter, "j1/velocity", 5.0);
  }
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 5.0), 2.0);
}

// Without an acceleration limit it slows only in the period that reaches the limit
TEST(CommandLimiter, a_velocity_command_toward_a_position_limit_ends_the_next_period_there)
{
  Component component;
  component.add("j1", limits(-1.0, 1.0, NaN), {"velocity"});
  auto limiter = component.limiter();
  component.state("j1/position", 0.9);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 5.0), 5.0);
  component.state("j1/position", 0.99);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 5.0), 1.0);
  EXPECT_TRUE(component.result.clamped);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 5.0, 0.0), 1.0)
    << "a write without time";
  component.state("j1/position", 1.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 5.0), 0.0);
  EXPECT_TRUE(component.result.clamped);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", -5.0), -5.0) << "away from it";
  component.state("j1/position", -1.2);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", -5.0), 0.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 0.5), 0.5) << "back into the range";
}

// It moves the joint by the limited commands, which must stop it at the limit without braking
// harder than the acceleration limit
TEST(CommandLimiter, a_velocity_command_brakes_before_a_position_limit_with_the_acceleration_limit)
{
  constexpr double ACCELERATION = 10.0;
  Component component;
  component.add("j1", limits(-1.0, 1.0, 2.0, ACCELERATION), {"velocity"});
  auto limiter = component.limiter();
  double position = 0.5;
  double speed = 2.0;
  component.state("j1/velocity", speed);
  double highest = position;
  for (int i = 0; i < 300; ++i)
  {
    component.state("j1/position", position);
    const double limited = component.limit(limiter, "j1/velocity", 2.0);
    EXPECT_GE(limited, speed - ACCELERATION * PERIOD * (1 + 1e-9)) << "at " << position;
    speed = limited;
    position += speed * PERIOD;
    highest = std::max(highest, position);
  }
  EXPECT_LE(highest, 1.0 + 1e-9);
  EXPECT_GT(highest, 0.999) << "precondition: it reached the limit";
  EXPECT_NEAR(speed, 0.0, 1e-6);
}

// A stalled or e-stopped drive does not follow an async command away from the limit
TEST(CommandLimiter, an_async_joint_at_its_limit_after_an_unfollowed_command_does_not_pass_it)
{
  Component component;
  component.info_.is_async = true;
  component.add("j1", limits(-1.0, 1.0, 2.0), {"velocity"});
  component.state("j1/position", 1.0);
  auto limiter = component.limiter();
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", -1.0), -1.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 1.0), 0.0);
  component.state("j1/position", 0.98);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 1.0), 1.0);
  component.state("j1/position", 0.99);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 1.0), 0.0) << "it still moves there";
}

// The capped period bounds the step, the joint moved by the last command for the whole overrun
TEST(CommandLimiter, an_async_overrun_does_not_command_past_the_position_limit)
{
  Component component;
  component.info_.is_async = true;
  component.add("j1", limits(-1.0, 1.0, 2.0), {"velocity"});
  component.state("j1/position", 0.97);
  auto limiter = component.limiter(PERIOD);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 2.0), 2.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 2.0, 2 * PERIOD), 0.0);
}

TEST(CommandLimiter, a_velocity_joint_without_a_known_position_holds_at_its_position_limits)
{
  Component component;
  component.add("j1", limits(-1.0, 1.0, 2.0), {"velocity"});
  component.state("j1/position", NaN);
  auto limiter = component.limiter();
  EXPECT_TRUE(std::isnan(component.limit(limiter, "j1/velocity", 0.5)));
  component.state("j1/position", 0.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 0.5), 0.5);
  component.state("j1/position", std::numeric_limits<double>::infinity());
  EXPECT_TRUE(std::isnan(component.limit(limiter, "j1/velocity", 0.5))) << "after it was known";
  component.state("j1/position", 0.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 0.5), 0.5);
}

TEST(CommandLimiter, a_busy_position_skips_the_write_of_a_velocity_command)
{
  Component component;
  component.add("j1", limits(-1.0, 1.0, 2.0), {"velocity"});
  auto limiter = component.limiter();
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/velocity", 0.1), 0.1);
  auto & command = component.command("j1/velocity");
  component.values()[component.index(command)] = 0.1;
  {
    const testing::HeldLock busy(component.state("j1/position").get_mutex());
    component.result = component.enforce(limiter);
  }
  EXPECT_FALSE(component.result.complete) << "the position toward the limit is busy";
}

TEST(CommandLimiter, an_effort_command_is_clamped_to_the_effort_limit)
{
  Component component;
  auto joint = limits(NaN, NaN, NaN);
  joint.has_effort_limits = true;
  joint.max_effort = 3.0;
  component.add("j1", joint, {"effort"});
  auto limiter = component.limiter();
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/effort", 5.0), 3.0);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/effort", -5.0), -3.0);
}

TEST(CommandLimiter, a_joint_with_disabled_limits_passes_unchanged)
{
  Component component;
  component.add("j1", limits(-1, 1, 1.0), {"position"}, false);
  auto limiter = component.limiter();
  EXPECT_EQ(limiter.size(), 0u);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 5.0);
}

TEST(CommandLimiter, invalid_limits_and_a_missing_position_state_are_rejected)
{
  Component zero;
  zero.add("j1", limits(-1, 1, 0.0), {"position"});
  EXPECT_THROW(zero.limiter(), std::runtime_error);

  Component stateless;
  stateless.add("j1", limits(-1, 1, 1.0), {"position"});
  std::vector<CommandInterface::SharedPtr> commands{
    std::make_shared<CommandInterface>("j1", "position")};
  EXPECT_THROW(CommandLimiter(stateless.info_, commands, {}, NO_CAP), std::runtime_error);

  Component stateless_velocity;
  stateless_velocity.add("j1", limits(-1, 1, NaN), {"velocity"});
  try
  {
    // NOLINTNEXTLINE(bugprone-unused-raii) only the constructor's throw matters
    CommandLimiter(
      stateless_velocity.info_, {std::make_shared<CommandInterface>("j1", "velocity")}, {}, NO_CAP);
    ADD_FAILURE() << "a velocity command with position limits but no position state was accepted";
  }
  catch (const std::runtime_error & error)
  {
    EXPECT_NE(std::string(error.what()).find("'j1'"), std::string::npos) << error.what();
  }

  auto no_effort = limits(-1, 1, 1.0);
  no_effort.has_effort_limits = true;
  no_effort.max_effort = 0.0;
  Component effort;
  effort.add("j1", no_effort, {"effort"});
  EXPECT_THROW(effort.limiter(), std::runtime_error);
}

// URDFs often say effort="0" for joints that are not effort controlled, as upstream they start
TEST(CommandLimiter, an_unused_zero_effort_limit_does_not_stop_a_position_joint)
{
  auto joint = limits(-1, 1, 1.0);
  joint.has_effort_limits = true;
  joint.max_effort = 0.0;
  Component component;
  component.add("j1", joint, {"position"});
  auto limiter = component.limiter();
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.01);
}

TEST(CommandLimiter, a_period_without_time_holds_the_position)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position"});
  auto limiter = component.limiter();
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0, 0.0), 0.0);
}

TEST(CommandLimiter, an_overrun_moves_no_further_than_one_regular_period)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position"});
  auto limiter = component.limiter(PERIOD);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0, 0.5), 0.01);
}

// The copy gives an unclaimed interface NaN, so a joint that nothing commands is not reported
TEST(CommandLimiter, an_unclaimed_interface_is_no_command)
{
  Component component;
  component.add("j1", limits(-1, 1, 1.0), {"position"});
  auto limiter = component.limiter();
  component.limit(limiter, "j1/position", 0.5);
  component.claim(0);
  component.state("j1/position", 1.05);
  EXPECT_TRUE(std::isnan(component.limit(limiter, "j1/position", NaN)));
  EXPECT_FALSE(component.result.out_of_range);
  EXPECT_FALSE(component.result.clamped);
  component.state("j1/position", 0.5);
  component.claim(2);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.51);
}

TEST(CommandLimiter, a_busy_state_skips_the_write_and_leaves_the_command)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position"});
  auto limiter = component.limiter();
  auto & command = component.command("j1/position");
  component.values()[component.index(command)] = 5.0;
  {
    const testing::HeldLock busy(component.state("j1/position").get_mutex());
    component.result = component.enforce(limiter);
  }
  EXPECT_FALSE(component.result.complete) << "the position to start from is busy";
  EXPECT_DOUBLE_EQ(component.values()[component.index(command)], 5.0)
    << "a busy state does not hold the joint";
  EXPECT_DOUBLE_EQ(component.limit(limiter, command, 5.0), 0.01);
  EXPECT_TRUE(component.result.complete);
}

// The commands of a skipped write never reached the hardware, the next one starts before them.
// The cap keeps the time since the last written step to one period.
TEST(CommandLimiter, an_unwritten_cycle_adds_no_step)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position"});
  auto limiter = component.limiter(PERIOD);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.01);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0, PERIOD, false), 0.02);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.02);
}

// gz writes on every physics step but updates at the control rate
TEST(CommandLimiter, only_the_first_enforce_of_an_update_moves_the_joints)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position"});
  auto limiter = component.limiter();
  double & command = component.values()[0];
  for (const int64_t time_ns : {10'000'000, 15'000'000})
  {
    command = 5.0;
    limiter.enforce(1, time_ns, PERIOD, component.claims(), component.values());
    limiter.written();
    EXPECT_DOUBLE_EQ(command, 0.01);
  }
}

TEST(CommandLimiter, an_unwritten_first_enforce_leaves_the_move_to_the_next_one_of_its_update)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position"});
  auto limiter = component.limiter(PERIOD);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.01);
  double & command = component.values()[0];
  for (const int64_t time_ns : {20'000'000, 21'000'000})
  {
    command = 5.0;
    limiter.enforce(2, time_ns, PERIOD, component.claims(), component.values());
    EXPECT_DOUBLE_EQ(command, 0.02);
  }
}

TEST(CommandLimiter, a_claim_change_in_an_unwritten_cycle_still_restarts)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.0), {"position"});
  auto limiter = component.limiter(PERIOD);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.01);
  component.state("j1/position", 0.5);
  component.claim(2);
  component.limit(limiter, "j1/position", 5.0, PERIOD, false);
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.51);
}

TEST(CommandLimiter, a_restart_continues_with_the_actual_speed)
{
  Component component;
  component.add("j1", limits(-10, 10, 1.5, 2.0), {"position"});
  component.state("j1/velocity", 1.0);
  auto limiter = component.limiter();
  EXPECT_DOUBLE_EQ(component.limit(limiter, "j1/position", 5.0), 0.0102);
}

TEST(CommandLimiter, sensor_noise_at_a_limit_is_not_out_of_range)
{
  Component component;
  component.add("j1", limits(0, 1, 1.0), {"position"});
  component.state("j1/position", -0.001);
  auto limiter = component.limiter();
  component.limit(limiter, "j1/position", 0.0);
  EXPECT_FALSE(component.result.out_of_range);
  component.state("j1/position", -0.01);
  component.limit(limiter, "j1/position", 0.0);
  EXPECT_TRUE(component.result.out_of_range);
}

TEST(CommandLimiter, a_velocity_joint_past_its_position_limit_is_out_of_range)
{
  Component component;
  component.add("j1", limits(0, 1, 1.0), {"velocity"});
  component.state("j1/position", 1.001);
  auto limiter = component.limiter();
  component.limit(limiter, "j1/velocity", -0.1);
  EXPECT_FALSE(component.result.out_of_range);
  component.state("j1/position", 1.01);
  component.limit(limiter, "j1/velocity", -0.1);
  EXPECT_TRUE(component.result.out_of_range);
}

TEST(CommandLimiter, parameters_tighten_the_urdf_limits_and_add_an_acceleration_limit)
{
  auto urdf = limits(-1, 1, 2.0);
  JointLimits parameters;
  parameters.has_velocity_limits = true;
  parameters.max_velocity = 3.0;
  parameters.has_acceleration_limits = true;
  parameters.max_acceleration = 4.0;
  apply_limit_parameters("j1", parameters, urdf);
  EXPECT_DOUBLE_EQ(urdf.max_velocity, 2.0);
  EXPECT_TRUE(urdf.has_acceleration_limits);
  EXPECT_DOUBLE_EQ(urdf.max_acceleration, 4.0);
  parameters.has_position_limits = true;
  parameters.min_position = -0.5;
  parameters.max_position = 2.0;
  apply_limit_parameters("j1", parameters, urdf);
  EXPECT_DOUBLE_EQ(urdf.min_position, -0.5);
  EXPECT_DOUBLE_EQ(urdf.max_position, 1.0);
  parameters.has_deceleration_limits = true;
  EXPECT_THROW(apply_limit_parameters("j1", parameters, urdf), std::runtime_error);
}

// 7 arm joints and 4 flippers with long names, every kind of command, NaN and restarts
TEST(CommandLimiter, enforcing_does_not_allocate)
{
  if (!testing::allocation_counting_available())
  {
    GTEST_SKIP() << "Sanitizers replace operator new";
  }
  Component component;
  for (int i = 1; i <= 7; ++i)
  {
    component.add("arm_joint_number_" + std::to_string(i), limits(-1, 1, 1.5, 5.0), {"position"});
  }
  component.add("wrist_joint", limits(-1, 1, 1.5, 5.0), {"velocity"});
  auto flipper = limits(NaN, NaN, 2.3, 8.0);
  flipper.has_effort_limits = true;
  flipper.max_effort = 2.0;
  for (const char * name : {"front_left", "front_right", "back_left", "back_right"})
  {
    component.add(
      std::string("flipper_") + name + "_joint", flipper, {"position", "velocity", "effort"});
  }
  auto limiter = component.limiter();
  ASSERT_EQ(limiter.size(), 20u);
  auto & arm = component.command("arm_joint_number_3/position");
  auto & arm_state = component.state("arm_joint_number_3/position");
  auto & velocity = component.command("flipper_back_right_joint/velocity");
  auto & wrist = component.command("wrist_joint/velocity");
  auto & wrist_state = component.state("wrist_joint/position");
  auto & effort = component.command("flipper_back_right_joint/effort");
  const uint64_t before = testing::counted_allocations();
  bool clamped = false;
  bool out_of_range = false;
  {
    testing::CountAllocations scope(true);
    for (int cycle = 0; cycle < 1000; ++cycle)
    {
      const double goal = cycle % 100 == 50 ? NaN : (cycle % 7) - 3.0;
      component.claim(static_cast<uint32_t>(cycle / 300));
      (void)arm_state.set_value(cycle % 200 == 0 ? 1.5 : 0.0);
      component.limit(limiter, arm, goal);
      clamped = clamped || component.result.clamped;
      out_of_range = out_of_range || component.result.out_of_range;
      component.limit(limiter, velocity, goal);
      component.limit(limiter, effort, goal);
      (void)wrist_state.set_value(cycle % 3 == 0 ? 1.0 : 0.95);
      component.limit(limiter, wrist, goal);
    }
  }
  EXPECT_TRUE(clamped && out_of_range) << "precondition: every path ran";
  EXPECT_EQ(testing::counted_allocations() - before, 0u);
}

}  // namespace
}  // namespace hector_controller_manager::runtime
