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

#include "hector_controller_manager/runtime/command_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
constexpr double INF = std::numeric_limits<double>::infinity();
/// Only a correction above this counts as clamped, rounding of a command at its limit does not.
constexpr double CLAMP_TOLERANCE = 1e-6;
/// A joint parked at a limit reads past it by sensor noise, a few Dynamixel ticks of 1.5 mrad or
/// millimetres of a gripper, which must not count as out of range.
constexpr double RANGE_TOLERANCE = 0.005;

const hardware_interface::StateInterface * find_state(
  const std::vector<hardware_interface::StateInterface::ConstSharedPtr> & states,
  const std::string & joint, const std::string & interface)
{
  for (const auto & state : states)
  {
    if (
      state->get_prefix_name() == joint && state->get_interface_name() == interface &&
      state->get_data_type() == hardware_interface::HandleDataType::DOUBLE)
    {
      return state.get();
    }
  }
  return nullptr;
}

/// Checks the limits a command of the kind uses, so an unused effort="0" does not stop a robot.
void validate(
  const std::string & joint, const joint_limits::JointLimits & limits, bool position, bool velocity,
  bool effort)
{
  const auto invalid = [&](const std::string & what)
  {
    throw std::runtime_error(
      "Joint '" + joint + "' has " + what +
      ", set a valid limit, disable its limits with <limits enable=\"false\"/> or exclude its "
      "hardware from the command limits");
  };
  if (position && limits.has_position_limits && !(limits.min_position <= limits.max_position))
  {
    invalid("a lower position limit above the upper one");
  }
  if (velocity && limits.has_velocity_limits && !(limits.max_velocity > 0.0))
  {
    invalid("a velocity limit that is not positive");
  }
  if (velocity && limits.has_acceleration_limits && !(limits.max_acceleration > 0.0))
  {
    invalid("an acceleration limit that is not positive");
  }
  if (effort && limits.has_effort_limits && !(limits.max_effort > 0.0))
  {
    invalid("an effort limit that is not positive");
  }
}

/// The band of speeds reachable within one period from `speed`. Slowing down is never limited.
std::pair<double, double> speed_band(
  const joint_limits::JointLimits & limits, double speed, double period)
{
  double low = -INF;
  double high = INF;
  if (limits.has_velocity_limits)
  {
    low = -limits.max_velocity;
    high = limits.max_velocity;
  }
  // A write without time, as gz makes one per physics step, keeps the speed of the write before
  double change = 0.0;
  if (period > 0.0)
  {
    change = limits.has_acceleration_limits ? limits.max_acceleration * period : INF;
  }
  low = std::max(low, std::min(speed, 0.0) - change);
  high = std::min(high, std::max(speed, 0.0) + change);
  return {low, high};
}

/// The fastest speed toward a position limit `distance` away from which the joint still stops at
/// it, braking at the acceleration limit after one more period, or without one the speed that
/// reaches it in one period. Zero at or past the limit.
double stopping_speed(const joint_limits::JointLimits & limits, double distance, double period)
{
  if (!(distance > 0.0))
  {
    return 0.0;
  }
  if (!limits.has_acceleration_limits)
  {
    // The speed band holds a write without time
    return period > 0.0 ? distance / period : INF;
  }
  const double acceleration = limits.max_acceleration;
  return acceleration * (std::sqrt(period * period + 2.0 * distance / acceleration) - period);
}

/// Noticeably outside the position limits, never for NaN.
bool outside(const joint_limits::JointLimits & limits, double position)
{
  return position < limits.min_position - RANGE_TOLERANCE ||
         position > limits.max_position + RANGE_TOLERANCE;
}

/// NaN without a state, nothing while the state is busy.
// NOLINTNEXTLINE(bugprone-exception-escape) find_state keeps only double states, which never throw
std::optional<double> actual(const hardware_interface::StateInterface * state) noexcept
{
  return state ? state->get_optional() : std::numeric_limits<double>::quiet_NaN();
}
}  // namespace

CommandLimiter::CommandLimiter(
  const hardware_interface::HardwareInfo & info,
  const std::vector<hardware_interface::CommandInterface::SharedPtr> & commands,
  const std::vector<hardware_interface::StateInterface::ConstSharedPtr> & states, double max_period)
: max_period_(max_period), async_(info.is_async)
{
  using namespace hardware_interface;
  for (uint32_t index = 0; index < commands.size(); ++index)
  {
    const auto & command = commands[index];
    const std::string & joint_name = command->get_prefix_name();
    const auto joint = std::find_if(
      info.joints.begin(), info.joints.end(), [&](const auto & j) { return j.name == joint_name; });
    const auto limits = info.limits.find(joint_name);
    if (
      joint == info.joints.end() || !joint->enable_limits || limits == info.limits.end() ||
      command->get_data_type() != HandleDataType::DOUBLE)
    {
      continue;
    }
    const std::string & name = command->get_interface_name();
    const auto description = std::find_if(
      joint->command_interfaces.begin(), joint->command_interfaces.end(),
      [&](const auto & interface) { return interface.name == name; });
    if (description != joint->command_interfaces.end() && !description->enable_limits)
    {
      continue;
    }
    const auto & l = limits->second;
    Entry entry{
      Kind::Effort,
      index,
      l,
      find_state(states, joint_name, HW_IF_POSITION),
      find_state(states, joint_name, HW_IF_VELOCITY),
      {},
      {},
      {},
      {}};
    if (
      name == HW_IF_POSITION &&
      (l.has_position_limits || l.has_velocity_limits || l.has_acceleration_limits))
    {
      entry.kind = Kind::Position;
      if (!entry.position)
      {
        throw std::runtime_error(
          "Joint '" + joint_name +
          "' has a limited position command but no position state to limit it from");
      }
    }
    else if (
      name == HW_IF_VELOCITY &&
      (l.has_velocity_limits || l.has_acceleration_limits || l.has_position_limits))
    {
      entry.kind = Kind::Velocity;
      if (l.has_position_limits && !entry.position)
      {
        throw std::runtime_error(
          "Joint '" + joint_name +
          "' has position limits for its velocity command but no position state to stop it at "
          "them");
      }
    }
    else if (name != HW_IF_EFFORT || !l.has_effort_limits)
    {
      continue;
    }
    validate(
      joint_name, l, entry.kind != Kind::Effort, entry.kind != Kind::Effort,
      entry.kind == Kind::Effort);
    for (uint32_t other = 0; other < commands.size(); ++other)
    {
      if (commands[other]->get_prefix_name() == joint_name)
      {
        entry.joint_interfaces.push_back(other);
      }
    }
    entry.seen.resize(entry.joint_interfaces.size());
    entries_.push_back(entry);
  }
}

CommandLimiter::Restart CommandLimiter::restart(Entry & entry) noexcept
{
  const auto velocity = actual(entry.velocity);
  const auto position = actual(entry.position);
  const bool positioned = entry.kind == Kind::Position || entry.limits.has_position_limits;
  if (!velocity || (positioned && !position))
  {
    return Restart::Busy;
  }
  if (positioned && !std::isfinite(*position))
  {
    return Restart::Unknown;
  }
  double speed = std::isfinite(*velocity) ? *velocity : 0.0;
  if (entry.limits.has_velocity_limits)
  {
    speed = std::clamp(speed, -entry.limits.max_velocity, entry.limits.max_velocity);
  }
  if (entry.kind == Kind::Velocity)
  {
    entry.motion.previous = speed;
    return Restart::Started;
  }
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access) only a Position entry gets here, positioned
  entry.motion.previous = *position;
  entry.motion.speed = speed;
  return Restart::Started;
}

double CommandLimiter::limit(
  Entry & entry, double goal, double period, double position, double lag) noexcept
{
  const auto & limits = entry.limits;
  if (entry.kind == Kind::Effort)
  {
    return std::clamp(goal, -limits.max_effort, limits.max_effort);
  }
  if (entry.kind == Kind::Velocity)
  {
    auto [low, high] = speed_band(limits, entry.motion.previous, period);
    // Both bands contain zero, slowing down is never limited
    if (limits.has_position_limits)
    {
      // The drive may not follow the last command, so only a move toward a limit counts
      const double predicted = position + entry.motion.previous * lag;
      const double lowest = std::min(position, predicted);
      const double highest = std::max(position, predicted);
      low = std::max(low, -stopping_speed(limits, lowest - limits.min_position, period));
      high = std::min(high, stopping_speed(limits, limits.max_position - highest, period));
    }
    entry.motion.previous = std::clamp(goal, low, high);
    return entry.motion.previous;
  }
  double lower = entry.motion.previous;
  double upper = entry.motion.previous;
  if (period > 0.0)
  {
    const auto [low, high] = speed_band(limits, entry.motion.speed, period);
    lower += low * period;
    upper += high * period;
  }
  if (limits.has_position_limits)
  {
    // The band contains the previous command, so a joint outside the range holds or moves back
    lower = std::max(lower, std::min(limits.min_position, entry.motion.previous));
    upper = std::min(upper, std::max(limits.max_position, entry.motion.previous));
  }
  const double limited = std::clamp(goal, lower, upper);
  if (period > 0.0)
  {
    entry.motion.speed = (limited - entry.motion.previous) / period;
  }
  entry.motion.previous = limited;
  return limited;
}

CommandLimiter::Result CommandLimiter::enforce(
  uint64_t update, int64_t time_ns, double period, const std::atomic<uint32_t> * claims,
  double * commands) noexcept
{
  if (unwritten_)
  {
    step_ = saved_step_;
    for (auto & entry : entries_)
    {
      entry.motion = entry.saved;
      entry.motion.primed = entry.motion.primed && !entry.claims_changed;
    }
  }
  saved_step_ = step_;
  unwritten_ = true;
  double seconds = 0.0;
  if (period > 0.0 && update != step_.update)
  {
    seconds = step_.moved ? 1e-9 * static_cast<double>(time_ns - step_.time_ns) : period;
    step_ = {update, time_ns, true};
  }
  period = std::min(seconds, max_period_);
  // An async owner reads right after its write, so the joint moved by the last command since, also
  // through an overrun
  const double lag = async_ ? seconds : 0.0;
  Result result;
  for (auto & entry : entries_)
  {
    entry.saved = entry.motion;
    entry.claims_changed = false;
    for (size_t i = 0; i < entry.joint_interfaces.size(); ++i)
    {
      const uint32_t word = claims[entry.joint_interfaces[i]].load(std::memory_order_relaxed);
      entry.claims_changed = entry.claims_changed || word != entry.seen[i];
      entry.seen[i] = word;
    }
    auto & motion = entry.motion;
    motion.primed = motion.primed && !entry.claims_changed;
    double & command = commands[entry.interface];
    if (std::isnan(command))
    {
      motion.primed = false;
      continue;
    }
    if (entry.kind == Kind::Position && entry.limits.has_position_limits)
    {
      result.out_of_range =
        result.out_of_range ||
        outside(entry.limits, actual(entry.position).value_or(entry.limits.min_position));
    }
    double position = std::numeric_limits<double>::quiet_NaN();
    if (entry.kind == Kind::Velocity && entry.limits.has_position_limits)
    {
      const auto state = actual(entry.position);
      if (!state)
      {
        result.complete = false;
        continue;
      }
      position = *state;
      motion.primed = motion.primed && std::isfinite(position);
      result.out_of_range = result.out_of_range || outside(entry.limits, position);
    }
    if (!motion.primed && entry.kind != Kind::Effort)
    {
      const Restart restarted = restart(entry);
      if (restarted == Restart::Busy)
      {
        result.complete = false;
        continue;
      }
      motion.primed = restarted == Restart::Started;
    }
    // Without an actual state to start from the joint holds
    const double limited = motion.primed || entry.kind == Kind::Effort
                             ? limit(entry, command, period, position, lag)
                             : std::numeric_limits<double>::quiet_NaN();
    if (std::isnan(limited) || std::abs(limited - command) > CLAMP_TOLERANCE)
    {
      result.clamped = true;
      result.last_clamped = entry.interface;
    }
    command = limited;
  }
  return result;
}

void apply_limit_parameters(
  const std::string & joint, const joint_limits::JointLimits & parameters,
  joint_limits::JointLimits & limits)
{
  if (parameters.has_deceleration_limits || parameters.has_jerk_limits)
  {
    throw std::runtime_error(
      "joint_limits." + joint +
      " sets a deceleration or jerk limit, the command limits enforce neither and never limit "
      "slowing down");
  }
  const auto tighten = [](bool & has, double & limit, bool has_parameter, double parameter)
  {
    if (has_parameter)
    {
      limit = has ? std::min(limit, parameter) : parameter;
      has = true;
    }
  };
  if (parameters.has_position_limits)
  {
    limits.min_position = limits.has_position_limits
                            ? std::max(limits.min_position, parameters.min_position)
                            : parameters.min_position;
    tighten(limits.has_position_limits, limits.max_position, true, parameters.max_position);
  }
  tighten(
    limits.has_velocity_limits, limits.max_velocity, parameters.has_velocity_limits,
    parameters.max_velocity);
  tighten(
    limits.has_acceleration_limits, limits.max_acceleration, parameters.has_acceleration_limits,
    parameters.max_acceleration);
  tighten(
    limits.has_effort_limits, limits.max_effort, parameters.has_effort_limits,
    parameters.max_effort);
}

}  // namespace hector_controller_manager::runtime
