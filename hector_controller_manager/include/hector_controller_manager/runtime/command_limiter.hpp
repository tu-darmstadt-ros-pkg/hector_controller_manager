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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__COMMAND_LIMITER_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__COMMAND_LIMITER_HPP_

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "joint_limits/joint_limits.hpp"

namespace hector_controller_manager::runtime
{
/// The claim word of a command interface. Bit 0 tells whether a controller claims it, the bits
/// above count claim changes, so a reader sees every change of the claimer.
constexpr uint32_t CLAIMED = 1;

/// Clamps the position, velocity and effort commands of one component to its joint limits.
///
/// Built off RT. `enforce` limits the commands of one write in a buffer, on the thread that copies
/// them into the plugin, and neither allocates, blocks nor throws. A position command moves at most
/// the velocity limit per period from the previous limited command, and its speed grows by at most
/// the acceleration limit per period. The period is capped, so an overrun allows no longer step.
/// Slowing down is never limited, so a goal is not overshot. A position goal is clamped to the
/// joint's range, a joint already outside it may hold there and only move back. A velocity command
/// is clamped the same way, and toward a position limit so the next period ends at the limit at
/// most. With an acceleration limit it slows earlier, so the joint can brake there. An async owner
/// reads right after its write, so the position it read lags the last command for the whole time
/// since that write, also an overrun, which the limiter adds only toward a limit. An effort command
/// is clamped to the effort limit. A NaN command passes unchanged, the copy makes the command of an
/// unclaimed interface NaN. The previous command restarts from the actual state after a NaN and
/// whenever the claim of any command interface of the joint changes, so also when a torque off
/// interface is released.
class CommandLimiter
{
public:
  struct Result
  {
    /// A command differed noticeably from its limited value.
    bool clamped = false;
    /// The actual position of a joint is noticeably outside its position limits.
    bool out_of_range = false;
    /// False if a state to start from or the position of a velocity command was busy, then the
    /// commands of this cycle must not be written.
    bool complete = true;
    /// Command interface index of the last clamped command.
    uint32_t last_clamped = 0;
  };

  /// Limits the commands of every joint that has limits enabled, a cycle moves them by at most
  /// `max_period`. Throws if a limit it uses is invalid or a limited position command or a velocity
  /// command with position limits has no position state.
  CommandLimiter(
    const hardware_interface::HardwareInfo & info,
    const std::vector<hardware_interface::CommandInterface::SharedPtr> & commands,
    const std::vector<hardware_interface::StateInterface::ConstSharedPtr> & states,
    double max_period);

  size_t size() const { return entries_.size(); }

  /// `claims` holds the claim word and `commands` the command of every command interface by
  /// index, the commands are limited in place. gz writes on every physics step but updates at the
  /// control rate, so only the first enforce of an `update` moves the joints, by the time since the
  /// last one that did. A zero `period` moves nothing. The next enforce undoes this one unless
  /// `written` followed, which the writing thread calls, ordered with the copying one.
  Result enforce(
    uint64_t update, int64_t time_ns, double period, const std::atomic<uint32_t> * claims,
    double * commands) noexcept;
  /// The commands of the last enforce were written.
  void written() noexcept { unwritten_ = false; }

private:
  enum class Kind : uint8_t
  {
    Position,
    Velocity,
    Effort,
  };
  enum class Restart : uint8_t
  {
    Started,
    Unknown,
    Busy,
  };
  /// The last limited command and, for a position, the velocity of its last step.
  struct Motion
  {
    bool primed = false;
    double previous = 0.0;
    double speed = 0.0;
  };
  struct Entry
  {
    Kind kind;
    uint32_t interface;
    joint_limits::JointLimits limits;
    const hardware_interface::StateInterface * position;
    const hardware_interface::StateInterface * velocity;
    /// Command interface indices of the joint and the claim words seen last.
    std::vector<uint32_t> joint_interfaces;
    std::vector<uint32_t> seen;
    Motion motion;
    /// Before the last enforce, and whether it saw a claim change, which an undo keeps.
    Motion saved;
    bool claims_changed = false;
  };

  static Restart restart(Entry & entry) noexcept;
  /// `position` is the actual one of a velocity command with position limits, which still moves
  /// by the previous command for `lag` seconds.
  static double limit(
    Entry & entry, double goal, double period, double position, double lag) noexcept;

  /// The update and time of the last enforce that moved the joints.
  struct Step
  {
    uint64_t update = 0;
    int64_t time_ns = 0;
    bool moved = false;
  };

  double max_period_;
  bool async_;
  std::vector<Entry> entries_;
  Step step_;
  /// Before the last enforce, which the next one restores while `unwritten_`.
  Step saved_step_;
  /// RT's enforce and an async owner's `written` are ordered by the Cycle word, a write through a
  /// block by the IoGate.
  bool unwritten_ = false;
};

/// Applies limits from the joint_limits parameters to the URDF limits of a joint. They may tighten
/// the position, velocity and effort limits and set the acceleration limit. Throws for the
/// deceleration and jerk limits, which the limiter does not enforce.
void apply_limit_parameters(
  const std::string & joint, const joint_limits::JointLimits & parameters,
  joint_limits::JointLimits & limits);

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__COMMAND_LIMITER_HPP_
