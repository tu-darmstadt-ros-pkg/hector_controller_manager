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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__SHADOW_HANDLES_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__SHADOW_HANDLES_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hector_controller_manager/runtime/command_limiter.hpp"
#include "hector_controller_manager/runtime/hardware_stats.hpp"
#include "hector_controller_manager/runtime/plan.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace hector_controller_manager::runtime
{
/// The handles controllers use in place of the double interfaces a plugin exports, and the copies
/// between the two.
///
/// The owner sets them up once before RT runs. RT records the claims of each update, relaxed. The
/// copying thread copies the states in and the commands, limited, out, while the plugin is idle.
/// It is RT, or the owner in a write through a block, which the IoGate of the component orders
/// after RT, so the claim words and the update count need no order of their own. A copy takes
/// every handle it writes or none, so a controller never sees joints of two reads and a write
/// never gets joints of two copies. Only a copy that is written advances the command limits.
class ShadowHandles
{
public:
  /// Counts the skipped copies and the clamps of the command limits.
  explicit ShadowHandles(HardwareStats & stats) : stats_(stats) {}

  /// Owner, once, before RT and any copy. Shadows the double interfaces. A sync component passes
  /// the others uncopied, an async one throws for them. `limit` clamps the commands to the joint
  /// limits of `info`.
  void init(
    const hardware_interface::HardwareInfo & info,
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> plugin_states,
    std::vector<hardware_interface::CommandInterface::SharedPtr> plugin_commands, bool limit);

  const std::vector<hardware_interface::StateInterface::ConstSharedPtr> & states() const
  {
    return states_;
  }
  const std::vector<hardware_interface::CommandInterface::SharedPtr> & commands() const
  {
    return commands_;
  }
  std::optional<uint32_t> command_index(const std::string & name) const;
  /// Throws for a name the plugin did not export.
  uint32_t command_at(const std::string & name) const { return command_index_.at(name); }
  /// How many command interfaces the limits clamp.
  size_t limited() const { return limiter_ ? limiter_->size() : 0; }

  /// RT, once per update.
  void rt_note_claims(const std::vector<Plan::Claim> & claims) noexcept;

  /// The copying thread, with the plugin idle. False if a handle was busy, then nothing is copied.
  bool copy_states() noexcept;
  /// Also false if the limits are incomplete. The write skips then.
  bool copy_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period, bool through_block) noexcept;
  /// Before the plugin writes a copy.
  void written() noexcept
  {
    if (limiter_)
    {
      limiter_->written();
    }
  }

private:
  bool limit_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period, bool through_block) noexcept;

  HardwareStats & stats_;
  /// What controllers use and what the plugin exported, by the same index.
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> states_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands_;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> plugin_states_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> plugin_commands_;
  std::map<std::string, uint32_t> command_index_;
  /// The shadowed interfaces and the buffers of their copy, preallocated by init.
  struct StateCopy
  {
    hardware_interface::StateInterface * shadow;
    const hardware_interface::StateInterface * plugin;
  };
  struct CommandCopy
  {
    uint32_t index;
    const hardware_interface::CommandInterface * shadow;
    hardware_interface::CommandInterface * plugin;
  };
  std::vector<StateCopy> state_copies_;
  std::vector<double> state_values_;
  std::vector<std::unique_lock<std::shared_mutex>> state_locks_;
  std::vector<CommandCopy> command_copies_;
  std::vector<std::unique_lock<std::shared_mutex>> command_locks_;
  /// By command index.
  std::vector<double> command_values_;
  std::unique_ptr<CommandLimiter> limiter_;
  /// By command index, written by RT. The copy sends NaN for an unclaimed interface.
  std::unique_ptr<std::atomic<uint32_t>[]> claim_words_;
  /// Counts the updates, written by RT.
  std::atomic<uint64_t> updates_{0};
  /// RT only, the claims of the last update.
  std::vector<Plan::Claim> rt_claims_;
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__SHADOW_HANDLES_HPP_
