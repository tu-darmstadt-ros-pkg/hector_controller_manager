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

#include "hector_controller_manager/runtime/shadow_handles.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

#include "hector_controller_manager/runtime/thread_role.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using hardware_interface::HandleDataType;

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

void ShadowHandles::init(
  const hardware_interface::HardwareInfo & info,
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> plugin_states,
  std::vector<hardware_interface::CommandInterface::SharedPtr> plugin_commands, bool limit)
{
  expect_precondition(
    [this] { return states_.empty() && commands_.empty(); }, "ShadowHandles::init twice");
  plugin_states_ = std::move(plugin_states);
  plugin_commands_ = std::move(plugin_commands);
  // RT on a sync component runs the controllers and the plugin, so these need no copy
  const auto shadowed = [&info](const hardware_interface::Handle & handle)
  {
    if (handle.get_data_type() == HandleDataType::DOUBLE)
    {
      return true;
    }
    if (info.is_async)
    {
      throw std::runtime_error(
        "The asynchronous component '" + info.name + "' exports '" + handle.get_name() +
        "', which does not hold a double. RT copies only double interfaces to and from an "
        "asynchronous component");
    }
    return false;
  };
  for (const auto & state : plugin_states_)
  {
    if (!shadowed(*state))
    {
      states_.push_back(state);
      continue;
    }
    auto shadow = std::make_shared<hardware_interface::StateInterface>(
      state->get_prefix_name(), state->get_interface_name());
    (void)shadow->set_value(state->get_optional().value_or(NaN), true);
    state_copies_.push_back({shadow.get(), state.get()});
    states_.push_back(std::move(shadow));
  }
  for (uint32_t i = 0; i < plugin_commands_.size(); ++i)
  {
    const auto & command = plugin_commands_[i];
    command_index_.emplace(command->get_name(), i);
    if (!shadowed(*command))
    {
      commands_.push_back(command);
      continue;
    }
    auto shadow = std::make_shared<hardware_interface::CommandInterface>(
      command->get_prefix_name(), command->get_interface_name());
    (void)shadow->set_value(command->get_optional().value_or(NaN), true);
    command_copies_.push_back({i, shadow.get(), command.get()});
    commands_.push_back(std::move(shadow));
  }
  state_values_.resize(state_copies_.size());
  state_locks_.resize(state_copies_.size());
  command_locks_.resize(command_copies_.size());
  command_values_.resize(commands_.size());
  if (limit)
  {
    // Plugins step by the update rate, a longer period after an overrun allows no longer step
    const double max_period =
      info.rw_rate > 0 ? 1.0 / info.rw_rate : std::numeric_limits<double>::infinity();
    limiter_ = std::make_unique<CommandLimiter>(info, plugin_commands_, plugin_states_, max_period);
  }
  claim_words_ = std::make_unique<std::atomic<uint32_t>[]>(commands_.size());
  rt_claims_.resize(commands_.size());
}

std::optional<uint32_t> ShadowHandles::command_index(const std::string & name) const
{
  const auto it = command_index_.find(name);
  return it == command_index_.end() ? std::nullopt : std::optional(it->second);
}

void ShadowHandles::rt_note_claims(const std::vector<Plan::Claim> & claims) noexcept
{
  updates_.fetch_add(1, std::memory_order_relaxed);
  for (size_t i = 0; i < rt_claims_.size(); ++i)
  {
    const Plan::Claim claim = i < claims.size() ? claims[i] : Plan::Claim{};
    if (claim != rt_claims_[i])
    {
      rt_claims_[i] = claim;
      const uint32_t changes = (claim_words_[i].load(std::memory_order_relaxed) >> 1) + 1;
      claim_words_[i].store(
        (changes << 1) | (claim.activation != 0 ? CLAIMED : 0), std::memory_order_relaxed);
    }
  }
}

bool ShadowHandles::copy_states() noexcept
{
  for (size_t i = 0; i < state_copies_.size(); ++i)
  {
    const auto value = state_copies_[i].plugin->get_optional();
    if (!value)
    {
      return false;
    }
    state_values_[i] = *value;
  }
  // All shadows at once, so a controller never sees joints of two reads
  size_t locked = 0;
  while (locked < state_copies_.size())
  {
    state_locks_[locked] =
      std::unique_lock(state_copies_[locked].shadow->get_mutex(), std::try_to_lock);
    if (!state_locks_[locked].owns_lock())
    {
      break;
    }
    ++locked;
  }
  const bool complete = locked == state_copies_.size();
  for (size_t i = 0; i < locked; ++i)
  {
    if (complete)
    {
      (void)state_copies_[i].shadow->set_value(state_locks_[i], state_values_[i]);
    }
    state_locks_[i].unlock();
  }
  return complete;
}

bool ShadowHandles::copy_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period, bool through_block) noexcept
{
  bool complete = true;
  for (const auto & copy : command_copies_)
  {
    // A detached claimer's goal stays in its shadow until the NaN of its release
    const bool claimed = claim_words_[copy.index].load(std::memory_order_relaxed) & CLAIMED;
    const auto value = claimed ? copy.shadow->get_optional() : std::optional(NaN);
    complete = complete && value.has_value();
    command_values_[copy.index] = value.value_or(NaN);
  }
  // Only a copy that is written advances the limits, else the next one would jump by its step
  complete = complete && (!limiter_ || limit_commands(time, period, through_block));
  // All plugin handles at once, so a write never gets joints of two copies
  size_t locked = 0;
  while (complete && locked < command_copies_.size())
  {
    command_locks_[locked] =
      std::unique_lock(command_copies_[locked].plugin->get_mutex(), std::try_to_lock);
    if (!command_locks_[locked].owns_lock())
    {
      complete = false;
      break;
    }
    ++locked;
  }
  for (size_t i = 0; i < locked; ++i)
  {
    if (complete)
    {
      const auto & copy = command_copies_[i];
      (void)copy.plugin->set_value(command_locks_[i], command_values_[copy.index]);
    }
    command_locks_[i].unlock();
  }
  if (!complete)
  {
    stats_.skip();
  }
  return complete;
}

bool ShadowHandles::limit_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period, bool through_block) noexcept
{
  const auto result = limiter_->enforce(
    updates_.load(std::memory_order_relaxed), time.nanoseconds(),
    through_block ? 0.0 : period.seconds(), claim_words_.get(), command_values_.data());
  if (result.out_of_range)
  {
    stats_.out_of_range();
  }
  if (result.clamped)
  {
    stats_.clamp(result.last_clamped);
  }
  return result.complete;
}

}  // namespace hector_controller_manager::runtime
