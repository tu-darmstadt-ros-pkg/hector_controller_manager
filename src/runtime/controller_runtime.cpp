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

#include "hector_controller_manager/runtime/controller_runtime.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "hector_controller_manager/runtime/lifecycle_state.hpp"
#include "hector_controller_manager/runtime/thrown.hpp"
#include "rclcpp/logging.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using controller_interface::interface_configuration_type;
using core::CallResult;
using core::Lifecycle;

/// INDIVIDUAL, INDIVIDUAL_BEST_EFFORT and NONE. ALL and REGEX are rejected loudly.
std::vector<std::string> names_of(
  const controller_interface::InterfaceConfiguration & configuration, bool & best_effort,
  const char * kind)
{
  switch (configuration.type)
  {
    case interface_configuration_type::NONE:
      return {};
    case interface_configuration_type::INDIVIDUAL:
      return configuration.names;
    case interface_configuration_type::INDIVIDUAL_BEST_EFFORT:
      best_effort = true;
      return configuration.names;
    case interface_configuration_type::ALL:
    case interface_configuration_type::REGEX:
      break;
  }
  throw std::runtime_error(
    std::string("The ") + kind +
    " interface configuration ALL or REGEX is not supported, list the interfaces explicitly");
}

/// Sets `reason` if the transition threw.
template <typename Transition>
Lifecycle guarded(
  const ControllerRuntime::Plugin & plugin, Transition transition, std::string & reason)
{
  Lifecycle state{};
  const auto error = thrown_by([&] { state = lifecycle_of_id(transition().id()); });
  if (!error)
  {
    return state;
  }
  reason = "A lifecycle transition threw: " + *error;
  RCLCPP_ERROR(plugin->get_node()->get_logger(), "%s", reason.c_str());
  return lifecycle_of_id(plugin->get_lifecycle_id());
}

template <typename Transition>
Lifecycle guarded(const ControllerRuntime::Plugin & plugin, Transition transition)
{
  std::string reason;
  return guarded(plugin, transition, reason);
}

/// Why a transition ended in the error path. rclcpp_lifecycle logs and swallows the exception of a
/// callback, so only an exception of the transition itself has a text.
std::string errored(const char * callback, Lifecycle state, const std::string & reason)
{
  return reason.empty() ? std::string(callback) + " returned ERROR or threw, the controller is " +
                            to_string(state)
                        : reason;
}
}  // namespace

ControllerRuntime::ControllerRuntime(std::string name, std::string type)
: name_(std::move(name)), type_(std::move(type)), owner_(std::make_unique<OwnerThread>())
{
}

Lifecycle ControllerRuntime::configure(unsigned int cm_update_rate, ControllerExports & exports)
{
  owner_->expect_owner();
  const auto node = plugin_->get_node();
  if (node->get_parameter("is_async").as_bool())
  {
    throw std::runtime_error("Asynchronous controllers are not supported");
  }
  const auto update_rate = node->get_parameter("update_rate").as_int();
  if (update_rate != 0 && update_rate != static_cast<int64_t>(cm_update_rate))
  {
    throw std::runtime_error(
      "Per controller update rates are not supported, 'update_rate' must equal the controller "
      "manager's " +
      std::to_string(cm_update_rate) + " Hz");
  }
  const Lifecycle state = guarded(plugin_, [&]() -> const auto & { return plugin_->configure(); });
  if (state != Lifecycle::Inactive)
  {
    return state;
  }
  try
  {
    bool command_best_effort = false;
    exports.interfaces.command =
      names_of(plugin_->command_interface_configuration(), command_best_effort, "command");
    if (command_best_effort)
    {
      throw std::runtime_error("INDIVIDUAL_BEST_EFFORT is supported for state interfaces only");
    }
    exports.interfaces.state = names_of(
      plugin_->state_interface_configuration(), exports.interfaces.best_effort_states, "state");
    exports.chainable = plugin_->is_chainable();
    if (exports.chainable)
    {
      exports.references = plugin_->export_reference_interfaces();
      exports.exported_states = plugin_->export_state_interfaces();
    }
  }
  catch (...)
  {
    cleanup();
    throw;
  }
  references_ = exports.references;
  return state;
}

Lifecycle ControllerRuntime::cleanup()
{
  owner_->expect_owner();
  references_.clear();
  return guarded(plugin_, [&]() -> const auto & { return plugin_->get_node()->cleanup(); });
}

core::Outcome ControllerRuntime::deactivate()
{
  owner_->expect_owner();
  std::string reason;
  const Lifecycle state =
    guarded(plugin_, [&]() -> const auto & { return plugin_->get_node()->deactivate(); }, reason);
  plugin_->release_interfaces();
  if (state == Lifecycle::Inactive)
  {
    return core::call_outcome(CallResult::Ok, state);
  }
  if (state == Lifecycle::Active)
  {
    // A refused stop counts as stopped, or the stop would be retried forever
    const Lifecycle after = shutdown();
    return core::call_outcome(
      CallResult::Failure, Lifecycle::Finalized,
      after == Lifecycle::Finalized
        ? "on_deactivate refused to stop, the controller is shut down"
        : std::string("on_deactivate refused to stop and the shutdown failed, the controller is ") +
            to_string(after) + " and never used again");
  }
  return core::call_outcome(CallResult::Error, state, errored("on_deactivate", state, reason));
}

core::Outcome ControllerRuntime::activate(InterfaceHandles handles, bool chained)
{
  owner_->expect_owner();
  if (plugin_->is_chainable())
  {
    if (auto refused = set_chained(chained); !refused.empty())
    {
      return core::call_outcome(CallResult::Failure, state(), std::move(refused));
    }
  }
  for (const auto & reference : references_)
  {
    (void)reference->set_value(std::numeric_limits<double>::quiet_NaN(), true);
  }
  std::vector<hardware_interface::LoanedCommandInterface> command;
  for (auto & handle : handles.command)
  {
    command.emplace_back(std::move(handle));
  }
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  for (auto & handle : handles.state)
  {
    state_interfaces.emplace_back(std::move(handle));
  }
  plugin_->assign_interfaces(std::move(command), std::move(state_interfaces));
  std::string reason;
  const Lifecycle state =
    guarded(plugin_, [&]() -> const auto & { return plugin_->get_node()->activate(); }, reason);
  if (state == Lifecycle::Active)
  {
    return core::call_outcome(CallResult::Ok, state);
  }
  plugin_->release_interfaces();
  if (state == Lifecycle::Inactive)
  {
    return core::call_outcome(CallResult::Failure, state, "on_activate returned FAILURE");
  }
  return core::call_outcome(CallResult::Error, state, errored("on_activate", state, reason));
}

std::string ControllerRuntime::set_chained(bool chained)
{
  owner_->expect_owner();
  bool accepted = false;
  if (const auto error = thrown_by([&] { accepted = plugin_->set_chained_mode(chained); }))
  {
    const std::string reason = "Setting the chained mode threw: " + *error;
    RCLCPP_ERROR(plugin_->get_node()->get_logger(), "%s", reason.c_str());
    return reason;
  }
  return accepted
           ? ""
           : std::string("The controller refused ") + (chained ? "chained" : "unchained") + " mode";
}

Lifecycle ControllerRuntime::shutdown()
{
  owner_->expect_owner();
  if (lifecycle_of_id(plugin_->get_lifecycle_id()) == Lifecycle::Finalized)
  {
    return Lifecycle::Finalized;
  }
  const Lifecycle state =
    guarded(plugin_, [&]() -> const auto & { return plugin_->get_node()->shutdown(); });
  plugin_->release_interfaces();
  return state;
}

Lifecycle ControllerRuntime::state() const { return lifecycle_of_id(plugin_->get_lifecycle_id()); }

bool ControllerRuntime::rt_update(
  const rclcpp::Time & time, const rclcpp::Duration & period) noexcept
{
  const auto start = Clock::now();
  controller_interface::return_type result = controller_interface::return_type::ERROR;
  // Copied without allocating, the supervisor logs it when it absorbs the failure
  const auto record = [this](const char * text)
  {
    size_t i = 0;
    for (; text[i] != '\0' && i + 1 < update_error_.size(); ++i)
    {
      update_error_[i] = text[i];
    }
    update_error_[i] = '\0';
  };
  try
  {
    result = plugin_->update(time, period);
    if (result != controller_interface::return_type::OK)
    {
      record("");
    }
  }
  catch (const std::exception & e)
  {
    record(e.what());
  }
  catch (...)
  {
    record("an exception of unknown type");
  }
  update_seconds_.store(
    std::chrono::duration<double>(Clock::now() - start).count(), std::memory_order_relaxed);
  return result == controller_interface::return_type::OK;
}

}  // namespace hector_controller_manager::runtime
