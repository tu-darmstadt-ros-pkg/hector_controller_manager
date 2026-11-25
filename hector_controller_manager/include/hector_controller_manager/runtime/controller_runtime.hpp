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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__CONTROLLER_RUNTIME_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__CONTROLLER_RUNTIME_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/core/reconciler.hpp"
#include "hector_controller_manager/runtime/fixed_text.hpp"
#include "hector_controller_manager/runtime/io_gate.hpp"
#include "hector_controller_manager/runtime/owner_thread.hpp"

namespace hector_controller_manager::runtime
{
/// Handles a controller claims, by full name.
struct InterfaceHandles
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state;
};

/// What a configured controller exposes.
struct ControllerExports
{
  core::InterfaceRequest interfaces;
  bool chainable = false;
  std::vector<hardware_interface::CommandInterface::SharedPtr> references;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> exported_states;
};

/// Why RT stopped a controller in an activation.
enum class StopKind
{
  None,
  /// Its own update failed.
  Own,
  /// The e-stop stopped it.
  EStop,
  /// Another failure of its scope stopped it.
  Scope,
};

/// One controller with its plugin and its owner thread. RT calls update, the owner everything else,
/// never at the same time, because the owner only touches a controller no adopted plan runs. The
/// owner's lifecycle calls hold `introspection_gate`, so RT never samples what they write.
class ControllerRuntime
{
public:
  using Plugin = controller_interface::ControllerInterfaceBaseSharedPtr;

  ControllerRuntime(std::string name, std::string type, std::shared_ptr<IoGate> introspection_gate);
  ~ControllerRuntime();
  ControllerRuntime(const ControllerRuntime &) = delete;
  ControllerRuntime & operator=(const ControllerRuntime &) = delete;

  const std::string & name() const { return name_; }
  const std::string & type() const { return type_; }
  OwnerThread & owner() { return *owner_; }

  /// Owner jobs.
  void set_plugin(Plugin plugin) { plugin_ = std::move(plugin); }
  const Plugin & plugin() const { return plugin_; }
  core::Lifecycle configure(unsigned int cm_update_rate, ControllerExports & exports);
  core::Lifecycle cleanup();
  /// FAILURE counts as stopped, the controller is shut down and ends finalized.
  core::Outcome deactivate();
  /// Sets the chained mode first, the plugin keeps its own across cleanup and errors.
  core::Outcome activate(InterfaceHandles handles, bool chained);
  /// Why the plugin refused the mode, empty if it took it.
  std::string set_chained(bool chained);
  core::Lifecycle shutdown();

  /// Any thread.
  core::Lifecycle state() const;
  double update_seconds() const noexcept { return update_seconds_.load(std::memory_order_relaxed); }

  /// RT. Activation number it failed in, written by RT only.
  std::atomic<uint64_t> failed_activation{0};
  /// RT. Activation number whose own update failed, stored before the failure of its scope.
  std::atomic<uint64_t> failed_update{0};
  /// RT. Activation number the e-stop stopped, stored before the failure of its scope.
  std::atomic<uint64_t> estop_activation{0};
  bool rt_update(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept;
  /// Any thread. Whether and why RT stopped the controller in the activation.
  StopKind stop_kind(uint64_t activation) const noexcept;
  /// Why the update of `failed_update` failed, the text of the exception it threw or empty if it
  /// returned ERROR. Read it only after `failed_activation` showed that activation.
  std::string update_error() const { return update_error_.get(); }

private:
  std::string name_;
  std::string type_;
  const std::shared_ptr<IoGate> introspection_gate_;
  Plugin plugin_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> references_;
  /// Written by RT only, sampled by the diagnostics.
  std::atomic<double> update_seconds_{0.0};
  /// Written by RT in the failing update, before it publishes the failure.
  FixedText update_error_;
  /// Last, so the thread ends before the members it uses.
  std::unique_ptr<OwnerThread> owner_;
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__CONTROLLER_RUNTIME_HPP_
