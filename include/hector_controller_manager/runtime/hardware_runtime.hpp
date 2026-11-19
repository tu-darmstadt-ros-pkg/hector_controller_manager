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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__HARDWARE_RUNTIME_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__HARDWARE_RUNTIME_HPP_

#include <array>
#include <atomic>
#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "hardware_interface/hardware_component_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/core/reconciler.hpp"
#include "hector_controller_manager/runtime/io_gate.hpp"
#include "hector_controller_manager/runtime/owner_thread.hpp"
#include "rclcpp/logger.hpp"

namespace hector_controller_manager::runtime
{
using HardwareFactory =
  std::function<std::unique_ptr<hardware_interface::HardwareComponentInterface>(
    const hardware_interface::HardwareInfo &)>;

enum class IoResult : uint8_t
{
  Ok,
  Skipped,
  Error,
  Deactivate,
};

/// One hardware component with its plugin, its owner thread and what RT and the owner share.
///
/// `read` and `write` of a sync component run on RT, everything else on the owner. An
/// async component runs everything on the owner, triggered by RT once per cycle. The IO fault word
/// is written by the IO thread (RT for sync, owner for async), the call fault and the lifecycle
/// word by the owner, the hang fault and stop requests by RT.
class HardwareRuntime
{
public:
  /// Called on the owner after it held interfaces RT asked to stop.
  using HoldListener = std::function<void(const std::string &, std::vector<std::string>)>;

  /// `gate_timeout` bounds how long the owner waits for RT to leave a read or write before the
  /// component faults.
  HardwareRuntime(
    hardware_interface::HardwareInfo info, HoldListener on_hold,
    Clock::duration gate_timeout = std::chrono::seconds(1));

  const std::string & name() const { return info_.name; }
  const hardware_interface::HardwareInfo & info() const { return info_; }
  bool is_async() const { return info_.is_async; }
  OwnerThread & owner() { return *owner_; }

  /// Owner jobs, call them only from the owner thread.
  /// Creates, initializes and exports the interfaces. Throws with the reason on failure.
  void initialize(
    const HardwareFactory & factory, hardware_interface::HardwareComponentParams params);
  /// The state the transition ended in, with the reason if a callback failed.
  core::Outcome transition(core::step::HardwareTransition transition);
  /// The validation prepare, whose error only rejects.
  bool validate(const std::vector<std::string> & start, const std::vector<std::string> & stop);
  /// prepare and perform for a split mode switch, which leaves out stops of what is held already.
  /// An error faults the component and returns its reason, success returns an empty one.
  std::string switch_modes(
    const std::vector<std::string> & start, const std::vector<std::string> & stop);

  /// Valid after initialize.
  const std::vector<hardware_interface::StateInterface::ConstSharedPtr> & state_interfaces() const
  {
    return states_;
  }
  const std::vector<hardware_interface::CommandInterface::SharedPtr> & command_interfaces() const
  {
    return commands_;
  }
  std::optional<uint32_t> command_index(const std::string & name) const;

  /// Any thread.
  core::Lifecycle state() const noexcept;
  core::HardwareFault fault() const noexcept;
  /// Why the component faulted since its last lifecycle transition, empty without a fault.
  std::string fault_reason() const;
  double read_seconds() const noexcept { return read_seconds_.load(std::memory_order_relaxed); }
  double write_seconds() const noexcept { return write_seconds_.load(std::memory_order_relaxed); }
  uint64_t skipped_cycles() const noexcept { return skipped_.load(std::memory_order_relaxed); }

  /// Supervisor. Writes stop before a mode switch job is submitted and resume only once it
  /// returned in time. An abandoned switch keeps them stopped until the next lifecycle transition,
  /// so a late perform never applies a command of a claimer that left meanwhile.
  void block_writes() noexcept;
  void unblock_writes() noexcept { write_block_.store(0, std::memory_order_release); }
  /// Owner job. One write through the block, once every live interface holds NaN after a hand
  /// over released it, so the hardware holds instead of keeping the last command.
  void write_through_block();

  /// RT.
  IoResult rt_read(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept;
  IoResult rt_write(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept;
  void rt_trigger(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept;
  void rt_request_stop(uint32_t interface) noexcept;
  /// True once per fault and plan, so RT stops the failure scope of every plan it adopts while
  /// the fault lasts, also one that attached a new claimer.
  bool rt_take_fault(uint64_t plan_generation) noexcept;
  /// Faults a component that did not complete a cycle within the timeout. True if it just did.
  bool rt_check_hang(Clock::time_point now, Clock::duration timeout) noexcept;

private:
  /// The reason of one fault word in fixed storage, so RT records it without allocating. Its
  /// writer copies it before the release store of the word, a reader loads the word first.
  class FaultReason
  {
  public:
    void set(std::initializer_list<std::string_view> parts) noexcept;
    std::string get() const;

  private:
    std::array<std::atomic<char>, 256> text_{};
  };

  /// Publishes a fault of the current generation. The first reason stays, a reader may copy it.
  void raise(
    std::atomic<uint64_t> & word, FaultReason & reason, core::HardwareFault kind,
    std::initializer_list<std::string_view> parts) noexcept;

  uint64_t generation() const noexcept { return lifecycle_.load(std::memory_order_acquire) >> 8; }
  void publish_state(core::Lifecycle state);
  /// Appends why the component did not recover to `reason`.
  core::Lifecycle error_path(std::string & reason);
  using Callback = hardware_interface::CallbackReturn (
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::*)(const rclcpp_lifecycle::State &);
  core::Outcome call(
    Callback callback, const char * name, core::Lifecycle on_success, core::Lifecycle on_failure);
  /// Runs a plugin call on the owner, excluding RT IO of a sync component. If RT does not leave a
  /// read or write in time, the component faults and nothing is returned.
  template <typename Call>
  auto exclusive(Call call) -> std::optional<decltype(call())>;
  void poll();
  /// RT, async only. Restarts the hang window whenever the owner completed a cycle.
  void rt_track_async_progress(Clock::time_point now) noexcept;
  void hold_requested();
  void async_cycle();
  IoResult io(
    bool write, const rclcpp::Time & time, const rclcpp::Duration & period,
    bool through_block = false) noexcept;

  hardware_interface::HardwareInfo info_;
  const Clock::duration gate_timeout_;
  /// A sensor is never written.
  const bool writable_;
  HoldListener on_hold_;
  rclcpp::Logger logger_;
  std::unique_ptr<hardware_interface::HardwareComponentInterface> plugin_;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> states_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands_;
  std::map<std::string, uint32_t> command_index_;

  IoGate gate_;
  /// Generation shifted by 8, lifecycle state in the low byte. Owner only.
  std::atomic<uint64_t> lifecycle_{1u << 8};
  /// Generation shifted by KIND_BITS, the core::HardwareFault in the low bits.
  std::atomic<uint64_t> io_fault_{0};
  std::atomic<uint64_t> call_fault_{0};
  std::atomic<uint64_t> hang_fault_{0};
  FaultReason io_reason_;
  FaultReason call_reason_;
  FaultReason hang_reason_;
  /// The plugin call the owner is in, named in the reason of a hang.
  std::atomic<const char *> owner_call_{nullptr};
  /// Lifecycle generation plus one while writes are blocked, 0 otherwise. Supervisor only.
  std::atomic<uint64_t> write_block_{0};
  /// Per command interface, counts RT stop requests. RT only.
  std::unique_ptr<std::atomic<uint32_t>[]> stop_requests_;
  std::atomic<uint32_t> stop_total_{0};
  /// The stop total the owner has held. Owner only.
  std::atomic<uint32_t> stops_done_{0};
  /// Async trigger, RT only. Time and period of the last write or trigger.
  std::atomic<uint64_t> triggers_{0};
  std::atomic<int64_t> trigger_time_ns_{0};
  std::atomic<int64_t> trigger_period_ns_{0};
  std::atomic<uint64_t> served_{0};

  std::atomic<double> read_seconds_{0.0};
  std::atomic<double> write_seconds_{0.0};
  std::atomic<uint64_t> skipped_{0};

  /// RT only.
  struct
  {
    uint64_t handled_generation = 0;
    uint64_t handled_plan_generation = 0;
    Clock::time_point stalled_since{};
    uint64_t last_served = 0;
  } rt_;
  /// Owner only.
  struct
  {
    std::vector<uint32_t> stops_seen;
    std::vector<bool> live;
    uint64_t triggers_seen = 0;
    rcl_clock_type_t clock_type = RCL_ROS_TIME;
  } owner_only_;

  /// Last, so the thread ends before the members it uses.
  std::unique_ptr<OwnerThread> owner_;
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__HARDWARE_RUNTIME_HPP_
