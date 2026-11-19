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

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/hardware_component_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/core/reconciler.hpp"
#include "hector_controller_manager/runtime/cycle_stamp.hpp"
#include "hector_controller_manager/runtime/fault_latch.hpp"
#include "hector_controller_manager/runtime/hang_watch.hpp"
#include "hector_controller_manager/runtime/hardware_stats.hpp"
#include "hector_controller_manager/runtime/io_gate.hpp"
#include "hector_controller_manager/runtime/owner_cycle.hpp"
#include "hector_controller_manager/runtime/owner_thread.hpp"
#include "hector_controller_manager/runtime/plan.hpp"
#include "hector_controller_manager/runtime/scheduling.hpp"
#include "hector_controller_manager/runtime/shadow_handles.hpp"
#include "hector_controller_manager/runtime/stop_batch.hpp"
#include "hector_controller_manager/runtime/write_block.hpp"
#include "rclcpp/logger.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace hector_controller_manager::runtime
{
using HardwareFactory =
  std::function<std::unique_ptr<hardware_interface::HardwareComponentInterface>(
    const hardware_interface::HardwareInfo &)>;

/// SCHED_FIFO priority of an async owner thread, its URDF thread_priority or one below the control
/// loop's `rt_priority`. The parser gives 50 when the URDF sets none, so an explicit 50 counts as
/// unset.
int owner_priority(const hardware_interface::HardwareAsyncParams & params, int rt_priority);

enum class IoResult : uint8_t
{
  Ok,
  Skipped,
  Error,
  Deactivate,
};

/// One hardware component with its plugin, its owner thread and what RT and the owner share.
///
/// - `read` and `write` of a sync component run on RT, everything else on the owner.
/// - An async component runs everything on the owner, triggered by RT once per cycle.
/// - The IO thread writes the IO fault. It is RT for a sync component and the owner for an async
///   one or a write through a block.
/// - The owner writes the call fault and the lifecycle word, RT the hang fault and the stop
///   requests.
/// - Controllers use shadows of the plugin's double interfaces. The copies between them and the
///   plugin run only while no plugin call runs, so a controller and the plugin never share a
///   handle and every copy is one consistent snapshot.
/// - RT makes the copies, the owner the command copy of its write through a block.
class HardwareRuntime
{
public:
  /// Called on the owner after it held interfaces RT asked to stop.
  using HoldListener = std::function<void(const std::string &, std::vector<std::string>)>;

  /// `gate_timeout` bounds how long the owner waits for RT to leave a read or write before the
  /// component faults. `limit_commands` clamps the commands to the joint limits of `info`.
  HardwareRuntime(
    hardware_interface::HardwareInfo info, HoldListener on_hold,
    Clock::duration gate_timeout = std::chrono::seconds(1), bool limit_commands = false);

  const std::string & name() const { return info_.name; }
  const hardware_interface::HardwareInfo & info() const { return info_; }
  bool is_async() const { return info_.is_async; }
  OwnerThread & owner() { return *owner_; }

  /// Owner jobs, call them only from the owner thread.

  /// Pins an async owner to the cores of its URDF and, on a realtime kernel, runs it with
  /// SCHED_FIFO at owner_priority. Without the privileges it warns, as the control loop does.
  /// Threads the plugin starts on the owner keep its cores but run with SCHED_OTHER.
  void schedule_owner(int rt_priority, const Scheduler & scheduler = {});
  /// What schedule_owner got, read once the job that called it returned.
  const Scheduling & owner_scheduling() const { return owner_scheduling_; }
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

  /// The handles controllers use, valid after initialize. A synchronous component passes its
  /// interfaces that do not hold a double uncopied.
  const std::vector<hardware_interface::StateInterface::ConstSharedPtr> & state_interfaces() const
  {
    return shadows_.states();
  }
  const std::vector<hardware_interface::CommandInterface::SharedPtr> & command_interfaces() const
  {
    return shadows_.commands();
  }
  std::optional<uint32_t> command_index(const std::string & name) const
  {
    return shadows_.command_index(name);
  }

  /// Any thread.
  core::Lifecycle state() const noexcept;
  core::HardwareFault fault() const noexcept;
  /// Why the component faulted since its last lifecycle transition, empty without a fault.
  std::string fault_reason() const;
  const HardwareStats & stats() const noexcept { return stats_; }
  /// The command interface the limits clamped last, empty before any clamp.
  std::string last_clamped() const;

  /// Supervisor. Writes stop before a mode switch job is submitted and resume only once it
  /// returned in time. An abandoned switch keeps them stopped until the next lifecycle transition,
  /// so a late perform never applies a command of a claimer that left meanwhile.
  void block_writes() noexcept { write_block_.block(generation()); }
  void unblock_writes() noexcept { write_block_.unblock(); }
  /// Owner job. One write through the block, once every live interface holds NaN after a hand
  /// over released it, so the hardware holds instead of keeping the last command.
  void write_through_block();

  /// RT.
  /// Records the claims of the plan the controllers update with, once per update.
  void rt_note_claims(const std::vector<Plan::Claim> & claims) noexcept;
  /// Reads a sync component. Copies the states into the shadows, for an async component only
  /// while its owner is free.
  IoResult rt_read(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept;
  IoResult rt_write(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept;
  /// Copies the commands of an async component and triggers its cycle, only while its owner is
  /// free, so two copies never go into one write.
  void rt_trigger(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept;
  /// Asks the owner to hold the interface. The requests reach it together at rt_publish_stops.
  void rt_request_stop(uint32_t interface) noexcept;
  void rt_publish_stops() noexcept;
  /// True once per fault and plan, so RT stops the failure scope of every plan it adopts while
  /// the fault lasts, also one that attached a new claimer.
  bool rt_take_fault(uint64_t plan_generation) noexcept;
  /// Faults a component in use whose owner stayed busy outside a mode switch for longer than the
  /// timeout, measured from the first check that found it so. True if it just did.
  bool rt_check_hang(Clock::time_point now, Clock::duration timeout, bool in_use) noexcept;
  /// Enters the gate while no plugin call and, for an async component, no cycle runs, so RT may
  /// read what the plugin registered for introspection. Every true return needs an rt_leave_idle.
  bool rt_enter_idle() noexcept;
  void rt_leave_idle() noexcept { gate_.rt_leave(); }

private:
  uint64_t generation() const noexcept
  {
    return lifecycle_.load(std::memory_order_acquire) >> STATE_BITS;
  }
  void publish_state(core::Lifecycle state);
  /// Appends why the component did not recover to `reason`.
  core::Lifecycle error_path(std::string & reason);
  using Callback = hardware_interface::CallbackReturn (
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::*)(const rclcpp_lifecycle::State &);
  core::Outcome call(
    Callback callback, const char * name, core::Lifecycle on_success, core::Lifecycle on_failure);
  /// Runs a plugin call on the owner, excluding RT IO and copies. If RT does not leave in time, the
  /// component faults and nothing is returned.
  template <typename Call>
  auto exclusive(Call call) -> std::optional<decltype(call())>;
  void poll();
  void hold_requested();
  void async_cycle();
  /// Whether a write may run now, checked before its copy and again before the plugin call.
  bool writes(bool through_block) const noexcept;
  /// RT, async only. No call of the owner runs and it served every trigger.
  bool rt_owner_free() const noexcept;
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
  const bool limit_commands_;

  IoGate gate_;
  static constexpr int STATE_BITS = 8;
  /// Generation from 1 up shifted by STATE_BITS, lifecycle state in the low bits. Written by the
  /// owner, read by every thread.
  std::atomic<uint64_t> lifecycle_{uint64_t{1} << STATE_BITS};
  /// Raised by the IO thread, by the owner and by RT, in this order.
  FaultLatch io_fault_;
  FaultLatch call_fault_;
  FaultLatch hang_fault_;
  Scheduling owner_scheduling_;
  HangWatch hang_;
  WriteBlock write_block_;
  /// What RT asked the owner to hold.
  StopBatch stops_;
  /// The async cycle RT triggered.
  OwnerCycle cycle_;
  CycleStamp stamp_;

  HardwareStats stats_;
  /// The handles controllers use, copied under the gate.
  ShadowHandles shadows_{stats_};

  /// RT only.
  struct
  {
    uint64_t handled_generation = 0;
    uint64_t handled_plan_generation = 0;
  } rt_;
  /// Owner only.
  struct
  {
    std::vector<bool> live;
    rcl_clock_type_t clock_type = RCL_ROS_TIME;
  } owner_only_;

  /// Last, so the thread ends before the members it uses.
  std::unique_ptr<OwnerThread> owner_;
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__HARDWARE_RUNTIME_HPP_
