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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__PLAN_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__PLAN_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include "hector_controller_manager/runtime/doorbell.hpp"

namespace hector_controller_manager::runtime
{
class ControllerRuntime;
class HardwareRuntime;

/// Everything RT runs, immutable once published. The supervisor builds it off RT, so RT never
/// allocates or looks anything up.
struct Plan
{
  struct Span
  {
    uint32_t begin = 0;
    uint32_t end = 0;
  };
  /// A command interface of a hardware component, by index.
  struct HardwareInterface
  {
    HardwareRuntime * hardware;
    uint32_t interface;
  };
  /// What RT stops when this fails, controllers by index and the hardware interfaces they command.
  struct FailureScope
  {
    Span controllers;
    Span holds;
  };
  struct Controller
  {
    ControllerRuntime * runtime;
    uint64_t activation;
    FailureScope scope;
    /// Claims command interfaces and is not protected, so the e-stop stops it.
    bool estop;
  };
  struct Hardware
  {
    HardwareRuntime * runtime;
    /// Running controllers claim its interfaces, so a stall of it faults it.
    bool in_use;
    FailureScope scope;
  };

  uint64_t generation = 0;
  std::vector<Hardware> hardware;
  /// In ChainGraph::update_order, so references and exported states arrive in the same cycle.
  std::vector<Controller> controllers;
  std::vector<uint32_t> scope_controllers;
  std::vector<HardwareInterface> scope_holds;
  /// Keeps the runtimes referenced above alive while RT may use the plan.
  std::vector<std::shared_ptr<const void>> keep_alive;
};

/// Hands plans to RT through one atomic and tells the supervisor what RT adopted.
///
/// RT sets its in-call flag, then loads the published plan. The supervisor stores the plan, then
/// loads the flag. Both sequentially consistent, so either RT sees the new plan or the supervisor
/// sees RT outside a call, which then cannot use the old plan any more.
class PlanSlot
{
public:
  /// RT rings `progress` when it adopts a plan, leaves a call or finished an update, but only while
  /// somebody waits for that.
  explicit PlanSlot(Doorbell & progress) : progress_(progress) {}

  /// Supervisor side.
  void publish(const Plan * plan) noexcept;
  /// Until no RT call runs with a plan older than the generation. False at the deadline.
  bool wait_quiescent(uint64_t generation, Clock::time_point deadline) noexcept;
  /// Until RT finished an update with the generation or a newer one. False at the deadline, or
  /// earlier when the doorbell rang for something else.
  bool wait_updated(uint64_t generation, Clock::time_point deadline) noexcept;
  /// Until no RT call runs with a plan older than the generation, or the doorbell rang after
  /// `seen` for something else. RT rings when it leaves a call meanwhile.
  void wait_quiescent_or_ring(
    uint64_t generation, uint32_t seen, Clock::time_point deadline) noexcept;

  /// RT side. Every enter needs a leave. Returns the adopted plan, null before the first.
  const Plan * rt_enter() noexcept;
  void rt_leave() noexcept;
  /// RT finished update with the adopted plan.
  void rt_updated() noexcept;

private:
  void ring_if_waiting() noexcept;
  bool quiescent(uint64_t generation) const noexcept;
  template <typename Reached>
  bool wait_for(Reached reached, uint32_t seen, Clock::time_point deadline) noexcept;

  std::atomic<const Plan *> published_{nullptr};
  /// Written by RT only. The generation is kept apart, the supervisor may free an old plan as soon
  /// as RT left its last call with it.
  const Plan * rt_adopted_ = nullptr;
  uint64_t rt_generation_ = 0;
  /// Adopted generation shifted by one, in-call flag in bit 0. Written by RT only.
  std::atomic<uint64_t> rt_state_{0};
  /// Written by RT only.
  std::atomic<uint64_t> rt_updated_{0};
  std::atomic<uint32_t> waiting_{0};
  Doorbell & progress_;
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__PLAN_HPP_
