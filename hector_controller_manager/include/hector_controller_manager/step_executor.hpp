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

#ifndef HECTOR_CONTROLLER_MANAGER__STEP_EXECUTOR_HPP_
#define HECTOR_CONTROLLER_MANAGER__STEP_EXECUTOR_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/core/reconciler.hpp"
#include "hector_controller_manager/entities.hpp"
#include "hector_controller_manager/inbox.hpp"
#include "hector_controller_manager/runtime/owner_thread.hpp"
#include "hector_controller_manager/runtime/plan.hpp"
#include "hector_controller_manager/runtime/thread_role.hpp"

namespace hector_controller_manager
{
/// Runs the reconciler's steps on the owner threads and hands plans to RT. Supervisor thread
/// only. Writes the model only through core::apply, for calls it gives up on or that return late.
class StepExecutor
{
public:
  StepExecutor(
    const runtime::ThreadRole & supervisor, core::Model & model, const Entities & entities,
    runtime::PlanSlot & slot, std::shared_ptr<Inbox> inbox)
  : supervisor_(supervisor),
    model_(model),
    entities_(entities),
    slot_(slot),
    inbox_(std::move(inbox))
  {
  }

  /// `interrupt` ends the wait for a hardware lifecycle call early.
  core::Outcome execute(
    const core::Step & step, Clock::time_point deadline,
    const runtime::WaitInterrupt & interrupt = {});
  /// Runs `call` on the owner. A call that outlives the deadline runs `undo` with its outcome on
  /// the owner when it returns, then posts `late` with the state it ended in.
  core::Outcome lifecycle_call(
    runtime::OwnerThread & owner, std::function<core::Outcome()> call, Clock::time_point deadline,
    std::function<void(core::Lifecycle)> late, std::function<void(core::Outcome &)> undo = {},
    const runtime::WaitInterrupt & interrupt = {});
  /// Publishes the plan of the model and waits until RT runs no older one. False at the deadline.
  bool publish_plan(const core::Model & model, Clock::time_point deadline);
  /// Drops the plans RT can no longer use, and the runtimes only they keep alive. False if RT is
  /// still in a call with an older plan at the deadline.
  bool release_old_plans(Clock::time_point deadline);
  /// Releases the controllers detached under a stuck RT call once no RT write can use the claims of
  /// a plan that ran them. True if it did.
  bool release_quiescent();
  bool awaiting_quiescence() const { return !awaiting_quiescence_.second.empty(); }
  /// Until RT left the call or the write that keeps the detached controllers quarantined, or the
  /// doorbell rang after `seen` for something else.
  void wait_quiescent_or_ring(uint32_t seen);
  /// Posts the return of a mode switch job of the hardware that outlived its deadline.
  std::function<void()> late_mode_switch(const std::string & hardware) const;
  /// Ends RT's use of plans. False if RT is still in a call at the deadline, then the plans it
  /// may use are leaked.
  bool retire(Clock::time_point deadline);

private:
  /// After `stopped` returned from its deactivation, writes the NaN of every taken component no
  /// other running claimer writes any more, once (section "NaN write" of the design).
  void write_released(
    const std::string & stopped, core::Outcome & outcome, Clock::time_point deadline);
  /// One mode switch job per component, all in parallel. Writes of a component resume once its
  /// job returned in time, if `give_back`.
  core::Outcome switch_hardware(
    const std::map<std::string, core::ModeSwitch> & modes,
    const std::function<bool(const std::string &)> & give_back, Clock::time_point deadline);
  /// Takes the controllers out of RT's plan, quarantined until RT is quiescent if it is stuck.
  void detach_controllers(const core::step::Detach & detach, Clock::time_point deadline);
  /// Writes the NaN of the released interfaces after on_deactivate.
  core::Outcome deactivate_controller(
    const core::step::Deactivate & deactivate, Clock::time_point deadline);
  /// Waits until every hardware component the claims read states from read them since its last
  /// transition. Returns the quoted names of those that did not by the deadline.
  std::string await_fresh_states(const core::Claims & claims, Clock::time_point deadline);
  /// A failed activation writes the NaN of the released interfaces, a late success is undone.
  core::Outcome activate_controller(
    const core::step::Activate & activate, Clock::time_point deadline);
  /// The model takes the chained mode the plugin reached, also late.
  core::Outcome set_chained(const core::step::SetChained & chained, Clock::time_point deadline);

  const runtime::ThreadRole & supervisor_;
  core::Model & model_;
  const Entities & entities_;
  runtime::PlanSlot & slot_;
  std::shared_ptr<Inbox> inbox_;
  std::vector<std::shared_ptr<runtime::Plan>> plans_;
  uint64_t generation_ = 0;
  /// Controllers detached while RT was stuck in a call, quarantined until RT is quiescent.
  std::pair<uint64_t, core::Names> awaiting_quiescence_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__STEP_EXECUTOR_HPP_
