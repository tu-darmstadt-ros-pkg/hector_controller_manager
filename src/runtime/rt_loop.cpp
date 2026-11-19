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

#include "hector_controller_manager/runtime/rt_loop.hpp"

#include "hector_controller_manager/runtime/controller_runtime.hpp"
#include "hector_controller_manager/runtime/hardware_runtime.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
/// Leaves the plan slot on every return path.
class Call
{
public:
  explicit Call(PlanSlot & slot) : slot_(slot), plan_(slot.rt_enter()) {}
  ~Call() { slot_.rt_leave(); }
  const Plan * plan() const { return plan_; }

private:
  PlanSlot & slot_;
  const Plan * plan_;
};
}  // namespace

RtLoop::RtLoop(PlanSlot & slot, Doorbell & supervisor, Clock::duration hang_timeout)
: slot_(slot), supervisor_(supervisor), hang_timeout_(hang_timeout)
{
}

void RtLoop::stop(const Plan & plan, const Plan::FailureScope & scope) noexcept
{
  for (uint32_t i = scope.controllers.begin; i < scope.controllers.end; ++i)
  {
    const auto & member = plan.controllers[plan.scope_controllers[i]];
    member.runtime->failed_activation.store(member.activation, std::memory_order_release);
  }
  for (uint32_t i = scope.holds.begin; i < scope.holds.end; ++i)
  {
    plan.scope_holds[i].hardware->rt_request_stop(plan.scope_holds[i].interface);
  }
  supervisor_.ring();
}

void RtLoop::check_hardware(const Plan & plan) noexcept
{
  const auto now = Clock::now();
  for (const auto & hardware : plan.hardware)
  {
    if (hardware.in_use)
    {
      hardware.runtime->rt_check_hang(now, hang_timeout_);
    }
    if (hardware.runtime->rt_take_fault(plan.generation))
    {
      stop(plan, hardware.scope);
    }
  }
}

void RtLoop::read(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept
{
  const ThreadRole::Call in_rt(rt_);
  cycle_start_ = Clock::now();
  const Call call(slot_);
  if (!call.plan())
  {
    return;
  }
  for (const auto & hardware : call.plan()->hardware)
  {
    if (!hardware.runtime->is_async())
    {
      hardware.runtime->rt_read(time, period);
    }
  }
  check_hardware(*call.plan());
}

void RtLoop::update(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept
{
  const ThreadRole::Call in_rt(rt_);
  const Call call(slot_);
  if (!call.plan())
  {
    return;
  }
  const Plan & plan = *call.plan();
  const bool estop = estop_.load(std::memory_order_acquire);
  for (const auto & controller : plan.controllers)
  {
    const bool failed = controller.runtime->failed_activation.load(std::memory_order_relaxed) ==
                        controller.activation;
    if (!failed && estop && controller.estop)
    {
      controller.runtime->estop_activation.store(controller.activation, std::memory_order_relaxed);
      stop(plan, controller.scope);
    }
    else if (!failed && !controller.runtime->rt_update(time, period))
    {
      controller.runtime->failed_update.store(controller.activation, std::memory_order_relaxed);
      stop(plan, controller.scope);
    }
  }
  slot_.rt_updated();
}

void RtLoop::write(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept
{
  const ThreadRole::Call in_rt(rt_);
  {
    const Call call(slot_);
    if (call.plan())
    {
      for (const auto & hardware : call.plan()->hardware)
      {
        if (!hardware.runtime->is_async())
        {
          hardware.runtime->rt_write(time, period);
        }
      }
      // After the sync faults stopped their scope, so the async owners see its stops first
      check_hardware(*call.plan());
      for (const auto & hardware : call.plan()->hardware)
      {
        if (hardware.runtime->is_async())
        {
          hardware.runtime->rt_trigger(time, period);
        }
      }
    }
  }
  cycle_seconds_.store(
    std::chrono::duration<double>(Clock::now() - cycle_start_).count(), std::memory_order_relaxed);
}

}  // namespace hector_controller_manager::runtime
