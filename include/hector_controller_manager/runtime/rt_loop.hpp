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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__RT_LOOP_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__RT_LOOP_HPP_

#include <atomic>

#include "hector_controller_manager/runtime/doorbell.hpp"
#include "hector_controller_manager/runtime/plan.hpp"
#include "hector_controller_manager/runtime/thread_role.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace hector_controller_manager::runtime
{
/// The control cycle. It only reads the adopted plan and the runtime atomics, so it neither
/// allocates nor takes a blocking lock, and no exception escapes. A failure stops its scope in the
/// same cycle and rings the supervisor, which does the rest off RT.
class RtLoop
{
public:
  RtLoop(PlanSlot & slot, Doorbell & supervisor, Clock::duration hang_timeout);

  void read(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept;
  void update(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept;
  void write(const rclcpp::Time & time, const rclcpp::Duration & period) noexcept;

  /// Any thread. While engaged, update stops every controller of the plan the e-stop stops.
  void set_estop(bool engaged) noexcept { estop_.store(engaged, std::memory_order_release); }
  bool estop() const noexcept { return estop_.load(std::memory_order_acquire); }

  /// Seconds the last read, update and write took together.
  double cycle_seconds() const noexcept { return cycle_seconds_.load(std::memory_order_relaxed); }

private:
  void stop(const Plan & plan, const Plan::FailureScope & scope) noexcept;
  void check_hardware(const Plan & plan) noexcept;

  PlanSlot & slot_;
  Doorbell & supervisor_;
  const Clock::duration hang_timeout_;
  /// Held during read, update and write.
  ThreadRole rt_{"RT"};
  /// RT only.
  Clock::time_point cycle_start_{};
  /// Written by RT only.
  std::atomic<double> cycle_seconds_{0.0};
  std::atomic<bool> estop_{false};
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__RT_LOOP_HPP_
