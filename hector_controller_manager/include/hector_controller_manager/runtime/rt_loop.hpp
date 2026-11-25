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
#include <cstdint>
#include <memory>

#include "hector_controller_manager/runtime/doorbell.hpp"
#include "hector_controller_manager/runtime/io_gate.hpp"
#include "hector_controller_manager/runtime/plan.hpp"
#include "hector_controller_manager/runtime/thread_role.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace hector_controller_manager
{
class Introspection;
}  // namespace hector_controller_manager

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

  /// Any thread. Every update while engaged and the first one after an engage stop every controller
  /// of the plan the e-stop stops, so an engage shorter than a cycle stops them too. An engage
  /// while no plan is published sets no pending bit, as no controller ran that it could miss.
  void set_estop(bool engaged) noexcept
  {
    if (engaged)
    {
      // The first plan holds no controller, so a publish racing this check starts nothing
      const uint8_t bits = slot_.has_plan() ? ENGAGED | PENDING : ENGAGED;
      estop_.fetch_or(bits, std::memory_order_release);
    }
    else
    {
      estop_.fetch_and(static_cast<uint8_t>(~ENGAGED), std::memory_order_release);
    }
  }
  bool estop() const noexcept { return estop_.load(std::memory_order_acquire) & ENGAGED; }
  /// Any thread. Every later update stops what an engage stops, without engaging, until a restart.
  void stop_command_claimers() noexcept { estop_.fetch_or(HELD, std::memory_order_release); }

  /// Seconds the last read, update and write took together.
  double cycle_seconds() const noexcept { return cycle_seconds_.load(std::memory_order_relaxed); }
  /// Any thread. How long RT has been in the cycle its last read opened, zero between cycles.
  Clock::duration open_cycle(Clock::time_point now) const noexcept
  {
    const Clock::rep start = cycle_start_.load(std::memory_order_relaxed);
    return start == 0 ? Clock::duration::zero() : now.time_since_epoch() - Clock::duration(start);
  }

  /// Off RT before the first plan, while no owner runs a plugin call. The registry queues only as
  /// many enables and disables as values are registered and applies them only when it samples.
  /// Skipped while another manager lives, whose writers may run.
  void flush_introspection();

  /// Held by a controller's owner for every lifecycle call, which may write what the controller
  /// registered for introspection.
  const std::shared_ptr<IoGate> & introspection_gate() const { return introspection_gate_; }

private:
  void stop(const Plan & plan, const Plan::FailureScope & scope) noexcept;
  void check_hardware(const Plan & plan) noexcept;
  /// Samples the introspection registry while RT holds every writer of it off, else skips.
  void sample_introspection(const Plan & plan) noexcept;

  PlanSlot & slot_;
  Doorbell & supervisor_;
  const Clock::duration hang_timeout_;
  /// Held during read, update and write.
  ThreadRole rt_{"RT"};
  /// The steady time of the read that opened the cycle, 0 after its write. Written by RT, sampled
  /// by the diagnostics.
  std::atomic<Clock::rep> cycle_start_{0};
  /// Written by RT only, sampled by the diagnostics.
  std::atomic<double> cycle_seconds_{0.0};
  static constexpr uint8_t ENGAGED = 1;
  /// Set by an engage while a plan is published, cleared by the update that acts on it.
  static constexpr uint8_t PENDING = 2;
  /// Set by `stop_command_claimers`, never cleared, so a plan published later is stopped too.
  static constexpr uint8_t HELD = 4;
  /// Released by the e-stop listener and the supervisor, acquired by RT at every update, by the
  /// latch and by the diagnostics. RT publishes nothing through it.
  std::atomic<uint8_t> estop_{0};
  /// Null while no controller manager joined it.
  Introspection * const introspection_;
  const std::shared_ptr<IoGate> introspection_gate_ = std::make_shared<IoGate>();
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__RT_LOOP_HPP_
