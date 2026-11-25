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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__SCHEDULING_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__SCHEDULING_HPP_

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace hector_controller_manager::runtime
{
/// Sets the policy and priority of the calling thread, threads it creates start with SCHED_OTHER.
/// False if the kernel refused it.
bool schedule_without_inheritance(int policy, int priority) noexcept;

/// The policy without SCHED_RESET_ON_FORK and the priority of the calling thread, read from the
/// kernel since glibc's pthread_getschedparam may report a stale cached policy.
std::pair<int, int> scheduling_in_effect() noexcept;

/// What a thread that should run SCHED_FIFO asks of the kernel, tests replace it.
struct Scheduler
{
  std::function<bool()> realtime_kernel = realtime_tools::has_realtime_kernel;
  /// Sets errno when it fails. Without inheritance, so a plugin's executor or bus thread does not
  /// take the priority and delay the thread that started it.
  std::function<bool(int policy, int priority)> set_policy = schedule_without_inheritance;
  std::function<std::pair<int, int>()> in_effect = scheduling_in_effect;
  /// Each returns whether it succeeded and else the reason.
  std::function<std::pair<bool, std::string>()> lock_memory = realtime_tools::lock_memory;
  std::function<std::pair<bool, std::string>(const std::vector<int> & cores)> set_affinity =
    [](const std::vector<int> & cores)
  { return realtime_tools::set_current_thread_affinity(cores); };
};

/// The policy and priority a thread runs with after it asked for SCHED_FIFO.
struct Scheduling
{
  int policy = 0;
  int priority = 0;
  /// Why the thread runs without SCHED_FIFO, empty only if the read back policy is SCHED_FIFO.
  std::string problem;
};

/// Asks for SCHED_FIFO at `priority` on a realtime kernel and reads back what is in effect.
Scheduling schedule_fifo(int priority, const Scheduler & scheduler);

/// "SCHED_FIFO 49", followed by the problem if there is one.
std::string to_string(const Scheduling & scheduling);

/// Reports the scheduling of the control loop and of the owner of each async component, by name.
/// OK, or ERROR naming every one that runs without SCHED_FIFO. Without `control_loop` an
/// embedding process runs the control loop, then only the owners count.
void report_scheduling(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  const std::optional<Scheduling> & control_loop,
  const std::vector<std::pair<std::string, Scheduling>> & owners);
}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__SCHEDULING_HPP_
