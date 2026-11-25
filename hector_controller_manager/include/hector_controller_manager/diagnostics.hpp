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

#ifndef HECTOR_CONTROLLER_MANAGER__DIAGNOSTICS_HPP_
#define HECTOR_CONTROLLER_MANAGER__DIAGNOSTICS_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <shared_mutex>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "hector_controller_manager/supervisor.hpp"
#include "rclcpp/node.hpp"

namespace hector_controller_manager
{
/// The diagnostics of the controller manager on its executor threads.
///
/// - "Controller Manager Activity" summarizes the node, the e-stop and the cycle time, lists each
///   entity's state and the cycle counters of each component.
/// - A counter that grew since the last period is logged once per period.
/// - "Realtime Scheduling" is a task of its own, so an ERROR off a realtime kernel does not mask
///   the e-stop's WARN.
/// - Each task reads the supervisor only under a shared `callbacks_mutex` while `closed` is false.
class Diagnostics
{
public:
  Diagnostics(
    rclcpp::Node & node, Supervisor & supervisor, std::shared_mutex & callbacks_mutex,
    const bool & closed, const std::atomic<bool> & initialization_failed);

private:
  void activity(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void add_hardware(
    diagnostic_updater::DiagnosticStatusWrapper & stat, const HardwareStatus & hardware);
  /// How much a counter grew since the last call with the same key.
  uint64_t growth(const std::string & key, uint64_t count);
  /// Warns while an async component skips cycles, unless its URDF turns print_warnings off.
  void report_overruns(const HardwareStatus & hardware);
  /// Adds the command limit counters and logs what changed since the last call.
  void report_command_limits(
    diagnostic_updater::DiagnosticStatusWrapper & stat, const HardwareStatus & hardware);

  Supervisor & supervisor_;
  std::shared_mutex & callbacks_mutex_;
  const bool & closed_;
  const std::atomic<bool> & initialization_failed_;
  const rclcpp::Logger logger_;
  /// Counters last reported, by component and counter.
  std::map<std::string, uint64_t> reported_;
  /// Last, its timer runs the tasks that use the members above.
  diagnostic_updater::Updater updater_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__DIAGNOSTICS_HPP_
