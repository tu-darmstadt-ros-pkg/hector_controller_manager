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

#include "hector_controller_manager/diagnostics.hpp"

#include <cinttypes>
#include <mutex>

#include "rclcpp/logging.hpp"

namespace hector_controller_manager
{
Diagnostics::Diagnostics(
  rclcpp::Node & node, Supervisor & supervisor, std::shared_mutex & callbacks_mutex,
  const bool & closed, const std::atomic<bool> & initialization_failed)
: supervisor_(supervisor),
  callbacks_mutex_(callbacks_mutex),
  closed_(closed),
  initialization_failed_(initialization_failed),
  logger_(node.get_logger()),
  updater_(&node)
{
  updater_.setHardwareID("ros2_control");
  updater_.add(
    "Controller Manager Activity",
    [this](diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
      const std::shared_lock lock(callbacks_mutex_);
      if (!closed_)
      {
        activity(stat);
      }
    });
  updater_.add(
    "Realtime Scheduling",
    [this](diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
      const std::shared_lock lock(callbacks_mutex_);
      if (!closed_)
      {
        supervisor_.report_scheduling(stat);
      }
    });
}

void Diagnostics::activity(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto status = supervisor_.status();
  if (status->initialized)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
  }
  else if (initialization_failed_)
  {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Initialization failed, the node needs a restart");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for the robot description");
  }
  if (supervisor_.rt().estop())
  {
    stat.mergeSummary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "E-stop engaged by " + supervisor_.estop_cause());
  }
  // A stuck cycle runs no update, so nothing stops, not even on an e-stop. gz writes in the step
  // after its read and update, so an embedded loop holds a cycle open across a whole gz step.
  if (
    const auto open = supervisor_.rt().open_cycle(Clock::now());
    supervisor_.runs_control_loop() && open > supervisor_.config().hang_timeout)
  {
    stat.mergeSummary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "The control loop is stuck in a cycle for " +
        std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(open).count()) +
        " ms, nothing stops meanwhile, also not on an e-stop");
  }
  stat.add("cycle_time_s", supervisor_.rt().cycle_seconds());
  for (const auto & controller : status->controllers)
  {
    stat.add(controller.name, to_string(controller.model.state));
  }
  for (const auto & hardware : status->hardware)
  {
    stat.add(hardware.name, to_string(hardware.model.state));
    add_hardware(stat, hardware);
  }
}

void Diagnostics::add_hardware(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const HardwareStatus & hardware)
{
  stat.add(hardware.name + ".skipped_cycles", hardware.runtime->stats().skipped());
  report_command_limits(stat, hardware);
  report_overruns(hardware);
}

uint64_t Diagnostics::growth(const std::string & key, uint64_t count)
{
  auto & reported = reported_[key];
  const uint64_t grown = count - reported;
  reported = count;
  return grown;
}

void Diagnostics::report_overruns(const HardwareStatus & hardware)
{
  const uint64_t skipped = growth(hardware.name + ".skipped", hardware.runtime->stats().skipped());
  if (hardware.info.is_async && hardware.info.async_params.print_warnings && skipped > 0)
  {
    RCLCPP_WARN(
      logger_,
      "The asynchronous component '%s' skipped %" PRIu64
      " cycles since the last report, its owner thread was busy",
      hardware.name.c_str(), skipped);
  }
}

void Diagnostics::report_command_limits(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const HardwareStatus & hardware)
{
  const uint64_t clamped = hardware.runtime->stats().clamped();
  const uint64_t out_of_range = hardware.runtime->stats().out_of_range_cycles();
  stat.add(hardware.name + ".clamped_cycles", clamped);
  stat.add(hardware.name + ".out_of_range_cycles", out_of_range);
  // Logged once per diagnostics period at most, RT only counts
  if (const uint64_t grown = growth(hardware.name + ".clamped", clamped); grown > 0)
  {
    RCLCPP_WARN(
      logger_,
      "The command limits of '%s' clamped %" PRIu64 " cycles since the last report, last '%s'",
      hardware.name.c_str(), grown, hardware.runtime->last_clamped().c_str());
  }
  if (const uint64_t grown = growth(hardware.name + ".out_of_range", out_of_range); grown > 0)
  {
    RCLCPP_WARN(
      logger_,
      "A joint of '%s' was outside its position limits for %" PRIu64
      " cycles since the last report, its position commands may only move back",
      hardware.name.c_str(), grown);
  }
}

}  // namespace hector_controller_manager
