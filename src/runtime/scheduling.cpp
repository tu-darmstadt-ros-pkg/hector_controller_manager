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

#include "hector_controller_manager/runtime/scheduling.hpp"

#include <sched.h>

#include <cerrno>
#include <cstring>
#include <tuple>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
std::string policy_name(int policy)
{
  switch (policy)
  {
    case SCHED_FIFO:
      return "SCHED_FIFO";
    case SCHED_RR:
      return "SCHED_RR";
    case SCHED_OTHER:
      return "SCHED_OTHER";
    default:
      return "policy " + std::to_string(policy);
  }
}
}  // namespace

bool schedule_without_inheritance(int policy, int priority) noexcept
{
  sched_param param{};
  param.sched_priority = priority;
  return sched_setscheduler(0, policy | SCHED_RESET_ON_FORK, &param) == 0;
}

std::pair<int, int> scheduling_in_effect() noexcept
{
  sched_param param{};
  sched_getparam(0, &param);
  return {sched_getscheduler(0) & ~SCHED_RESET_ON_FORK, param.sched_priority};
}

Scheduling schedule_fifo(int priority, const Scheduler & scheduler)
{
  Scheduling result;
  if (!scheduler.realtime_kernel())
  {
    result.problem = "no realtime kernel";
  }
  else if (!scheduler.set_policy(SCHED_FIFO, priority))
  {
    const int error = errno;
    result.problem = "the kernel refused SCHED_FIFO at priority " + std::to_string(priority) +
                     ": " + std::strerror(error);
  }
  std::tie(result.policy, result.priority) = scheduler.in_effect();
  if (result.problem.empty() && result.policy != SCHED_FIFO)
  {
    result.problem =
      "the kernel accepted SCHED_FIFO but the thread runs " + policy_name(result.policy);
  }
  return result;
}

std::string to_string(const Scheduling & scheduling)
{
  const std::string text =
    policy_name(scheduling.policy) + " " + std::to_string(scheduling.priority);
  return scheduling.problem.empty() ? text : text + ", " + scheduling.problem;
}

void report_scheduling(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  const std::optional<Scheduling> & control_loop,
  const std::vector<std::pair<std::string, Scheduling>> & owners)
{
  const auto report =
    [&stat](const std::string & thread, const std::string & key, const Scheduling & scheduling)
  {
    stat.add(key, to_string(scheduling));
    if (!scheduling.problem.empty())
    {
      stat.mergeSummary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        thread + " runs without SCHED_FIFO, " + scheduling.problem);
    }
  };
  stat.summary(
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    "Every thread of the controller manager that should run SCHED_FIFO runs it");
  if (control_loop)
  {
    report("The control loop", "control_loop.scheduling", *control_loop);
  }
  else
  {
    stat.add("control_loop.scheduling", "run by the embedding process, not reported");
  }
  for (const auto & [name, scheduling] : owners)
  {
    report("The owner of '" + name + "'", name + ".scheduling", scheduling);
  }
}
}  // namespace hector_controller_manager::runtime
