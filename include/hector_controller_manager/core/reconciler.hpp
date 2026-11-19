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

#ifndef HECTOR_CONTROLLER_MANAGER__CORE__RECONCILER_HPP_
#define HECTOR_CONTROLLER_MANAGER__CORE__RECONCILER_HPP_

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "hector_controller_manager/core/model.hpp"

/// The reconciler decides which single safe step moves the actual state toward the desired one and
/// how its outcome changes the actual state. Pure, the supervisor executes the steps.
namespace hector_controller_manager::core
{
using InterfacesByHardware = std::map<std::string, std::vector<std::string>>;

namespace step
{
/// The hardware holds these interfaces through perform(∅, stop).
struct Hold
{
  InterfacesByHardware interfaces;
};
/// Block the writes of these components, so their live interfaces can change the claimer without
/// a mode switch.
struct Take
{
  Names hardware;
};
/// Publish a plan without these controllers and wait until RT no longer runs them.
struct Detach
{
  Names controllers;
};
struct Deactivate
{
  std::string controller;
};
enum class HardwareTransition : uint8_t
{
  Configure,
  Activate,
  Deactivate,
  Cleanup,
  /// on_error after a fault.
  Error,
  Shutdown,
};
struct Hardware
{
  std::string hardware;
  HardwareTransition transition;
};
struct SetChained
{
  std::string controller;
  bool chained;
};
/// Assign the claimed interfaces, then on_activate.
struct Activate
{
  std::string controller;
  Claims claims;
};
/// Publish a plan with these controllers.
struct Attach
{
  Names controllers;
};
/// Wait until RT updated the attached controllers once. Only RT progress completes it, a stalled
/// loop leaves the step pending, never failed, and the hardware holds meanwhile.
struct AwaitUpdate
{
  Names controllers;
};
/// One perform(start, stop) per component, none if both are empty, then its writes resume. Starts
/// interfaces whose claimers updated in their current activation, and stops the live interfaces of
/// a taken component that have no such claimer.
struct SwitchModes
{
  std::map<std::string, ModeSwitch> modes;
};
}  // namespace step

/// Actual equals desired.
struct Done
{
};
/// Only a quarantined callback or an unreachable desired state is left.
struct Blocked
{
  std::string reason;
};

using Step = std::variant<
  Done, Blocked, step::Hold, step::Take, step::Detach, step::Deactivate, step::Hardware,
  step::SetChained, step::Activate, step::Attach, step::AwaitUpdate, step::SwitchModes>;

/// A short text of the step for logs.
std::string describe(const Step & step);

/// The next step, in the priority order hold, detach, deactivate, hardware down, chained mode,
/// hardware up, activate, attach, await update, start.
Step next_step(const Model & actual, const Desired & desired);

/// Whether nothing is left to do and desired is reached except for quarantined entities and a
/// component taken for one. Only their late returns can change them, and a switch that touches them
/// is rejected before it starts.
bool reached_apart_from_quarantine(const Model & actual, const Desired & desired);

enum class CallResult : uint8_t
{
  Ok,
  /// The callback returned FAILURE, or a controller refused to stop.
  Failure,
  /// The callback returned ERROR or threw.
  Error,
  /// The callback outlived its deadline. It still runs, the entity is quarantined.
  TimedOut,
};

struct Outcome
{
  CallResult result = CallResult::Ok;
  /// The primary state a returned lifecycle callback ended in.
  Lifecycle state = Lifecycle::Unconfigured;
  /// Hardware whose mode switch or NaN write failed or outlived the deadline.
  Names faulted_hardware;
  /// Interfaces a SwitchModes did not start because their claimer failed meanwhile.
  Names skipped;
  /// RT did not update the controllers of an AwaitUpdate before the deadline.
  bool not_updated = false;
  /// Faulted hardware whose call outlived the deadline and still runs. apply quarantines it.
  Names timed_out_hardware;
  /// Why the call failed, as the plugin or the runtime reported it.
  std::string reason;
};

/// The outcome of a lifecycle call that returned.
inline Outcome call_outcome(CallResult result, Lifecycle state, std::string reason = {})
{
  Outcome outcome;
  outcome.result = result;
  outcome.state = state;
  outcome.reason = std::move(reason);
  return outcome;
}

/// Adds `text` to a failure reason, separated by a semicolon.
inline void append_reason(std::string & reason, const std::string & text)
{
  reason += (reason.empty() ? "" : "; ") + text;
}

struct Failures
{
  /// Controllers and hardware that failed and cannot run as they did before.
  Names unusable;
  /// Controllers that refused a chained mode. They still work in the mode they have.
  Names refused;

  bool empty() const { return unusable.empty() && refused.empty(); }
};

/// Applies the outcome of an executed step to the actual state. A failure also shrinks desired by
/// the failure closure, so the reconciler never retries a failed step.
Failures apply(Model & actual, Desired & desired, const Step & step, const Outcome & outcome);

/// Calls given up at their deadline that still run. Each quarantines its entity until it returns.
struct GivenUp
{
  Names controllers;
  Names hardware;
};

/// A given up call returned at last. Each releases one quarantine of its entity.
namespace returned
{
/// A lifecycle callback, which left the controller in this state.
struct ControllerCallback
{
  std::string controller;
  Lifecycle state;
};
/// A lifecycle callback, which left the component in this state.
struct HardwareCallback
{
  std::string hardware;
  Lifecycle state;
};
/// The chained mode the plugin reached, if it applied the change.
struct ChainedMode
{
  std::string controller;
  std::optional<bool> chained;
};
/// RT left every call that may still have run the controllers after their Detach, and a mode
/// switch or NaN write of the hardware returned.
struct Released
{
  Names controllers;
  Names hardware;
};
}  // namespace returned

using Returned = std::variant<
  returned::ControllerCallback, returned::HardwareCallback, returned::ChainedMode,
  returned::Released>;

void apply(Model & actual, const GivenUp & given_up);
void apply(Model & actual, const Returned & returned);

/// RT stopped these controllers or faulted this hardware. Desired drops what can no longer run.
void apply_failures(
  Model & actual, Desired & desired, const Names & failed_controllers,
  const std::map<std::string, HardwareFault> & faulted_hardware);

/// Removes what cannot run any more from desired, the failed seeds and their stop closure in the
/// desired graph. Never adds anything.
void shrink(const Model & actual, Desired & desired, const Names & seeds);

/// Invariants every actual state reached by the steps satisfies. Returns violations.
std::vector<std::string> check_invariants(const Model & actual);

}  // namespace hector_controller_manager::core

#endif  // HECTOR_CONTROLLER_MANAGER__CORE__RECONCILER_HPP_
