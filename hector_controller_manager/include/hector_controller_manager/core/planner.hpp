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

#ifndef HECTOR_CONTROLLER_MANAGER__CORE__PLANNER_HPP_
#define HECTOR_CONTROLLER_MANAGER__CORE__PLANNER_HPP_

#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "hector_controller_manager/core/model.hpp"

namespace hector_controller_manager::core
{
enum class Strictness : uint8_t
{
  Strict,
  Auto,
  ForceAuto,
};

struct SwitchRequest
{
  Names activate;
  Names deactivate;
  Strictness strictness = Strictness::Strict;
};

struct SwitchPlan
{
  Desired desired;
  /// Deactivated by the switch, restarts included.
  Names stopped;
  /// Activated by the switch, restarts included.
  Names started;
  /// Full lists for the validation prepare, by hardware.
  std::map<std::string, ModeSwitch> validation;
};

struct Rejection
{
  std::string reason;
};

/// Computes the desired state of a switch once, before anything executes, or rejects it. Its
/// controllers must be settled. The hardware targets are kept as given, so a pending fault keeps
/// its target.
std::variant<SwitchPlan, Rejection> plan_switch(
  const Model & model, const std::map<std::string, Lifecycle> & hardware,
  const SwitchRequest & request);

/// What an AUTO switch that activates the controller starts, the controller and every provider it
/// needs that is not running, transitively. Rejects like that switch if the controller or a needed
/// provider cannot start.
std::variant<Names, Rejection> auto_start_set(const Model & model, const std::string & controller);

struct StartupSelection
{
  Names activate;
  /// "'name' (reason)" for each controller left out.
  std::vector<std::string> skipped;
};

/// What the startup's AUTO switch activates of `wanted`. It leaves out a controller that is
/// protected on a restore, an unprotected command claimer while the e-stop is engaged by
/// `estop_cause`, one whose joint or reference a running controller claims, and one that cannot
/// start, itself or through a needed provider. An active controller is left alone, an unknown one
/// is kept for the switch to reject.
StartupSelection select_startup(
  const Model & model, const std::vector<std::string> & wanted, bool restore,
  const std::optional<std::string> & estop_cause);

/// Resolves the claims of a controller against the interfaces available to the given active set.
/// Rejects a missing or unusable interface, unless it is a best effort state interface.
std::variant<Claims, Rejection> resolve_claims(
  const Model & model, const InterfaceIndex & index, const std::string & controller,
  const Names & active);

/// Restarts every running provider desired keeps chained while a writer of its references leaves,
/// together with its restart scope, so no provider keeps a departed writer's last reference. A
/// writer counts if it is active now or was desired in `before`, so one already stopped by the
/// failed switch still counts. A switch plans this itself, a rollback needs it on top.
void restart_for_departed_writers(const Model & model, const Desired & before, Desired & desired);

/// Resolves again the claims of the controllers desired still has to start, so a best effort reader
/// reads what is available now. Returns those whose claims cannot be resolved any more.
Names resolve_starting(const Model & model, Desired & desired);

}  // namespace hector_controller_manager::core

#endif  // HECTOR_CONTROLLER_MANAGER__CORE__PLANNER_HPP_
