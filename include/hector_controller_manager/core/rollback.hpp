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

#ifndef HECTOR_CONTROLLER_MANAGER__CORE__ROLLBACK_HPP_
#define HECTOR_CONTROLLER_MANAGER__CORE__ROLLBACK_HPP_

#include "hector_controller_manager/core/model.hpp"

namespace hector_controller_manager::core
{
/// Where a failed switch goes back to, the desired state before it without what failed for good,
/// what runs on hardware that became unusable, and what a quarantined plugin may still touch. A
/// controller that failed in its new activation comes back in the one it had before. Hardware
/// targets stay as `current` holds them.
Desired plan_rollback(
  const Model & model, const Desired & before, const Desired & current, const Names & unusable);

/// Keeps only what already runs in the activation `desired` asks for, so reaching it only stops.
Desired keep_running(const Model & model, const Desired & desired);

}  // namespace hector_controller_manager::core

#endif  // HECTOR_CONTROLLER_MANAGER__CORE__ROLLBACK_HPP_
