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

#include "hector_controller_manager/core/rollback.hpp"

#include "hector_controller_manager/core/chain_rules.hpp"
#include "hector_controller_manager/core/planner.hpp"
#include "hector_controller_manager/core/reconciler.hpp"

namespace hector_controller_manager::core
{
Desired plan_rollback(
  const Model & model, const Desired & before, const Desired & current, const Names & unusable)
{
  Desired rollback = before;
  rollback.hardware = current.hardware;
  const InterfaceIndex index(model);
  Names seeds;
  for (const auto & name : unusable)
  {
    if (model.hardware.count(name))
    {
      for (const auto & [controller, want] : before.active)
      {
        if (
          claimers_of_hardware(model, index, name).count(controller) ||
          claims_hardware(index, want.claims, name))
        {
          seeds.insert(controller);
        }
      }
    }
    else if (!before.active.count(name))
    {
      seeds.insert(name);
    }
  }
  for (const auto & [name, controller] : model.controllers)
  {
    if (controller.quarantined || controller.failed)
    {
      seeds.insert(name);
    }
    // A quarantined plugin may still write the references it claimed
    if (controller.quarantined)
    {
      seeds.merge(providers_written(index, controller.claims));
    }
  }
  shrink(model, rollback, seeds);
  restart_for_departed_writers(model, before, rollback);
  // What starts again reads the best effort states available now, which may differ from before
  shrink(model, rollback, resolve_starting(model, rollback));
  return rollback;
}

Desired keep_running(const Model & model, const Desired & desired)
{
  Desired settled = desired;
  Names missing;
  for (const auto & [name, want] : desired.active)
  {
    const auto & controller = model.controllers.at(name);
    if (
      !is_running_member(controller) || controller.activation < want.min_activation ||
      !controller.running)
    {
      missing.insert(name);
    }
  }
  shrink(model, settled, missing);
  return settled;
}

}  // namespace hector_controller_manager::core
