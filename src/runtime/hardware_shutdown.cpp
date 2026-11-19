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

#include "hector_controller_manager/runtime/hardware_shutdown.hpp"

#include <utility>
#include <vector>

namespace hector_controller_manager::runtime
{
std::map<std::string, std::optional<core::Lifecycle>> shut_down_all(
  const std::map<std::string, HardwareRuntime *> & hardware, Clock::time_point deadline)
{
  using core::Lifecycle;
  std::vector<std::pair<std::string, OwnerThread::JobPtr>> stopping;
  for (const auto & [name, component] : hardware)
  {
    stopping.emplace_back(
      name, component->owner().submit(
              [component = component]
              {
                if (component->state() == Lifecycle::Active)
                {
                  component->transition(core::step::HardwareTransition::Deactivate);
                }
                if (component->state() != Lifecycle::Finalized)
                {
                  component->transition(core::step::HardwareTransition::Shutdown);
                }
              }));
  }
  // A transition publishes its state before it returns
  std::map<std::string, std::optional<Lifecycle>> result;
  for (const auto & [name, job] : stopping)
  {
    auto * component = hardware.at(name);
    result.emplace(
      name, component->owner().wait(job, deadline, [] {})
              ? std::optional<Lifecycle>(component->state())
              : std::nullopt);
  }
  return result;
}

}  // namespace hector_controller_manager::runtime
