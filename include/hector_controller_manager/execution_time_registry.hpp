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

#ifndef HECTOR_CONTROLLER_MANAGER__EXECUTION_TIME_REGISTRY_HPP_
#define HECTOR_CONTROLLER_MANAGER__EXECUTION_TIME_REGISTRY_HPP_

#include <functional>
#include <mutex>
#include <set>
#include <string>

#include "hector_controller_manager/supervisor.hpp"

namespace hector_controller_manager
{
/// The execution times on `~/statistics`, sampled off RT from atomics. Safe from any thread.
class ExecutionTimeRegistry
{
public:
  /// `cycle_seconds` samples the control cycle, registered as `total`.
  ExecutionTimeRegistry(std::string total, std::function<double()> cycle_seconds);

  /// Registers the cycle and every controller and component not registered yet.
  void add(const Status & status);
  /// Unregisters everything, before what the samples read is gone.
  void remove_all();

private:
  const std::string total_;
  const std::function<double()> cycle_seconds_;
  std::mutex mutex_;
  std::set<std::string> registered_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__EXECUTION_TIME_REGISTRY_HPP_
