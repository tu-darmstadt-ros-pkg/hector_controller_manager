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

#ifndef HECTOR_CONTROLLER_MANAGER__EXECUTOR_BARRIER_HPP_
#define HECTOR_CONTROLLER_MANAGER__EXECUTOR_BARRIER_HPP_

#include <mutex>
#include <vector>

#include "hector_controller_manager/clock.hpp"
#include "hector_controller_manager/runtime/doorbell.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hector_controller_manager
{
/// Confirms that the executor spinning `node` rebuilt its entities, by a timer created now that
/// fires. Safe from any thread except an executor thread.
class ExecutorBarrier
{
public:
  explicit ExecutorBarrier(rclcpp::Node & node);

  /// False if the timer did not fire by `deadline`.
  bool pass(Clock::time_point deadline);

private:
  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  /// Rung by the timers on an executor thread.
  runtime::Doorbell bell_;
  /// Cancelled timers, destroyed with the barrier after its executor stopped.
  std::mutex mutex_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__EXECUTOR_BARRIER_HPP_
