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

#include "hector_controller_manager/executor_barrier.hpp"

#include <atomic>
#include <memory>
#include <utility>

namespace hector_controller_manager
{
ExecutorBarrier::ExecutorBarrier(rclcpp::Node & node)
: node_(node), group_(node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
}

bool ExecutorBarrier::pass(Clock::time_point deadline)
{
  // Each call has its own flag set by its timer, so another call's timer confirms nothing
  auto fired = std::make_shared<std::atomic<bool>>(false);
  auto timer = node_.create_wall_timer(
    std::chrono::nanoseconds(0),
    [this, fired]
    {
      fired->store(true);
      bell_.ring();
    },
    group_);
  // The bell is read before the flag, so a ring between the two is not missed
  for (uint32_t rung = bell_.value(); !fired->load() && bell_.wait_until(rung, deadline);
       rung = bell_.value())
  {
  }
  const bool confirmed = fired->load();
  timer->cancel();
  {
    // An executor thread may hold the last reference otherwise, and destroy the timer while another
    // barrier's creation frees what is left of it
    std::lock_guard<std::mutex> lock(mutex_);
    timers_.push_back(std::move(timer));
  }
  return confirmed;
}

}  // namespace hector_controller_manager
