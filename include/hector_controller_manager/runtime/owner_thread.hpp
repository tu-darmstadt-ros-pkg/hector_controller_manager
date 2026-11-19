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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__OWNER_THREAD_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__OWNER_THREAD_HPP_

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "hector_controller_manager/runtime/doorbell.hpp"
#include "hector_controller_manager/runtime/thread_role.hpp"

namespace hector_controller_manager::runtime
{
/// Ends a wait for a job before its deadline once `give_up` holds, checked every `period`.
struct WaitInterrupt
{
  std::function<bool()> give_up;
  Clock::duration period{};
};

/// The one thread that calls into a plugin, apart from RT. Jobs run in submission order. A caller
/// waits for a job with a deadline. A job that outlives it keeps running, and its `late` callback
/// runs on this thread when it returns.
class OwnerThread
{
public:
  class Job;
  using JobPtr = std::shared_ptr<Job>;

  /// `poll` runs on this thread after every wakeup, before the next job.
  explicit OwnerThread(std::function<void()> poll = {});
  /// Stops with a short deadline. A thread stuck in a job is left running, it owns everything it
  /// references, so nothing is destroyed under it.
  ~OwnerThread();

  OwnerThread(const OwnerThread &) = delete;
  OwnerThread & operator=(const OwnerThread &) = delete;

  JobPtr submit(std::function<void()> work);
  /// True if the job finished by the deadline. Otherwise `late` runs when it finishes.
  [[nodiscard]] bool wait(
    const JobPtr & job, Clock::time_point deadline, std::function<void()> late = {},
    const WaitInterrupt & interrupt = {});
  /// Wakes the thread for `poll`, safe on RT.
  void wake() noexcept;
  bool is_owner_thread() const;
  /// Aborts in checked builds unless called from a job of this thread.
  void expect_owner() const noexcept { expect_thread(is_owner_thread(), "owner"); }

private:
  struct State;
  std::shared_ptr<State> state_;
  std::thread thread_;
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__OWNER_THREAD_HPP_
