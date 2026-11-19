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

#include "hector_controller_manager/runtime/owner_thread.hpp"

#include <utility>

namespace hector_controller_manager::runtime
{
namespace
{
constexpr std::chrono::milliseconds STOP_TIMEOUT{500};

enum JobState : uint8_t
{
  RUNNING,
  FINISHED,
  ABANDONED,
};
}  // namespace

class OwnerThread::Job
{
public:
  explicit Job(std::function<void()> work) : work_(std::move(work)) {}

  void run()
  {
    work_();
    uint8_t expected = RUNNING;
    if (!state_.compare_exchange_strong(expected, FINISHED, std::memory_order_acq_rel) && late_)
    {
      late_();
    }
    done_.ring();
  }

  bool wait(Clock::time_point deadline, std::function<void()> late, const WaitInterrupt & interrupt)
  {
    while (state_.load(std::memory_order_acquire) == RUNNING)
    {
      const uint32_t seen = done_.value();
      if (state_.load(std::memory_order_acquire) != RUNNING)
      {
        break;
      }
      const auto until =
        interrupt.give_up ? std::min(deadline, Clock::now() + interrupt.period) : deadline;
      if (
        !done_.wait_until(seen, until) &&
        (until == deadline || Clock::now() >= deadline || interrupt.give_up()))
      {
        late_ = std::move(late);
        uint8_t expected = RUNNING;
        return !state_.compare_exchange_strong(expected, ABANDONED, std::memory_order_acq_rel);
      }
    }
    return true;
  }

private:
  std::function<void()> work_;
  std::function<void()> late_;
  /// The owner moves it to FINISHED and a waiter that gives up to ABANDONED, the first CAS wins.
  /// The waiter sets the late callback before its CAS, so an owner that loses runs it.
  std::atomic<uint8_t> state_{RUNNING};
  Doorbell done_;
};

struct OwnerThread::State
{
  std::function<void()> poll;
  std::mutex mutex;
  std::deque<JobPtr> jobs;
  bool stopping = false;
  Doorbell bell;
  Doorbell exit_bell;
  /// Set by the owner thread when it starts, read by expect_owner from any thread.
  std::atomic<std::thread::id> id;
};

OwnerThread::OwnerThread(std::function<void()> poll) : state_(std::make_shared<State>())
{
  state_->poll = std::move(poll);
  thread_ = std::thread(
    [state = state_]
    {
      state->id = std::this_thread::get_id();
      while (true)
      {
        const uint32_t seen = state->bell.value();
        if (state->poll)
        {
          state->poll();
        }
        JobPtr job;
        {
          std::lock_guard<std::mutex> lock(state->mutex);
          if (state->stopping)
          {
            break;
          }
          if (!state->jobs.empty())
          {
            job = std::move(state->jobs.front());
            state->jobs.pop_front();
          }
        }
        if (job)
        {
          job->run();
          continue;
        }
        state->bell.wait_until(seen, Clock::time_point::max());
      }
      state->exit_bell.ring();
    });
}

OwnerThread::~OwnerThread()
{
  const uint32_t seen = state_->exit_bell.value();
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->stopping = true;
  }
  state_->bell.ring();
  const auto deadline = Clock::now() + STOP_TIMEOUT;
  if (state_->exit_bell.wait_until(seen, deadline))
  {
    thread_.join();
  }
  else
  {
    thread_.detach();
  }
}

OwnerThread::JobPtr OwnerThread::submit(std::function<void()> work)
{
  auto job = std::make_shared<Job>(std::move(work));
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->jobs.push_back(job);
  }
  state_->bell.ring();
  return job;
}

bool OwnerThread::wait(
  const JobPtr & job, Clock::time_point deadline, std::function<void()> late,
  const WaitInterrupt & interrupt)
{
  return job->wait(deadline, std::move(late), interrupt);
}

void OwnerThread::wake() noexcept { state_->bell.ring(); }

bool OwnerThread::is_owner_thread() const
{
  return std::this_thread::get_id() == state_->id.load();
}

}  // namespace hector_controller_manager::runtime
