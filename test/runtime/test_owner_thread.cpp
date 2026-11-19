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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "gate.hpp"
#include "hector_controller_manager/runtime/owner_thread.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using namespace std::chrono_literals;

TEST(OwnerThread, a_job_that_outlives_a_wait_without_a_late_callback_returns_quietly)
{
  testing::Gate gate;
  OwnerThread owner;
  gate.close();
  const auto held = owner.submit([&] { gate.pass(); });
  ASSERT_TRUE(gate.holds());
  EXPECT_FALSE(owner.wait(held, Clock::now()));
  gate.open();
  const auto next = owner.submit([] {});
  EXPECT_TRUE(owner.wait(next, Clock::now() + 5s)) << "the owner thread runs no more jobs";
}

std::atomic<bool> thread_ended{false};

/// Ends after the thread's own code, so only a join waits for it.
struct MarkEnd
{
  ~MarkEnd()
  {
    std::this_thread::sleep_for(20ms);
    thread_ended = true;
  }
};

TEST(OwnerThread, the_destructor_joins_a_thread_that_stops_in_time)
{
  thread_ended = false;
  {
    OwnerThread owner([] { thread_local MarkEnd mark; });
  }
  EXPECT_TRUE(thread_ended) << "the thread still runs after the destructor returned";
}

}  // namespace
}  // namespace hector_controller_manager::runtime
