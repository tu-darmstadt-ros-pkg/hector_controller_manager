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
#include <thread>

#include "hector_controller_manager/runtime/io_gate.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
/// RT and the owner both increment a plain counter inside the gate. Under TSan a missing
/// happens-before between them is a reported race, and the inside flag catches overlap directly.
TEST(IoGate, rt_and_owner_never_overlap)
{
  IoGate gate;
  uint64_t counter = 0;
  std::atomic<int> inside{0};
  std::atomic<bool> overlap{false};
  std::atomic<bool> stop{false};
  // Relaxed, so it adds no happens-before that only the gate may provide
  std::atomic<uint64_t> rt_entries{0};
  const auto critical = [&]
  {
    if (inside.fetch_add(1) != 0)
    {
      overlap = true;
    }
    ++counter;
    inside.fetch_sub(1);
  };
  std::thread rt(
    [&]
    {
      while (!stop.load())
      {
        if (gate.rt_enter())
        {
          critical();
          rt_entries.fetch_add(1, std::memory_order_relaxed);
          gate.rt_leave();
        }
      }
    });
  uint64_t owner_entries = 0;
  // RT gets in only between a release and the next acquire, so the owner goes on until it did
  const auto rt_deadline = Clock::now() + std::chrono::seconds(10);
  for (int i = 0;
       i < 20000 || (rt_entries.load(std::memory_order_relaxed) == 0 && Clock::now() < rt_deadline);
       ++i)
  {
    ASSERT_TRUE(gate.owner_acquire(Clock::now() + std::chrono::seconds(5)));
    critical();
    ++owner_entries;
    gate.owner_release();
  }
  stop = true;
  rt.join();
  EXPECT_FALSE(overlap.load());
  EXPECT_EQ(counter, rt_entries.load() + owner_entries);
  EXPECT_GT(rt_entries.load(), 0u);
}

TEST(IoGate, owner_gives_up_while_rt_stays_inside)
{
  IoGate gate;
  ASSERT_TRUE(gate.rt_enter());
  EXPECT_FALSE(gate.owner_acquire(Clock::now() + std::chrono::milliseconds(20)));
  // The owner gave up, so RT is not locked out
  gate.rt_leave();
  EXPECT_TRUE(gate.rt_enter());
  gate.rt_leave();
}

TEST(IoGate, rt_skips_while_the_owner_holds_it)
{
  IoGate gate;
  ASSERT_TRUE(gate.owner_acquire(Clock::now() + std::chrono::milliseconds(20)));
  EXPECT_FALSE(gate.rt_enter());
  gate.owner_release();
  EXPECT_TRUE(gate.rt_enter());
  gate.rt_leave();
}

TEST(IoGate, rt_skips_until_the_last_of_several_owners_released_it)
{
  IoGate gate;
  ASSERT_TRUE(gate.owner_acquire(Clock::now() + std::chrono::milliseconds(20)));
  ASSERT_TRUE(gate.owner_acquire(Clock::now() + std::chrono::milliseconds(20)));
  gate.owner_release();
  EXPECT_FALSE(gate.rt_enter());
  gate.owner_release();
  EXPECT_TRUE(gate.rt_enter());
  gate.rt_leave();
}

}  // namespace
}  // namespace hector_controller_manager::runtime
