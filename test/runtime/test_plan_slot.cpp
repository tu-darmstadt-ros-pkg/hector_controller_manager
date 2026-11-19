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
#include "hector_controller_manager/runtime/plan.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using namespace std::chrono_literals;

/// RT in a call with the plan of generation 1 until `leave` is set.
class PlanSlotTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    first_.generation = 1;
    second_.generation = 2;
    slot_.publish(&first_);
    rt_ = std::thread(
      [this]
      {
        slot_.rt_enter();
        entered_ = true;
        testing::eventually([this] { return leave_.load(); });
        slot_.rt_leave();
      });
    ASSERT_TRUE(testing::eventually([this] { return entered_.load(); }));
    slot_.publish(&second_);
  }

  void TearDown() override
  {
    leave_ = true;
    rt_.join();
  }

  Doorbell bell_;
  PlanSlot slot_{bell_};
  Plan first_;
  Plan second_;
  std::atomic<bool> entered_{false};
  std::atomic<bool> leave_{false};
  std::thread rt_;
};

TEST_F(PlanSlotTest, rt_leaving_its_call_wakes_a_quiescence_waiter)
{
  const uint32_t seen = bell_.value();
  std::thread release(
    [this]
    {
      std::this_thread::sleep_for(50ms);
      leave_ = true;
    });
  const auto start = Clock::now();
  slot_.wait_quiescent_or_ring(2, seen, start + 5s);
  EXPECT_LT(Clock::now() - start, 2s);
  EXPECT_TRUE(slot_.wait_quiescent(2, Clock::now()));
  release.join();
}

TEST_F(PlanSlotTest, a_ring_for_something_else_ends_the_wait)
{
  const uint32_t seen = bell_.value();
  bell_.ring();
  const auto start = Clock::now();
  slot_.wait_quiescent_or_ring(2, seen, start + 5s);
  EXPECT_LT(Clock::now() - start, 2s);
  EXPECT_FALSE(slot_.wait_quiescent(2, Clock::now()));
}

}  // namespace
}  // namespace hector_controller_manager::runtime
