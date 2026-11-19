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

#include <chrono>
#include <thread>

#include "gate.hpp"
#include "hector_controller_manager/runtime/plan.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using namespace std::chrono_literals;

/// RT in a call with the plan of generation 1 until `leave_` opens.
class PlanSlotTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    first_.generation = 1;
    second_.generation = 2;
    slot_.publish(&first_);
    leave_.close();
    rt_ = std::thread(
      [this]
      {
        slot_.rt_enter();
        leave_.pass();
        slot_.rt_leave();
      });
    ASSERT_TRUE(leave_.holds());
    slot_.publish(&second_);
  }

  void TearDown() override
  {
    leave_.open();
    rt_.join();
  }

  Doorbell bell_;
  PlanSlot slot_{bell_};
  Plan first_;
  Plan second_;
  testing::Gate leave_;
  std::thread rt_;
};

TEST_F(PlanSlotTest, rt_leaving_its_call_wakes_a_quiescence_waiter)
{
  const uint32_t seen = bell_.value();
  std::thread release(
    [this]
    {
      std::this_thread::sleep_for(50ms);
      leave_.open();
    });
  const auto start = Clock::now();
  slot_.wait_claims_current_or_ring(2, seen, start + 5s);
  EXPECT_LT(Clock::now() - start, 2s);
  EXPECT_TRUE(slot_.wait_quiescent(2, Clock::now()));
  release.join();
}

TEST_F(PlanSlotTest, a_ring_for_something_else_ends_the_wait)
{
  const uint32_t seen = bell_.value();
  bell_.ring();
  const auto start = Clock::now();
  slot_.wait_claims_current_or_ring(2, seen, start + 5s);
  EXPECT_LT(Clock::now() - start, 2s);
  EXPECT_FALSE(slot_.wait_quiescent(2, Clock::now()));
}

// RT records the claims in its update, a write after an update of an older plan still uses them
TEST(PlanSlotClaims, claims_are_current_after_an_update_of_the_plan_or_the_write_after_an_older)
{
  Doorbell bell;
  PlanSlot slot(bell);
  Plan first;
  first.generation = 1;
  Plan second;
  second.generation = 2;
  slot.publish(&first);
  EXPECT_TRUE(slot.wait_claims_current(2, Clock::now())) << "RT has not run";
  slot.rt_enter();
  slot.rt_updated();
  slot.rt_leave();
  slot.publish(&second);
  EXPECT_FALSE(slot.wait_claims_current(2, Clock::now() + 50ms));
  slot.rt_enter();
  slot.rt_leave();
  slot.rt_written();
  EXPECT_TRUE(slot.wait_claims_current(2, Clock::now()));
}

TEST(PlanSlotUpdates, only_an_update_with_the_plan_ends_the_wait)
{
  Doorbell bell;
  PlanSlot slot(bell);
  Plan first;
  first.generation = 1;
  Plan second;
  second.generation = 2;
  slot.publish(&first);
  slot.rt_enter();
  slot.rt_updated();
  slot.rt_leave();
  EXPECT_TRUE(slot.wait_updated(1, Clock::now()));
  slot.publish(&second);
  slot.rt_enter();
  slot.rt_leave();
  EXPECT_FALSE(slot.wait_updated(2, Clock::now() + 50ms)) << "RT adopted the plan, no update yet";
  std::thread rt(
    [&]
    {
      std::this_thread::sleep_for(50ms);
      slot.rt_enter();
      slot.rt_updated();
      slot.rt_leave();
    });
  const auto start = Clock::now();
  EXPECT_TRUE(slot.wait_updated(2, start + 5s));
  EXPECT_LT(Clock::now() - start, 2s);
  rt.join();
}

}  // namespace
}  // namespace hector_controller_manager::runtime
