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

#include <thread>

#include "gate.hpp"
#include "hector_controller_manager/runtime/cycle_stamp.hpp"
#include "hector_controller_manager/runtime/owner_cycle.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using Kind = OwnerCycle::Kind;

TEST(OwnerCycle, a_started_cycle_is_outstanding_until_the_owner_completes_it)
{
  OwnerCycle cycle;
  EXPECT_EQ(cycle.pending(), Kind::None);
  cycle.rt_start(true);
  EXPECT_EQ(cycle.pending(), Kind::Write);
  cycle.owner_complete();
  EXPECT_EQ(cycle.pending(), Kind::None);
  cycle.rt_start(false);
  EXPECT_EQ(cycle.pending(), Kind::Read);
}

TEST(CycleStamp, holds_the_last_time_and_period)
{
  CycleStamp stamp;
  stamp.rt_store(5, 2);
  stamp.rt_store(7, 3);
  EXPECT_EQ(stamp.load().time_ns, 7);
  EXPECT_EQ(stamp.load().period_ns, 3);
}

TEST(OwnerCycle, a_dropped_write_still_reads_and_a_drop_starts_nothing)
{
  OwnerCycle cycle;
  cycle.owner_drop_write();
  EXPECT_EQ(cycle.pending(), Kind::None);
  cycle.rt_start(true);
  cycle.owner_drop_write();
  EXPECT_EQ(cycle.pending(), Kind::Read);
  cycle.owner_drop_write();
  EXPECT_EQ(cycle.pending(), Kind::Read);
}

/// RT hands a plain command and its stamp to the owner with each cycle and reads the owner's plain
/// state once it completed. Under TSan a missing order in either direction is a reported race.
TEST(OwnerCycle, rt_and_the_owner_hand_over_what_a_cycle_uses)
{
  constexpr int CYCLES = 5000;
  OwnerCycle cycle;
  CycleStamp stamp;
  int command = 0;
  int state = 0;
  int stale = 0;
  std::thread owner(
    [&]
    {
      for (int served = 0; served < CYCLES; ++served)
      {
        if (!testing::spin_until([&] { return cycle.pending() != Kind::None; }))
        {
          return;
        }
        stale += stamp.load().time_ns != command ? 1 : 0;
        state = command;
        cycle.owner_complete();
      }
    });
  int unseen = 0;
  int completed = 0;
  for (int i = 1; i <= CYCLES; ++i)
  {
    command = i;
    stamp.rt_store(i, 1);
    cycle.rt_start(true);
    if (!testing::spin_until([&] { return cycle.pending() == Kind::None; }))
    {
      break;
    }
    unseen += state != i ? 1 : 0;
    ++completed;
  }
  owner.join();
  EXPECT_EQ(completed, CYCLES) << "the owner stopped completing cycles";
  EXPECT_EQ(stale, 0);
  EXPECT_EQ(unseen, 0);
}

}  // namespace
}  // namespace hector_controller_manager::runtime
