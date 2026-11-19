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
#include "hector_controller_manager/runtime/fault_latch.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using core::HardwareFault;

TEST(FaultLatch, a_second_fault_of_a_generation_keeps_the_first_kind_and_reason)
{
  FaultLatch latch;
  EXPECT_EQ(latch.get(1), HardwareFault::None);
  latch.raise(1, HardwareFault::Deactivate, {"read", " returned DEACTIVATE"});
  latch.raise(1, HardwareFault::Error, {"write returned ERROR"});
  EXPECT_EQ(latch.get(1), HardwareFault::Deactivate);
  EXPECT_EQ(latch.reason(), "read returned DEACTIVATE");
}

TEST(FaultLatch, a_fault_of_an_older_generation_reads_as_none_and_a_new_one_latches)
{
  FaultLatch latch;
  latch.raise(1, HardwareFault::Error, {"old"});
  EXPECT_EQ(latch.get(2), HardwareFault::None);
  latch.raise(2, HardwareFault::Deactivate, {"new"});
  EXPECT_EQ(latch.get(2), HardwareFault::Deactivate);
  EXPECT_EQ(latch.get(1), HardwareFault::None);
  EXPECT_EQ(latch.reason(), "new");
}

/// The payload is plain, so under TSan a raise that does not publish what came before it is a
/// reported race.
TEST(FaultLatch, a_reader_that_sees_the_fault_sees_its_reason_and_what_came_before)
{
  FaultLatch latch;
  int payload = 0;
  std::thread writer(
    [&]
    {
      payload = 42;
      latch.raise(1, HardwareFault::Error, {"bus ", "4711"});
    });
  const bool raised = testing::spin_until([&] { return latch.get(1) != HardwareFault::None; });
  const int seen = raised ? payload : 0;
  const std::string reason = latch.reason();
  writer.join();
  ASSERT_TRUE(raised) << "the fault never showed";
  EXPECT_EQ(seen, 42);
  EXPECT_EQ(reason, "bus 4711");
}

}  // namespace
}  // namespace hector_controller_manager::runtime
