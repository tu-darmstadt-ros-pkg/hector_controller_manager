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

#include <algorithm>
#include <atomic>
#include <optional>
#include <thread>
#include <vector>

#include "gate.hpp"
#include "hector_controller_manager/runtime/stop_batch.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using Interfaces = std::vector<uint32_t>;

TEST(StopBatch, the_owner_takes_a_published_batch_once)
{
  StopBatch stops;
  stops.reset(3);
  Interfaces requested;
  stops.rt_request(0);
  stops.rt_request(2);
  EXPECT_FALSE(stops.owner_take(requested)) << "the batch is not published";
  EXPECT_FALSE(stops.rt_pending());
  EXPECT_TRUE(stops.rt_publish());
  EXPECT_FALSE(stops.rt_publish()) << "nothing new to publish";
  EXPECT_TRUE(stops.rt_pending());
  EXPECT_TRUE(stops.pending());
  const auto total = stops.owner_take(requested);
  ASSERT_TRUE(total);
  EXPECT_EQ(requested, (Interfaces{0, 2}));
  stops.owner_done(*total);
  EXPECT_FALSE(stops.rt_pending());
  EXPECT_FALSE(stops.pending());
  requested.clear();
  EXPECT_FALSE(stops.owner_take(requested));
  EXPECT_TRUE(requested.empty());
}

TEST(StopBatch, the_owner_takes_no_part_of_a_batch_rt_is_still_writing)
{
  StopBatch stops;
  stops.reset(3);
  Interfaces requested;
  stops.rt_request(0);
  stops.rt_publish();
  stops.rt_request(1);
  EXPECT_FALSE(stops.owner_take(requested));
  EXPECT_TRUE(requested.empty());
  stops.rt_request(2);
  stops.rt_publish();
  const auto total = stops.owner_take(requested);
  ASSERT_TRUE(total);
  EXPECT_EQ(requested, (Interfaces{0, 1, 2}));
  stops.owner_done(*total);
  // A second request of an interface since the last batch is new again
  stops.rt_request(1);
  stops.rt_publish();
  requested.clear();
  ASSERT_TRUE(stops.owner_take(requested));
  EXPECT_EQ(requested, Interfaces{1});
}

/// RT stops two interfaces per batch and waits for the hold before the next one, as its writes
/// skip meanwhile. The payloads are plain, so under TSan a missing order is a reported race.
TEST(StopBatch, the_owner_holds_every_batch_whole_and_rt_sees_each_hold)
{
  constexpr int BATCHES = 2000;
  StopBatch stops;
  stops.reset(4);
  int sent = 0;
  int held = 0;
  int torn = 0;
  std::thread owner(
    [&]
    {
      Interfaces requested;
      for (int taken = 0; taken < BATCHES; ++taken)
      {
        std::optional<uint32_t> total;
        const auto take = [&]
        {
          requested.clear();
          total = stops.owner_take(requested);
          return total.has_value();
        };
        if (!testing::spin_until(take))
        {
          return;
        }
        const uint32_t first = static_cast<uint32_t>(sent % 2) * 2;
        torn += requested != Interfaces{first, first + 1} ? 1 : 0;
        held = sent;
        stops.owner_done(*total);
      }
    });
  int unseen = 0;
  int sent_batches = 0;
  for (int batch = 1; batch <= BATCHES; ++batch)
  {
    sent = batch;
    const uint32_t first = static_cast<uint32_t>(batch % 2) * 2;
    stops.rt_request(first);
    stops.rt_request(first + 1);
    stops.rt_publish();
    if (!testing::spin_until([&] { return !stops.rt_pending(); }))
    {
      break;
    }
    unseen += held != batch ? 1 : 0;
    ++sent_batches;
  }
  owner.join();
  EXPECT_EQ(sent_batches, BATCHES) << "the owner stopped holding";
  EXPECT_EQ(torn, 0);
  EXPECT_EQ(unseen, 0);
}

/// RT starts the next batch before the owner held the last one and pauses inside it, so the owner
/// takes while a batch is half written. It must never hold one interface of a pair without the
/// other.
TEST(StopBatch, an_owner_that_takes_during_a_half_written_batch_holds_no_part_of_it)
{
  constexpr int BATCHES = 20000;
  StopBatch stops;
  stops.reset(4);
  std::atomic<bool> finished{false};
  int torn = 0;
  int takes = 0;
  std::thread owner(
    [&]
    {
      Interfaces requested;
      while (!finished.load())
      {
        requested.clear();
        const auto total = stops.owner_take(requested);
        if (!total)
        {
          std::this_thread::yield();
          continue;
        }
        for (const uint32_t first : {0u, 2u})
        {
          const auto has = [&](uint32_t i)
          { return std::find(requested.begin(), requested.end(), i) != requested.end(); };
          torn += has(first) != has(first + 1) ? 1 : 0;
        }
        ++takes;
        stops.owner_done(*total);
      }
    });
  for (int batch = 1; batch <= BATCHES; ++batch)
  {
    const uint32_t first = static_cast<uint32_t>(batch % 2) * 2;
    stops.rt_request(first);
    std::this_thread::yield();
    stops.rt_request(first + 1);
    stops.rt_publish();
  }
  const bool held = testing::spin_until([&] { return !stops.rt_pending(); });
  finished = true;
  owner.join();
  EXPECT_TRUE(held) << "the owner never held the last batch";
  EXPECT_GT(takes, 0) << "precondition: the owner took batches";
  EXPECT_EQ(torn, 0);
}

}  // namespace
}  // namespace hector_controller_manager::runtime
