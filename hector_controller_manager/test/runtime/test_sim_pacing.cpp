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
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include "gate.hpp"
#include "hector_controller_manager/node.hpp"
#include "rcl/time.h"
#include "rclcpp/logging.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace std::chrono_literals;

constexpr int64_t MS = 1'000'000;

/// The control loop on a ROS clock that the test sets, as /clock would.
class SimPacing : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(rcl_enable_ros_time_override(clock_.get_clock_handle()), RCL_RET_OK);
    loop_ = std::thread(
      [this]
      {
        run_on_ros_clock(
          clock_, 20ms, [this] { return !stop_; },
          [this](const rclcpp::Time &, const rclcpp::Duration & period)
          {
            period_ns_ = period.nanoseconds();
            cycles_.fetch_add(1);
          });
      });
  }

  void TearDown() override
  {
    stop_ = true;
    loop_.join();
  }

  /// Under the clock mutex, as rclcpp's TimeSource sets the time from /clock.
  void set_ms(int64_t ms)
  {
    std::lock_guard<std::mutex> lock(clock_.get_clock_mutex());
    ASSERT_EQ(rcl_set_ros_time_override(clock_.get_clock_handle(), ms * MS), RCL_RET_OK);
  }

  /// The first /clock, then time for the loop to take it as the start of its schedule.
  void start_at_ms(int64_t ms)
  {
    set_ms(ms);
    std::this_thread::sleep_for(100ms);
  }

  /// Whether the cycle count reaches `count`.
  bool cycles_reach(int count)
  {
    return testing::eventually([&] { return cycles_ >= count; });
  }

  rclcpp::Clock clock_{RCL_ROS_TIME};
  std::atomic<bool> stop_{false};
  std::atomic<int> cycles_{0};
  std::atomic<int64_t> period_ns_{0};
  std::thread loop_;
};

TEST_F(SimPacing, nothing_runs_before_the_clock_starts_then_one_cycle_per_period)
{
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(cycles_, 0) << "the clock has not started";
  // gz ran for minutes before the first /clock
  start_at_ms(600'000);
  EXPECT_EQ(cycles_, 0) << "the first cycle waits one period";
  const auto begin = std::chrono::steady_clock::now();
  for (int i = 1; i <= 50; ++i)
  {
    for (int step = 1; step <= 4; ++step)
    {
      set_ms(600'000 + (i - 1) * 20 + step * 5);
    }
    ASSERT_TRUE(cycles_reach(i));
    EXPECT_EQ(period_ns_, 20 * MS);
  }
  EXPECT_EQ(cycles_, 50) << "one advanced sim second";
  // Each tick wakes the loop, the 100 ms stop check alone would take about 5 s
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 2500ms);
}

TEST_F(SimPacing, a_paused_clock_stops_the_cycles)
{
  start_at_ms(1'000);
  set_ms(1'020);
  ASSERT_TRUE(cycles_reach(1));
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(cycles_, 1);
  set_ms(1'040);
  EXPECT_TRUE(cycles_reach(2));
}

TEST_F(SimPacing, a_jump_neither_bursts_nor_freezes_the_loop)
{
  start_at_ms(1'000);
  set_ms(1'020);
  ASSERT_TRUE(cycles_reach(1));
  set_ms(11'020);
  ASSERT_TRUE(cycles_reach(2));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(cycles_, 2) << "no catch up after a forward jump";
  EXPECT_EQ(period_ns_, 10'000 * MS) << "the measured period";
  // A world reset
  set_ms(500);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(cycles_, 2);
  set_ms(520);
  EXPECT_TRUE(cycles_reach(3)) << "the schedule restarted at the reset";
  EXPECT_EQ(period_ns_, 20 * MS);
}

// A sync read that stalls on a bus timeout
TEST(SteadyPacing, an_overrun_skips_the_missed_cycles_and_gets_the_measured_period)
{
  constexpr int STALLED = 3;
  std::vector<std::chrono::steady_clock::time_point> starts;
  std::vector<int64_t> periods;
  std::chrono::steady_clock::time_point stall_end;
  run_on_steady_clock(
    10ms, [&] { return starts.size() < 20; },
    [&](const rclcpp::Time &, const rclcpp::Duration & period)
    {
      starts.push_back(std::chrono::steady_clock::now());
      periods.push_back(period.nanoseconds());
      if (starts.size() == STALLED)
      {
        std::this_thread::sleep_for(105ms);
        stall_end = std::chrono::steady_clock::now();
      }
    },
    rclcpp::get_logger("steady_pacing"));
  EXPECT_GE(periods[STALLED], 105 * MS) << "the measured period";
  int within_30ms = 0;
  for (const auto & start : starts)
  {
    within_30ms += start >= stall_end && start - stall_end <= 30ms ? 1 : 0;
  }
  // Each cycle after the stall runs at a point of the schedule after it, three fit into 30 ms
  EXPECT_LE(within_30ms, 3) << "the missed cycles ran in a burst";
}

}  // namespace
}  // namespace hector_controller_manager
