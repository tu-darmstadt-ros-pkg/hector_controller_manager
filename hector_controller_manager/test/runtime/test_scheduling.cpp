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

// What the control loop and the async owners got of SCHED_FIFO, in the log and the diagnostics,
// on sync and async hardware.

#include <gtest/gtest.h>
#include <sched.h>

#include <atomic>
#include <cerrno>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "log_capture.hpp"
#include "token_rig.hpp"

namespace hector_controller_manager
{
namespace
{
using diagnostic_msgs::msg::DiagnosticStatus;
using namespace testing::tokens;

class Scheduling : public TokenRig
{
protected:
  /// The kernel accepts or refuses SCHED_FIFO, or is no realtime kernel.
  void start_on_kernel(bool realtime_kernel, bool accepted)
  {
    scheduler_.realtime_kernel = [realtime_kernel] { return realtime_kernel; };
    scheduler_.set_policy = [accepted](int policy, int)
    {
      errno = EPERM;
      return accepted && policy == SCHED_FIFO;
    };
    if (accepted)
    {
      scheduler_.in_effect = [] { return std::make_pair(SCHED_FIFO, 50); };
    }
    TokenRig::SetUp();
  }

  void SetUp() override {}

  DiagnosticStatus report()
  {
    diagnostic_updater::DiagnosticStatusWrapper stat;
    supervisor_->report_scheduling(stat);
    return static_cast<const DiagnosticStatus &>(stat);
  }

  static std::string value(const DiagnosticStatus & stat, const std::string & key)
  {
    for (const auto & entry : stat.values)
    {
      if (entry.key == key)
      {
        return entry.value;
      }
    }
    return "missing";
  }

  testing::LogCapture log_;
};

TEST_P(Scheduling, without_a_realtime_kernel_it_logs_once_and_reports_an_error)
{
  start_on_kernel(false, false);
  supervisor_->schedule_control_loop();
  EXPECT_EQ(log_.count("No realtime kernel"), 1u) << log_.text();
  EXPECT_FALSE(log_.contains("Could not enable")) << log_.text();
  const auto stat = report();
  EXPECT_EQ(stat.level, DiagnosticStatus::ERROR);
  EXPECT_NE(
    stat.message.find("The control loop runs without SCHED_FIFO, no realtime kernel"),
    std::string::npos)
    << stat.message;
  EXPECT_EQ(value(stat, "control_loop.scheduling"), "SCHED_OTHER 0, no realtime kernel");
  EXPECT_EQ(
    stat.message.find("The owner of 'system' runs without SCHED_FIFO, no realtime kernel") !=
      std::string::npos,
    GetParam())
    << stat.message;
  EXPECT_EQ(value(stat, "system.scheduling") != "missing", GetParam());
}

TEST_P(Scheduling, a_refused_fifo_on_a_realtime_kernel_is_an_error_with_the_reason)
{
  start_on_kernel(true, false);
  supervisor_->schedule_control_loop();
  const std::string refused =
    "runs without SCHED_FIFO, the kernel refused SCHED_FIFO at priority 50: Operation not "
    "permitted";
  EXPECT_FALSE(log_.contains("No realtime kernel")) << log_.text();
  EXPECT_TRUE(log_.contains(
    "Could not enable FIFO RT scheduling policy for the control loop, the kernel refused "
    "SCHED_FIFO at priority 50: Operation not permitted"))
    << log_.text();
  const auto stat = report();
  EXPECT_EQ(stat.level, DiagnosticStatus::ERROR);
  EXPECT_NE(stat.message.find("The control loop " + refused), std::string::npos) << stat.message;
  const std::string owner =
    "The owner of 'system' runs without SCHED_FIFO, the kernel refused "
    "SCHED_FIFO at priority 49: Operation not permitted";
  EXPECT_EQ(stat.message.find(owner) != std::string::npos, GetParam()) << stat.message;
}

TEST_P(Scheduling, every_thread_with_fifo_is_ok)
{
  start_on_kernel(true, true);
  supervisor_->schedule_control_loop();
  const auto stat = report();
  EXPECT_EQ(stat.level, DiagnosticStatus::OK) << stat.message;
  EXPECT_EQ(
    stat.message, "Every thread of the controller manager that should run SCHED_FIFO runs it");
  EXPECT_NE(value(stat, "control_loop.scheduling"), "missing");
  EXPECT_FALSE(log_.contains("No realtime kernel")) << log_.text();
  EXPECT_FALSE(log_.contains("Could not enable")) << log_.text();
}

// Without privileges the kernel grants SCHED_OTHER, as a set_policy that wrongly reported FIFO
TEST_P(Scheduling, a_thread_that_reads_back_other_than_fifo_is_an_error)
{
  scheduler_.realtime_kernel = [] { return true; };
  scheduler_.set_policy = [](int, int)
  { return runtime::schedule_without_inheritance(SCHED_OTHER, 0); };
  TokenRig::SetUp();  // NOLINT(bugprone-parent-virtual-call) the fixture defers it to the stubs
  std::thread([&] { supervisor_->schedule_control_loop(); }).join();
  const auto stat = report();
  EXPECT_EQ(stat.level, DiagnosticStatus::ERROR) << stat.message;
  EXPECT_EQ(
    value(stat, "control_loop.scheduling"),
    "SCHED_OTHER 0, the kernel accepted SCHED_FIFO but the thread runs SCHED_OTHER");
  EXPECT_EQ(
    value(stat, "system.scheduling") ==
      "SCHED_OTHER 0, the kernel accepted SCHED_FIFO but the thread runs SCHED_OTHER",
    GetParam());
}

TEST_P(Scheduling, asked_to_it_locks_the_memory)
{
  int locks = 0;
  scheduler_.lock_memory = [&locks]
  {
    ++locks;
    return std::make_pair(true, std::string());
  };
  lock_memory_ = true;
  start_on_kernel(true, true);
  supervisor_->schedule_control_loop();
  EXPECT_EQ(locks, 1);
  const auto stat = report();
  EXPECT_EQ(stat.level, DiagnosticStatus::OK) << stat.message;
  EXPECT_EQ(value(stat, "control_loop.memory"), "locked");
}

TEST_P(Scheduling, unasked_it_leaves_the_memory_unlocked)
{
  int locks = 0;
  scheduler_.lock_memory = [&locks]
  {
    ++locks;
    return std::make_pair(true, std::string());
  };
  start_on_kernel(true, true);
  supervisor_->schedule_control_loop();
  EXPECT_EQ(locks, 0);
  const auto stat = report();
  EXPECT_EQ(stat.level, DiagnosticStatus::OK) << stat.message;
  EXPECT_EQ(value(stat, "control_loop.memory"), "not locked, lock_memory is off");
}

TEST_P(Scheduling, a_refused_memory_lock_warns_and_is_an_error)
{
  scheduler_.lock_memory = [] { return std::make_pair(false, std::string("mlockall refused")); };
  lock_memory_ = true;
  start_on_kernel(true, true);
  supervisor_->schedule_control_loop();
  EXPECT_TRUE(log_.contains("Could not lock the memory: mlockall refused")) << log_.text();
  const auto stat = report();
  EXPECT_EQ(stat.level, DiagnosticStatus::ERROR);
  EXPECT_NE(stat.message.find("The memory is not locked, mlockall refused"), std::string::npos)
    << stat.message;
  EXPECT_EQ(value(stat, "control_loop.memory"), "not locked, mlockall refused");
}

TEST_P(Scheduling, the_cores_reach_the_affinity_setter_and_a_refused_one_warns)
{
  std::vector<std::vector<int>> pinned;
  scheduler_.set_affinity = [&pinned](const std::vector<int> & cores)
  {
    pinned.push_back(cores);
    return std::make_pair(false, std::string("core 99 does not exist"));
  };
  cpu_affinity_ = {2, 99};
  start_on_kernel(true, true);
  supervisor_->schedule_control_loop();
  ASSERT_EQ(pinned.size(), 1u);
  EXPECT_EQ(pinned.front(), (std::vector<int>{2, 99}));
  EXPECT_TRUE(log_.contains("Could not pin the control loop to its cores: core 99 does not exist"))
    << log_.text();
}

TEST_P(Scheduling, without_cores_it_pins_nothing)
{
  bool pinned = false;
  scheduler_.set_affinity = [&pinned](const std::vector<int> &)
  {
    pinned = true;
    return std::make_pair(true, std::string());
  };
  start_on_kernel(true, true);
  supervisor_->schedule_control_loop();
  EXPECT_FALSE(pinned);
}

// gz runs the control loop itself, only the owners count
TEST_P(Scheduling, an_embedded_control_loop_is_not_reported)
{
  start_on_kernel(false, false);
  const auto stat = report();
  EXPECT_EQ(value(stat, "control_loop.scheduling"), "run by the embedding process, not reported");
  EXPECT_EQ(stat.message.find("control loop"), std::string::npos) << stat.message;
  EXPECT_EQ(stat.level, GetParam() ? DiagnosticStatus::ERROR : DiagnosticStatus::OK)
    << stat.message;
}

// The diagnostics timer reports while requests publish new snapshots and free the old one. A
// sanitizer build catches a report that reads a freed snapshot.
TEST_P(Scheduling, a_report_during_requests_reads_a_snapshot_it_holds)
{
  start_on_kernel(false, false);
  std::atomic<bool> done{false};
  std::thread diagnostics(
    [&]
    {
      while (!done)
      {
        report();
      }
    });
  const testing::OnExit join(
    [&]
    {
      done = true;
      diagnostics.join();
    });
  for (int i = 0; i < 20; ++i)
  {
    ASSERT_TRUE(supervisor_->load("a", "test/Writer").ok);
    ASSERT_TRUE(supervisor_->unload("a").ok);
  }
}

INSTANTIATE_TEST_SUITE_P(
  Hardware, Scheduling, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "async" : "sync"; });

}  // namespace
}  // namespace hector_controller_manager
