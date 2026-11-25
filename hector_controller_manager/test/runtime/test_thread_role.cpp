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

#include "fakes.hpp"
#include "hector_controller_manager/runtime/controller_runtime.hpp"
#include "hector_controller_manager/runtime/hardware_runtime.hpp"
#include "hector_controller_manager/runtime/io_gate.hpp"
#include "hector_controller_manager/runtime/owner_cycle.hpp"
#include "hector_controller_manager/runtime/rt_loop.hpp"
#include "hector_controller_manager/runtime/shadow_handles.hpp"
#include "hector_controller_manager/runtime/stop_batch.hpp"
#include "hector_controller_manager/runtime/thread_role.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
class ThreadRoleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!CHECKED_BUILD)
    {
      GTEST_SKIP() << "thread roles are checked in debug and sanitizer builds";
    }
    GTEST_FLAG_SET(death_test_style, "threadsafe");
  }
};

template <typename Call>
void on_other_thread(Call call)
{
  std::thread(call).join();
}

TEST_F(ThreadRoleTest, only_the_bound_thread_acts_in_a_role)
{
  ThreadRole role("test");
  on_other_thread([&] { role.expect(); });
  role.bind();
  role.expect();
  EXPECT_DEATH(on_other_thread([&] { role.expect(); }), "Called off the test thread");
  role.unbind();
  on_other_thread([&] { role.expect(); });
}

TEST_F(ThreadRoleTest, a_call_in_a_role_may_come_from_any_thread_but_never_overlaps)
{
  ThreadRole role("test");
  {
    const ThreadRole::Call call(role);
  }
  on_other_thread([&] { const ThreadRole::Call call(role); });
  const ThreadRole::Call held(role);
  EXPECT_DEATH(
    on_other_thread([&] { const ThreadRole::Call call(role); }), "Called off the test thread");
}

// gz runs write from one of its threads, read and update from another
TEST_F(ThreadRoleTest, rt_cycles_may_come_from_changing_threads)
{
  Doorbell wake;
  PlanSlot slot(wake);
  RtLoop loop(slot, wake, std::chrono::milliseconds(100));
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  const rclcpp::Duration period(0, 0);
  loop.write(time, period);
  on_other_thread(
    [&]
    {
      loop.read(time, period);
      loop.update(time, period);
    });
  loop.write(time, period);
}

TEST_F(ThreadRoleTest, owner_jobs_run_only_on_the_owner)
{
  HardwareRuntime hardware(testing::hardware_info("hw"), [](const auto &, auto) {});
  EXPECT_DEATH(
    hardware.transition(core::step::HardwareTransition::Configure), "Called off the owner thread");
  ControllerRuntime controller("c", "test/Controller", std::make_shared<IoGate>());
  EXPECT_DEATH(controller.set_chained(true), "Called off the owner thread");
}

TEST(Precondition, only_a_checked_build_evaluates_it)
{
  bool evaluated = false;
  expect_precondition(
    [&]
    {
      evaluated = true;
      return true;
    },
    "never broken");
  EXPECT_EQ(evaluated, CHECKED_BUILD);
}

using PreconditionTest = ThreadRoleTest;

TEST_F(PreconditionTest, a_second_cycle_never_starts_while_one_is_outstanding)
{
  OwnerCycle cycle;
  cycle.rt_start(false);
  EXPECT_DEATH(cycle.rt_start(true), "OwnerCycle::rt_start while a cycle is outstanding");
}

TEST_F(PreconditionTest, the_gate_is_left_only_by_who_entered_it)
{
  IoGate gate;
  EXPECT_DEATH(gate.rt_leave(), "IoGate::rt_leave without RT in");
  EXPECT_DEATH(gate.owner_release(), "IoGate::owner_release without an owner in");
}

TEST_F(PreconditionTest, a_stop_batch_is_reset_only_before_use)
{
  StopBatch stops;
  stops.reset(1);
  stops.rt_request(0);
  stops.rt_publish();
  EXPECT_DEATH(stops.reset(1), "StopBatch::reset after RT or the owner used it");
}

TEST_F(PreconditionTest, shadow_handles_are_set_up_once)
{
  HardwareStats stats;
  ShadowHandles shadows(stats);
  const auto info = testing::hardware_info("hw");
  const auto state = std::make_shared<hardware_interface::StateInterface>("j1", "position");
  shadows.init(info, {state}, {}, false);
  EXPECT_DEATH(shadows.init(info, {state}, {}, false), "ShadowHandles::init twice");
}

}  // namespace
}  // namespace hector_controller_manager::runtime
