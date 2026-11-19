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
#include <pthread.h>
#include <sched.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fakes.hpp"
#include "gate.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/runtime/hardware_runtime.hpp"
#include "hector_controller_manager/runtime/rt_loop.hpp"
#include "log_capture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_environment.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using hardware_interface::return_type;

/// What the plugin's next calls return, the test sets it.
struct Script
{
  std::atomic<return_type> read{return_type::OK};
  std::atomic<return_type> write{return_type::OK};
  std::atomic<bool> throw_on_read{false};
  std::atomic<return_type> prepare{return_type::OK};
  std::atomic<return_type> perform{return_type::OK};
  testing::Gate perform_gate;
  testing::Gate prepare_gate;
  testing::Gate deactivate_gate;
  std::atomic<int> writes{0};
  std::vector<std::string> stopped;
};

/// A system whose read can be held, like a driver stuck on its bus.
class BlockingSystem : public hardware_interface::SystemInterface
{
public:
  BlockingSystem(testing::Gate & read_gate, std::atomic<int> & switches, Script & script)
  : read_gate_(read_gate), switches_(switches), script_(script)
  {
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    read_gate_.pass();
    if (script_.throw_on_read)
    {
      throw std::runtime_error("bus 4711");
    }
    return script_.read;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    script_.writes.fetch_add(1);
    return script_.write;
  }

  return_type prepare_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override
  {
    script_.prepare_gate.pass();
    return script_.prepare;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    script_.deactivate_gate.pass();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  return_type perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> & stop) override
  {
    script_.stopped = stop;
    switches_.fetch_add(1);
    script_.perform_gate.pass();
    return script_.perform;
  }

private:
  testing::Gate & read_gate_;
  std::atomic<int> & switches_;
  Script & script_;
};

/// Completes a cycle every few milliseconds, slower than one RT period.
class SlowBus : public hardware_interface::SystemInterface
{
public:
  explicit SlowBus(std::atomic<int> & cycles) : cycles_(cycles) {}
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    cycles_.fetch_add(1);
    return return_type::OK;
  }
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    return return_type::OK;
  }

private:
  std::atomic<int> & cycles_;
};

/// Writes the number of its read into the position of j1, waits at the gate and then writes it into
/// j2 and j3, like a bus that transfers one joint after the other.
class NumberedReads : public hardware_interface::SystemInterface
{
public:
  NumberedReads(testing::Gate & between_joints, std::atomic<int> & finished)
  : between_joints_(between_joints), finished_(finished)
  {
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    ++reads_;
    set_state("j1/position", reads_);
    between_joints_.pass();
    set_state("j2/position", reads_);
    set_state("j3/position", reads_);
    finished_.fetch_add(1);
    return return_type::OK;
  }
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return return_type::OK;
  }

private:
  testing::Gate & between_joints_;
  std::atomic<int> & finished_;
  double reads_ = 0.0;
};

/// An async component with the positions of j1, j2 and j3, read by NumberedReads, in a plan RT
/// runs.
struct NumberedRig
{
  NumberedRig()
  {
    auto info = testing::hardware_info("numbered", true);
    for (const char * name : {"j2", "j3"})
    {
      auto joint = info.joints.front();
      joint.name = name;
      joint.command_interfaces.clear();
      info.joints.push_back(joint);
    }
    hardware = std::make_shared<HardwareRuntime>(info, [](const auto &, auto) {});
    EXPECT_TRUE(
      testing::bring_up(
        *hardware, [&] { return std::make_unique<NumberedReads>(between_joints, finished); }));
    plan.generation = 1;
    plan.hardware.push_back({hardware.get(), false, {}, {{}}});
    slot.publish(&plan);
  }
  ~NumberedRig()
  {
    between_joints.open();
    slot.publish(nullptr);
  }

  rclcpp::Time time() const { return rclcpp::Time(cycle * 1000000, RCL_STEADY_TIME); }

  std::optional<double> state(size_t index) const
  {
    return hardware->state_interfaces()[index]->get_optional();
  }

  testing::Gate between_joints;
  std::atomic<int> finished{0};
  std::shared_ptr<HardwareRuntime> hardware;
  Doorbell bell;
  PlanSlot slot{bell};
  RtLoop loop{slot, bell, std::chrono::seconds(1)};
  Plan plan;
  int64_t cycle = 0;
  const rclcpp::Duration period = rclcpp::Duration(0, 1000000);
};

class HardwareRuntimeTest : public ::testing::Test
{
protected:
  void SetUp() override { start(false); }

  void TearDown() override
  {
    read_gate_.open();
    script_.perform_gate.open();
    script_.prepare_gate.open();
    script_.deactivate_gate.open();
  }

  void start(
    bool async, const std::string & type = "system",
    const std::vector<std::string> & extra_commands = {},
    Clock::duration gate_timeout = std::chrono::seconds(1))
  {
    auto info = testing::hardware_info("blocking", async);
    info.type = type;
    for (const auto & name : extra_commands)
    {
      hardware_interface::InterfaceInfo command{};
      command.name = name;
      info.joints.front().command_interfaces.push_back(command);
    }
    runtime_ = std::make_shared<HardwareRuntime>(info, [](const auto &, auto) {}, gate_timeout);
    ASSERT_TRUE(
      testing::bring_up(
        *runtime_,
        [&] { return std::make_unique<BlockingSystem>(read_gate_, switches_, script_); }));
  }

  /// A mode switch that holds the owner for three hang timeouts while RT cycles every millisecond
  /// from `now` on and checks the component as in use, none of the checks may fault it. Returns
  /// the time after the switch.
  Clock::time_point long_switch(bool async, Clock::time_point now = Clock::now())
  {
    script_.perform_gate.close();
    auto job = runtime_->owner().submit([&] { runtime_->switch_modes({"j1/position"}, {}); });
    EXPECT_TRUE(script_.perform_gate.holds());
    const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
    for (const auto end = now + 3 * HANG_TIMEOUT; now < end; now += std::chrono::milliseconds(1))
    {
      EXPECT_FALSE(runtime_->rt_check_hang(now, HANG_TIMEOUT, true));
      if (async)
      {
        runtime_->rt_trigger(time, rclcpp::Duration(0, 1000000));
      }
      else
      {
        runtime_->rt_read(time, rclcpp::Duration(0, 1000000));
      }
    }
    script_.perform_gate.open();
    EXPECT_TRUE(runtime_->owner().wait(job, Clock::now() + std::chrono::seconds(5)));
    return now;
  }

  static constexpr std::chrono::milliseconds HANG_TIMEOUT{100};

  template <typename Work>
  void run(Work work)
  {
    auto job = runtime_->owner().submit(work);
    ASSERT_TRUE(runtime_->owner().wait(job, Clock::now() + std::chrono::seconds(5)));
  }

  testing::Gate read_gate_;
  std::atomic<int> switches_{0};
  Script script_;
  std::shared_ptr<HardwareRuntime> runtime_;
};

TEST_F(HardwareRuntimeTest, an_async_owner_that_keeps_completing_cycles_is_not_hung)
{
  std::atomic<int> cycles{0};
  auto hw = std::make_shared<HardwareRuntime>(
    testing::hardware_info("slow_bus", true), [](const auto &, auto) {});
  ASSERT_TRUE(testing::bring_up(*hw, [&] { return std::make_unique<SlowBus>(cycles); }));

  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  // The owner needs at least 3 ms per cycle, so these cycles take longer than the hang timeout
  constexpr auto hang_timeout = std::chrono::milliseconds(300);
  constexpr int CYCLES = 150;
  const auto give_up = Clock::now() + std::chrono::seconds(10);
  bool hung = false;
  while (cycles.load() < CYCLES && Clock::now() < give_up && !hung)
  {
    // RT at 1 kHz, read checks and write triggers, as RtLoop does for a component in use
    hung = hw->rt_check_hang(Clock::now(), hang_timeout, true);
    hw->rt_trigger(time, rclcpp::Duration(0, 1000000));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_FALSE(hung) << "hang fault although the owner completed " << cycles.load() << " cycles";
  EXPECT_GE(cycles.load(), CYCLES) << "precondition: the owner made progress";
}

TEST_F(HardwareRuntimeTest, a_call_while_rt_is_stuck_in_read_faults_the_component_for_good)
{
  start(false, "system", {}, std::chrono::milliseconds(100));
  read_gate_.close();
  std::thread rt(
    [&] { runtime_->rt_read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0)); });
  EXPECT_TRUE(read_gate_.holds());
  bool switched = true;
  run([&] { switched = runtime_->switch_modes({"j1/position"}, {}).empty(); });
  read_gate_.open();
  rt.join();
  EXPECT_FALSE(switched);
  EXPECT_EQ(switches_.load(), 0);
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::Error);
  EXPECT_EQ(runtime_->fault_reason(), "RT did not leave a read or write within 100 ms");
  EXPECT_EQ(
    runtime_->rt_read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0)),
    IoResult::Skipped);
}

TEST_F(HardwareRuntimeTest, a_transition_while_rt_is_stuck_in_read_fails_with_the_reason)
{
  start(false, "system", {}, std::chrono::milliseconds(100));
  read_gate_.close();
  std::thread rt(
    [&] { runtime_->rt_read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0)); });
  EXPECT_TRUE(read_gate_.holds());
  core::Outcome outcome;
  run([&] { outcome = runtime_->transition(core::step::HardwareTransition::Deactivate); });
  read_gate_.open();
  rt.join();
  EXPECT_EQ(outcome.result, core::CallResult::Error);
  EXPECT_EQ(outcome.state, core::Lifecycle::Active) << "the plugin was never called";
  EXPECT_EQ(outcome.reason, "RT did not leave a read or write within 100 ms");
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::Error);
}

TEST_F(HardwareRuntimeTest, writes_resume_once_the_owner_held_what_rt_stopped)
{
  run([&] { ASSERT_EQ(runtime_->switch_modes({"j1/position"}, {}), ""); });
  const int before = switches_.load();
  runtime_->rt_request_stop(*runtime_->command_index("j1/position"));
  runtime_->rt_publish_stops();
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  EXPECT_TRUE(
    testing::eventually(
      [&] { return runtime_->rt_write(time, rclcpp::Duration(0, 0)) == IoResult::Ok; }));
  EXPECT_EQ(switches_.load(), before + 1);
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
}

// The supervisor planned the hold before it heard that the owner already held what RT stopped
TEST_F(HardwareRuntimeTest, a_hold_of_what_rt_already_stopped_does_not_reach_the_plugin)
{
  run([&] { ASSERT_EQ(runtime_->switch_modes({"j1/position"}, {}), ""); });
  runtime_->rt_request_stop(*runtime_->command_index("j1/position"));
  runtime_->rt_publish_stops();
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  ASSERT_TRUE(
    testing::eventually(
      [&] { return runtime_->rt_write(time, rclcpp::Duration(0, 0)) == IoResult::Ok; }))
    << "precondition: the owner held j1/position";
  const int before = switches_.load();
  run([&] { EXPECT_EQ(runtime_->switch_modes({}, {"j1/position"}), ""); });
  EXPECT_EQ(switches_.load(), before) << "the plugin stopped j1/position twice";
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
}

// A plugin such as Dynamixel refuses to hold a current cap without its position, so the owner holds
// the stops of one failure together
TEST_F(HardwareRuntimeTest, the_owner_holds_the_stops_rt_requested_together_in_one_switch)
{
  start(false, "system", {"velocity"});
  run([&] { ASSERT_EQ(runtime_->switch_modes({"j1/position", "j1/velocity"}, {}), ""); });
  const int before = switches_.load();
  runtime_->rt_request_stop(*runtime_->command_index("j1/position"));
  // The owner polls before it takes a job
  run([] {});
  runtime_->rt_request_stop(*runtime_->command_index("j1/velocity"));
  runtime_->rt_publish_stops();
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  ASSERT_TRUE(
    testing::eventually(
      [&] { return runtime_->rt_write(time, rclcpp::Duration(0, 0)) == IoResult::Ok; }));
  EXPECT_EQ(switches_.load(), before + 1);
  EXPECT_EQ(script_.stopped, (std::vector<std::string>{"j1/velocity", "j1/position"}));
}

// RT requests B while the owner is in a read and publishes it only with C. The owner must not
// hold B before its batch is published.
TEST_F(HardwareRuntimeTest, the_owner_holds_no_stop_of_a_batch_rt_has_not_published)
{
  start(true, "system", {"velocity", "effort"});
  run([&]
      { ASSERT_EQ(runtime_->switch_modes({"j1/position", "j1/velocity", "j1/effort"}, {}), ""); });
  const int before = switches_.load();
  read_gate_.close();
  runtime_->rt_trigger(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 1000000));
  ASSERT_TRUE(read_gate_.holds()) << "precondition: the owner is in a read";
  runtime_->rt_request_stop(*runtime_->command_index("j1/position"));
  runtime_->rt_publish_stops();
  runtime_->rt_request_stop(*runtime_->command_index("j1/velocity"));
  read_gate_.open();
  // The owner polls before it takes a job
  run([] {});
  EXPECT_EQ(switches_.load(), before) << "the owner held part of the unpublished batch";
  runtime_->rt_request_stop(*runtime_->command_index("j1/effort"));
  runtime_->rt_publish_stops();
  ASSERT_TRUE(testing::eventually([&] { return switches_.load() == before + 1; }));
  // A second hold would come right after the first
  run([] {});
  EXPECT_EQ(switches_.load(), before + 1);
  EXPECT_EQ(script_.stopped, (std::vector<std::string>{"j1/effort", "j1/position", "j1/velocity"}));
}

TEST_F(HardwareRuntimeTest, a_late_hold_stops_only_what_rt_left_live)
{
  start(false, "system", {"velocity"});
  run([&] { ASSERT_EQ(runtime_->switch_modes({"j1/position", "j1/velocity"}, {}), ""); });
  runtime_->rt_request_stop(*runtime_->command_index("j1/position"));
  runtime_->rt_publish_stops();
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  ASSERT_TRUE(
    testing::eventually(
      [&] { return runtime_->rt_write(time, rclcpp::Duration(0, 0)) == IoResult::Ok; }))
    << "precondition: the owner held j1/position";
  const int before = switches_.load();
  run([&] { EXPECT_EQ(runtime_->switch_modes({}, {"j1/position", "j1/velocity"}), ""); });
  EXPECT_EQ(switches_.load(), before + 1);
  EXPECT_EQ(script_.stopped, std::vector<std::string>{"j1/velocity"});
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
}

TEST_F(HardwareRuntimeTest, a_stall_of_rt_itself_does_not_fault_an_idle_async_component)
{
  start(true);
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  read_gate_.close();
  runtime_->rt_trigger(time, rclcpp::Duration(0, 0));
  ASSERT_TRUE(read_gate_.holds());
  // The owner is still serving the first trigger, so a check starts the hang window
  const auto begin = Clock::now();
  EXPECT_FALSE(runtime_->rt_check_hang(begin, std::chrono::milliseconds(100), true));
  runtime_->rt_trigger(time, rclcpp::Duration(0, 0));
  read_gate_.open();
  run([] {});
  // RT itself stalled for 300 ms, its next check finds the owner free
  EXPECT_FALSE(runtime_->rt_check_hang(
    begin + std::chrono::milliseconds(300), std::chrono::milliseconds(100), true));
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
}

// RT skips its copies while the owner holds the gate in a mode switch, which is no hang
TEST_F(HardwareRuntimeTest, an_async_owner_busy_shorter_than_the_hang_timeout_is_not_hung)
{
  start(true);
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  constexpr auto hang_timeout = std::chrono::milliseconds(100);
  script_.perform_gate.close();
  auto job = runtime_->owner().submit([&] { runtime_->switch_modes({"j1/position"}, {}); });
  ASSERT_TRUE(script_.perform_gate.holds());
  const uint64_t skipped = runtime_->stats().skipped();
  const auto begin = Clock::now();
  for (int i = 0; i < 30; ++i)
  {
    EXPECT_FALSE(runtime_->rt_check_hang(begin + std::chrono::milliseconds(i), hang_timeout, true));
    runtime_->rt_trigger(time, rclcpp::Duration(0, 1000000));
  }
  EXPECT_GT(runtime_->stats().skipped(), skipped) << "precondition: RT skipped its copies";
  script_.perform_gate.open();
  ASSERT_TRUE(runtime_->owner().wait(job, Clock::now() + std::chrono::seconds(5)));
  // RT itself stalls once the owner is free, a late check must not count that
  EXPECT_FALSE(runtime_->rt_check_hang(begin + std::chrono::milliseconds(330), hang_timeout, true));
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
}

// A check found the owner busy and opened a window. A trigger that found it free since ends that
// window, a check during the next cycle does not count it.
TEST_F(HardwareRuntimeTest, a_busy_owner_before_a_free_trigger_is_not_hung)
{
  start(true);
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  constexpr auto hang_timeout = std::chrono::milliseconds(100);
  const auto begin = Clock::now();
  read_gate_.close();
  runtime_->rt_trigger(time, rclcpp::Duration(0, 1000000));
  ASSERT_TRUE(read_gate_.holds());
  EXPECT_FALSE(runtime_->rt_check_hang(begin, hang_timeout, true));
  read_gate_.open();
  run([] {});
  read_gate_.close();
  runtime_->rt_trigger(time, rclcpp::Duration(0, 1000000));
  ASSERT_TRUE(read_gate_.holds()) << "precondition: the owner serves the trigger";
  EXPECT_FALSE(runtime_->rt_check_hang(
    begin + hang_timeout + std::chrono::milliseconds(50), hang_timeout, true));
  read_gate_.open();
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
}

// RT checks before it triggers, so a check never finds an owner free that finishes every cycle
// between the two, for example with writes of 60 ms
TEST_F(HardwareRuntimeTest, an_async_owner_that_finishes_every_cycle_after_the_check_is_not_hung)
{
  start(true);
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  const auto begin = Clock::now();
  for (int cycle = 0; cycle < 3; ++cycle)
  {
    read_gate_.close();
    runtime_->rt_trigger(time, rclcpp::Duration(0, 1000000));
    ASSERT_TRUE(read_gate_.holds()) << "precondition: the owner serves trigger " << cycle;
    EXPECT_FALSE(
      runtime_->rt_check_hang(begin + cycle * std::chrono::milliseconds(60), HANG_TIMEOUT, true))
      << "cycle " << cycle;
    read_gate_.open();
    // The owner takes a job only after it finished its cycle
    run([] {});
  }
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
}

// The owner stays busy while the component goes out of use and back, which restarts the window
TEST_F(HardwareRuntimeTest, a_component_out_of_use_ends_its_hang_window)
{
  start(true);
  read_gate_.close();
  runtime_->rt_trigger(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0));
  ASSERT_TRUE(read_gate_.holds());
  const auto begin = Clock::now();
  EXPECT_FALSE(runtime_->rt_check_hang(begin, HANG_TIMEOUT, true));
  EXPECT_FALSE(runtime_->rt_check_hang(begin + HANG_TIMEOUT / 2, HANG_TIMEOUT, false));
  EXPECT_FALSE(runtime_->rt_check_hang(begin + 2 * HANG_TIMEOUT, HANG_TIMEOUT, true));
  EXPECT_FALSE(runtime_->rt_check_hang(begin + 3 * HANG_TIMEOUT, HANG_TIMEOUT, false))
    << "a component out of use never faults";
  read_gate_.open();
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
}

// A trigger between the owner's poll and its next job copies commands as old as that job
TEST_F(HardwareRuntimeTest, an_async_copy_taken_before_a_long_prepare_is_not_written_after_it)
{
  start(true);
  testing::Gate before_job;
  before_job.close();
  script_.prepare_gate.close();
  auto job = runtime_->owner().submit(
    [&]
    {
      before_job.pass();
      runtime_->validate({"j1/position"}, {});
    });
  ASSERT_TRUE(before_job.holds());
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  runtime_->rt_trigger(time, rclcpp::Duration(0, 1000000));
  before_job.open();
  ASSERT_TRUE(script_.prepare_gate.holds()) << "precondition: the owner is in prepare";
  script_.prepare_gate.open();
  ASSERT_TRUE(runtime_->owner().wait(job, Clock::now() + std::chrono::seconds(5)));
  run([] {});
  EXPECT_EQ(script_.writes.load(), 0);
  runtime_->rt_trigger(time, rclcpp::Duration(0, 1000000));
  run([] {});
  EXPECT_EQ(script_.writes.load(), 1) << "the next trigger copies and writes";
}

// A mode switch has a deadline of its own, the hang timeout covers read and write
TEST_F(HardwareRuntimeTest, an_async_mode_switch_longer_than_the_hang_timeout_does_not_fault)
{
  start(true);
  long_switch(true);
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
}

// A check opened the window while the owner read, then the owner went straight into a long switch
// and another call. The first check in that call must not count the switch.
TEST_F(HardwareRuntimeTest, a_long_async_mode_switch_ends_a_hang_window_opened_before_it)
{
  start(true);
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  read_gate_.close();
  runtime_->rt_trigger(time, rclcpp::Duration(0, 1000000));
  ASSERT_TRUE(read_gate_.holds());
  const auto begin = Clock::now();
  EXPECT_FALSE(runtime_->rt_check_hang(begin, HANG_TIMEOUT, true));
  runtime_->rt_trigger(time, rclcpp::Duration(0, 1000000));
  read_gate_.open();
  const auto after = long_switch(true, begin);
  script_.prepare_gate.close();
  auto job = runtime_->owner().submit([&] { runtime_->validate({"j1/position"}, {}); });
  ASSERT_TRUE(script_.prepare_gate.holds()) << "precondition: the owner is in another call";
  EXPECT_FALSE(runtime_->rt_check_hang(after, HANG_TIMEOUT, true))
    << "the window counted the mode switch";
  script_.prepare_gate.open();
  ASSERT_TRUE(runtime_->owner().wait(job, Clock::now() + std::chrono::seconds(5)));
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
}

TEST_F(HardwareRuntimeTest, a_read_that_hangs_after_a_long_async_mode_switch_faults_in_time)
{
  start(true);
  const auto stuck = long_switch(true);
  read_gate_.close();
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  runtime_->rt_trigger(time, rclcpp::Duration(0, 1000000));
  ASSERT_TRUE(read_gate_.holds());
  runtime_->rt_trigger(time, rclcpp::Duration(0, 1000000));
  EXPECT_FALSE(runtime_->rt_check_hang(stuck, HANG_TIMEOUT, true))
    << "the window counted the mode switch";
  EXPECT_FALSE(runtime_->rt_check_hang(stuck + HANG_TIMEOUT / 2, HANG_TIMEOUT, true));
  EXPECT_TRUE(runtime_->rt_check_hang(stuck + 2 * HANG_TIMEOUT, HANG_TIMEOUT, true));
  read_gate_.open();
  const std::string reason = runtime_->fault_reason();
  EXPECT_NE(reason.find("while its owner was in read"), std::string::npos) << reason;
}

// The owner of an async component writes its states while the controllers update
TEST_F(HardwareRuntimeTest, a_controller_sees_every_joint_of_one_async_read)
{
  NumberedRig rig;
  int busy = 0;
  int torn = 0;
  double seen = 0.0;
  for (; rig.cycle < 300; ++rig.cycle)
  {
    rig.loop.read(rig.time(), rig.period);
    std::array<std::optional<double>, 3> states;
    for (size_t i = 0; i < states.size(); ++i)
    {
      states[i] = rig.state(i);
    }
    busy += !states[0] || !states[1] || !states[2] ? 1 : 0;
    const auto same = [](double a, double b) { return a == b || (std::isnan(a) && std::isnan(b)); };
    torn += states[0] && states[1] && states[2] &&
                (!same(*states[0], *states[1]) || !same(*states[0], *states[2]))
              ? 1
              : 0;
    seen = states[0].value_or(seen);
    rig.loop.update(rig.time(), rig.period);
    rig.loop.write(rig.time(), rig.period);
    if (rig.cycle == 100)
    {
      rig.between_joints.close();
    }
    if (rig.cycle == 200)
    {
      rig.between_joints.open();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(busy, 0);
  EXPECT_EQ(torn, 0);
  EXPECT_GT(seen, 50.0) << "precondition: the owner kept reading";
}

// The owner is busy at every RT read and free at every write. Its states still reach the
// controllers, and a state copy that cannot take a shadow counts as skipped.
TEST_F(HardwareRuntimeTest, an_async_owner_that_reads_past_rt_read_still_delivers_its_states)
{
  NumberedRig rig;
  rig.between_joints.close();
  rig.loop.write(rig.time(), rig.period);
  for (rig.cycle = 1; rig.cycle <= 50; ++rig.cycle)
  {
    rig.loop.read(rig.time(), rig.period);
    rig.between_joints.open();
    ASSERT_TRUE(testing::eventually([&] { return rig.finished.load() >= rig.cycle; }));
    rig.between_joints.close();
    rig.loop.update(rig.time(), rig.period);
    rig.loop.write(rig.time(), rig.period);
  }
  EXPECT_EQ(rig.state(0), 50.0);
  EXPECT_EQ(rig.state(2), 50.0);
  const uint64_t skipped = rig.hardware->stats().skipped();
  rig.loop.read(rig.time(), rig.period);
  rig.between_joints.open();
  ASSERT_TRUE(testing::eventually([&] { return rig.finished.load() >= rig.cycle; }));
  {
    auto & shadow =
      const_cast<hardware_interface::StateInterface &>(*rig.hardware->state_interfaces()[1]);
    const testing::HeldLock busy(shadow.get_mutex());
    rig.loop.write(rig.time(), rig.period);
  }
  EXPECT_EQ(rig.hardware->stats().skipped(), skipped + 1);
  EXPECT_EQ(rig.state(0), 50.0) << "a copy that cannot take every shadow takes none";
}

TEST(OwnerPriority, an_owner_runs_below_the_control_loop_unless_its_urdf_sets_a_priority)
{
  hardware_interface::HardwareAsyncParams params;
  EXPECT_EQ(owner_priority(params, 50), 49);
  EXPECT_EQ(owner_priority(params, 80), 79);
  EXPECT_EQ(owner_priority(params, 1), 1);
  params.thread_priority = 70;
  EXPECT_EQ(owner_priority(params, 50), 70);
}

// Without realtime privileges only the cores apply, the owner keeps its policy
TEST_F(HardwareRuntimeTest, an_async_owner_runs_on_the_cores_of_its_urdf)
{
  cpu_set_t allowed;
  ASSERT_EQ(sched_getaffinity(0, sizeof(allowed), &allowed), 0);
  int core = 0;
  while (!CPU_ISSET(core, &allowed))
  {
    ++core;
  }
  auto info = testing::hardware_info("pinned", true);
  info.async_params.cpu_affinity_cores = {core};
  auto hardware = std::make_shared<HardwareRuntime>(info, [](const auto &, auto) {});
  cpu_set_t pinned;
  int policy = -1;
  sched_param param{};
  auto job = hardware->owner().submit(
    [&]
    {
      hardware->schedule_owner(50);
      sched_getaffinity(0, sizeof(pinned), &pinned);
      pthread_getschedparam(pthread_self(), &policy, &param);
      policy &= ~SCHED_RESET_ON_FORK;
    });
  ASSERT_TRUE(hardware->owner().wait(job, Clock::now() + std::chrono::seconds(5)));
  EXPECT_EQ(CPU_COUNT(&pinned), 1);
  EXPECT_TRUE(CPU_ISSET(core, &pinned));
  if (realtime_tools::has_realtime_kernel() && policy == SCHED_FIFO)
  {
    EXPECT_EQ(param.sched_priority, 49);
  }
  else
  {
    EXPECT_EQ(policy, SCHED_OTHER);
  }
}

// Off a realtime kernel too, the owner asks for SCHED_FIFO without inheritance and says why it
// did not get it
TEST_F(HardwareRuntimeTest, an_async_owner_asks_for_fifo_below_the_control_loop)
{
  auto hardware = std::make_shared<HardwareRuntime>(
    testing::hardware_info("fifo", true), [](const auto &, auto) {});
  std::vector<std::pair<int, int>> asked;
  Scheduler scheduler;
  scheduler.realtime_kernel = [] { return true; };
  scheduler.set_policy = [&](int policy, int priority)
  {
    asked.emplace_back(policy, priority);
    errno = EPERM;
    return false;
  };
  const testing::LogCapture log;
  auto job = hardware->owner().submit([&] { hardware->schedule_owner(60, scheduler); });
  ASSERT_TRUE(hardware->owner().wait(job, Clock::now() + std::chrono::seconds(5)));
  ASSERT_EQ(asked.size(), 1u);
  EXPECT_EQ(asked.front(), std::make_pair(SCHED_FIFO, 59));
  EXPECT_TRUE(log.contains("at priority 59: Operation not permitted")) << log.text();
}

/// The policy and cores of a thread that started on the calling one.
struct StartedThread
{
  StartedThread()
  {
    std::thread(
      [this]
      {
        sched_param param{};
        pthread_getschedparam(pthread_self(), &policy, &param);
        sched_getaffinity(0, sizeof(cores), &cores);
      })
      .join();
  }
  int policy = -1;
  cpu_set_t cores{};
};

// SCHED_OTHER is allowed without privileges, the flag that stops the inheritance is the same
TEST(OwnerScheduling, a_policy_set_without_inheritance_does_not_reach_new_threads)
{
  int own = -1;
  int started = -1;
  bool applied = false;
  std::thread(
    [&]
    {
      applied = schedule_without_inheritance(SCHED_OTHER, 0);
      own = sched_getscheduler(0);
      started = StartedThread().policy;
    })
    .join();
  ASSERT_TRUE(applied);
  EXPECT_EQ(own, SCHED_OTHER | SCHED_RESET_ON_FORK);
  EXPECT_EQ(started, SCHED_OTHER);
}

/// Starts a thread in on_init, as Dynamixel does for its executor.
class ThreadStartingSystem : public BlockingSystem
{
public:
  ThreadStartingSystem(
    testing::Gate & read_gate, std::atomic<int> & switches, Script & script,
    std::optional<StartedThread> & started)
  : BlockingSystem(read_gate, switches, script), started_(started)
  {
  }
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    started_.emplace();
    return BlockingSystem::on_init(params);
  }

private:
  std::optional<StartedThread> & started_;
};

// The policy part needs a realtime kernel with privileges, else the owner keeps SCHED_OTHER
TEST_F(HardwareRuntimeTest, a_plugin_thread_keeps_the_owner_cores_but_not_its_priority)
{
  cpu_set_t allowed;
  ASSERT_EQ(sched_getaffinity(0, sizeof(allowed), &allowed), 0);
  int core = 0;
  while (!CPU_ISSET(core, &allowed))
  {
    ++core;
  }
  auto info = testing::hardware_info("threads", true);
  info.async_params.cpu_affinity_cores = {core};
  auto hardware = std::make_shared<HardwareRuntime>(info, [](const auto &, auto) {});
  std::optional<StartedThread> started;
  int owner_policy = -1;
  auto job = hardware->owner().submit(
    [&]
    {
      hardware->schedule_owner(50);
      owner_policy = sched_getscheduler(0);
      hardware->initialize(
        [&](const auto &)
        { return std::make_unique<ThreadStartingSystem>(read_gate_, switches_, script_, started); },
        testing::steady_params(info));
    });
  ASSERT_TRUE(hardware->owner().wait(job, Clock::now() + std::chrono::seconds(5)));
  ASSERT_TRUE(started.has_value());
  EXPECT_EQ(started->policy, SCHED_OTHER);
  EXPECT_EQ(CPU_COUNT(&started->cores), 1);
  EXPECT_TRUE(CPU_ISSET(core, &started->cores));
  if (realtime_tools::has_realtime_kernel() && (owner_policy & ~SCHED_RESET_ON_FORK) == SCHED_FIFO)
  {
    EXPECT_EQ(owner_policy, SCHED_FIFO | SCHED_RESET_ON_FORK);
  }
}

// RT copies only doubles, a sync component's controllers use its other handles directly
TEST_F(HardwareRuntimeTest, only_a_sync_component_may_export_an_interface_without_a_double)
{
  for (const bool async : {false, true})
  {
    auto info = testing::hardware_info("brakes", async);
    hardware_interface::InterfaceInfo brake{};
    brake.name = "brake";
    brake.data_type = "bool";
    info.joints.front().command_interfaces.push_back(brake);
    auto hardware = std::make_shared<HardwareRuntime>(info, [](const auto &, auto) {});
    std::string error;
    auto job = hardware->owner().submit(
      [&]
      {
        try
        {
          hardware->initialize(
            [&](const auto &)
            { return std::make_unique<BlockingSystem>(read_gate_, switches_, script_); },
            testing::steady_params(info));
        }
        catch (const std::exception & e)
        {
          error = e.what();
        }
      });
    ASSERT_TRUE(hardware->owner().wait(job, Clock::now() + std::chrono::seconds(5)));
    if (async)
    {
      EXPECT_NE(error.find("'j1/brake', which does not hold a double"), std::string::npos) << error;
      continue;
    }
    ASSERT_EQ(error, "");
    const auto & brakes = hardware->command_interfaces()[*hardware->command_index("j1/brake")];
    EXPECT_EQ(brakes->get_data_type(), hardware_interface::HandleDataType::BOOL);
  }
}

TEST_F(HardwareRuntimeTest, an_io_fault_names_the_call_and_its_result)
{
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  EXPECT_EQ(runtime_->fault_reason(), "");
  script_.read = return_type::ERROR;
  ASSERT_EQ(runtime_->rt_read(time, rclcpp::Duration(0, 0)), IoResult::Error);
  EXPECT_EQ(runtime_->fault_reason(), "read returned ERROR");
  // A new generation starts without a fault, also the write's first one stays
  run([&] { runtime_->transition(core::step::HardwareTransition::Error); });
  run(
    [&]
    {
      runtime_->transition(core::step::HardwareTransition::Configure);
      runtime_->transition(core::step::HardwareTransition::Activate);
    });
  EXPECT_EQ(runtime_->fault_reason(), "");
  script_.read = return_type::OK;
  script_.write = return_type::DEACTIVATE;
  ASSERT_EQ(runtime_->rt_write(time, rclcpp::Duration(0, 0)), IoResult::Deactivate);
  EXPECT_EQ(runtime_->fault_reason(), "write returned DEACTIVATE");
}

TEST_F(HardwareRuntimeTest, a_throwing_read_names_the_exception)
{
  script_.throw_on_read = true;
  ASSERT_EQ(
    runtime_->rt_read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0)),
    IoResult::Error);
  EXPECT_EQ(runtime_->fault_reason(), "read threw: bus 4711");
}

TEST_F(HardwareRuntimeTest, a_failed_perform_names_the_call)
{
  script_.perform = return_type::ERROR;
  run([&] { runtime_->switch_modes({"j1/position"}, {}); });
  EXPECT_EQ(runtime_->fault_reason(), "perform_command_mode_switch returned ERROR");
}

TEST_F(HardwareRuntimeTest, a_second_call_fault_keeps_the_first_reason)
{
  script_.perform = return_type::ERROR;
  run([&] { runtime_->switch_modes({"j1/position"}, {}); });
  script_.prepare = return_type::ERROR;
  run([&] { runtime_->switch_modes({"j1/position"}, {}); });
  EXPECT_EQ(runtime_->fault_reason(), "perform_command_mode_switch returned ERROR");
}

TEST_F(HardwareRuntimeTest, a_sync_mode_switch_longer_than_the_hang_timeout_does_not_fault)
{
  long_switch(false);
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
}

// RT skips its reads while the owner holds the gate in a slow deactivation after a long switch
TEST_F(HardwareRuntimeTest, a_sync_hang_names_its_duration_and_the_owner_call)
{
  const auto stuck = long_switch(false);
  script_.deactivate_gate.close();
  auto job = runtime_->owner().submit(
    [&] { runtime_->transition(core::step::HardwareTransition::Deactivate); });
  ASSERT_TRUE(script_.deactivate_gate.holds());
  ASSERT_EQ(
    runtime_->rt_read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0)),
    IoResult::Skipped);
  EXPECT_FALSE(runtime_->rt_check_hang(stuck, HANG_TIMEOUT, true))
    << "the window counted the mode switch";
  EXPECT_FALSE(runtime_->rt_check_hang(stuck + HANG_TIMEOUT / 2, HANG_TIMEOUT, true));
  EXPECT_TRUE(runtime_->rt_check_hang(stuck + std::chrono::milliseconds(250), HANG_TIMEOUT, true));
  // Before the transition ends, which starts a generation without the fault
  const std::string reason = runtime_->fault_reason();
  script_.deactivate_gate.open();
  ASSERT_TRUE(runtime_->owner().wait(job, Clock::now() + std::chrono::seconds(5)));
  const std::string prefix = "no read or write ran for ";
  ASSERT_EQ(reason.rfind(prefix, 0), 0u) << reason;
  EXPECT_GE(std::stoi(reason.substr(prefix.size())), 250) << reason;
  const std::string rest =
    " ms, longer than the hang timeout of 100 ms, while its owner was in on_deactivate";
  EXPECT_NE(reason.find(rest), std::string::npos) << reason;
}

TEST_F(HardwareRuntimeTest, an_async_hang_names_the_owner_call)
{
  start(true);
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  read_gate_.close();
  runtime_->rt_trigger(time, rclcpp::Duration(0, 0));
  ASSERT_TRUE(read_gate_.holds());
  runtime_->rt_trigger(time, rclcpp::Duration(0, 0));
  const auto stuck = Clock::now();
  EXPECT_FALSE(runtime_->rt_check_hang(stuck, std::chrono::milliseconds(100), true));
  EXPECT_TRUE(runtime_->rt_check_hang(
    stuck + std::chrono::milliseconds(250), std::chrono::milliseconds(100), true));
  read_gate_.open();
  const std::string reason = runtime_->fault_reason();
  EXPECT_NE(reason.find("while its owner was in read"), std::string::npos) << reason;
}

TEST_F(HardwareRuntimeTest, an_active_sensor_is_read_but_never_written)
{
  start(false, "sensor");
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  EXPECT_EQ(runtime_->rt_read(time, rclcpp::Duration(0, 0)), IoResult::Ok);
  EXPECT_EQ(runtime_->rt_write(time, rclcpp::Duration(0, 0)), IoResult::Skipped);
}

// A block left by an abandoned mode switch ends with the next lifecycle transition
TEST_F(HardwareRuntimeTest, a_write_block_expires_at_the_next_lifecycle_transition)
{
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  runtime_->block_writes();
  ASSERT_EQ(runtime_->rt_write(time, rclcpp::Duration(0, 0)), IoResult::Skipped);
  run(
    [&]
    {
      runtime_->transition(core::step::HardwareTransition::Deactivate);
      runtime_->transition(core::step::HardwareTransition::Activate);
    });
  ASSERT_EQ(runtime_->state(), core::Lifecycle::Active);
  EXPECT_EQ(runtime_->rt_write(time, rclcpp::Duration(0, 0)), IoResult::Ok);
}

}  // namespace
}  // namespace hector_controller_manager::runtime
