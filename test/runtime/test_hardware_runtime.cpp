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
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "fakes.hpp"
#include "gate.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/runtime/hardware_runtime.hpp"
#include "rclcpp/rclcpp.hpp"

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
    return script_.write;
  }

  return_type prepare_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override
  {
    return script_.prepare;
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

class HardwareRuntimeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override { start(false); }

  void TearDown() override
  {
    read_gate_.open();
    script_.perform_gate.open();
  }

  void start(
    bool async, const std::string & type = "system",
    const std::vector<std::string> & extra_commands = {})
  {
    auto info = testing::hardware_info("blocking", async);
    info.type = type;
    for (const auto & name : extra_commands)
    {
      hardware_interface::InterfaceInfo command;
      command.name = name;
      info.joints.front().command_interfaces.push_back(command);
    }
    runtime_ = std::make_shared<HardwareRuntime>(info, [](const auto &, auto) {});
    ASSERT_TRUE(
      testing::bring_up(
        *runtime_,
        [&] { return std::make_unique<BlockingSystem>(read_gate_, switches_, script_); }));
  }

  template <typename Work>
  void run(Work work)
  {
    auto job = runtime_->owner().submit(work);
    ASSERT_TRUE(runtime_->owner().wait(job, Clock::now() + std::chrono::seconds(5), [] {}));
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
  const auto end = Clock::now() + std::chrono::milliseconds(400);
  bool hung = false;
  int hung_after_cycles = -1;
  while (Clock::now() < end && !hung)
  {
    // RT at 1 kHz, read checks and write triggers, as RtLoop does for a component in use
    hung = hw->rt_check_hang(Clock::now(), std::chrono::milliseconds(100));
    if (hung)
    {
      hung_after_cycles = cycles.load();
    }
    hw->rt_trigger(time, rclcpp::Duration(0, 1000000));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_FALSE(hung) << "hang fault although the owner completed " << hung_after_cycles
                     << " cycles";
  EXPECT_GT(cycles.load(), 30) << "precondition: the owner made progress";
}

TEST_F(HardwareRuntimeTest, a_call_while_rt_is_stuck_in_read_faults_the_component)
{
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
  EXPECT_EQ(runtime_->fault_reason(), "RT did not leave a read or write within 1000 ms");
}

TEST_F(HardwareRuntimeTest, a_faulted_component_is_not_read_again)
{
  read_gate_.close();
  std::thread rt(
    [&] { runtime_->rt_read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0)); });
  EXPECT_TRUE(read_gate_.holds());
  run([&] { runtime_->switch_modes({"j1/position"}, {}); });
  read_gate_.open();
  rt.join();
  EXPECT_EQ(
    runtime_->rt_read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0)),
    IoResult::Skipped);
}

TEST_F(HardwareRuntimeTest, writes_resume_once_the_owner_held_what_rt_stopped)
{
  run([&] { ASSERT_EQ(runtime_->switch_modes({"j1/position"}, {}), ""); });
  const int before = switches_.load();
  runtime_->rt_request_stop(*runtime_->command_index("j1/position"));
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

TEST_F(HardwareRuntimeTest, a_late_hold_stops_only_what_rt_left_live)
{
  start(false, "system", {"velocity"});
  run([&] { ASSERT_EQ(runtime_->switch_modes({"j1/position", "j1/velocity"}, {}), ""); });
  runtime_->rt_request_stop(*runtime_->command_index("j1/position"));
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
  // The owner is still serving the first trigger, so the second one starts the hang window
  runtime_->rt_trigger(time, rclcpp::Duration(0, 0));
  read_gate_.open();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(runtime_->rt_check_hang(Clock::now(), std::chrono::milliseconds(100)));
  EXPECT_EQ(runtime_->fault(), core::HardwareFault::None);
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

// RT skips its reads while the owner holds the gate in a slow perform
TEST_F(HardwareRuntimeTest, a_sync_hang_names_its_duration_and_the_owner_call)
{
  script_.perform_gate.close();
  auto job = runtime_->owner().submit([&] { runtime_->switch_modes({"j1/position"}, {}); });
  ASSERT_TRUE(script_.perform_gate.holds());
  ASSERT_EQ(
    runtime_->rt_read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0)),
    IoResult::Skipped);
  EXPECT_TRUE(runtime_->rt_check_hang(
    Clock::now() + std::chrono::milliseconds(250), std::chrono::milliseconds(100)));
  script_.perform_gate.open();
  ASSERT_TRUE(runtime_->owner().wait(job, Clock::now() + std::chrono::seconds(5), [] {}));
  const std::string reason = runtime_->fault_reason();
  const std::string prefix = "no read or write ran for ";
  ASSERT_EQ(reason.rfind(prefix, 0), 0u) << reason;
  EXPECT_GE(std::stoi(reason.substr(prefix.size())), 250) << reason;
  const std::string rest =
    " ms, longer than the hang timeout of 100 ms, while its owner was in "
    "perform_command_mode_switch";
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
  EXPECT_TRUE(runtime_->rt_check_hang(
    Clock::now() + std::chrono::milliseconds(250), std::chrono::milliseconds(100)));
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

}  // namespace
}  // namespace hector_controller_manager::runtime
