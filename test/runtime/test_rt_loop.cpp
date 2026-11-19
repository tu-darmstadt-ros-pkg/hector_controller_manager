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
#include <memory>

#include "fakes.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/runtime/controller_runtime.hpp"
#include "hector_controller_manager/runtime/hardware_runtime.hpp"
#include "hector_controller_manager/runtime/rt_loop.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using hardware_interface::return_type;

class FaultyRead : public hardware_interface::SystemInterface
{
public:
  explicit FaultyRead(std::atomic<bool> & fail) : fail_(fail) {}
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return fail_ ? return_type::ERROR : return_type::OK;
  }
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return return_type::OK;
  }

private:
  std::atomic<bool> & fail_;
};

class Counting : public testing::NoInterfaces<>
{
public:
  explicit Counting(std::atomic<int> & updates) : updates_(updates) {}
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    updates_.fetch_add(1);
    return controller_interface::return_type::OK;
  }

private:
  std::atomic<int> & updates_;
};

class RtLoopTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// RT stops a hardware fault once per lifecycle generation, with the scope of the plan it holds
// in that cycle. A claimer that a later plan attaches to the faulted component never runs.
TEST_F(RtLoopTest, a_claimer_attached_after_rt_handled_a_fault_is_still_stopped)
{
  std::atomic<bool> fail{false};
  auto hw =
    std::make_shared<HardwareRuntime>(testing::hardware_info("bus"), [](const auto &, auto) {});
  ASSERT_TRUE(testing::bring_up(*hw, [&] { return std::make_unique<FaultyRead>(fail); }));

  std::atomic<int> updates{0};
  auto b = std::make_shared<ControllerRuntime>("b", "test/Counting");
  b->set_plugin(testing::initialized(std::make_shared<Counting>(updates), "b"));

  Doorbell bell;
  PlanSlot slot(bell);
  RtLoop loop(slot, bell, std::chrono::milliseconds(100));
  // Nobody claims the component yet
  Plan p1;
  p1.generation = 1;
  p1.hardware.push_back({hw.get(), false, {}});
  // b claims the component, so the component's scope stops b
  Plan p2;
  p2.generation = 2;
  p2.controllers.push_back({b.get(), 1, {{0, 1}, {0, 0}}, false});
  p2.scope_controllers.push_back(0);
  p2.hardware.push_back({hw.get(), true, {{0, 1}, {0, 0}}});

  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  const rclcpp::Duration period(0, 1000000);
  slot.publish(&p1);
  loop.read(time, period);
  fail = true;
  // The fault fires and is handled with the empty scope of p1
  loop.read(time, period);
  ASSERT_NE(hw->fault(), core::HardwareFault::None);
  // The supervisor attaches b before it absorbed the fault
  slot.publish(&p2);
  for (int i = 0; i < 5; ++i)
  {
    loop.read(time, period);
    loop.update(time, period);
    loop.write(time, period);
  }
  EXPECT_EQ(updates.load(), 0) << "b ran " << updates.load()
                               << " updates on a component RT knows is faulted";
  slot.publish(nullptr);
  loop.read(time, period);
  b->set_plugin(nullptr);
}

}  // namespace
}  // namespace hector_controller_manager::runtime
