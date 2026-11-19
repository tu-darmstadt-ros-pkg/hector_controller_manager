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
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fakes.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/runtime/hardware_shutdown.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using core::Lifecycle;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using namespace std::chrono_literals;

/// Its on_deactivate takes `delay`.
class SlowSystem : public hardware_interface::SystemInterface
{
public:
  explicit SlowSystem(std::chrono::milliseconds delay) : delay_(delay) {}
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    std::this_thread::sleep_for(delay_);
    return CallbackReturn::SUCCESS;
  }
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return return_type::OK;
  }
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return return_type::OK;
  }

private:
  std::chrono::milliseconds delay_;
};

class HardwareShutdownTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  HardwareRuntime * add(const std::string & name, std::chrono::milliseconds delay, bool activate)
  {
    auto runtime =
      std::make_unique<HardwareRuntime>(testing::hardware_info(name), [](const auto &, auto) {});
    EXPECT_TRUE(
      testing::bring_up(*runtime, [&] { return std::make_unique<SlowSystem>(delay); }, activate));
    runtimes_.push_back(std::move(runtime));
    return runtimes_.back().get();
  }

  std::vector<std::unique_ptr<HardwareRuntime>> runtimes_;
};

TEST_F(HardwareShutdownTest, active_and_inactive_components_end_finalized)
{
  const auto states = shut_down_all(
    {{"active", add("active", 0ms, true)}, {"inactive", add("inactive", 0ms, false)}},
    Clock::now() + 5s);
  EXPECT_EQ(states.at("active"), Lifecycle::Finalized);
  EXPECT_EQ(states.at("inactive"), Lifecycle::Finalized);
}

// The slow one comes first, so waiting for it before the fast one starts would miss the deadline
TEST_F(HardwareShutdownTest, a_slow_component_does_not_use_up_the_time_of_the_others)
{
  auto * slow = add("a_slow", 300ms, true);
  auto * fast = add("b_fast", 50ms, true);
  const auto states = shut_down_all({{"a_slow", slow}, {"b_fast", fast}}, Clock::now() + 150ms);
  EXPECT_EQ(states.at("a_slow"), std::nullopt);
  EXPECT_EQ(states.at("b_fast"), Lifecycle::Finalized);
  // The slow call outlives the deadline and uses its runtime until it returns
  auto returned = slow->owner().submit([] {});
  EXPECT_TRUE(slow->owner().wait(returned, Clock::now() + 5s, [] {}));
}

}  // namespace
}  // namespace hector_controller_manager::runtime
