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

#include <memory>
#include <stdexcept>

#include "fakes.hpp"
#include "hector_controller_manager/runtime/controller_runtime.hpp"
#include "log_capture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_environment.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
class ThrowingChainable : public testing::NoInterfacesChainable
{
protected:
  bool on_set_chained_mode(bool) override { throw std::runtime_error("chain mode refused 4711"); }
};

using ControllerRuntimeTest = ::testing::Test;

TEST_F(ControllerRuntimeTest, a_throwing_chained_mode_change_fails_and_logs_the_reason)
{
  const auto plugin = testing::initialized(std::make_shared<ThrowingChainable>(), "thrower");
  ControllerRuntime runtime("thrower", "test/ThrowingChainable", std::make_shared<IoGate>());
  const testing::LogCapture log;
  bool set = true;
  auto job = runtime.owner().submit(
    [&]
    {
      runtime.set_plugin(plugin);
      set = runtime.set_chained(true).empty();
    });
  ASSERT_TRUE(runtime.owner().wait(job, Clock::now() + std::chrono::seconds(5)));
  EXPECT_FALSE(set);
  EXPECT_TRUE(log.contains("chain mode refused 4711")) << log.text();
  runtime.set_plugin(nullptr);
}

}  // namespace
}  // namespace hector_controller_manager::runtime
