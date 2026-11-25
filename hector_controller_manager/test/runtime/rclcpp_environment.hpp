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

#ifndef RUNTIME__RCLCPP_ENVIRONMENT_HPP_
#define RUNTIME__RCLCPP_ENVIRONMENT_HPP_

#include <gtest/gtest.h>

#include "rclcpp/utilities.hpp"

namespace hector_controller_manager::testing
{
/// One rclcpp context for every suite of a test binary, a second init after a shutdown crashes at
/// exit.
class Rclcpp : public ::testing::Environment
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
inline ::testing::Environment * const rclcpp_environment =
  ::testing::AddGlobalTestEnvironment(new Rclcpp());

}  // namespace hector_controller_manager::testing

#endif  // RUNTIME__RCLCPP_ENVIRONMENT_HPP_
