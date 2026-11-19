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

#ifndef HECTOR_CONTROLLER_MANAGER__NODE_HPP_
#define HECTOR_CONTROLLER_MANAGER__NODE_HPP_

#include <chrono>
#include <functional>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

namespace hector_controller_manager
{
using Cycle = std::function<void(const rclcpp::Time & now, const rclcpp::Duration & period)>;

/// The control loop under use_sim_time, paced like upstream ros2_control_node. The first cycle runs
/// one period after the clock started, each cycle gets the measured period, a late cycle does not
/// catch up, and a backward jump restarts the schedule. Returns once `running` is false.
void run_on_ros_clock(
  rclcpp::Clock & clock, std::chrono::nanoseconds period, const std::function<bool()> & running,
  const Cycle & cycle);

/// The control loop on the steady clock, paced like upstream ros2_control_node. Cycles keep a fixed
/// schedule, each gets the measured period, and a late cycle skips the cycles it missed instead of
/// catching up in a burst. Returns once `running` is false.
void run_on_steady_clock(
  std::chrono::nanoseconds period, const std::function<bool()> & running, const Cycle & cycle,
  const rclcpp::Logger & logger);

/// What ros2_control_node runs, the controller manager node with its control loop until SIGINT or
/// SIGTERM. Returns the exit code, 1 if the shutdown did not stop everything cleanly.
int run_node(int argc, char ** argv);

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__NODE_HPP_
