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

#include "hector_controller_manager/node.hpp"

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "hector_controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace hector_controller_manager
{
namespace
{
/// How long a wait for the clock goes before it checks for a stop, as no tick may come.
constexpr std::chrono::milliseconds STOP_CHECK{100};
}  // namespace

void run_on_ros_clock(
  rclcpp::Clock & clock, std::chrono::nanoseconds period, const std::function<bool()> & running,
  const Cycle & cycle)
{
  std::mutex mutex;
  std::condition_variable ticked;
  // Clock::sleep_until ignores a backward jump and would wait for the old target
  rcl_jump_threshold_t threshold{};
  threshold.on_clock_change = true;
  threshold.min_forward.nanoseconds = 1;
  threshold.min_backward.nanoseconds = -1;
  const auto on_jump = clock.create_jump_callback(
    nullptr,
    [&](const rcl_time_jump_t &)
    {
      std::lock_guard<std::mutex> lock(mutex);
      ticked.notify_all();
    },
    threshold);
  const auto wait = [&](const auto & ready)
  {
    std::unique_lock<std::mutex> lock(mutex);
    while (running() && !ready())
    {
      ticked.wait_for(lock, STOP_CHECK);
    }
    return running();
  };
  if (!wait([&] { return clock.started(); }))
  {
    return;
  }
  const rclcpp::Duration ros_period(period);
  rclcpp::Time previous = clock.now();
  while (wait(
    [&]
    {
      const rclcpp::Time now = clock.now();
      return now < previous || now >= previous + ros_period;
    }))
  {
    const rclcpp::Time now = clock.now();
    // A backward jump, such as a world reset, starts the schedule again from there
    if (now >= previous)
    {
      cycle(now, now - previous);
    }
    previous = now;
  }
}

void run_on_steady_clock(
  std::chrono::nanoseconds period, const std::function<bool()> & running, const Cycle & cycle,
  const rclcpp::Logger & logger)
{
  rclcpp::Clock clock(RCL_STEADY_TIME);
  rclcpp::Time previous = clock.now();
  auto next = std::chrono::steady_clock::now();
  while (true)
  {
    next += period;
    const auto late = std::chrono::steady_clock::now() - next;
    if (late > std::chrono::nanoseconds::zero())
    {
      // A burst of cycles would send the bus every missed write at once
      const auto missed = late / period + 1;
      next += missed * period;
      RCLCPP_WARN_THROTTLE(
        logger, clock, 1000, "The control loop overran its period, %ld cycles skipped",
        static_cast<long>(missed));
    }
    std::this_thread::sleep_until(next);
    if (!running())
    {
      return;
    }
    const rclcpp::Time now = clock.now();
    cycle(now, now - previous);
    previous = now;
  }
}

int run_node(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto cm = std::make_shared<ControllerManager>(nullptr, "controller_manager");
  const auto executor = cm->executor();
  executor->add_node(cm);
  const int priority = cm->has_parameter("thread_priority")
                         ? static_cast<int>(cm->get_parameter("thread_priority").as_int())
                         : cm->declare_parameter<int>("thread_priority", 50);
  const bool use_sim_time = cm->get_parameter("use_sim_time").as_bool();
  std::thread control_loop(
    [cm, priority, use_sim_time]
    {
      if (realtime_tools::has_realtime_kernel() && !realtime_tools::configure_sched_fifo(priority))
      {
        RCLCPP_WARN(cm->get_logger(), "Could not enable FIFO RT scheduling policy");
      }
      const auto period = std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());
      const auto running = [] { return rclcpp::ok(); };
      const auto cycle = [&cm](const rclcpp::Time & now, const rclcpp::Duration & measured)
      {
        cm->read(now, measured);
        cm->update(now, measured);
        cm->write(now, measured);
      };
      if (use_sim_time)
      {
        run_on_ros_clock(*cm->get_clock(), period, running, cycle);
      }
      else
      {
        run_on_steady_clock(period, running, cycle, cm->get_logger());
      }
    });

  executor->spin();
  control_loop.join();
  // The pre-shutdown callback ran the shutdown that counts, this one reports it
  const bool clean = cm->shutdown();
  rclcpp::shutdown();
  return clean ? 0 : 1;
}
}  // namespace hector_controller_manager
