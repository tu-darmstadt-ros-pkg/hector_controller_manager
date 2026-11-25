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

#ifndef RUNTIME__RT_THREAD_HPP_
#define RUNTIME__RT_THREAD_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace hector_controller_manager::testing
{
/// A control loop on a thread of its own until stopped. Each cycle gets a steady time that starts
/// at zero and advances by the period, and the thread sleeps for `pace` after it.
class RtThread
{
public:
  using Cycle = std::function<void(const rclcpp::Time &, const rclcpp::Duration &)>;

  RtThread() = default;
  explicit RtThread(
    Cycle cycle, double period = 0.001,
    std::chrono::milliseconds pace = std::chrono::milliseconds(1))
  {
    start(std::move(cycle), period, pace);
  }
  ~RtThread() { stop(); }
  RtThread(const RtThread &) = delete;
  RtThread & operator=(const RtThread &) = delete;

  /// Starts the loop, once per RtThread since a stopped one stays stopped.
  void start(
    Cycle cycle, double period = 0.001,
    std::chrono::milliseconds pace = std::chrono::milliseconds(1))
  {
    thread_ = std::thread(
      [this, cycle = std::move(cycle), step = rclcpp::Duration::from_seconds(period), pace]
      {
        rclcpp::Time time(0, 0, RCL_STEADY_TIME);
        while (!stop_)
        {
          if (!paused)
          {
            cycle(time, step);
            cycles_.fetch_add(1);
            time += step;
          }
          std::this_thread::sleep_for(pace);
        }
      });
  }

  /// Ends the loop after its current cycle.
  void stop()
  {
    stop_ = true;
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  int cycles() const { return cycles_.load(); }

  /// Skips the cycles while set, the time stands still meanwhile.
  std::atomic<bool> paused{false};

private:
  std::atomic<bool> stop_{false};
  std::atomic<int> cycles_{0};
  std::thread thread_;
};

/// One cycle of a controller manager or of a supervisor's RT.
template <typename Rt>
void read_update_write(Rt & rt, const rclcpp::Time & time, const rclcpp::Duration & period)
{
  rt.read(time, period);
  rt.update(time, period);
  rt.write(time, period);
}

}  // namespace hector_controller_manager::testing

#endif  // RUNTIME__RT_THREAD_HPP_
