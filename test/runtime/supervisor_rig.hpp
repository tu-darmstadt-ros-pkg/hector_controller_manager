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

#ifndef RUNTIME__SUPERVISOR_RIG_HPP_
#define RUNTIME__SUPERVISOR_RIG_HPP_

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "fakes.hpp"
#include "gate.hpp"
#include "hector_controller_manager/supervisor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hector_controller_manager::testing
{
/// A supervisor whose RT runs about every millisecond on a thread of its own.
template <typename Base = ::testing::Test>
class SupervisorRig : public Base
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  /// Creates the supervisor, starts RT and initializes the supervisor with the description.
  void start(const SupervisorConfig & config, PluginHost & host, const std::string & urdf)
  {
    supervisor_ = std::make_unique<Supervisor>(config, host, rclcpp::get_logger("test"), nullptr);
    loop_ = std::thread(
      [this]
      {
        const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.001);
        rclcpp::Time time(0, 0, RCL_STEADY_TIME);
        while (!stop_)
        {
          if (!pause_rt_)
          {
            cycle(time, period);
            cycles_.fetch_add(1);
            time += period;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
    ASSERT_TRUE(supervisor_->initialize(urdf, {}).ok);
  }

  /// One RT cycle, which a test that watches RT wraps.
  virtual void cycle(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    supervisor_->rt().read(time, period);
    supervisor_->rt().update(time, period);
    supervisor_->rt().write(time, period);
  }

  /// Shuts the supervisor down and ends RT.
  void stop()
  {
    if (!supervisor_)
    {
      return;
    }
    supervisor_->shutdown(FINISH_TIMEOUT);
    stop_ = true;
    loop_.join();
    supervisor_.reset();
  }

  void TearDown() override { stop(); }

  Response switch_controllers(
    core::Names activate, core::Names deactivate, Clock::duration timeout = FINISH_TIMEOUT)
  {
    return supervisor_->switch_controllers(
      {std::move(activate), std::move(deactivate), core::Strictness::Strict}, timeout);
  }

  void load_and_configure(
    const std::string & name, const std::string & type, bool is_protected = false)
  {
    const auto loaded = supervisor_->load(name, type, is_protected);
    ASSERT_TRUE(loaded.ok) << loaded.message;
    const auto configured = supervisor_->configure(name);
    ASSERT_TRUE(configured.ok) << configured.message;
  }

  /// The status entry of the controller, with its runtime.
  std::optional<ControllerStatus> entry(const std::string & name)
  {
    // The snapshot must outlive the loop, a temporary would die before its body runs
    const auto status = supervisor_->status();
    for (const auto & entry : status->controllers)
    {
      if (entry.name == name)
      {
        return entry;
      }
    }
    return std::nullopt;
  }

  std::optional<core::Controller> controller(const std::string & name)
  {
    const auto found = entry(name);
    return found ? std::optional<core::Controller>(found->model) : std::nullopt;
  }

  core::Hardware hardware() { return supervisor_->status()->hardware.front().model; }

  bool quarantined()
  {
    const auto status = supervisor_->status();
    const auto pending = [](const auto & entry) { return entry.model.quarantined > 0; };
    return std::any_of(status->hardware.begin(), status->hardware.end(), pending) ||
           std::any_of(status->controllers.begin(), status->controllers.end(), pending);
  }

  /// Requests run in FIFO order and first reconcile what is left, so once a no-op switch was not
  /// answered busy the supervisor has done what the desired state asks for.
  bool settled() { return switch_controllers({}, {}).message != RequestQueue::busy().message; }

  /// Waits until RT completed `count` more cycles.
  bool cycles(int count)
  {
    const int start = cycles_.load();
    return eventually([&] { return cycles_.load() >= start + count; });
  }

  std::unique_ptr<Supervisor> supervisor_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> pause_rt_{false};
  std::atomic<int> cycles_{0};
  std::thread loop_;
};

}  // namespace hector_controller_manager::testing

#endif  // RUNTIME__SUPERVISOR_RIG_HPP_
