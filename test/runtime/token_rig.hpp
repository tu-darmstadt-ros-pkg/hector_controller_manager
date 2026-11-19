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

#ifndef RUNTIME__TOKEN_RIG_HPP_
#define RUNTIME__TOKEN_RIG_HPP_

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fakes.hpp"
#include "gate.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/supervisor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "supervisor_rig.hpp"

// Fakes that write each update's token, the epoch of its activation and its count, so the hardware
// can tell whether a command it applies belongs to a running activation.
namespace hector_controller_manager::testing::tokens
{
using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::return_type;
using namespace std::chrono_literals;

std::string urdf(bool async)
{
  return std::string(R"(<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <link name="l1"/>
  <link name="l2"/>
  <joint name="j1" type="continuous"><parent link="base"/><child link="l1"/></joint>
  <joint name="j2" type="continuous"><parent link="base"/><child link="l2"/></joint>
  <ros2_control name="system" type="system" is_async=")") +
         (async ? "true" : "false") + R"(">
    <hardware><plugin>test/System</plugin></hardware>
    <joint name="j1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="j2">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>)";
}

const std::array<std::string, 3> INTERFACES = {"j1/position", "j1/velocity", "j2/velocity"};
constexpr size_t J1_POSITION = 0;
constexpr size_t J1_VELOCITY = 1;
constexpr size_t J2_VELOCITY = 2;
/// A command is a token, the activation's epoch times this plus the update that wrote it.
constexpr double EPOCH = 1e6;
/// The update token of an update RT was stuck in.
constexpr double LATE = 999998.0;
/// The update token of a command a deactivation leaves behind.
constexpr double HOLD = 999999.0;

struct PerformCall
{
  std::vector<std::string> start;
  std::vector<std::string> stop;
};

struct Observed
{
  std::array<std::atomic<bool>, 3> live{};
  std::array<std::atomic<uint32_t>, 3> applied_epoch{};
  std::atomic<int> applied_j2{0};
  /// Applied tokens of an update RT was stuck in.
  std::atomic<int> late_applied{0};
  /// Writes that carry NaN on a live interface.
  std::array<std::atomic<int>, 3> nan_writes{};
  std::atomic<int> writes{0};
  /// Applied commands whose activation had ended, and the last one of them.
  std::atomic<int> violations{0};
  std::atomic<double> violation{0.0};
  /// Writes that carry a command on an interface the hardware stopped.
  std::atomic<int> stopped_commands{0};
  std::atomic<int> performs{0};
  std::mutex calls_mutex;
  std::vector<PerformCall> calls;

  std::array<std::atomic<bool>, 64> active_epochs{};
  std::atomic<uint32_t> next_epoch{1};

  std::atomic<int> skip_first_updates{0};
  std::atomic<bool> write_on_deactivate{false};
  std::atomic<bool> stall_j1_update{false};
  std::atomic<bool> in_stalled_update{false};
  std::atomic<uint32_t> failing_epoch{0};
  /// The next activation writes a command, then returns this.
  std::atomic<CallbackReturn> failing_activation{CallbackReturn::SUCCESS};
  std::atomic<CallbackReturn> deactivation{CallbackReturn::SUCCESS};
  std::atomic<int> configures{0};
  std::atomic<int> configure_delay_ms{0};
  std::atomic<int> activate_delay_once_ms{0};
  std::atomic<int> perform_delay_ms{0};
  Gate activate_gate;
  Gate deactivate_gate;
  Gate perform_gate;
  /// The next performs fail, by an error or by an exception.
  std::atomic<int> perform_failures{0};
  std::atomic<bool> perform_throws{false};
  std::atomic<bool> fail_read{false};
  Gate read_gate;
  /// Plugin callbacks that run right now, every one that uses this state. The CM leaks a plugin
  /// whose call it gave up on, so a test waits for them before this state goes away.
  std::atomic<int> running{0};
};

class System : public hardware_interface::SystemInterface
{
public:
  explicit System(Observed & observed) : observed_(observed) {}

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    observed_.read_gate.pass();
    return observed_.fail_read ? return_type::ERROR : return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    const InFlight running(observed_.running);
    observed_.writes.fetch_add(1);
    for (size_t i = 0; i < INTERFACES.size(); ++i)
    {
      double value = 0.0;
      if (!get_command(get_command_interface_handle(INTERFACES[i]), value, false))
      {
        continue;
      }
      if (std::isnan(value))
      {
        observed_.nan_writes[i].fetch_add(observed_.live[i] ? 1 : 0);
        continue;
      }
      if (!observed_.live[i])
      {
        observed_.stopped_commands.fetch_add(1);
        continue;
      }
      observed_.late_applied.fetch_add(std::fmod(value, EPOCH) == LATE ? 1 : 0);
      const auto epoch = static_cast<uint32_t>(value / EPOCH);
      if (epoch >= observed_.active_epochs.size() || !observed_.active_epochs[epoch])
      {
        observed_.violations.fetch_add(1);
        observed_.violation = value;
        continue;
      }
      observed_.applied_epoch[i] = epoch;
      if (i == J2_VELOCITY)
      {
        observed_.applied_j2.fetch_add(1);
      }
    }
    return return_type::OK;
  }

  return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override
  {
    const InFlight running(observed_.running);
    std::this_thread::sleep_for(std::chrono::milliseconds(observed_.perform_delay_ms.load()));
    observed_.perform_gate.pass();
    observed_.performs.fetch_add(1);
    if (observed_.perform_failures.load() > 0)
    {
      observed_.perform_failures.fetch_sub(1);
      if (observed_.perform_throws)
      {
        throw std::runtime_error("perform threw");
      }
      return return_type::ERROR;
    }
    {
      std::lock_guard<std::mutex> lock(observed_.calls_mutex);
      observed_.calls.push_back({start, stop});
    }
    for (size_t i = 0; i < INTERFACES.size(); ++i)
    {
      for (const auto & name : stop)
      {
        observed_.live[i] = observed_.live[i] && name != INTERFACES[i];
      }
      for (const auto & name : start)
      {
        observed_.live[i] = observed_.live[i] || name == INTERFACES[i];
      }
    }
    return return_type::OK;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    const InFlight running(observed_.running);
    for (auto & live : observed_.live)
    {
      live = false;
    }
    return CallbackReturn::SUCCESS;
  }

private:
  Observed & observed_;
};

/// Writes tokens of its activation into one command interface, chosen by its name.
class Writer : public NoInterfaces<>
{
public:
  Writer(Observed & observed, std::string interface)
  : observed_(observed), interface_(std::move(interface))
  {
  }

  InterfaceConfiguration command_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, {interface_}};
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const InFlight running(observed_.running);
    observed_.configures.fetch_add(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(observed_.configure_delay_ms.load()));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    const InFlight running(observed_.running);
    if (
      const auto result = observed_.failing_activation.exchange(CallbackReturn::SUCCESS);
      result != CallbackReturn::SUCCESS)
    {
      // A command of an activation that never runs
      (void)command_interfaces_[0].set_value(observed_.next_epoch.fetch_add(1) * EPOCH + 1.0);
      return result;
    }
    std::this_thread::sleep_for(
      std::chrono::milliseconds(observed_.activate_delay_once_ms.exchange(0)));
    observed_.activate_gate.pass();
    epoch_ = observed_.next_epoch.fetch_add(1);
    observed_.active_epochs[epoch_] = true;
    updates_ = 0;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    const InFlight running(observed_.running);
    observed_.active_epochs[epoch_] = false;
    observed_.deactivate_gate.pass();
    if (observed_.write_on_deactivate)
    {
      // Like a controller that leaves a hold command behind
      (void)command_interfaces_[0].set_value(epoch_ * EPOCH + HOLD);
    }
    return observed_.deactivation.load();
  }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    const InFlight running(observed_.running);
    bool stalled = false;
    while (observed_.stall_j1_update && interface_ != INTERFACES[J2_VELOCITY])
    {
      stalled = true;
      observed_.in_stalled_update = true;
      std::this_thread::sleep_for(1ms);
    }
    observed_.in_stalled_update = false;
    if (stalled)
    {
      (void)command_interfaces_[0].set_value(epoch_ * EPOCH + LATE);
      return controller_interface::return_type::OK;
    }
    if (epoch_ == observed_.failing_epoch)
    {
      return controller_interface::return_type::ERROR;
    }
    ++updates_;
    if (updates_ > observed_.skip_first_updates)
    {
      (void)command_interfaces_[0].set_value(epoch_ * EPOCH + updates_);
    }
    return controller_interface::return_type::OK;
  }

private:
  Observed & observed_;
  std::string interface_;
  uint32_t epoch_ = 0;
  int updates_ = 0;
};

/// A supervisor on one system with j1 position, j1 velocity and j2 velocity, sync or async by the
/// parameter. Every test ends with no command of an ended activation applied.
class TokenRig : public SupervisorRig<::testing::TestWithParam<bool>>
{
protected:
  void SetUp() override
  {
    SupervisorConfig config;
    config.default_timeout = FINISH_TIMEOUT;
    config.hang_timeout = hang_timeout_;
    start(config, host_, urdf(GetParam()));
  }

  void TearDown() override
  {
    observed_.stall_j1_update = false;
    pause_rt_ = false;
    // Shutdown must not start a callback that outlives it, the plugin would be leaked with it
    for (auto * gate :
         {&observed_.activate_gate, &observed_.deactivate_gate, &observed_.perform_gate,
          &observed_.read_gate})
    {
      gate->open();
    }
    for (auto * delay :
         {&observed_.configure_delay_ms, &observed_.activate_delay_once_ms,
          &observed_.perform_delay_ms})
    {
      *delay = 0;
    }
    // A call given up on runs later on its owner and still uses observed_, which ends with the test
    EXPECT_TRUE(eventually([&] { return !quarantined(); }));
    stop();
    EXPECT_TRUE(eventually([&] { return observed_.running.load() == 0; }));
    EXPECT_EQ(observed_.violations.load(), 0) << observed_.violation.load();
  }

  void add(const std::string & name) { load_and_configure(name, "test/Writer"); }

  /// Adds and starts the controllers, true once the hardware applies the commands of vel, the
  /// newest activation, and of j2 if started.
  bool running(const core::Names & names)
  {
    for (const auto & name : names)
    {
      add(name);
    }
    return switch_controllers(names, {}).ok && applies_newest(J1_VELOCITY) &&
           (!names.count("j2") || eventually([&] { return observed_.applied_j2.load() > 0; }));
  }

  /// Waits until the hardware applies the newest activation on the interface.
  bool applies_newest(size_t interface)
  {
    return eventually(
      [&] { return observed_.applied_epoch[interface].load() + 1 == observed_.next_epoch.load(); });
  }

  bool stalled()
  {
    return eventually([&] { return observed_.in_stalled_update.load(); });
  }

  /// True if the hardware applies writes again over a few cycles.
  bool writes_go_on()
  {
    const int before = observed_.writes.load();
    return eventually([&] { return observed_.writes.load() > before + 5; }, FINISH_TIMEOUT);
  }

  std::vector<PerformCall> calls()
  {
    std::lock_guard<std::mutex> lock(observed_.calls_mutex);
    return observed_.calls;
  }

  std::chrono::milliseconds hang_timeout_{100};
  Observed observed_;
  FakeHost host_{
    [this](const std::string & name, const std::string &)
    {
      const std::string interface = name.rfind("pos", 0) == 0  ? INTERFACES[J1_POSITION]
                                    : name.rfind("j2", 0) == 0 ? INTERFACES[J2_VELOCITY]
                                                               : INTERFACES[J1_VELOCITY];
      return std::make_shared<Writer>(observed_, interface);
    },
    [this] { return std::make_unique<System>(observed_); }};
};

}  // namespace hector_controller_manager::testing::tokens

#endif  // RUNTIME__TOKEN_RIG_HPP_
