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

// Measures the control cycle of one component with N joints and one controller that commands every
// joint. Not a test, ctest does not run it.
//
// bench_cycle --joints 8,32,128 --async 0|1 --limits 0|1 --cycles 200000 [--rate 1000]
//   [--command position|velocity] [--fifo 0|1] [--lock 0|1]

#include <sys/resource.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "fakes.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/supervisor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hector_controller_manager
{
namespace
{
using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::return_type;

struct Options
{
  std::vector<int> joints{8, 32, 128};
  bool async = false;
  bool limits = true;
  int cycles = 200000;
  int warmup = 2000;
  int rate = 1000;
  std::string command = "position";
  bool fifo = false;
  bool lock = false;
};

std::string joint(int index) { return "j" + std::to_string(index); }

std::string urdf(int joints, bool async, const std::string & command)
{
  std::ostringstream out;
  out << R"(<?xml version="1.0"?><robot name="bench"><link name="l0"/>)";
  for (int i = 0; i < joints; ++i)
  {
    out << "<link name=\"l" << i + 1 << "\"/><joint name=\"" << joint(i)
        << "\" type=\"revolute\"><parent link=\"l" << i << "\"/><child link=\"l" << i + 1
        << "\"/><limit lower=\"-3\" upper=\"3\" velocity=\"2\" effort=\"10\"/></joint>";
  }
  out << "<ros2_control name=\"system\" type=\"system\" is_async=\"" << (async ? "true" : "false")
      << "\"><hardware><plugin>bench/System</plugin></hardware>";
  for (int i = 0; i < joints; ++i)
  {
    out << "<joint name=\"" << joint(i) << "\"><command_interface name=\"" << command
        << "\"/><state_interface name=\"position\"/><state_interface name=\"velocity\"/>"
        << "<state_interface name=\"effort\"/></joint>";
  }
  out << "</ros2_control></robot>";
  return out.str();
}

/// Follows a position command at once, integrates a velocity command.
class System : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    joints_.clear();
    for (const auto & info : get_hardware_info().joints)
    {
      const auto & command = info.command_interfaces.front().name;
      joints_.push_back(
        {get_command_interface_handle(info.name + "/" + command).get(),
         get_state_interface_handle(info.name + "/position").get(),
         get_state_interface_handle(info.name + "/velocity").get(),
         get_state_interface_handle(info.name + "/effort").get(), command == "velocity", 0.0, 0.0});
    }
    return CallbackReturn::SUCCESS;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (auto & j : joints_)
    {
      (void)j.position->set_value(j.actual, false);
      (void)j.velocity->set_value(j.speed, false);
      (void)j.effort->set_value(0.0, false);
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    for (auto & j : joints_)
    {
      const double value = j.command->get_optional().value_or(NAN);
      if (std::isnan(value))
      {
        continue;
      }
      j.speed = j.velocity_command ? value : j.speed;
      j.actual = j.velocity_command ? j.actual + value * period.seconds() : value;
    }
    return return_type::OK;
  }

private:
  struct Joint
  {
    hardware_interface::CommandInterface * command;
    hardware_interface::StateInterface * position;
    hardware_interface::StateInterface * velocity;
    hardware_interface::StateInterface * effort;
    bool velocity_command;
    double actual;
    double speed;
  };
  std::vector<Joint> joints_;
};

/// Reads every position and commands every joint along a slow sine.
class Writer : public controller_interface::ControllerInterface
{
public:
  Writer(int joints, std::string command) : joints_(joints), command_(std::move(command)) {}

  InterfaceConfiguration command_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, names(command_)};
  }
  InterfaceConfiguration state_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, names("position")};
  }
  CallbackReturn on_init() override { return CallbackReturn::SUCCESS; }
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    double sum = 0.0;
    for (const auto & state : state_interfaces_)
    {
      sum += state.get_optional().value_or(0.0);
    }
    sink_ = sum;
    const double goal = 0.5 * std::sin(time.seconds());
    for (auto & command : command_interfaces_)
    {
      (void)command.set_value(goal);
    }
    return controller_interface::return_type::OK;
  }

private:
  std::vector<std::string> names(const std::string & interface) const
  {
    std::vector<std::string> result;
    for (int i = 0; i < joints_; ++i)
    {
      result.push_back(joint(i) + "/" + interface);
    }
    return result;
  }

  int joints_;
  std::string command_;
  volatile double sink_ = 0.0;
};

class Host : public PluginHost
{
public:
  Host(int joints, std::string command, unsigned int rate)
  : joints_(joints), command_(std::move(command)), rate_(rate)
  {
  }

  runtime::ControllerRuntime::Plugin create_controller(
    const std::string & name, const std::string &) override
  {
    auto controller = std::make_shared<Writer>(joints_, command_);
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = name;
    params.update_rate = rate_;
    params.controller_manager_update_rate = rate_;
    if (controller->init(params) != controller_interface::return_type::OK)
    {
      throw std::runtime_error("init failed");
    }
    return controller;
  }
  bool forget_controller(const runtime::ControllerRuntime::Plugin &) override { return true; }
  std::unique_ptr<hardware_interface::HardwareComponentInterface> create_hardware(
    const hardware_interface::HardwareInfo &) override
  {
    return std::make_unique<System>();
  }
  hardware_interface::HardwareComponentParams hardware_params(
    const hardware_interface::HardwareInfo & info) override
  {
    return testing::steady_params(info);
  }

private:
  int joints_;
  std::string command_;
  unsigned int rate_;
};

struct Samples
{
  explicit Samples(size_t count) : cycle(count), read(count), update(count), write(count) {}
  std::vector<int64_t> cycle, read, update, write;
};

int64_t percentile(std::vector<int64_t> values, double fraction)
{
  std::sort(values.begin(), values.end());
  return values[static_cast<size_t>(fraction * static_cast<double>(values.size() - 1))];
}

int64_t ns(Clock::duration duration)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

void check(const Response & response, const char * what)
{
  if (!response.ok)
  {
    throw std::runtime_error(std::string(what) + ": " + response.message);
  }
}

void run(int joints, const Options & options)
{
  SupervisorConfig config;
  config.update_rate = static_cast<unsigned int>(options.rate);
  config.enforce_command_limits = options.limits;
  if (options.limits)
  {
    for (int i = 0; i < joints; ++i)
    {
      config.limit_parameters[joint(i)].has_acceleration_limits = true;
      config.limit_parameters[joint(i)].max_acceleration = 50.0;
    }
  }
  config.lock_memory = options.lock;
  config.scheduler.realtime_kernel = [&options] { return options.fifo; };
  Host host(joints, options.command, config.update_rate);
  Supervisor supervisor(config, host, rclcpp::get_logger("bench"), nullptr);

  const auto count = static_cast<size_t>(options.cycles);
  Samples samples(count);
  rusage before{};
  rusage after{};
  std::atomic<bool> measure{false};
  std::atomic<bool> done{false};
  std::thread loop(
    [&]
    {
      if (options.fifo || options.lock)
      {
        supervisor.schedule_control_loop();
      }
      const auto period = std::chrono::nanoseconds(1'000'000'000 / options.rate);
      const rclcpp::Duration measured(period);
      auto next = Clock::now();
      int warmup = options.warmup;
      size_t index = 0;
      while (!done.load(std::memory_order_relaxed))
      {
        const auto start = Clock::now();
        const rclcpp::Time now(ns(start.time_since_epoch()), RCL_STEADY_TIME);
        supervisor.rt().read(now, measured);
        const auto read = Clock::now();
        supervisor.rt().update(now, measured);
        const auto update = Clock::now();
        supervisor.rt().write(now, measured);
        const auto write = Clock::now();
        if (measure.load(std::memory_order_relaxed) && warmup-- <= 0)
        {
          if (index == 0)
          {
            getrusage(RUSAGE_THREAD, &before);
          }
          samples.cycle[index] = ns(write - start);
          samples.read[index] = ns(read - start);
          samples.update[index] = ns(update - read);
          samples.write[index] = ns(write - update);
          if (++index == count)
          {
            getrusage(RUSAGE_THREAD, &after);
            done = true;
          }
        }
        next += period;
        std::this_thread::sleep_until(next);
      }
    });

  try
  {
    check(supervisor.initialize(urdf(joints, options.async, options.command), {}), "initialize");
    check(supervisor.load("writer", "bench/Writer"), "load");
    check(supervisor.configure("writer"), "configure");
    check(
      supervisor.switch_controllers(
        {{"writer"}, {}, core::Strictness::Strict}, std::chrono::seconds(5)),
      "switch");
  }
  catch (...)
  {
    done = true;
    loop.join();
    throw;
  }
  measure = true;
  loop.join();
  const auto status = supervisor.status();
  const auto & stats = status->hardware.front().runtime->stats();
  const uint64_t skipped = stats.skipped();
  const uint64_t clamped = stats.clamped();
  check(supervisor.shutdown(std::chrono::seconds(5)), "shutdown");

  std::printf(
    "joints %3d handles %3d async %d limits %d command %s cycles %zu | cycle p50 %6ld p99 %6ld "
    "max %7ld | read p50 %5ld p99 %5ld | update p50 %5ld p99 %5ld | write p50 %5ld p99 %5ld | "
    "minflt %ld majflt %ld nvcsw %ld nivcsw %ld skipped %lu clamped %lu\n",
    joints, joints * 4, options.async, options.limits, options.command.c_str(), count,
    percentile(samples.cycle, 0.5), percentile(samples.cycle, 0.99),
    *std::max_element(samples.cycle.begin(), samples.cycle.end()), percentile(samples.read, 0.5),
    percentile(samples.read, 0.99), percentile(samples.update, 0.5),
    percentile(samples.update, 0.99), percentile(samples.write, 0.5),
    percentile(samples.write, 0.99), after.ru_minflt - before.ru_minflt,
    after.ru_majflt - before.ru_majflt, after.ru_nvcsw - before.ru_nvcsw,
    after.ru_nivcsw - before.ru_nivcsw, static_cast<unsigned long>(skipped),
    static_cast<unsigned long>(clamped));
  std::fflush(stdout);
}

std::vector<int> list(const std::string & text)
{
  std::vector<int> values;
  std::stringstream stream(text);
  for (std::string item; std::getline(stream, item, ',');)
  {
    values.push_back(std::stoi(item));
  }
  return values;
}

/// `args` without the ROS arguments, the program name first.
Options parse(const std::vector<std::string> & args)
{
  Options options;
  for (size_t i = 1; i + 1 < args.size(); i += 2)
  {
    const std::string & key = args[i];
    const std::string & value = args[i + 1];
    if (key == "--joints")
    {
      options.joints = list(value);
    }
    else if (key == "--async")
    {
      options.async = value == "1";
    }
    else if (key == "--limits")
    {
      options.limits = value == "1";
    }
    else if (key == "--cycles")
    {
      options.cycles = std::stoi(value);
    }
    else if (key == "--warmup")
    {
      options.warmup = std::stoi(value);
    }
    else if (key == "--rate")
    {
      options.rate = std::stoi(value);
    }
    else if (key == "--command")
    {
      options.command = value;
    }
    else if (key == "--fifo")
    {
      options.fifo = value == "1";
    }
    else if (key == "--lock")
    {
      options.lock = value == "1";
    }
    else
    {
      throw std::invalid_argument("unknown option " + key);
    }
  }
  if (options.cycles <= 0)
  {
    // The percentiles need at least one cycle
    throw std::invalid_argument("--cycles must be positive");
  }
  return options;
}
}  // namespace
}  // namespace hector_controller_manager

int main(int argc, char ** argv)
{
  using namespace hector_controller_manager;
  const Options options = parse(rclcpp::init_and_remove_ros_arguments(argc, argv));
  rclcpp::get_logger("bench").set_level(rclcpp::Logger::Level::Warn);
  for (const int joints : options.joints)
  {
    run(joints, options);
  }
  rclcpp::shutdown();
  return 0;
}
