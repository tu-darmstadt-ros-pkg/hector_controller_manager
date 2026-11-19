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
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "cm_test_suite/faults.hpp"
#include "cm_test_suite/rig.hpp"
#include "hector_controller_manager/controller_manager.hpp"
#include "hector_controller_manager/entities.hpp"
#include "hector_controller_manager/node.hpp"
#include "rcl/init.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

/// The default timeout of requests the test node runs with, and the supervisor's default grace.
constexpr auto DEFAULT_TIMEOUT = 1000ms;
constexpr auto GRACE = 400ms;
/// rclcpp and the RMW tear down after the shutdown, and a loaded machine schedules late.
constexpr auto TEARDOWN = 2500ms;

std::string list(const std::set<std::string> & names)
{
  std::string result = "[";
  for (const auto & name : names)
  {
    result += (result.size() > 1 ? ", " : "") + name;
  }
  return result + "]";
}

/// The rig's startup as a parameter file of the node "controller_manager".
std::string parameters(const std::string & extra)
{
  std::string yaml = "controller_manager:\n  ros__parameters:\n    robot_description: |\n";
  std::istringstream urdf(cm_test_suite::rig::urdf());
  for (std::string line; std::getline(urdf, line);)
  {
    yaml += "      " + line + "\n";
  }
  yaml += "    default_timeout_ms: " + std::to_string(DEFAULT_TIMEOUT.count()) + "\n";
  for (const auto & controller : cm_test_suite::rig::controllers())
  {
    yaml += "    " + controller.name +
            ":\n      type: " + cm_test_suite::rig::plugin_type(controller) + "\n";
  }
  const auto & startup = cm_test_suite::rig::startup_profile();
  yaml += "    startup:\n      hardware_active: " + list(startup.hardware_active) +
          "\n      controllers_loaded: " + list(startup.controllers_loaded) +
          "\n      controllers_active: " + list(startup.controllers_active) + "\n";
  return yaml + extra;
}

/// An e-stop topic of this test run, so concurrent runs in one domain do not cross-talk. A child
/// process passes the pid of the test process, its parent.
std::string run_topic(const std::string & name, pid_t test_process = ::getpid())
{
  return "/hcm_signals_" + name + "_" + std::to_string(test_process);
}

/// A scenario the child process runs, see main.
enum class Scenario
{
  Node,
  RequestInFlight,
  ContextFirst,
  ContextGone,
  Release,
};

/// This binary again in a child process that runs the scenario, its output in a pipe. A fresh
/// process, since threads may not start after a fork of a threaded one (TSan has its own).
class ChildProcess
{
public:
  ChildProcess(const std::string & extra, Scenario scenario)
  : file_(
      std::filesystem::temp_directory_path() /
      ("hcm_signals_" + std::to_string(::getpid()) + "_" + std::to_string(++children_) + ".yaml"))
  {
    std::ofstream(file_) << parameters(extra);
    std::vector<std::string> args = {
      "/proc/self/exe", "--child", std::to_string(static_cast<int>(scenario)), file_};
    std::vector<char *> argv;
    for (auto & arg : args)
    {
      argv.push_back(arg.data());
    }
    argv.push_back(nullptr);
    int fds[2];
    if (::pipe(fds) != 0)
    {
      throw std::runtime_error("pipe failed");
    }
    pid_ = ::fork();
    if (pid_ == 0)
    {
      ::dup2(fds[1], STDOUT_FILENO);
      ::dup2(fds[1], STDERR_FILENO);
      ::close(fds[0]);
      ::close(fds[1]);
      ::execv(argv[0], argv.data());
      ::_exit(127);
    }
    ::close(fds[1]);
    out_ = fds[0];
  }

  ~ChildProcess()
  {
    if (pid_ > 0 && !exit_code_)
    {
      ::kill(pid_, SIGKILL);
      ::waitpid(pid_, nullptr, 0);
    }
    ::close(out_);
    std::filesystem::remove(file_);
  }

  /// Reads the output until it contains `text`.
  bool wait_for(const std::string & text, SteadyClock::duration timeout = 20s)
  {
    const auto deadline = SteadyClock::now() + timeout;
    while (output_.find(text) == std::string::npos && SteadyClock::now() < deadline)
    {
      read_some(10ms);
    }
    return output_.find(text) != std::string::npos;
  }

  /// Sends the signal, none for 0, and returns how long the process took to exit, if it did within
  /// `timeout`.
  std::optional<SteadyClock::duration> stop(int signal, SteadyClock::duration timeout)
  {
    const auto begin = SteadyClock::now();
    ::kill(pid_, signal);
    while (SteadyClock::now() - begin < timeout)
    {
      int status = 0;
      if (::waitpid(pid_, &status, WNOHANG) == pid_)
      {
        exit_code_ = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
        return SteadyClock::now() - begin;
      }
      read_some(10ms);
    }
    return std::nullopt;
  }

  std::optional<int> exit_code() const { return exit_code_; }
  const std::string & output() const { return output_; }

private:
  void read_some(std::chrono::milliseconds wait)
  {
    pollfd fd{out_, POLLIN, 0};
    if (::poll(&fd, 1, static_cast<int>(wait.count())) > 0)
    {
      char buffer[4096];
      const ssize_t count = ::read(out_, buffer, sizeof(buffer));
      if (count > 0)
      {
        output_.append(buffer, static_cast<size_t>(count));
      }
      else
      {
        std::this_thread::sleep_for(wait);
      }
    }
  }

  static inline int children_ = 0;
  std::filesystem::path file_;
  pid_t pid_ = -1;
  int out_ = -1;
  std::string output_;
  std::optional<int> exit_code_;
};

/// Waits for the ready lines, sends the signal, none for 0, and expects a clean exit in time. The
/// controllers refuse their stop only where the context ended before it.
void ends_in_time(
  ChildProcess & node, int signal, const std::vector<std::string> & ready, bool refused = false)
{
  for (const auto & text : ready)
  {
    ASSERT_TRUE(node.wait_for(text)) << node.output();
  }
  const auto took = node.stop(signal, DEFAULT_TIMEOUT + GRACE + TEARDOWN);
  const std::string output = node.output().substr(0, 20000);
  ASSERT_TRUE(took.has_value()) << "still running, output:\n" << output;
  EXPECT_EQ(node.exit_code(), 0) << output;
  EXPECT_EQ(node.output().find("Sanitizer"), std::string::npos) << output;
  EXPECT_EQ(node.output().find("refused to stop") != std::string::npos, refused) << output;
}

TEST(Signals, sigint_ends_a_running_node)
{
  ChildProcess node("", Scenario::Node);
  ends_in_time(node, SIGINT, {"Startup activated"});
}

TEST(Signals, sigterm_ends_a_running_node)
{
  ChildProcess node("", Scenario::Node);
  ends_in_time(node, SIGTERM, {"Startup activated"});
}

TEST(Signals, sigint_ends_a_node_with_an_estop_topic)
{
  ChildProcess node("    estop_topics: [" + run_topic("estop") + "]\n", Scenario::Node);
  ends_in_time(node, SIGINT, {"Startup activated"});
}

/// The autostart's activation of base_vel hangs.
void arm_a_hanging_autostart()
{
  cm_test_suite::faults().arm(
    {cm_test_suite::TargetKind::Controller, *cm_test_suite::rig::controller_index("base_vel")},
    cm_test_suite::Hook::Activate, cm_test_suite::Fault::Hang);
}

/// Releases the e-stop topic and keeps its publisher until a signal ends the process.
int release(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("signals_estop");
  auto publisher = node->create_publisher<std_msgs::msg::Bool>(
    run_topic("inflight", ::getppid()), rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::Bool released;
  released.data = false;
  publisher->publish(released);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// The autostart's switch outlives its deadline in on_activate, the shutdown queues behind it. The
// activation still hangs when the shutdown ends, so the exit code reports an incomplete shutdown.
TEST(Signals, sigint_ends_a_node_with_a_request_in_flight)
{
  ChildProcess node(
    "    estop_topics: [" + run_topic("inflight") + "]\n    autostart_cooldown_ms: 0\n",
    Scenario::RequestInFlight);
  ASSERT_TRUE(node.wait_for("Startup activated")) << node.output();
  // A process of its own, a thread beside the node's startup would race rclcpp's initialization
  ChildProcess releaser("", Scenario::Release);
  ASSERT_TRUE(node.wait_for("E-stop released")) << node.output();
  ASSERT_TRUE(node.wait_for("Step: activate base_vel")) << "precondition: the switch is in flight\n"
                                                        << node.output();
  const auto took = node.stop(SIGINT, 2 * (DEFAULT_TIMEOUT + GRACE) + TEARDOWN);
  ASSERT_TRUE(took.has_value()) << "still running, output:\n" << node.output().substr(0, 20000);
  EXPECT_EQ(node.exit_code(), 1) << node.output();
  EXPECT_NE(node.output().find("Shutdown incomplete"), std::string::npos) << node.output();
  EXPECT_EQ(node.output().find("Sanitizer"), std::string::npos) << node.output();
}

/// The controller manager in a process whose context another part shuts down first, as a simulator
/// that embeds it may. With `bypass` the context ends without its pre-shutdown callbacks.
int shut_down_after_the_context(int argc, char ** argv, bool bypass)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto cm = std::make_shared<ControllerManager>(nullptr, "controller_manager");
  const auto executor = cm->executor();
  executor->add_node(cm);
  std::thread spinner([&] { executor->spin(); });
  std::atomic<bool> stop{false};
  std::thread loop(
    [&]
    {
      const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);
      rclcpp::Time time(0, 0, RCL_STEADY_TIME);
      while (!stop)
      {
        cm->read(time, period);
        cm->update(time, period);
        cm->write(time, period);
        time += period;
        std::this_thread::sleep_for(10ms);
      }
    });
  const auto deadline = SteadyClock::now() + 20s;
  while (SteadyClock::now() < deadline)
  {
    auto response = std::make_shared<ControllerManager::ListControllers::Response>();
    cm->list_controllers_srv_cb(
      std::make_shared<ControllerManager::ListControllers::Request>(), response);
    if (
      std::any_of(
        response->controller.begin(), response->controller.end(), [](const auto & controller)
        { return controller.name == "jsb" && controller.state == "active"; }))
    {
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
  std::cout << "Context shuts down" << std::endl;
  if (bypass)
  {
    const auto context = rclcpp::contexts::get_global_default_context()->get_rcl_context();
    if (rcl_shutdown(context.get()) != RCL_RET_OK)
    {
      throw std::runtime_error("rcl_shutdown failed");
    }
    // Unlike rclcpp::shutdown, rcl does not wake the executor
    executor->cancel();
  }
  else
  {
    rclcpp::shutdown();
  }
  spinner.join();
  cm->shutdown();
  stop = true;
  loop.join();
  return 0;
}

// The controllers cannot transition any more, their stop must still end
TEST(Signals, a_shutdown_after_the_context_ends)
{
  ChildProcess node("", Scenario::ContextGone);
  ends_in_time(node, 0, {"Context shuts down"}, true);
}

// The controller manager stops before the context ends, without help of the embedding process
TEST(Signals, an_embedding_process_that_shuts_the_context_down_stops_the_controllers)
{
  ChildProcess node("", Scenario::ContextFirst);
  ends_in_time(node, 0, {"Context shuts down"});
}

}  // namespace
}  // namespace hector_controller_manager

/// Runs the tests, or as a child process the scenario named by `--child <scenario> <parameters>`.
int main(int argc, char ** argv)
{
  using hector_controller_manager::Scenario;
  if (argc == 4 && std::string(argv[1]) == "--child")
  {
    const auto scenario = static_cast<Scenario>(std::stoi(argv[2]));
    std::vector<std::string> args = {"ros2_control_node", "--ros-args", "--params-file", argv[3]};
    if (scenario == Scenario::RequestInFlight)
    {
      hector_controller_manager::arm_a_hanging_autostart();
      // The supervisor logs each step at debug level, which shows the activation in flight
      args.insert(args.begin() + 2, {"--log-level", "controller_manager:=debug"});
    }
    std::vector<char *> node_argv;
    for (auto & arg : args)
    {
      node_argv.push_back(arg.data());
    }
    node_argv.push_back(nullptr);
    const auto run = [&]
    {
      const int count = static_cast<int>(args.size());
      switch (scenario)
      {
        case Scenario::ContextFirst:
        case Scenario::ContextGone:
          return hector_controller_manager::shut_down_after_the_context(
            count, node_argv.data(), scenario == Scenario::ContextGone);
        case Scenario::Release:
          return hector_controller_manager::release(count, node_argv.data());
        default:
          return hector_controller_manager::run_node(count, node_argv.data());
      }
    };
    hector_controller_manager::exit_process(run());
  }
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
