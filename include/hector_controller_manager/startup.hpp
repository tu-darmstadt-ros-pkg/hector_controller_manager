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

#ifndef HECTOR_CONTROLLER_MANAGER__STARTUP_HPP_
#define HECTOR_CONTROLLER_MANAGER__STARTUP_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "hector_controller_manager/request_threads.hpp"
#include "hector_controller_manager/supervisor.hpp"
#include "rclcpp/node.hpp"

namespace hector_controller_manager
{
/// The startup configuration (the `startup.*` and `hardware_components_initial_state.*`
/// parameters) at the bring up and again after an e-stop release, see "Autostart on release" in the
/// design.
///
/// - The startup thread reads the hardware targets before the initialization and applies the rest
///   once it is done.
/// - A release starts an autostart on a request thread. It brings the hardware up and runs the
///   startup switch that restores.
/// - An explicit hardware activation runs the same switch on the request's thread.
class Startup
{
public:
  struct Settings
  {
    std::chrono::milliseconds cooldown;
    int64_t attempts;
  };

  /// `engaged` reads the e-stop without a lock, `load` loads a controller by its parameters.
  Startup(
    rclcpp::Node & node, Supervisor & supervisor, RequestThreads & threads, Settings settings,
    std::function<bool()> engaged, std::function<Response(const std::string &)> load);

  /// The target state of every component the startup configuration names, the others go active.
  /// Kept for the autostart, call it once before the initialization.
  std::map<std::string, core::Lifecycle> read_hardware_targets();
  /// Loads and configures the startup controllers, then runs the startup switch.
  void apply();
  /// The startup switch after a hardware activation brought a component up.
  void restore();
  /// Called with the EStopMonitor's lock held when the e-stop flipped. A release starts an
  /// autostart on a request thread.
  void on_change(bool engaged, const std::string & cause);
  /// Ends every autostart at its next pause.
  void stop();

private:
  /// One AUTO switch to the startup controllers that are not active. `restore` skips protected
  /// ones, only an explicit request brings those back after a fault.
  void startup_switch(bool restore);
  void run(uint64_t release);
  /// False once the e-stop engaged, `stop` was called or a newer release won.
  bool pause(uint64_t release, std::chrono::milliseconds time);

  rclcpp::Node & node_;
  Supervisor & supervisor_;
  RequestThreads & threads_;
  const rclcpp::Logger logger_;
  const Settings settings_;
  const std::function<bool()> engaged_;
  const std::function<Response(const std::string &)> load_;
  std::map<std::string, core::Lifecycle> startup_hardware_;
  /// Taken after the EStopMonitor's lock, guards the members below up to the condition variable.
  std::mutex mutex_;
  /// Counts releases, a newer one supersedes the autostart of an older one.
  uint64_t releases_ = 0;
  bool stopping_ = false;
  std::condition_variable changed_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__STARTUP_HPP_
