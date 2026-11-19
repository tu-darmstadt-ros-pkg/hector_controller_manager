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

#ifndef HECTOR_CONTROLLER_MANAGER__PLUGIN_HOST_HPP_
#define HECTOR_CONTROLLER_MANAGER__PLUGIN_HOST_HPP_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "hardware_interface/hardware_component_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hector_controller_manager/runtime/controller_runtime.hpp"
#include "hector_controller_manager/runtime/owner_thread.hpp"

namespace hector_controller_manager
{
using runtime::Clock;

/// What the shell provides to create plugins. Called on the owner thread of the new entity.
class PluginHost
{
public:
  virtual ~PluginHost() = default;
  /// Creates and initializes the controller. Throws with the reason on failure.
  virtual runtime::ControllerRuntime::Plugin create_controller(
    const std::string & name, const std::string & type) = 0;
  /// The controller is about to be destroyed. False if destroying it is not safe yet, then it is
  /// abandoned instead.
  virtual bool forget_controller(const runtime::ControllerRuntime::Plugin & plugin) = 0;
  virtual std::unique_ptr<hardware_interface::HardwareComponentInterface> create_hardware(
    const hardware_interface::HardwareInfo & info) = 0;
  virtual hardware_interface::HardwareComponentParams hardware_params(
    const hardware_interface::HardwareInfo & info) = 0;
};

/// The host for owner jobs, which may outlive the supervisor after a timeout. Closed when the
/// supervisor goes, so a late job never calls a destroyed host.
class HostAccess
{
public:
  /// Calls `use` with the host unless the supervisor is gone. No lock is held during the call,
  /// so one hanging job never delays another.
  template <typename Use>
  void with_host(Use use)
  {
    PluginHost * host = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!host_)
      {
        return;
      }
      host = host_;
      ++in_use_;
    }
    struct Release
    {
      HostAccess & access;
      ~Release()
      {
        std::lock_guard<std::mutex> lock(access.mutex_);
        --access.in_use_;
        access.released_.notify_all();
      }
    } release{*this};
    use(*host);
  }
  void open(PluginHost & host) { host_ = &host; }
  /// Ends access and waits for calls in flight. False if one still runs at the deadline.
  bool close(Clock::time_point deadline)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    host_ = nullptr;
    return released_.wait_until(lock, deadline, [this] { return in_use_ == 0; });
  }

private:
  std::mutex mutex_;
  std::condition_variable released_;
  PluginHost * host_ = nullptr;
  int in_use_ = 0;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__PLUGIN_HOST_HPP_
