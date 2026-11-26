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

#include "hector_controller_manager/startup.hpp"

#include <cinttypes>
#include <utility>

#include "hector_controller_manager/core/names.hpp"
#include "hector_controller_manager/parameters.hpp"
#include "hector_controller_manager/runtime/thread_role.hpp"
#include "hector_controller_manager/runtime/thrown.hpp"
#include "rclcpp/logging.hpp"

namespace hector_controller_manager
{
namespace
{
constexpr std::chrono::milliseconds RETRY_PAUSE{100};
}  // namespace

Startup::Startup(
  rclcpp::Node & node, Supervisor & supervisor, RequestThreads & threads, Settings settings,
  std::function<bool()> engaged, std::function<Response(const std::string &)> load)
: node_(node),
  supervisor_(supervisor),
  threads_(threads),
  logger_(node.get_logger()),
  settings_(settings),
  engaged_(std::move(engaged)),
  load_(std::move(load))
{
}

std::map<std::string, core::Lifecycle> Startup::read_hardware_targets()
{
  runtime::expect_precondition(
    [this]
    {
      const auto status = supervisor_.status();
      return !status->initialized;
    },
    "Startup::read_hardware_targets after the initialization");
  std::map<std::string, core::Lifecycle> targets;
  for (const auto & name : string_list(node_, "hardware_components_initial_state.unconfigured"))
  {
    targets[name] = core::Lifecycle::Unconfigured;
  }
  for (const auto & name : string_list(node_, "hardware_components_initial_state.inactive"))
  {
    targets[name] = core::Lifecycle::Inactive;
  }
  for (const auto & name : string_list(node_, "startup.hardware_active"))
  {
    targets[name] = core::Lifecycle::Active;
  }
  startup_hardware_ = targets;
  return targets;
}

void Startup::apply()
{
  for (const auto & name : string_list(node_, "startup.controllers_loaded"))
  {
    const Response loaded = load_(name);
    const Response configured = loaded.ok ? supervisor_.configure(name) : loaded;
    if (!configured.ok)
    {
      RCLCPP_ERROR(logger_, "Startup: '%s' failed: %s", name.c_str(), configured.message.c_str());
    }
  }
  startup_switch(false);
}

void Startup::restore() { startup_switch(true); }

void Startup::startup_switch(bool restore)
{
  const auto [selected, response] = supervisor_.startup_switch(
    string_list(node_, "startup.controllers_active"), restore,
    supervisor_.config().default_timeout);
  if (response.ok && selected.empty())
  {
    return;
  }
  if (response.ok)
  {
    RCLCPP_INFO(logger_, "Startup activated %s", core::text(selected).c_str());
  }
  else if (selected.empty())
  {
    RCLCPP_ERROR(logger_, "Startup switch rejected: %s", response.message.c_str());
  }
  else
  {
    // The whole startup set is rejected, the process keeps running
    RCLCPP_ERROR(
      logger_, "Startup activates none of %s, the switch was rejected: %s",
      core::text(selected).c_str(), response.message.c_str());
  }
}

void Startup::on_change(bool engaged, const std::string & cause)
{
  std::lock_guard<std::mutex> lock(mutex_);
  changed_.notify_all();
  if (engaged)
  {
    RCLCPP_WARN(logger_, "E-stop engaged by %s", cause.c_str());
    return;
  }
  RCLCPP_INFO(logger_, "E-stop released, autostart follows");
  threads_.run(
    [this, release = ++releases_]
    {
      if (const auto error = runtime::thrown_by([&] { run(release); }))
      {
        RCLCPP_ERROR(logger_, "Autostart failed: %s", error->c_str());
      }
    });
}

void Startup::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  stopping_ = true;
  changed_.notify_all();
}

bool Startup::pause(uint64_t release, std::chrono::milliseconds time)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!changed_.wait_for(
        lock, time, [&] { return engaged_() || stopping_ || releases_ != release; }))
  {
    return true;
  }
  if (!stopping_)
  {
    RCLCPP_WARN(logger_, "Autostart aborted, the e-stop engaged or a newer release won");
  }
  return false;
}

void Startup::run(uint64_t release)
{
  // Initialization runs the startup itself once the description arrived
  if (const auto status = supervisor_.status(); !status->initialized)
  {
    return;
  }
  for (int64_t attempt = 1;; ++attempt)
  {
    // Hardware that lost power needs a moment to boot
    if (!pause(release, settings_.cooldown))
    {
      return;
    }
    std::string failures;
    // The status may lag a fault RT already saw, and activating active hardware is a no op
    const auto status = supervisor_.status();
    for (const auto & hardware : status->hardware)
    {
      const auto target = startup_hardware_.find(hardware.name);
      if (target != startup_hardware_.end() && target->second != core::Lifecycle::Active)
      {
        continue;
      }
      // A request that came first is no failed attempt, ask again within the state timeout
      const auto timeout = supervisor_.config().hardware_state_timeout;
      const auto give_up = Clock::now() + timeout;
      Response response =
        supervisor_.set_hardware_state(hardware.name, core::Lifecycle::Active, timeout);
      RCLCPP_INFO_EXPRESSION(
        logger_, !response.ok && response.try_again, "Autostart asks again for '%s': %s",
        hardware.name.c_str(), response.message.c_str());
      while (!response.ok && response.try_again && Clock::now() < give_up)
      {
        if (!pause(release, RETRY_PAUSE))
        {
          return;
        }
        response = supervisor_.set_hardware_state(hardware.name, core::Lifecycle::Active, timeout);
      }
      if (!response.ok)
      {
        failures += " " + response.message;
      }
    }
    if (failures.empty())
    {
      break;
    }
    if (attempt == settings_.attempts)
    {
      RCLCPP_ERROR(logger_, "Autostart gave up on the hardware:%s", failures.c_str());
      return;
    }
    RCLCPP_WARN(logger_, "Autostart attempt %" PRId64 " failed:%s", attempt, failures.c_str());
  }
  if (!pause(release, std::chrono::milliseconds::zero()))
  {
    return;
  }
  startup_switch(true);
}

}  // namespace hector_controller_manager
