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

#include "hector_controller_manager/hardware_preparation.hpp"

#include <algorithm>
#include <exception>
#include <utility>

#include "hardware_interface/component_parser.hpp"
#include "hector_controller_manager/runtime/command_limiter.hpp"

namespace hector_controller_manager
{
namespace
{
std::optional<std::string> check_infos(
  std::vector<hardware_interface::HardwareInfo> & infos, const SupervisorConfig & config,
  const std::map<std::string, core::Lifecycle> & initial)
{
  const auto known = [&](const std::string & name)
  {
    return std::any_of(
      infos.begin(), infos.end(), [&](const auto & info) { return info.name == name; });
  };
  for (auto & info : infos)
  {
    if (info.rw_rate != 0 && info.rw_rate != config.update_rate)
    {
      return "Hardware '" + info.name + "' sets rw_rate " + std::to_string(info.rw_rate) +
             ", but read and write run at the controller manager's " +
             std::to_string(config.update_rate) + " Hz. Remove rw_rate or set it equal";
    }
    if (info.is_async && info.async_params.scheduling_policy != "synchronized")
    {
      return "Hardware '" + info.name + "' sets the scheduling_policy '" +
             info.async_params.scheduling_policy +
             "', but the controller manager runs every asynchronous component synchronized "
             "with its control loop. Remove scheduling_policy or set it to synchronized";
    }
    // Plugins derive periods from it, Dynamixel refuses to configure its bus watchdog with 0
    info.rw_rate = config.update_rate;
  }
  for (const auto & [name, state] : initial)
  {
    if (!known(name))
    {
      return "The initial state names unknown hardware '" + name + "'";
    }
  }
  for (const auto & name : config.command_limits_excluded)
  {
    if (!known(name))
    {
      return "The command limits exclude unknown hardware '" + name + "'";
    }
  }
  return std::nullopt;
}

}  // namespace

HardwarePreparation prepare_hardware(
  const std::string & description, const SupervisorConfig & config,
  const std::map<std::string, core::Lifecycle> & initial)
{
  std::vector<hardware_interface::HardwareInfo> infos;
  try
  {
    infos = hardware_interface::parse_control_resources_from_urdf(description);
  }
  catch (const std::exception & e)
  {
    return {{}, std::string("Invalid robot description: ") + e.what()};
  }
  if (auto error = check_infos(infos, config, initial))
  {
    return {{}, std::move(error)};
  }
  const auto limited = [&](const auto & info)
  { return config.enforce_command_limits && !config.command_limits_excluded.count(info.name); };
  for (const auto & [joint, parameters] : config.limit_parameters)
  {
    bool found = false;
    for (auto & info : infos)
    {
      const auto limits = info.limits.find(joint);
      if (limits != info.limits.end() && limited(info))
      {
        found = true;
        try
        {
          runtime::apply_limit_parameters(joint, parameters, limits->second);
        }
        catch (const std::exception & e)
        {
          return {{}, e.what()};
        }
      }
    }
    // A shared parameter file may keep its limits while the enforcement is off
    if (!found && config.enforce_command_limits)
    {
      return {{}, "joint_limits." + joint + " names no joint whose commands are limited"};
    }
  }
  HardwarePreparation result;
  for (auto & info : infos)
  {
    const bool limit_commands = limited(info);
    result.hardware.push_back({std::move(info), limit_commands});
  }
  return result;
}

}  // namespace hector_controller_manager
