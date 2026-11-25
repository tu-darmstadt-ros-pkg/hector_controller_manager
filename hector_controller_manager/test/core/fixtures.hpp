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

#ifndef CORE__FIXTURES_HPP_
#define CORE__FIXTURES_HPP_

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/core/planner.hpp"
#include "hector_controller_manager/core/reconciler.hpp"

namespace hector_controller_manager::core::testing
{
struct ControllerSpec
{
  std::string name;
  bool chainable;
  std::vector<std::string> command;
  std::vector<std::string> state;
  /// Short names, exported with the controller name as prefix.
  std::vector<std::string> references;
  std::vector<std::string> exported_states;
  bool best_effort_states = false;
};

/// Whether both positions exist and the first comes before the second.
inline bool before(std::optional<size_t> first, std::optional<size_t> second)
{
  return first && second && *first < *second;
}

inline void add_hardware(
  Model & model, const std::string & name, const std::vector<std::string> & joints,
  const std::vector<std::string> & modes, const std::vector<std::string> & states)
{
  Hardware hw;
  hw.state = Lifecycle::Active;
  for (const auto & joint : joints)
  {
    for (const auto & mode : modes)
    {
      hw.command_interfaces.push_back({joint + "/" + mode, joint, true});
    }
  }
  hw.state_interfaces = states;
  model.hardware.emplace(name, hw);
}

/// A configured, inactive controller.
inline void add_controller(Model & model, const ControllerSpec & spec)
{
  Controller controller;
  controller.state = Lifecycle::Inactive;
  controller.chainable = spec.chainable;
  controller.interfaces = {spec.command, spec.state, spec.best_effort_states};
  for (const auto & reference : spec.references)
  {
    controller.references.push_back(spec.name + "/" + reference);
  }
  for (const auto & exported : spec.exported_states)
  {
    controller.exported_states.push_back(spec.name + "/" + exported);
  }
  model.controllers.emplace(spec.name, controller);
}

/// The rig of the common test suite, all hardware active, all controllers inactive.
inline Model rig()
{
  Model model;
  add_hardware(
    model, "arm", {"a1", "a2", "a3"}, {"position", "velocity"},
    {"a1/position", "a1/velocity", "a2/position", "a2/velocity", "a3/position", "a3/velocity"});
  add_hardware(
    model, "base", {"wl", "wr"}, {"velocity"},
    {"wl/position", "wl/velocity", "wr/position", "wr/velocity"});
  add_hardware(model, "flipper", {"f1"}, {"position", "velocity"}, {"f1/position"});
  add_hardware(model, "imu", {}, {}, {"imu/yaw"});
  const std::vector<ControllerSpec> specs = {
    {"jsb",
     false,
     {},
     {"a1/position", "a2/position", "a3/position", "wl/position", "wr/position", "f1/position",
      "f1/effort"},
     {},
     {},
     true},
    {"arm_vel",
     true,
     {"a1/velocity", "a2/velocity", "a3/velocity"},
     {"a1/velocity", "a2/velocity", "a3/velocity"},
     {"a1", "a2", "a3"},
     {}},
    {"arm_pos", false, {"a1/position", "a2/position", "a3/position"}, {}, {}, {}},
    {"arm_ik", true, {"arm_vel/a1", "arm_vel/a2"}, {}, {"x", "z"}, {}},
    {"wrist", true, {"arm_vel/a3"}, {}, {"angle"}, {}},
    {"base_vel",
     true,
     {"wl/velocity", "wr/velocity"},
     {"wl/velocity", "wr/velocity"},
     {"wl", "wr"},
     {}},
    {"diff_drive", true, {"base_vel/wl", "base_vel/wr"}, {"imu/yaw"}, {"vx", "wz"}, {"odom"}},
    {"flipper_pos", true, {"f1/position"}, {}, {"f1"}, {}},
    {"flipper_vel", false, {"f1/velocity"}, {}, {}, {}},
    {"whole_body",
     true,
     {"arm_ik/x", "arm_ik/z", "wrist/angle", "diff_drive/vx", "diff_drive/wz", "flipper_pos/f1"},
     {},
     {"a", "b"},
     {}},
    {"nav", false, {"diff_drive/vx", "diff_drive/wz"}, {}, {}, {}},
    {"teleop", false, {"whole_body/a", "whole_body/b"}, {}, {}, {}},
    {"odom_pub", false, {}, {"diff_drive/odom", "imu/yaw"}, {}, {}},
    {"monitor", false, {}, {}, {}, {}},
  };
  for (const auto & spec : specs)
  {
    add_controller(model, spec);
  }
  return model;
}

/// The outcome of a step that succeeded.
inline Outcome outcome_for(const Step & step)
{
  Outcome outcome;
  if (std::holds_alternative<step::Activate>(step))
  {
    outcome.state = Lifecycle::Active;
  }
  if (const auto * hw = std::get_if<step::Hardware>(&step))
  {
    switch (hw->transition)
    {
      case step::HardwareTransition::Configure:
      case step::HardwareTransition::Deactivate:
        outcome.state = Lifecycle::Inactive;
        break;
      case step::HardwareTransition::Activate:
        outcome.state = Lifecycle::Active;
        break;
      case step::HardwareTransition::Cleanup:
      case step::HardwareTransition::Error:
        outcome.state = Lifecycle::Unconfigured;
        break;
      case step::HardwareTransition::Shutdown:
        outcome.state = Lifecycle::Finalized;
        break;
    }
  }
  return outcome;
}

/// Executes steps with successful outcomes until done, checking the invariants after each step.
/// Returns the executed steps.
inline std::vector<Step> settle(Model & model, Desired desired, size_t max_steps = 200)
{
  std::vector<Step> steps;
  for (size_t i = 0; i < max_steps; ++i)
  {
    Step step = next_step(model, desired);
    if (std::holds_alternative<Done>(step))
    {
      return steps;
    }
    if (const auto * blocked = std::get_if<Blocked>(&step))
    {
      ADD_FAILURE() << "blocked: " << blocked->reason;
      return steps;
    }
    apply(model, desired, step, outcome_for(step));
    for (const auto & violation : check_invariants(model))
    {
      ADD_FAILURE() << "after step " << i << ": " << violation;
    }
    steps.push_back(std::move(step));
  }
  ADD_FAILURE() << "did not settle within " << max_steps << " steps";
  return steps;
}

/// The hardware components a step calls perform_command_mode_switch on.
inline Names performed_on(const Step & step)
{
  Names hardware;
  if (const auto * hold = std::get_if<step::Hold>(&step))
  {
    for (const auto & [name, interfaces] : hold->interfaces)
    {
      if (!interfaces.empty())
      {
        hardware.insert(name);
      }
    }
  }
  if (const auto * switch_modes = std::get_if<step::SwitchModes>(&step))
  {
    for (const auto & [name, modes] : switch_modes->modes)
    {
      if (!modes.start.empty() || !modes.stop.empty())
      {
        hardware.insert(name);
      }
    }
  }
  return hardware;
}

/// The desired state that keeps everything as it is.
inline Desired keep(const Model & model)
{
  Desired desired;
  for (const auto & name : running_members(model))
  {
    const auto & controller = model.controllers.at(name);
    desired.active.emplace(name, DesiredController{controller.activation, controller.claims});
  }
  for (const auto & [name, hw] : model.hardware)
  {
    desired.hardware.emplace(name, hw.state);
  }
  return desired;
}

/// Plans and executes a switch, expecting acceptance.
inline SwitchPlan switch_to(
  Model & model, Names activate, Names deactivate, Strictness strictness = Strictness::Strict)
{
  auto result = plan_switch(
    model, keep(model).hardware, {std::move(activate), std::move(deactivate), strictness});
  if (const auto * rejection = std::get_if<Rejection>(&result))
  {
    ADD_FAILURE() << "rejected: " << rejection->reason;
    return {};
  }
  auto plan = std::get<SwitchPlan>(result);
  settle(model, plan.desired);
  return plan;
}

/// Plans a switch, expecting acceptance.
inline SwitchPlan plan_of(const Model & model, const SwitchRequest & request)
{
  auto result = plan_switch(model, keep(model).hardware, request);
  if (const auto * rejection = std::get_if<Rejection>(&result))
  {
    ADD_FAILURE() << rejection->reason;
    return {};
  }
  return std::get<SwitchPlan>(result);
}

/// Executes the steps before the SwitchModes successfully and returns the SwitchModes.
inline Step until_switch_modes(Model & model, Desired & desired)
{
  // Far more than any rig switch takes, so a plan without a SwitchModes fails instead of hanging
  constexpr int MAX_STEPS = 1000;
  Step step = next_step(model, desired);
  for (int steps = 0; !std::holds_alternative<step::SwitchModes>(step); ++steps)
  {
    if (steps == MAX_STEPS)
    {
      ADD_FAILURE() << "no SwitchModes within " << MAX_STEPS << " steps, the last is "
                    << describe(step);
      return step;
    }
    apply(model, desired, step, outcome_for(step));
    step = next_step(model, desired);
  }
  return step;
}

/// The keys of a map by name.
template <typename Map>
Names names_of(const Map & map)
{
  Names names;
  for (const auto & [name, unused] : map)
  {
    names.insert(name);
  }
  return names;
}

inline Names active_names(const Model & model)
{
  Names names;
  for (const auto & [name, controller] : model.controllers)
  {
    if (controller.state == Lifecycle::Active)
    {
      names.insert(name);
    }
  }
  return names;
}

inline const Names & startup_active()
{
  static const Names names = {"jsb",      "arm_vel",    "arm_ik",      "wrist",
                              "base_vel", "diff_drive", "flipper_pos", "odom_pub"};
  return names;
}

/// The rig with the startup set active.
inline Model started_rig()
{
  Model model = rig();
  switch_to(model, startup_active(), {});
  return model;
}

}  // namespace hector_controller_manager::core::testing

#endif  // CORE__FIXTURES_HPP_
