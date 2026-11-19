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

#ifndef HECTOR_CONTROLLER_MANAGER__CORE__MODEL_HPP_
#define HECTOR_CONTROLLER_MANAGER__CORE__MODEL_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

/// Pure value types of the controller manager state.
namespace hector_controller_manager::core
{
using Names = std::set<std::string>;

/// Unconfigured, Inactive and Active are in their order up, the reconciler compares them.
enum class Lifecycle : uint8_t
{
  Unconfigured,
  Inactive,
  Active,
  Finalized,
};

const char * to_string(Lifecycle state);

/// What a controller claims in one activation. Names are full interface names.
struct Claims
{
  std::vector<std::string> command;
  std::vector<std::string> state;

  bool operator==(const Claims &) const = default;
};

/// The interface configuration of a configured controller.
struct InterfaceRequest
{
  std::vector<std::string> command;
  std::vector<std::string> state;
  /// Claims only the available state interfaces, as INDIVIDUAL_BEST_EFFORT.
  bool best_effort_states = false;

  bool operator==(const InterfaceRequest &) const = default;
};

struct Controller
{
  Lifecycle state = Lifecycle::Unconfigured;
  bool chainable = false;
  bool chained = false;
  /// Calls of it given up on that still run. Nothing new starts on it while any is pending.
  uint32_t quarantined = 0;
  /// RT stopped it after a failure in the current activation.
  bool failed = false;
  /// Never stopped implicitly, only by a request that names it or by a failure.
  bool is_protected = false;
  /// Contained in the plan RT runs.
  bool running = false;
  /// Ran update in the plan it was attached with.
  bool updated = false;
  /// Number of activations so far. RT failure flags refer to it.
  uint64_t activation = 0;
  /// Valid from configure until cleanup.
  InterfaceRequest interfaces;
  std::vector<std::string> references;
  std::vector<std::string> exported_states;
  /// Resolved claims of the current activation, empty while not active.
  Claims claims;

  bool operator==(const Controller &) const = default;
};

enum class HardwareFault : uint8_t
{
  None,
  /// read, write, prepare or perform returned ERROR or threw, so on_error follows.
  Error,
  /// read or write returned DEACTIVATE, so on_deactivate follows.
  Deactivate,
};

struct CommandInterface
{
  std::string name;
  /// Interfaces of one joint are command modes of it.
  std::string joint;
  /// The handle holds a double, so NaN can clear a released command.
  bool takes_nan = false;

  bool operator==(const CommandInterface &) const = default;
};

struct Hardware
{
  Lifecycle state = Lifecycle::Unconfigured;
  HardwareFault fault = HardwareFault::None;
  /// Calls of it given up on that still run. Nothing new starts on it while any is pending.
  uint32_t quarantined = 0;
  std::vector<CommandInterface> command_interfaces;
  std::vector<std::string> state_interfaces;
  /// Command interfaces the hardware applies, the others it holds.
  Names live;
  /// Writes are blocked while its live interfaces change their claimer.
  bool taken = false;

  bool operator==(const Hardware &) const = default;
};

/// Interfaces a hardware component starts and stops in one mode switch.
struct ModeSwitch
{
  std::vector<std::string> start;
  std::vector<std::string> stop;

  bool operator==(const ModeSwitch &) const = default;
};

struct Model
{
  std::map<std::string, Controller> controllers;
  std::map<std::string, Hardware> hardware;

  bool operator==(const Model &) const = default;
};

/// Where an interface comes from.
struct InterfaceOwner
{
  enum class Kind : uint8_t
  {
    HardwareCommand,
    HardwareState,
    Reference,
    ExportedState,
  };
  Kind kind;
  std::string owner;
  /// Joint of a hardware command interface.
  std::string joint;
};

inline bool is_provider_kind(InterfaceOwner::Kind kind)
{
  return kind == InterfaceOwner::Kind::Reference || kind == InterfaceOwner::Kind::ExportedState;
}

inline bool is_hardware_kind(InterfaceOwner::Kind kind)
{
  return kind == InterfaceOwner::Kind::HardwareCommand ||
         kind == InterfaceOwner::Kind::HardwareState;
}

/// Identifies the joint of a hardware command interface across components.
inline std::string joint_key(const InterfaceOwner & owner)
{
  return owner.owner + "/" + owner.joint;
}

/// Index of every interface the model knows, built from the configured controllers and hardware.
/// Command and state interfaces have separate name spaces, a joint usually has both a position
/// command and a position state.
class InterfaceIndex
{
public:
  explicit InterfaceIndex(const Model & model);

  /// Hardware command interface or reference.
  const InterfaceOwner * command(const std::string & name) const;
  /// Hardware command interface only.
  const InterfaceOwner * hardware_command(const std::string & name) const;
  /// Hardware state interface or exported state.
  const InterfaceOwner * state(const std::string & name) const;

private:
  std::map<std::string, InterfaceOwner> commands_;
  std::map<std::string, InterfaceOwner> states_;
};

struct DesiredController
{
  /// The activation number it must have reached at least. A restart asks for the next one.
  uint64_t min_activation = 0;
  Claims claims;

  bool operator==(const DesiredController &) const = default;
};

/// The desired state the reconciler moves the actual state toward.
struct Desired
{
  /// Controllers that must be active.
  std::map<std::string, DesiredController> active;
  std::map<std::string, Lifecycle> hardware;

  bool operator==(const Desired &) const = default;
};

/// A controller may claim its command interfaces.
bool commandable(const Hardware & hw);
/// A controller may claim its state interfaces.
bool readable(const Hardware & hw);

/// Active and not stopped by a failure, so it keeps running unless something changes.
bool is_running_member(const Controller & controller);

Names running_members(const Model & model);

/// A running member in at least the given activation.
bool activated(const Controller & controller, uint64_t min_activation);

/// Unprotected and claims command interfaces, so an engaged e-stop stops it.
bool estop_stops(const Controller & controller);

/// The plugin holds interfaces, active or quarantined in on_activate with interfaces assigned.
bool holds_claims(const Controller & controller);

using ControllerEntry = std::pair<const std::string, Controller>;

/// The controller that holds the command interface, if any.
const ControllerEntry * command_claimer(const Model & model, const std::string & interface);

/// A claimer of the component that is detached but not yet stopped, active and quarantined. It may
/// still write its handles, also of an interface held meanwhile, so a taken component waits for it.
std::optional<std::string> hung_claimer(const Model & model, const Hardware & hw);

/// The controllers desired keeps active.
Names names_of(const Desired & desired);

/// The claims of the controllers desired keeps active, by name.
std::map<std::string, Claims> claims_of(const Desired & desired);

}  // namespace hector_controller_manager::core

#endif  // HECTOR_CONTROLLER_MANAGER__CORE__MODEL_HPP_
