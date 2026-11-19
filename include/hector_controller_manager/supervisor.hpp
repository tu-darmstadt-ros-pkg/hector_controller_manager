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

#ifndef HECTOR_CONTROLLER_MANAGER__SUPERVISOR_HPP_
#define HECTOR_CONTROLLER_MANAGER__SUPERVISOR_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hector_controller_manager/controller_lifecycle.hpp"
#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/core/planner.hpp"
#include "hector_controller_manager/core/reconciler.hpp"
#include "hector_controller_manager/entities.hpp"
#include "hector_controller_manager/inbox.hpp"
#include "hector_controller_manager/plugin_host.hpp"
#include "hector_controller_manager/request_queue.hpp"
#include "hector_controller_manager/runtime/controller_runtime.hpp"
#include "hector_controller_manager/runtime/hardware_runtime.hpp"
#include "hector_controller_manager/runtime/plan.hpp"
#include "hector_controller_manager/runtime/rt_loop.hpp"
#include "hector_controller_manager/step_executor.hpp"
#include "rclcpp/logger.hpp"

namespace hector_controller_manager
{
using runtime::Clock;

struct SupervisorConfig
{
  unsigned int update_rate = 100;
  /// A component in use faults after this long without a completed cycle.
  Clock::duration hang_timeout = std::chrono::milliseconds(100);
  /// Deadline of requests without a timeout of their own, and of fault handling.
  Clock::duration default_timeout = std::chrono::seconds(5);
  /// Deadline of loading one hardware component. Bus setup of a real driver takes long.
  Clock::duration hardware_init_timeout = std::chrono::seconds(60);
  /// Deadline of hardware state requests and of bringing the hardware up after loading it. A
  /// Dynamixel chain configures and activates in seconds, bus retries included.
  Clock::duration hardware_state_timeout = std::chrono::seconds(30);
  /// Time of a hardware lifecycle call in fault handling, a bus driver spends bus timeouts there.
  Clock::duration hardware_fault_timeout = std::chrono::seconds(5);
  /// Added to a switch's deadline for rolling back after a failure.
  Clock::duration grace = std::chrono::milliseconds(400);
};

struct ControllerStatus
{
  std::string name;
  std::string type;
  core::Controller model;
  std::shared_ptr<const runtime::ControllerRuntime> runtime;
};

struct HardwareStatus
{
  std::string name;
  hardware_interface::HardwareInfo info;
  core::Hardware model;
  std::shared_ptr<const runtime::HardwareRuntime> runtime;
};

/// A consistent copy of the state for the list services, the activity topic and diagnostics.
struct Status
{
  bool initialized = false;
  std::vector<ControllerStatus> controllers;
  std::vector<HardwareStatus> hardware;
};

/// The only writer of the model. Runs requests one after another in FIFO order on its own thread,
/// executes the reconciler's steps on the owners and absorbs the failures RT reports.
class Supervisor
{
public:
  using StatusListener = std::function<void(std::shared_ptr<const Status>)>;

  Supervisor(
    SupervisorConfig config, PluginHost & host, rclcpp::Logger logger, StatusListener on_status);
  ~Supervisor();

  Supervisor(const Supervisor &) = delete;
  Supervisor & operator=(const Supervisor &) = delete;

  runtime::RtLoop & rt() { return rt_loop_; }
  const SupervisorConfig & config() const { return config_; }

  /// Each call blocks until the request was handled. Safe from any thread except RT.
  Response initialize(
    const std::string & robot_description, const std::map<std::string, core::Lifecycle> & initial);
  Response load(const std::string & name, const std::string & type, bool is_protected = false);
  Response configure(const std::string & name);
  Response cleanup(const std::string & name);
  Response unload(const std::string & name);
  Response switch_controllers(const core::SwitchRequest & request, Clock::duration timeout);
  Response set_hardware_state(
    const std::string & name, core::Lifecycle target, Clock::duration timeout);
  /// Goes ahead of the waiting requests and runs after the request in flight, `timeout` counts
  /// from its start.
  Response shutdown(Clock::duration timeout);

  std::shared_ptr<const Status> status() const;

  /// Any thread. While engaged, RT stops every controller that claims command interfaces and is
  /// not protected in its next update, and a switch that would start one is rejected. An engage's
  /// `cause` names the topic and why, for the answers of requests.
  void set_estop(bool engaged, std::string cause = {});
  /// The cause of the latest engage, kept after a release for a stop that is reported later.
  std::string estop_cause() const;

private:
  struct Reconciled
  {
    bool done = false;
    /// Only the first update of attached controllers is missing, RT has not cycled yet.
    bool waiting_for_rt = false;
    std::string reason;
    core::Failures failed;
  };

  Response submit(RequestQueue::Work work, Clock::duration timeout);
  void close_host_access();
  /// Runs a request's work. An escaping exception holds everything instead of ending the thread.
  Response perform(RequestQueue::Request & request);
  void run();
  /// Returns why hardware faulted, for the answer of a request.
  std::string absorb();
  /// How the failure closure reached a controller it stopped in this activation.
  std::string stop_cause(const std::string & name, uint64_t activation) const;
  /// Moves actual toward desired until done, blocked or the deadline. Idle reconciliation returns
  /// while it waits for RT, so requests are not held up.
  Reconciled reconcile(Clock::time_point deadline, bool idle = false);
  void publish_status();
  void check_invariants();
  /// Stops and holds every controller for good, aborts in a checked build.
  void hold_everything();

  Response do_initialize(
    const std::string & description, const std::map<std::string, core::Lifecycle> & initial,
    Clock::time_point deadline);
  /// The rejection of a request that changes what runs, if the controller manager does not run.
  std::optional<Response> not_running() const;
  /// Protection does not hold for a protected controller in the failure scope of one the e-stop
  /// stops, the chain rules stop it too.
  void warn_unprotected_chains(const core::Desired & desired) const;
  Response do_switch(const core::SwitchRequest & request, Clock::time_point deadline);
  /// Runs prepare with the full lists on every touched component. Returns why the hardware
  /// rejected the switch, empty if it accepted.
  std::string validate_modes(const core::SwitchPlan & plan, Clock::time_point deadline);
  /// Moves desired back to `before` without what failed or cannot run. Returns why the rollback
  /// did not complete, if it did not.
  std::optional<std::string> roll_back(
    const core::Desired & before, const core::Names & unusable, core::Names failed_in_rt,
    Clock::time_point deadline);
  Response do_set_hardware_state(
    const std::string & name, core::Lifecycle target, Clock::time_point deadline);
  Response do_shutdown(Clock::time_point deadline);

  const SupervisorConfig config_;
  PluginHost & host_;
  std::shared_ptr<HostAccess> host_access_;
  rclcpp::Logger logger_;
  StatusListener on_status_;

  /// The supervisor thread while it runs. It alone touches the members below and the units that
  /// take them.
  runtime::ThreadRole role_{"supervisor"};
  core::Model model_;
  core::Desired desired_;
  /// The last stop cause of each controller, with the activation it stopped in.
  std::map<std::string, std::pair<uint64_t, std::string>> stop_causes_;
  Entities entities_;
  bool initialized_ = false;
  bool shut_down_ = false;
  bool broken_ = false;

  /// What RT touches during a call, on the heap so it can outlive a supervisor torn down under a
  /// stuck RT call.
  struct RtSide
  {
    explicit RtSide(Clock::duration hang_timeout) : loop(slot, wake, hang_timeout) {}
    runtime::Doorbell wake;
    runtime::PlanSlot slot{wake};
    runtime::RtLoop loop;
  };
  std::unique_ptr<RtSide> rt_side_;
  runtime::Doorbell & wake_;
  runtime::PlanSlot & slot_;
  runtime::RtLoop & rt_loop_;
  std::shared_ptr<Inbox> inbox_;
  StepExecutor executor_;
  RequestQueue requests_;
  ControllerLifecycle lifecycle_;

  mutable std::mutex status_mutex_;
  std::shared_ptr<const Status> status_;
  mutable std::mutex estop_mutex_;
  std::string estop_cause_;
  std::thread thread_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__SUPERVISOR_HPP_
