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

#include "hector_controller_manager/supervisor.hpp"

#include <algorithm>
#include <cstdlib>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "hector_controller_manager/core/chain_rules.hpp"
#include "hector_controller_manager/core/names.hpp"
#include "hector_controller_manager/core/rollback.hpp"
#include "hector_controller_manager/hardware_preparation.hpp"
#include "hector_controller_manager/leak.hpp"
#include "hector_controller_manager/runtime/hardware_shutdown.hpp"
#include "hector_controller_manager/runtime/thrown.hpp"
#include "rclcpp/logging.hpp"

namespace hector_controller_manager
{
namespace
{
using core::Lifecycle;
using core::Names;
using runtime::HardwareRuntime;
using runtime::StopKind;

/// A request reserves this much to validate a mode switch, a component that does not answer the
/// validation in time is quarantined.
constexpr std::chrono::milliseconds MIN_VALIDATION_TIME{100};
/// What a shutdown that waited behind a slow call keeps of the grace to stop the controllers. The
/// grace must be longer, so the hardware shutdown keeps time.
constexpr std::chrono::milliseconds MIN_SHUTDOWN_STOP_TIME{200};
/// An owner waits this many hang timeouts for RT to leave a read or write, 1 s by default.
constexpr int GATE_TIMEOUT_IN_HANG_TIMEOUTS = 10;
constexpr const char * BROKEN = "A broken invariant stopped the controller manager, restart it";

/// Whether the step runs a mode switch job on a component's owner, which gets mode_switch_timeout.
bool switches_modes(const core::Step & step)
{
  if (const auto * switch_modes = std::get_if<core::step::SwitchModes>(&step))
  {
    return std::any_of(
      switch_modes->modes.begin(), switch_modes->modes.end(),
      [](const auto & entry) { return !entry.second.start.empty() || !entry.second.stop.empty(); });
  }
  return std::holds_alternative<core::step::Hold>(step) ||
         std::holds_alternative<core::step::Take>(step);
}

/// The text of the exception the failed update threw, for a log line or an answer.
std::string thrown(const runtime::ControllerRuntime & runtime)
{
  const std::string error = runtime.update_error();
  return error.empty() ? std::string() : ", it threw: " + error;
}

/// The interfaces of a loaded component as the model knows them.
core::Hardware model_of(const HardwareRuntime & hardware)
{
  core::Hardware model;
  for (const auto & handle : hardware.command_interfaces())
  {
    model.command_interfaces.push_back(
      {handle->get_name(), handle->get_prefix_name(),
       handle->get_data_type() == hardware_interface::HandleDataType::DOUBLE});
  }
  for (const auto & handle : hardware.state_interfaces())
  {
    model.state_interfaces.push_back(handle->get_name());
  }
  return model;
}

/// The first claim of an interface the owner provides, as "reads 'x'" or "writes 'x'".
std::string claim_on(
  const core::InterfaceIndex & index, const core::Claims & claims, const std::string & owner)
{
  for (const auto & interface : claims.command)
  {
    if (const auto * found = index.command(interface); found && found->owner == owner)
    {
      return "writes '" + interface + "'";
    }
  }
  for (const auto & interface : claims.state)
  {
    if (const auto * found = index.state(interface); found && found->owner == owner)
    {
      return "reads '" + interface + "'";
    }
  }
  return {};
}

/// For each controller of desired that the failures stop, the faulted hardware or the stopped
/// controller through which the failure closure reached it, one step toward a seed.
std::map<std::string, std::string> stop_causes(
  const core::Model & model, const core::Desired & desired, const Names & seeds,
  const std::map<std::string, core::HardwareFault> & faulted)
{
  const core::InterfaceIndex index(model);
  const auto claims = core::claims_of(desired);
  const core::ChainGraph graph(index, claims);
  std::map<std::string, std::string> causes;
  std::vector<std::string> reached(seeds.begin(), seeds.end());
  for (const auto & [hardware, fault] : faulted)
  {
    for (const auto & [name, claimed] : claims)
    {
      const std::string claim = claim_on(index, claimed, hardware);
      if (
        !claim.empty() && !seeds.count(name) &&
        causes.emplace(name, "with hardware '" + hardware + "' (" + claim + ")").second)
      {
        reached.push_back(name);
      }
    }
  }
  // Breadth first, so each cause lies on a shortest path to a seed
  for (size_t i = 0; i < reached.size(); ++i)
  {
    const std::string cause = reached[i];
    const auto add = [&](const std::string & name, const std::string & text)
    {
      if (!seeds.count(name) && causes.emplace(name, text).second)
      {
        reached.push_back(name);
      }
    };
    for (const auto & user : graph.users_of(cause))
    {
      add(
        user, "with controller '" + cause + "' (" + claim_on(index, claims.at(user), cause) + ")");
    }
    for (const auto & provider : graph.written_by(cause))
    {
      add(
        provider,
        "with controller '" + cause + "', which " + claim_on(index, claims.at(cause), provider));
    }
  }
  return causes;
}

std::string join(const std::vector<std::string> & names)
{
  std::string result;
  for (const auto & name : names)
  {
    result += (result.empty() ? "" : ", ") + name;
  }
  return result;
}
}  // namespace

Supervisor::Supervisor(
  SupervisorConfig config, PluginHost & host, rclcpp::Logger logger, StatusListener on_status)
: config_(config),
  host_(host),
  host_access_(std::make_shared<HostAccess>()),
  logger_(std::move(logger)),
  on_status_(std::move(on_status)),
  rt_side_(std::make_unique<RtSide>(config_.hang_timeout)),
  wake_(rt_side_->wake),
  slot_(rt_side_->slot),
  rt_loop_(rt_side_->loop),
  inbox_(std::make_shared<Inbox>(wake_)),
  executor_(role_, model_, entities_, slot_, inbox_),
  requests_(wake_),
  lifecycle_(
    role_, model_, entities_, executor_, requests_,
    {host_access_, inbox_, logger_, [this] { publish_status(); }, config_.update_rate,
     config_.default_timeout, rt_loop_.introspection_gate()})
{
  RCLCPP_WARN_EXPRESSION(
    logger_, !config_.scheduler.realtime_kernel(),
    "No realtime kernel, the async owners and the control loop of ros2_control_node run without "
    "realtime scheduling");
  host_access_->open(host_);
  publish_status();
  thread_ = std::thread([this] { run(); });
}

void Supervisor::schedule_control_loop()
{
  std::optional<std::string> memory_lock_failure;
  if (config_.lock_memory)
  {
    const auto [locked, error] = config_.scheduler.lock_memory();
    RCLCPP_WARN_EXPRESSION(logger_, !locked, "Could not lock the memory: %s", error.c_str());
    if (!locked)
    {
      memory_lock_failure = error;
    }
  }
  if (!config_.cpu_affinity.empty())
  {
    const auto [pinned, error] = config_.scheduler.set_affinity(config_.cpu_affinity);
    RCLCPP_WARN_EXPRESSION(
      logger_, !pinned, "Could not pin the control loop to its cores: %s", error.c_str());
  }
  auto scheduling = runtime::schedule_fifo(config_.thread_priority, config_.scheduler);
  RCLCPP_WARN_EXPRESSION(
    logger_, config_.scheduler.realtime_kernel() && !scheduling.problem.empty(),
    "Could not enable FIFO RT scheduling policy for the control loop, %s",
    scheduling.problem.c_str());
  std::lock_guard<std::mutex> lock(scheduling_mutex_);
  control_loop_scheduling_ = std::move(scheduling);
  memory_lock_failure_ = std::move(memory_lock_failure);
}

bool Supervisor::runs_control_loop() const
{
  std::lock_guard<std::mutex> lock(scheduling_mutex_);
  return control_loop_scheduling_.has_value();
}

void Supervisor::report_scheduling(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  std::vector<std::pair<std::string, runtime::Scheduling>> owners;
  // A range-for over status()->hardware would free the snapshot before its body runs
  const auto snapshot = status();
  for (const auto & hardware : snapshot->hardware)
  {
    if (hardware.runtime->is_async())
    {
      owners.emplace_back(hardware.name, hardware.runtime->owner_scheduling());
    }
  }
  std::lock_guard<std::mutex> lock(scheduling_mutex_);
  runtime::report_scheduling(stat, control_loop_scheduling_, owners);
  if (memory_lock_failure_)
  {
    stat.add("control_loop.memory", "not locked, " + *memory_lock_failure_);
    stat.mergeSummary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "The memory is not locked, " + *memory_lock_failure_);
  }
  else if (control_loop_scheduling_)
  {
    stat.add(
      "control_loop.memory", config_.lock_memory ? "locked" : "not locked, lock_memory is off");
  }
}

Supervisor::~Supervisor()
{
  requests_.close();
  thread_.join();
  inbox_->close();
  if (!executor_.retire(Clock::now() + config_.grace))
  {
    // RT returns into the plan, the runtimes and the slot, so all of them stay alive
    RCLCPP_ERROR(logger_, "RT is still in a call while the supervisor goes, its side is abandoned");
    static_cast<void>(rt_side_.release());
    for (auto & [name, controller] : entities_.controllers)
    {
      leak(std::move(controller));
    }
    for (auto & [name, hardware] : entities_.hardware)
    {
      leak(std::move(hardware));
    }
    close_host_access();
    return;
  }
  for (auto & [name, controller] : entities_.controllers)
  {
    if (model_.controllers.at(name).quarantined)
    {
      leak(std::move(controller));
    }
    else if (controller->plugin() && !host_.forget_controller(controller->plugin()))
    {
      leak(std::move(controller));
    }
  }
  for (auto & [name, hardware] : entities_.hardware)
  {
    if (model_.hardware.at(name).quarantined)
    {
      leak(std::move(hardware));
    }
  }
  close_host_access();
}

void Supervisor::close_host_access()
{
  if (!host_access_->close(Clock::now() + config_.default_timeout))
  {
    // The host belongs to the caller, who destroys it next under the running call
    RCLCPP_FATAL(logger_, "A plugin call into the host still runs while the supervisor goes");
    std::abort();
  }
}

std::string Supervisor::stop_cause(const std::string & name, uint64_t activation) const
{
  // A later stop the walk did not reach must not show the cause of an earlier activation
  const auto it = stop_causes_.find(name);
  return it != stop_causes_.end() && it->second.first == activation
           ? it->second.second
           : "because another controller or hardware failed";
}

std::shared_ptr<const Status> Supervisor::status() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return status_;
}

void Supervisor::set_estop(bool engaged, std::string cause)
{
  std::lock_guard<std::mutex> lock(estop_mutex_);
  if (engaged)
  {
    estop_cause_ = std::move(cause);
  }
  rt_loop_.set_estop(engaged);
}

std::string Supervisor::estop_cause() const
{
  std::lock_guard<std::mutex> lock(estop_mutex_);
  return estop_cause_;
}

Response Supervisor::submit(RequestQueue::Work work, Clock::duration timeout)
{
  return requests_.call(std::move(work), timeout, config_.grace);
}

void Supervisor::run()
{
  role_.bind();
  while (true)
  {
    const uint32_t seen = wake_.value();
    const bool absorbed = !guarded("Fault handling", [this] { absorb(); });
    const auto [request, closed] = requests_.pop();
    if (closed)
    {
      role_.unbind();
      return;
    }
    if (request)
    {
      if (Clock::now() > request->deadline)
      {
        request->promise.set_value(RequestQueue::busy());
      }
      else
      {
        request->promise.set_value(perform(*request));
      }
      continue;
    }
    bool settled = true;
    const auto idle_work = [this, &settled]
    {
      const core::Step next = core::next_step(model_, desired_);
      settled =
        std::holds_alternative<core::Done>(next) || std::holds_alternative<core::Blocked>(next);
      if (!settled)
      {
        reconcile(
          Clock::now() + std::max(config_.default_timeout, config_.hardware_fault_timeout), true);
      }
    };
    if (!absorbed || guarded("Idle work", idle_work))
    {
      // A throw that repeats would spin, so it is retried only at the next wake
      const uint32_t thrown = wake_.value();
      if (!requests_.waiting())
      {
        wake_.wait_until(thrown, Clock::time_point::max());
      }
    }
    else if (!settled)
    {
      continue;
    }
    else if (executor_.awaiting_quiescence())
    {
      executor_.wait_quiescent_or_ring(seen);
    }
    else
    {
      wake_.wait_until(seen, Clock::time_point::max());
    }
  }
}

Response Supervisor::perform(RequestQueue::Request & request)
{
  Response response;
  if (const auto reason = guarded("A request", [&] { response = request.work(request.deadline); }))
  {
    return {false, "The request failed: " + *reason};
  }
  return response;
}

std::optional<std::string> Supervisor::guarded(
  const char * what, const std::function<void()> & work)
{
  auto reason = runtime::thrown_by(work);
  if (reason)
  {
    // The model may be half updated
    RCLCPP_FATAL(logger_, "%s threw: %s", what, reason->c_str());
    hold_everything();
  }
  return reason;
}

std::string Supervisor::absorb()
{
  role_.expect();
  auto events = inbox_->take();
  for (auto & event : events)
  {
    event();
  }
  if (!events.empty())
  {
    check_invariants();
  }
  // One publish after the failures are logged, a throw in it must not hide their cause
  bool changed = executor_.release_quiescent() || !events.empty();
  std::string hardware_failures;
  Names failed;
  Names seeds;
  std::map<std::string, StopKind> stops;
  std::map<std::string, core::HardwareFault> faulted;
  for (const auto & [name, controller] : model_.controllers)
  {
    if (controller.state != Lifecycle::Active || controller.failed)
    {
      continue;
    }
    if (const auto kind = entities_.controllers.at(name)->stop_kind(controller.activation);
        kind != StopKind::None)
    {
      failed.insert(name);
      stops.emplace(name, kind);
      if (kind != StopKind::Scope)
      {
        seeds.insert(name);
      }
    }
  }
  for (const auto & [name, hw] : model_.hardware)
  {
    const auto fault = entities_.hardware.at(name)->fault();
    if (hw.fault == core::HardwareFault::None && fault != core::HardwareFault::None)
    {
      faulted.emplace(name, fault);
    }
  }
  if (!failed.empty() || !faulted.empty())
  {
    for (auto & [name, cause] : stop_causes(model_, desired_, seeds, faulted))
    {
      // Keyed by the model activation the lookups pass, a pending restart desires a higher one
      if (const auto & controller = model_.controllers.at(name);
          controller.state == Lifecycle::Active)
      {
        stop_causes_[name] = {controller.activation, std::move(cause)};
      }
    }
    for (const auto & [name, kind] : stops)
    {
      if (kind == StopKind::Own)
      {
        RCLCPP_ERROR(
          logger_, "Controller '%s' failed in an update and stopped with its dependents%s",
          name.c_str(), thrown(*entities_.controllers.at(name)).c_str());
      }
      else if (kind == StopKind::EStop)
      {
        RCLCPP_ERROR(
          logger_,
          broken_ ? "Controller '%s' stopped by holding everything"
                  : "Controller '%s' stopped by the e-stop",
          name.c_str());
      }
      else
      {
        RCLCPP_ERROR(
          logger_, "Controller '%s' stopped %s", name.c_str(),
          stop_cause(name, model_.controllers.at(name).activation).c_str());
      }
    }
    for (const auto & [name, fault] : faulted)
    {
      const std::string why = entities_.hardware.at(name)->fault_reason();
      RCLCPP_ERROR(
        logger_, "Hardware '%s' failed, its controllers stopped: %s", name.c_str(), why.c_str());
      core::append_reason(hardware_failures, "Hardware '" + name + "' failed: " + why);
    }
    core::apply_failures(model_, desired_, failed, faulted);
    check_invariants();
    changed = true;
  }
  if (changed)
  {
    publish_status();
  }
  return hardware_failures;
}

Clock::time_point Supervisor::step_deadline(
  const core::Step & step, bool idle, Clock::time_point deadline) const
{
  const auto step_time = std::holds_alternative<core::step::Hardware>(step)
                           ? config_.hardware_fault_timeout
                           : config_.hang_timeout;
  return switches_modes(step) ? Clock::now() + config_.mode_switch_timeout
         : idle               ? std::min(deadline, Clock::now() + step_time)
                              : deadline;
}

Supervisor::Reconciled Supervisor::reconcile(
  Clock::time_point deadline, bool idle, Clock::time_point modes_until)
{
  role_.expect();
  Reconciled result;
  while (true)
  {
    if (const std::string failures = absorb(); !failures.empty())
    {
      core::append_reason(result.reason, failures);
    }
    // Idle work yields to a waiting request, which continues the same desired state
    if (idle && requests_.waiting())
    {
      return result;
    }
    const core::Step step = core::next_step(model_, desired_);
    if (std::holds_alternative<core::Done>(step))
    {
      result.done = true;
      return result;
    }
    if (const auto * blocked = std::get_if<core::Blocked>(&step))
    {
      result.reason = blocked->reason;
      return result;
    }
    if (Clock::now() >= deadline || (switches_modes(step) && Clock::now() >= modes_until))
    {
      // Only the hardware start of activated controllers is left, it follows without the request
      const bool starting = std::holds_alternative<core::step::AwaitUpdate>(step) ||
                            std::holds_alternative<core::step::SwitchModes>(step);
      result.waiting_for_rt = starting;
      core::append_reason(result.reason, "deadline passed before " + core::describe(step));
      return result;
    }
    RCLCPP_DEBUG(logger_, "Step: %s", core::describe(step).c_str());
    runtime::WaitInterrupt interrupt;
    if (idle && std::holds_alternative<core::step::Hardware>(step))
    {
      interrupt = {[this] { return requests_.waiting(); }, config_.hang_timeout};
    }
    const core::Outcome outcome =
      executor_.execute(step, step_deadline(step, idle, deadline), interrupt);
    const core::Failures failed = core::apply(model_, desired_, step, outcome);
    if (
      std::holds_alternative<core::step::AwaitUpdate>(step) && outcome.not_updated &&
      (idle || Clock::now() >= deadline))
    {
      result.waiting_for_rt = true;
      return result;
    }
    if (!failed.empty())
    {
      std::string what = core::describe(step) + " failed for " +
                         core::text(core::merged(failed.unusable, failed.refused));
      if (!outcome.reason.empty())
      {
        what += ": " + outcome.reason;
      }
      RCLCPP_ERROR(logger_, "%s", what.c_str());
      core::append_reason(result.reason, what);
    }
    result.failed.unusable.merge(Names(failed.unusable));
    result.failed.refused.merge(Names(failed.refused));
    check_invariants();
    publish_status();
  }
}

void Supervisor::publish_status()
{
  role_.expect();
  auto status = std::make_shared<Status>();
  status->initialized = initialized_;
  for (const auto & [name, controller] : model_.controllers)
  {
    status->controllers.push_back(
      {name, entities_.controllers.at(name)->type(), controller, entities_.controllers.at(name)});
  }
  for (const auto & [name, hw] : model_.hardware)
  {
    const auto & runtime = entities_.hardware.at(name);
    status->hardware.push_back({name, runtime->info(), hw, runtime});
  }
  // The old snapshot may hold the last reference of an unloaded runtime, readers must not wait for
  // its destruction
  std::shared_ptr<const Status> previous;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    previous = std::exchange(status_, status);
  }
  if (on_status_)
  {
    on_status_(status);
  }
}

void Supervisor::check_invariants()
{
  const auto violations = core::check_invariants(model_);
  if (violations.empty())
  {
    return;
  }
  for (const auto & violation : violations)
  {
    RCLCPP_FATAL(logger_, "Broken invariant: %s", violation.c_str());
  }
  hold_everything();
}

void Supervisor::hold_everything()
{
  if (runtime::CHECKED_BUILD)
  {
    std::abort();
  }
  // Rejects every further switch until a restart
  broken_ = true;
  desired_.active.clear();
  // The reconciler that would stop the controllers may be what threw
  rt_loop_.stop_command_claimers();
}

Response Supervisor::initialize(
  const std::string & robot_description, const std::map<std::string, Lifecycle> & initial)
{
  return submit(
    [this, robot_description, initial](Clock::time_point deadline)
    { return do_initialize(robot_description, initial, deadline); },
    config_.hardware_state_timeout);
}

std::variant<std::shared_ptr<HardwareRuntime>, std::string> Supervisor::load_hardware(
  const hardware_interface::HardwareInfo & info, bool limit_commands)
{
  auto inbox = inbox_;
  auto hardware = std::make_shared<HardwareRuntime>(
    info,
    [this, inbox](const std::string & name, std::vector<std::string> held)
    {
      inbox->post(
        [this, name, held]
        {
          for (const auto & interface : held)
          {
            model_.hardware.at(name).live.erase(interface);
          }
        });
    },
    GATE_TIMEOUT_IN_HANG_TIMEOUTS * config_.hang_timeout, limit_commands);
  auto error = std::make_shared<std::string>();
  auto job = hardware->owner().submit(
    [access = host_access_, hardware = hardware.get(), error, priority = config_.thread_priority,
     scheduler = config_.scheduler]
    {
      if (hardware->is_async())
      {
        hardware->schedule_owner(priority, scheduler);
      }
      *error = runtime::thrown_by(
                 [&]
                 {
                   access->with_host(
                     [&](PluginHost & host)
                     {
                       hardware->initialize(
                         [&host](const auto & component_info)
                         { return host.create_hardware(component_info); },
                         host.hardware_params(hardware->info()));
                     });
                 })
                 .value_or("");
    });
  if (!hardware->owner().wait(job, Clock::now() + config_.hardware_init_timeout))
  {
    leak(std::move(hardware));
    return "Loading hardware '" + info.name + "' did not finish in time, it is abandoned";
  }
  if (!error->empty())
  {
    return "Loading hardware '" + info.name + "' failed: " + *error;
  }
  return hardware;
}

Response Supervisor::do_initialize(
  const std::string & description, const std::map<std::string, Lifecycle> & initial,
  Clock::time_point deadline)
{
  if (initialized_)
  {
    return {false, "Already initialized, a new robot description is ignored"};
  }
  const auto prepared = prepare_hardware(description, config_, initial);
  if (prepared.error)
  {
    return {false, *prepared.error};
  }
  std::map<std::string, std::shared_ptr<HardwareRuntime>> created;
  for (const auto & [info, limit_commands] : prepared.hardware)
  {
    auto loaded = load_hardware(info, limit_commands);
    if (const auto * error = std::get_if<std::string>(&loaded))
    {
      return {false, *error};
    }
    created.emplace(info.name, std::get<std::shared_ptr<HardwareRuntime>>(std::move(loaded)));
  }
  core::Model next = model_;
  for (const auto & [name, hardware] : created)
  {
    next.hardware.emplace(name, model_of(*hardware));
  }
  try
  {
    const core::InterfaceIndex index(next);
  }
  catch (const std::invalid_argument & e)
  {
    return {false, std::string("The robot description is invalid: ") + e.what()};
  }
  for (const auto & [name, hardware] : created)
  {
    for (const auto & handle : hardware->command_interfaces())
    {
      entities_.command_handles.emplace(handle->get_name(), handle);
    }
    for (const auto & handle : hardware->state_interfaces())
    {
      entities_.state_handles.emplace(handle->get_name(), handle);
    }
    model_.hardware.emplace(name, next.hardware.at(name));
    entities_.hardware.emplace(name, hardware);
    const auto it = initial.find(name);
    desired_.hardware[name] = it == initial.end() ? Lifecycle::Active : it->second;
  }
  initialized_ = true;
  // Loading has a deadline of its own, the bring up gets its full time after it
  const auto bring_up = std::max(deadline, Clock::now() + config_.hardware_state_timeout);
  // Before the bring up enables the values of the components
  rt_loop_.flush_introspection();
  executor_.publish_plan(model_, bring_up);
  const auto result = reconcile(bring_up);
  publish_status();
  return {result.done && result.failed.empty(), result.reason};
}

Response Supervisor::submit_lifecycle(RequestQueue::Work work)
{
  return submit(
    [this, work = std::move(work)](Clock::time_point)
    {
      if (broken_)
      {
        return Response{false, BROKEN};
      }
      if (shut_down_)
      {
        return Response{false, "The controller manager is shut down"};
      }
      // A plugin call given up at its deadline is leaked, so time spent queued must not shorten it
      return work(Clock::now() + config_.default_timeout);
    },
    config_.default_timeout);
}

Response Supervisor::load(const std::string & name, const std::string & type, bool is_protected)
{
  return submit_lifecycle([this, name, type, is_protected](Clock::time_point deadline)
                          { return lifecycle_.load(name, type, is_protected, deadline); });
}

Response Supervisor::configure(const std::string & name)
{
  return submit_lifecycle([this, name](Clock::time_point deadline)
                          { return lifecycle_.configure(name, deadline); });
}

Response Supervisor::cleanup(const std::string & name)
{
  return submit_lifecycle([this, name](Clock::time_point deadline)
                          { return lifecycle_.cleanup(name, deadline); });
}

Response Supervisor::unload(const std::string & name)
{
  return submit_lifecycle([this, name](Clock::time_point deadline)
                          { return lifecycle_.unload(name, deadline); });
}

Response Supervisor::switch_controllers(
  const core::SwitchRequest & request, Clock::duration timeout)
{
  return submit(
    [this, request](Clock::time_point deadline) { return do_switch(request, deadline); }, timeout);
}

Response Supervisor::set_hardware_state(
  const std::string & name, Lifecycle target, Clock::duration timeout)
{
  return submit(
    [this, name, target](Clock::time_point deadline)
    { return do_set_hardware_state(name, target, deadline); }, timeout);
}

StartupSwitch Supervisor::startup_switch(
  const std::vector<std::string> & wanted, bool restore, Clock::duration timeout)
{
  StartupSwitch result;
  result.response = submit(
    [this, &wanted, restore, &selected = result.selected](Clock::time_point deadline)
    {
      if (auto rejection = not_running())
      {
        return *rejection;
      }
      reconcile(deadline - MIN_VALIDATION_TIME);
      const auto selection = core::select_startup(
        model_, wanted, restore,
        rt_loop_.estop() ? std::optional<std::string>(estop_cause()) : std::nullopt);
      for (const auto & entry : selection.skipped)
      {
        RCLCPP_WARN(logger_, "Startup skips %s", entry.c_str());
      }
      selected = selection.activate;
      if (selected.empty())
      {
        return Response{true, ""};
      }
      return do_switch({selected, {}, core::Strictness::Auto}, deadline);
    },
    timeout);
  return result;
}

Response Supervisor::shutdown(Clock::duration timeout)
{
  // A shutdown dropped behind a long request would leave the hardware up
  return requests_.call_first([this, timeout](Clock::time_point)
                              { return do_shutdown(Clock::now() + timeout); });
}

Response Supervisor::do_switch(const core::SwitchRequest & request, Clock::time_point deadline)
{
  if (auto rejection = not_running())
  {
    return *rejection;
  }
  // Validation gets time of its own, because a component that does not answer it is quarantined
  reconcile(deadline - MIN_VALIDATION_TIME);
  // A switch is planned on a settled model, only a quarantine may still block it
  if (const core::Step next = core::next_step(model_, desired_);
      !std::holds_alternative<core::Done>(next) && !std::holds_alternative<core::Blocked>(next))
  {
    return RequestQueue::busy();
  }
  auto planned = core::plan_switch(model_, desired_.hardware, request);
  if (const auto * rejection = std::get_if<core::Rejection>(&planned))
  {
    return {false, rejection->reason};
  }
  const auto plan = std::get<core::SwitchPlan>(std::move(planned));
  if (rt_loop_.estop())
  {
    for (const auto & name : plan.started)
    {
      if (core::estop_stops(model_.controllers.at(name)))
      {
        return {
          false, "The e-stop is engaged by " + estop_cause() + ", '" + name +
                   "' claims command interfaces"};
      }
    }
  }
  if (!plan.validation.empty() && deadline - Clock::now() < MIN_VALIDATION_TIME)
  {
    return RequestQueue::busy();
  }

  if (const std::string rejected = validate_modes(plan, deadline); !rejected.empty())
  {
    return {false, "Hardware rejected the mode switch:" + rejected};
  }

  const core::Desired before = desired_;
  desired_ = plan.desired;
  auto first = reconcile(deadline);
  // RT stops a controller that fails in its first update with its scope, the reconcile only
  // absorbs that
  Names failed_in_rt;
  const auto stopped_with_another = [&](const std::string & name)
  {
    return "'" + name + "' stopped after its activation " +
           stop_cause(name, plan.desired.active.at(name).min_activation);
  };
  for (const auto & name : plan.started)
  {
    const auto & runtime = *entities_.controllers.at(name);
    const auto kind = runtime.stop_kind(plan.desired.active.at(name).min_activation);
    if (
      kind != StopKind::None && !first.failed.unusable.count(name) &&
      !first.failed.refused.count(name))
    {
      failed_in_rt.insert(name);
      first.failed.unusable.insert(name);
      core::append_reason(
        first.reason,
        kind == StopKind::Own
          ? "'" + name + "' failed in an update after its activation" + thrown(runtime)
        : kind == StopKind::EStop
          ? "'" + name + "' stopped after its activation by the e-stop, engaged by " + estop_cause()
          : stopped_with_another(name));
    }
  }
  // A quarantine the switch does not touch ends on its own, it does not fail the switch
  const bool reached =
    first.done || first.waiting_for_rt || core::reached_apart_from_quarantine(model_, desired_);
  // A failure the reconcile absorbed takes what it stops out of desired
  bool kept = true;
  if (first.failed.empty())
  {
    for (const auto & name : plan.started)
    {
      if (!desired_.active.count(name))
      {
        kept = false;
        core::append_reason(first.reason, stopped_with_another(name));
      }
    }
  }
  if (reached && first.failed.empty() && kept)
  {
    warn_unprotected_chains(plan.desired);
    return {
      true, "Deactivated " + core::text(plan.stopped) + ", activated " + core::text(plan.started)};
  }
  const auto incomplete = roll_back(before, first.failed.unusable, failed_in_rt, deadline);
  return {
    false, "Switch failed and was rolled back: " + first.reason +
             (incomplete ? "; rollback incomplete: " + *incomplete : std::string())};
}

std::string Supervisor::validate_modes(const core::SwitchPlan & plan, Clock::time_point deadline)
{
  struct Validation
  {
    std::string name;
    std::shared_ptr<bool> ok;
    runtime::OwnerThread::JobPtr job;
  };
  std::vector<Validation> validations;
  for (const auto & [name, modes] : plan.validation)
  {
    auto * hardware = entities_.hardware.at(name).get();
    auto ok = std::make_shared<bool>(false);
    auto job = hardware->owner().submit([hardware, modes = modes, ok]
                                        { *ok = hardware->validate(modes.start, modes.stop); });
    validations.push_back({name, ok, job});
  }
  std::string rejected;
  for (const auto & [name, ok, job] : validations)
  {
    if (!entities_.hardware.at(name)->owner().wait(job, deadline, executor_.late_mode_switch(name)))
    {
      core::apply(model_, core::GivenUp{{}, {name}});
      rejected += " '" + name + "' did not answer prepare in time.";
    }
    else if (!*ok)
    {
      const auto & modes = plan.validation.at(name);
      rejected += " '" + name + "' rejected starting [" + join(modes.start) + "] and stopping [" +
                  join(modes.stop) + "].";
    }
  }
  if (!rejected.empty())
  {
    publish_status();
  }
  return rejected;
}

std::optional<std::string> Supervisor::roll_back(
  const core::Desired & before, const Names & unusable, Names failed_in_rt,
  Clock::time_point deadline)
{
  core::Desired rollback = core::plan_rollback(model_, before, desired_, unusable);
  // A fault or the e-stop never reactivates, also not a restarted controller in its old activation
  for (const auto & [name, want] : before.active)
  {
    if (
      entities_.controllers.at(name)->failed_activation.load(std::memory_order_acquire) >=
      want.min_activation)
    {
      failed_in_rt.insert(name);
    }
  }
  core::shrink(model_, rollback, failed_in_rt);
  desired_ = rollback;
  // Mode switches only before the deadline, one started later would extend the answer
  const auto grace = deadline + config_.grace;
  const auto second = reconcile(grace, false, deadline);
  if (second.done)
  {
    return std::nullopt;
  }
  desired_ = core::keep_running(model_, desired_);
  reconcile(grace, false, deadline);
  const Names lost = core::without(core::names_of(before), core::names_of(desired_));
  return second.reason + (lost.empty() ? "" : "; not restored: " + core::text(lost));
}

void Supervisor::warn_unprotected_chains(const core::Desired & desired) const
{
  Names stoppable;
  for (const auto & [name, unused] : desired.active)
  {
    if (core::estop_stops(model_.controllers.at(name)))
    {
      stoppable.insert(name);
    }
  }
  const core::InterfaceIndex index(model_);
  const core::ChainGraph graph(index, core::claims_of(desired));
  for (const auto & name : core::stop_closure(graph, stoppable))
  {
    if (model_.controllers.at(name).is_protected)
    {
      RCLCPP_WARN(
        logger_, "Protected '%s' is chained to a controller the e-stop stops, it stops with it",
        name.c_str());
    }
  }
}

std::optional<Response> Supervisor::not_running() const
{
  if (initialized_ && !shut_down_ && !broken_)
  {
    return std::nullopt;
  }
  return Response{false, broken_ ? BROKEN : "The controller manager is not running"};
}

Response Supervisor::do_set_hardware_state(
  const std::string & name, Lifecycle target, Clock::time_point deadline)
{
  if (auto rejection = not_running())
  {
    return *rejection;
  }
  const auto it = model_.hardware.find(name);
  if (it == model_.hardware.end())
  {
    return {false, "Hardware '" + name + "' is not loaded"};
  }
  if (target != Lifecycle::Active && target != Lifecycle::Inactive)
  {
    return {false, "Only the target states active and inactive are supported"};
  }
  // A call given up on returns through the inbox, fault handling on a dead bus takes a while. A
  // request queued meanwhile goes first, a pushed request rings wake_.
  while (it->second.quarantined)
  {
    const uint32_t seen = wake_.value();
    absorb();
    if (!it->second.quarantined)
    {
      break;
    }
    if (requests_.waiting())
    {
      return {
        false,
        "Hardware '" + name +
          "' is quarantined, a call of it still runs and another request waits, try again",
        true};
    }
    if (!wake_.wait_until(seen, deadline))
    {
      return {false, "Hardware '" + name + "' is quarantined, a call of it still runs"};
    }
  }
  reconcile(deadline);
  if (target == Lifecycle::Inactive)
  {
    Names commanders;
    const core::InterfaceIndex index(model_);
    for (const auto & claimer : core::claimers_of_hardware(model_, index, name))
    {
      if (
        model_.controllers.at(claimer).state == Lifecycle::Active &&
        core::commands_hardware(index, model_.controllers.at(claimer).claims, name))
      {
        commanders.insert(claimer);
      }
    }
    if (!commanders.empty())
    {
      return {false, "Hardware '" + name + "' is commanded by " + core::text(commanders)};
    }
  }
  const Lifecycle previous = desired_.hardware[name];
  desired_.hardware[name] = target;
  const auto result = reconcile(deadline);
  const auto & hw = model_.hardware.at(name);
  const bool reached = hw.state == target && hw.fault == core::HardwareFault::None;
  if (!reached && !hw.quarantined)
  {
    // The caller hears it failed, so nothing may reach the target later. A fault still
    // being handled keeps its own target.
    desired_.hardware[name] = hw.fault == core::HardwareFault::None ? hw.state : previous;
  }
  return {
    reached,
    reached ? "" : "Hardware '" + name + "' is " + to_string(hw.state) + ". " + result.reason};
}

Response Supervisor::do_shutdown(Clock::time_point deadline)
{
  desired_.active.clear();
  for (auto & [name, state] : desired_.hardware)
  {
    state = Lifecycle::Finalized;
  }
  const auto stop_deadline = std::max(deadline, Clock::now() + MIN_SHUTDOWN_STOP_TIME);
  const auto hardware_time = std::max(deadline, Clock::now()) + config_.grace - stop_deadline;
  const auto result = reconcile(stop_deadline);
  // A mode switch may run past the deadline, the hardware keeps its share of the grace after it
  const auto bound = std::max(stop_deadline, Clock::now()) + hardware_time;
  // A failed step steers back to where it started, but whatever returns late must still go down
  for (auto & [name, state] : desired_.hardware)
  {
    state = Lifecycle::Finalized;
  }
  // A hung controller keeps its hardware claimed, so the hardware stops with its interfaces held
  // and the controller out of the plan
  std::map<std::string, HardwareRuntime *> running;
  for (const auto & [name, hw] : model_.hardware)
  {
    if (hw.state != Lifecycle::Finalized && !hw.quarantined)
    {
      running.emplace(name, entities_.hardware.at(name).get());
    }
  }
  for (const auto & [name, state] : runtime::shut_down_all(running, bound))
  {
    if (!state)
    {
      core::apply(model_, core::GivenUp{{}, {name}});
      continue;
    }
    core::apply(model_, core::returned::HardwareCallback{name, *state});
  }
  shut_down_ = true;
  publish_status();
  return {result.done, result.reason};
}

}  // namespace hector_controller_manager
