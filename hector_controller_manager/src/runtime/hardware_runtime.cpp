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

#include "hector_controller_manager/runtime/hardware_runtime.hpp"

#include <algorithm>
#include <charconv>
#include <optional>
#include <stdexcept>
#include <utility>

#include "hector_controller_manager/runtime/lifecycle_state.hpp"
#include "hector_controller_manager/runtime/thrown.hpp"
#include "rclcpp/logging.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using core::CallResult;
using core::Lifecycle;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using Transition = core::step::HardwareTransition;
using Fault = core::HardwareFault;

double seconds_since(Clock::time_point start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

/// Milliseconds as text, without allocating.
class Millis
{
public:
  explicit Millis(Clock::duration duration) noexcept
  {
    const auto count = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    size_ = static_cast<size_t>(std::to_chars(text_, text_ + sizeof(text_), count).ptr - text_);
  }
  operator std::string_view() const noexcept { return {text_, size_}; }

private:
  char text_[24];
  size_t size_ = 0;
};

}  // namespace

int owner_priority(const hardware_interface::HardwareAsyncParams & params, int rt_priority)
{
  constexpr int PARSER_DEFAULT = 50;
  return params.thread_priority == PARSER_DEFAULT ? std::max(1, rt_priority - 1)
                                                  : params.thread_priority;
}

std::string HardwareRuntime::fault_reason() const
{
  const uint64_t current = generation();
  std::string result;
  for (const auto * latch : {&io_fault_, &call_fault_, &hang_fault_})
  {
    if (latch->get(current) != Fault::None)
    {
      core::append_reason(result, latch->reason());
    }
  }
  return result;
}

HardwareRuntime::HardwareRuntime(
  hardware_interface::HardwareInfo info, HoldListener on_hold, Clock::duration gate_timeout,
  bool limit_commands)
: info_(std::move(info)),
  gate_timeout_(gate_timeout),
  writable_(info_.type != "sensor"),
  on_hold_(std::move(on_hold)),
  logger_(rclcpp::get_logger("hector_controller_manager").get_child(info_.name)),
  limit_commands_(limit_commands)
{
  owner_ = std::make_unique<OwnerThread>([this] { poll(); });
}

void HardwareRuntime::schedule_owner(int rt_priority, const Scheduler & scheduler)
{
  owner_->expect_owner();
  const auto & params = info_.async_params;
  if (!params.cpu_affinity_cores.empty())
  {
    const auto [pinned, error] =
      realtime_tools::set_current_thread_affinity(params.cpu_affinity_cores);
    RCLCPP_WARN_EXPRESSION(
      logger_, !pinned, "Could not pin the owner thread to its cores: %s", error.c_str());
  }
  owner_scheduling_ = schedule_fifo(owner_priority(params, rt_priority), scheduler);
  // The supervisor logs once that the kernel is no realtime kernel
  RCLCPP_WARN_EXPRESSION(
    logger_, scheduler.realtime_kernel() && !owner_scheduling_.problem.empty(),
    "Could not enable FIFO RT scheduling policy for the owner thread, %s",
    owner_scheduling_.problem.c_str());
}

void HardwareRuntime::initialize(
  const HardwareFactory & factory, hardware_interface::HardwareComponentParams params)
{
  owner_->expect_owner();
  plugin_ = factory(info_);
  if (!plugin_)
  {
    throw std::runtime_error("The plugin of '" + info_.name + "' could not be created");
  }
  // The owner thread runs read and write, the plugin must not start its own async handler
  params.hardware_info.is_async = false;
  owner_only_.clock_type = params.clock ? params.clock->get_clock_type() : RCL_ROS_TIME;
  if (plugin_->init(params) != CallbackReturn::SUCCESS)
  {
    throw std::runtime_error("Initialization of '" + info_.name + "' failed");
  }
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> states;
  for (const auto & state : plugin_->on_export_state_interfaces())
  {
    states.push_back(state);
  }
  shadows_.init(
    info_, std::move(states),
    writable_ ? plugin_->on_export_command_interfaces()
              : std::vector<hardware_interface::CommandInterface::SharedPtr>{},
    limit_commands_);
  RCLCPP_INFO_EXPRESSION(
    logger_, limit_commands_, "Limits %zu command interfaces to the joint limits",
    shadows_.limited());
  stops_.reset(command_interfaces().size());
  owner_only_.live.assign(command_interfaces().size(), false);
  publish_state(Lifecycle::Unconfigured);
}

Lifecycle HardwareRuntime::state() const noexcept
{
  return static_cast<Lifecycle>(
    lifecycle_.load(std::memory_order_acquire) & ((1u << STATE_BITS) - 1));
}

core::HardwareFault HardwareRuntime::fault() const noexcept
{
  const uint64_t current = generation();
  Fault result = Fault::None;
  for (const auto * latch : {&io_fault_, &call_fault_, &hang_fault_})
  {
    const Fault fault = latch->get(current);
    if (fault == Fault::Error)
    {
      return fault;
    }
    result = fault == Fault::None ? result : fault;
  }
  return result;
}

void HardwareRuntime::publish_state(Lifecycle state)
{
  plugin_->set_lifecycle_state(primary_state(state));
  std::fill(owner_only_.live.begin(), owner_only_.live.end(), false);
  const uint64_t next = generation() + 1;
  lifecycle_.store((next << STATE_BITS) | static_cast<uint64_t>(state), std::memory_order_release);
}

template <typename Call>
auto HardwareRuntime::exclusive(Call call) -> std::optional<decltype(call())>
{
  if (!gate_.owner_acquire(Clock::now() + gate_timeout_))
  {
    const std::string stuck =
      "RT did not leave a read or write within " + std::string(Millis(gate_timeout_)) + " ms";
    RCLCPP_ERROR(logger_, "%s, the component faults", stuck.c_str());
    call_fault_.raise(generation(), Fault::Error, {stuck});
    return std::nullopt;
  }
  // A copy RT took before this call is as old as the call, the cycle after it only reads
  cycle_.owner_drop_write();
  struct Release
  {
    HardwareRuntime * self;
    ~Release() { self->gate_.owner_release(); }
  } release{this};
  return call();
}

Lifecycle HardwareRuntime::error_path(std::string & reason)
{
  plugin_->enable_introspection(false);
  bool recovered = false;
  const auto on_error = thrown_by(
    [&]
    {
      const HangWatch::InCall in_call(hang_, "on_error");
      recovered = plugin_->on_error(plugin_->get_lifecycle_state()) == CallbackReturn::SUCCESS;
    });
  if (recovered)
  {
    return Lifecycle::Unconfigured;
  }
  RCLCPP_ERROR_EXPRESSION(logger_, on_error, "on_error threw: %s", on_error.value_or("").c_str());
  core::append_reason(reason, on_error ? "on_error threw: " + *on_error : "on_error failed");
  // The component did not recover, on_shutdown is the last chance to release the drive
  const auto error = thrown_by(
    [&]
    {
      const HangWatch::InCall in_call(hang_, "on_shutdown");
      plugin_->on_shutdown(plugin_->get_lifecycle_state());
    });
  RCLCPP_ERROR_EXPRESSION(
    logger_, error, "on_shutdown after a failed on_error threw: %s", error.value_or("").c_str());
  return Lifecycle::Finalized;
}

core::Outcome HardwareRuntime::call(
  Callback callback, const char * name, Lifecycle on_success, Lifecycle on_failure)
{
  CallbackReturn result = CallbackReturn::ERROR;
  std::string reason = std::string(name) + " returned ERROR";
  const auto error = thrown_by(
    [&]
    {
      const HangWatch::InCall in_call(hang_, name);
      result = ((*plugin_).*callback)(plugin_->get_lifecycle_state());
    });
  if (error)
  {
    reason = std::string(name) + " threw: " + *error;
    RCLCPP_ERROR(logger_, "%s", reason.c_str());
  }
  switch (result)
  {
    case CallbackReturn::SUCCESS:
      return core::call_outcome(CallResult::Ok, on_success);
    case CallbackReturn::FAILURE:
      return core::call_outcome(
        CallResult::Failure, on_failure, std::string(name) + " returned FAILURE");
    case CallbackReturn::ERROR:
      break;
  }
  const Lifecycle state = error_path(reason);
  return core::call_outcome(CallResult::Error, state, reason);
}

core::Outcome HardwareRuntime::transition(Transition transition)
{
  owner_->expect_owner();
  using Interface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
  auto outcome = exclusive(
    [&]
    {
      core::Outcome next;
      switch (transition)
      {
        case Transition::Configure:
          next = call(
            &Interface::on_configure, "on_configure", Lifecycle::Inactive, Lifecycle::Unconfigured);
          break;
        case Transition::Activate:
          plugin_->prepare_for_activation();
          next =
            call(&Interface::on_activate, "on_activate", Lifecycle::Active, Lifecycle::Inactive);
          plugin_->enable_introspection(next.state == Lifecycle::Active);
          break;
        case Transition::Deactivate:
          plugin_->enable_introspection(false);
          // A component that does not stop goes through on_error, it must not keep running
          next = call(
            &Interface::on_deactivate, "on_deactivate", Lifecycle::Inactive, Lifecycle::Active);
          if (next.state == Lifecycle::Active)
          {
            next.state = error_path(next.reason);
          }
          break;
        case Transition::Cleanup:
          next = call(
            &Interface::on_cleanup, "on_cleanup", Lifecycle::Unconfigured, Lifecycle::Inactive);
          break;
        case Transition::Error:
          next.state = error_path(next.reason);
          break;
        case Transition::Shutdown:
          plugin_->enable_introspection(false);
          next = call(
            &Interface::on_shutdown, "on_shutdown", Lifecycle::Finalized, Lifecycle::Finalized);
          break;
      }
      publish_state(next.state);
      return next;
    });
  if (!outcome)
  {
    return core::call_outcome(CallResult::Error, state(), call_fault_.reason());
  }
  return *std::move(outcome);
}

bool HardwareRuntime::validate(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  owner_->expect_owner();
  return exclusive(
           [&]
           {
             bool ok = false;
             const auto error = thrown_by(
               [&]
               {
                 const HangWatch::InCall in_call(hang_, "prepare_command_mode_switch");
                 ok = plugin_->prepare_command_mode_switch(start, stop) == return_type::OK;
               });
             RCLCPP_ERROR_EXPRESSION(
               logger_, error, "prepare_command_mode_switch threw: %s", error.value_or("").c_str());
             return ok;
           })
    .value_or(false);
}

std::string HardwareRuntime::switch_modes(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  owner_->expect_owner();
  // RT may have held an interface since the plan was made, the plugin must not stop it twice
  std::vector<std::string> stopping;
  for (const auto & name : stop)
  {
    if (owner_only_.live[shadows_.command_at(name)])
    {
      stopping.push_back(name);
    }
  }
  if (start.empty() && stopping.empty())
  {
    return {};
  }
  // The fallback names why exclusive faulted the component, it is evaluated after the call
  const std::string reason =
    exclusive(
      [&]
      {
        std::string failed;
        hang_.owner_switching(true);
        const auto error = thrown_by(
          [&]
          {
            const HangWatch::InCall prepare(hang_, "prepare_command_mode_switch");
            if (plugin_->prepare_command_mode_switch(start, stopping) != return_type::OK)
            {
              failed = "prepare_command_mode_switch returned ERROR";
              return;
            }
            const HangWatch::InCall perform(hang_, "perform_command_mode_switch");
            if (plugin_->perform_command_mode_switch(start, stopping) != return_type::OK)
            {
              failed = "perform_command_mode_switch returned ERROR";
            }
          });
        hang_.owner_switching(false);
        if (error)
        {
          failed = "A command mode switch threw: " + *error;
          RCLCPP_ERROR(logger_, "%s", failed.c_str());
        }
        return failed;
      })
      .value_or(call_fault_.reason());
  for (const auto & name : stopping)
  {
    owner_only_.live[shadows_.command_at(name)] = false;
  }
  if (!reason.empty())
  {
    call_fault_.raise(generation(), Fault::Error, {reason});
    return reason;
  }
  for (const auto & name : start)
  {
    owner_only_.live[shadows_.command_at(name)] = true;
  }
  return reason;
}

void HardwareRuntime::poll()
{
  if (!plugin_)
  {
    return;
  }
  hold_requested();
  if (is_async())
  {
    async_cycle();
  }
}

void HardwareRuntime::hold_requested()
{
  std::vector<uint32_t> requested;
  const auto total = stops_.owner_take(requested);
  if (!total)
  {
    return;
  }
  // A plugin may refuse to hold part of a controller's claims, so the batch goes in one switch
  std::vector<std::string> stop;
  for (const uint32_t i : requested)
  {
    if (owner_only_.live[i])
    {
      stop.push_back(command_interfaces()[i]->get_name());
    }
  }
  if (!stop.empty())
  {
    switch_modes({}, stop);
    on_hold_(info_.name, stop);
  }
  stops_.owner_done(*total);
}

void HardwareRuntime::async_cycle()
{
  const OwnerCycle::Kind cycle = cycle_.pending();
  if (cycle == OwnerCycle::Kind::None)
  {
    return;
  }
  const auto stamp = stamp_.load();
  const rclcpp::Time time(stamp.time_ns, owner_only_.clock_type);
  const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(stamp.period_ns);
  if (cycle == OwnerCycle::Kind::Read || io(true, time, period) != IoResult::Error)
  {
    io(false, time, period);
  }
  cycle_.owner_complete();
}

void HardwareRuntime::write_through_block()
{
  owner_->expect_owner();
  const auto stamp = stamp_.load();
  const rclcpp::Time time(stamp.time_ns, owner_only_.clock_type);
  const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(stamp.period_ns);
  exclusive([&] { return io(true, time, period, true); });
}

bool HardwareRuntime::writes(bool through_block) const noexcept
{
  // An async write may not apply what RT asked to stop since the owner last held
  const bool stop_pending = is_async() && stops_.pending();
  const bool blocked = stop_pending || (!through_block && write_block_.blocks(generation()));
  return state() == Lifecycle::Active && writable_ && !blocked &&
         fault() == core::HardwareFault::None;
}

IoResult HardwareRuntime::io(
  bool write, const rclcpp::Time & time, const rclcpp::Duration & period,
  bool through_block) noexcept
{
  const Lifecycle current = state();
  const bool allowed = write ? writes(through_block)
                             : (current == Lifecycle::Active || current == Lifecycle::Inactive) &&
                                 fault() == core::HardwareFault::None;
  if (!allowed)
  {
    return IoResult::Skipped;
  }
  // RT copied the commands of an async cycle when it triggered it
  if (
    write && (!is_async() || through_block) && !shadows_.copy_commands(time, period, through_block))
  {
    return IoResult::Skipped;
  }
  if (write)
  {
    shadows_.written();
  }
  const char * name = write ? "write" : "read";
  const auto start = Clock::now();
  return_type result = return_type::ERROR;
  bool threw = false;
  try
  {
    // RT runs the IO of a sync component, only the owner names its call
    const HangWatch::InCall in_call(hang_, is_async() || through_block ? name : nullptr);
    result = write ? plugin_->write(time, period) : plugin_->read(time, period);
  }
  catch (const std::exception & e)
  {
    threw = true;
    io_fault_.raise(generation(), Fault::Error, {name, " threw: ", e.what()});
  }
  catch (...)
  {
    threw = true;
    io_fault_.raise(generation(), Fault::Error, {name, " threw an exception of unknown type"});
  }
  stats_.record_io(write, seconds_since(start));
  if (threw)
  {
    return IoResult::Error;
  }
  if (result == return_type::OK)
  {
    return IoResult::Ok;
  }
  const bool deactivate = result == return_type::DEACTIVATE;
  io_fault_.raise(
    generation(), deactivate ? Fault::Deactivate : Fault::Error,
    {name, deactivate ? " returned DEACTIVATE" : " returned ERROR"});
  return deactivate ? IoResult::Deactivate : IoResult::Error;
}

std::string HardwareRuntime::last_clamped() const
{
  // A clamp happens only after initialize exported the commands
  const auto index = stats_.last_clamped();
  return index ? command_interfaces()[*index]->get_name() : "";
}

void HardwareRuntime::rt_note_claims(const std::vector<Plan::Claim> & claims) noexcept
{
  shadows_.rt_note_claims(claims);
}

IoResult HardwareRuntime::rt_read(
  const rclcpp::Time & time, const rclcpp::Duration & period) noexcept
{
  if (!gate_.rt_enter())
  {
    // The trigger counts the skips of an async owner
    if (!is_async())
    {
      stats_.skip();
    }
    return IoResult::Skipped;
  }
  IoResult result = IoResult::Skipped;
  if (!is_async())
  {
    result = io(false, time, period);
  }
  // A busy owner may be in its read, the trigger copies once it is free
  if ((!is_async() || rt_owner_free()) && !shadows_.copy_states())
  {
    stats_.skip();
  }
  gate_.rt_leave();
  return result;
}

IoResult HardwareRuntime::rt_write(
  const rclcpp::Time & time, const rclcpp::Duration & period) noexcept
{
  // Stops RT asked for must land before any further write
  if (stops_.rt_pending())
  {
    return IoResult::Skipped;
  }
  stamp_.rt_store(time.nanoseconds(), period.nanoseconds());
  if (!gate_.rt_enter())
  {
    return IoResult::Skipped;
  }
  const IoResult result = io(true, time, period);
  gate_.rt_leave();
  return result;
}

void HardwareRuntime::rt_trigger(
  const rclcpp::Time & time, const rclcpp::Duration & period) noexcept
{
  const bool entered = gate_.rt_enter();
  if (!entered || !rt_owner_free())
  {
    if (entered)
    {
      gate_.rt_leave();
    }
    stats_.skip();
    return;
  }
  // The owner may have finished its cycle after the check, which then still found it busy
  hang_.rt_end_window();
  // An owner that reads past RT's read still gets its states to the controllers
  if (!shadows_.copy_states())
  {
    stats_.skip();
  }
  stamp_.rt_store(time.nanoseconds(), period.nanoseconds());
  // Without a write the cycle still reads
  const bool write = writes(false) && shadows_.copy_commands(time, period, false);
  cycle_.rt_start(write);
  gate_.rt_leave();
  owner_->wake();
}

void HardwareRuntime::rt_request_stop(uint32_t interface) noexcept { stops_.rt_request(interface); }

void HardwareRuntime::rt_publish_stops() noexcept
{
  if (stops_.rt_publish())
  {
    owner_->wake();
  }
}

bool HardwareRuntime::rt_take_fault(uint64_t plan_generation) noexcept
{
  const uint64_t current = generation();
  const bool handled =
    rt_.handled_generation == current && rt_.handled_plan_generation == plan_generation;
  if (handled || fault() == core::HardwareFault::None)
  {
    return false;
  }
  rt_.handled_generation = current;
  rt_.handled_plan_generation = plan_generation;
  return true;
}

bool HardwareRuntime::rt_owner_free() const noexcept
{
  return cycle_.pending() == OwnerCycle::Kind::None && !gate_.owner_in();
}

bool HardwareRuntime::rt_enter_idle() noexcept
{
  if (!gate_.rt_enter())
  {
    return false;
  }
  if (is_async() && !rt_owner_free())
  {
    gate_.rt_leave();
    return false;
  }
  return true;
}

bool HardwareRuntime::rt_check_hang(
  Clock::time_point now, Clock::duration timeout, bool in_use) noexcept
{
  const bool busy = is_async() ? !rt_owner_free() : gate_.owner_in();
  const auto stall = hang_.rt_check(now, timeout, in_use && busy);
  if (!stall || fault() != core::HardwareFault::None)
  {
    return false;
  }
  const char * call = hang_.call();
  hang_fault_.raise(
    generation(), Fault::Error,
    {"no read or write ran for ", Millis(*stall), " ms, longer than the hang timeout of ",
     Millis(timeout), " ms, while its owner ", call ? "was in " : "ran no plugin call",
     call ? call : ""});
  return true;
}

}  // namespace hector_controller_manager::runtime
