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

#include <charconv>
#include <stdexcept>
#include <utility>

#include "hector_controller_manager/runtime/lifecycle_state.hpp"
#include "hector_controller_manager/runtime/thrown.hpp"
#include "rclcpp/logging.hpp"

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

constexpr int KIND_BITS = 2;
static_assert(static_cast<int>(Fault::Deactivate) < (1 << KIND_BITS));

Fault fault_in(uint64_t word, uint64_t generation) noexcept
{
  return (word >> KIND_BITS) == generation ? static_cast<Fault>(word & ((1u << KIND_BITS) - 1))
                                           : Fault::None;
}

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

/// Names the plugin call the owner is in while it lives, nothing for a null name.
class InCall
{
public:
  InCall(std::atomic<const char *> & slot, const char * name) noexcept
  : slot_(name ? &slot : nullptr)
  {
    if (slot_)
    {
      slot_->store(name, std::memory_order_relaxed);
    }
  }
  ~InCall()
  {
    if (slot_)
    {
      slot_->store(nullptr, std::memory_order_relaxed);
    }
  }
  InCall(const InCall &) = delete;
  InCall & operator=(const InCall &) = delete;

private:
  std::atomic<const char *> * slot_;
};
}  // namespace

void HardwareRuntime::FaultReason::set(std::initializer_list<std::string_view> parts) noexcept
{
  size_t size = 0;
  for (const auto part : parts)
  {
    for (const char c : part)
    {
      if (size + 1 < text_.size())
      {
        text_[size++].store(c, std::memory_order_relaxed);
      }
    }
  }
  text_[size].store('\0', std::memory_order_relaxed);
}

std::string HardwareRuntime::FaultReason::get() const
{
  std::string text;
  for (const auto & c : text_)
  {
    const char value = c.load(std::memory_order_relaxed);
    if (value == '\0')
    {
      break;
    }
    text += value;
  }
  return text;
}

void HardwareRuntime::raise(
  std::atomic<uint64_t> & word, FaultReason & reason, Fault kind,
  std::initializer_list<std::string_view> parts) noexcept
{
  const uint64_t current = generation();
  if (fault_in(word.load(std::memory_order_relaxed), current) != Fault::None)
  {
    return;
  }
  reason.set(parts);
  word.store((current << KIND_BITS) | static_cast<uint64_t>(kind), std::memory_order_release);
}

std::string HardwareRuntime::fault_reason() const
{
  const uint64_t current = generation();
  const std::pair<const std::atomic<uint64_t> *, const FaultReason *> faults[] = {
    {&io_fault_, &io_reason_}, {&call_fault_, &call_reason_}, {&hang_fault_, &hang_reason_}};
  std::string result;
  for (const auto & [word, reason] : faults)
  {
    if (fault_in(word->load(std::memory_order_acquire), current) != Fault::None)
    {
      core::append_reason(result, reason->get());
    }
  }
  return result;
}

HardwareRuntime::HardwareRuntime(
  hardware_interface::HardwareInfo info, HoldListener on_hold, Clock::duration gate_timeout)
: info_(std::move(info)),
  gate_timeout_(gate_timeout),
  writable_(info_.type != "sensor"),
  on_hold_(std::move(on_hold)),
  logger_(rclcpp::get_logger("hector_controller_manager").get_child(info_.name))
{
  owner_ = std::make_unique<OwnerThread>([this] { poll(); });
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
  for (const auto & state : plugin_->on_export_state_interfaces())
  {
    states_.push_back(state);
  }
  if (writable_)
  {
    commands_ = plugin_->on_export_command_interfaces();
  }
  for (uint32_t i = 0; i < commands_.size(); ++i)
  {
    command_index_.emplace(commands_[i]->get_name(), i);
  }
  stop_requests_ = std::make_unique<std::atomic<uint32_t>[]>(commands_.size());
  owner_only_.stops_seen.assign(commands_.size(), 0);
  owner_only_.live.assign(commands_.size(), false);
  publish_state(Lifecycle::Unconfigured);
}

std::optional<uint32_t> HardwareRuntime::command_index(const std::string & name) const
{
  const auto it = command_index_.find(name);
  return it == command_index_.end() ? std::nullopt : std::optional(it->second);
}

Lifecycle HardwareRuntime::state() const noexcept
{
  return static_cast<Lifecycle>(lifecycle_.load(std::memory_order_acquire) & 0xff);
}

core::HardwareFault HardwareRuntime::fault() const noexcept
{
  const uint64_t current = generation();
  Fault result = Fault::None;
  for (const auto * word : {&io_fault_, &call_fault_, &hang_fault_})
  {
    const Fault fault = fault_in(word->load(std::memory_order_acquire), current);
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
  lifecycle_.store((next << 8) | static_cast<uint64_t>(state), std::memory_order_release);
}

template <typename Call>
auto HardwareRuntime::exclusive(Call call) -> std::optional<decltype(call())>
{
  if (!is_async() && !gate_.owner_acquire(Clock::now() + gate_timeout_))
  {
    const std::string stuck =
      "RT did not leave a read or write within " + std::string(Millis(gate_timeout_)) + " ms";
    RCLCPP_ERROR(logger_, "%s, the component faults", stuck.c_str());
    raise(call_fault_, call_reason_, Fault::Error, {stuck});
    return std::nullopt;
  }
  struct Release
  {
    HardwareRuntime * self;
    ~Release()
    {
      if (!self->is_async())
      {
        self->gate_.owner_release();
      }
    }
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
      const InCall in_call(owner_call_, "on_error");
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
      const InCall in_call(owner_call_, "on_shutdown");
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
      const InCall in_call(owner_call_, name);
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
    return core::call_outcome(CallResult::Error, state(), call_reason_.get());
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
                 const InCall in_call(owner_call_, "prepare_command_mode_switch");
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
    if (owner_only_.live[command_index_.at(name)])
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
        const auto error = thrown_by(
          [&]
          {
            const InCall prepare(owner_call_, "prepare_command_mode_switch");
            if (plugin_->prepare_command_mode_switch(start, stopping) != return_type::OK)
            {
              failed = "prepare_command_mode_switch returned ERROR";
              return;
            }
            const InCall perform(owner_call_, "perform_command_mode_switch");
            if (plugin_->perform_command_mode_switch(start, stopping) != return_type::OK)
            {
              failed = "perform_command_mode_switch returned ERROR";
            }
          });
        if (error)
        {
          failed = "A command mode switch threw: " + *error;
          RCLCPP_ERROR(logger_, "%s", failed.c_str());
        }
        return failed;
      })
      .value_or(call_reason_.get());
  for (const auto & name : stopping)
  {
    owner_only_.live[command_index_.at(name)] = false;
  }
  if (!reason.empty())
  {
    raise(call_fault_, call_reason_, Fault::Error, {reason});
    return reason;
  }
  for (const auto & name : start)
  {
    owner_only_.live[command_index_.at(name)] = true;
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
  const uint32_t total = stop_total_.load(std::memory_order_acquire);
  if (total == stops_done_.load(std::memory_order_relaxed))
  {
    return;
  }
  std::vector<std::string> stop;
  for (uint32_t i = 0; i < commands_.size(); ++i)
  {
    const uint32_t requests = stop_requests_[i].load(std::memory_order_acquire);
    if (requests != owner_only_.stops_seen[i])
    {
      owner_only_.stops_seen[i] = requests;
      if (owner_only_.live[i])
      {
        stop.push_back(commands_[i]->get_name());
      }
    }
  }
  if (!stop.empty())
  {
    switch_modes({}, stop);
    on_hold_(info_.name, stop);
  }
  stops_done_.store(total, std::memory_order_release);
}

void HardwareRuntime::async_cycle()
{
  const uint64_t triggers = triggers_.load(std::memory_order_acquire);
  if (triggers == owner_only_.triggers_seen)
  {
    return;
  }
  owner_only_.triggers_seen = triggers;
  const rclcpp::Time time(trigger_time_ns_.load(std::memory_order_relaxed), owner_only_.clock_type);
  const rclcpp::Duration period =
    rclcpp::Duration::from_nanoseconds(trigger_period_ns_.load(std::memory_order_relaxed));
  if (io(true, time, period) != IoResult::Error)
  {
    io(false, time, period);
  }
  served_.store(triggers, std::memory_order_release);
}

void HardwareRuntime::block_writes() noexcept
{
  write_block_.store(generation() + 1, std::memory_order_release);
}

void HardwareRuntime::write_through_block()
{
  owner_->expect_owner();
  const rclcpp::Time time(trigger_time_ns_.load(std::memory_order_relaxed), owner_only_.clock_type);
  const rclcpp::Duration period =
    rclcpp::Duration::from_nanoseconds(trigger_period_ns_.load(std::memory_order_relaxed));
  exclusive([&] { return io(true, time, period, true); });
}

IoResult HardwareRuntime::io(
  bool write, const rclcpp::Time & time, const rclcpp::Duration & period,
  bool through_block) noexcept
{
  const Lifecycle current = state();
  const bool allowed = write ? current == Lifecycle::Active && writable_
                             : current == Lifecycle::Active || current == Lifecycle::Inactive;
  // An async write may not apply what RT asked to stop since the owner last held
  const bool stop_pending =
    write && is_async() &&
    stop_total_.load(std::memory_order_acquire) != stops_done_.load(std::memory_order_relaxed);
  const bool blocked =
    stop_pending ||
    (write && !through_block && write_block_.load(std::memory_order_acquire) == generation() + 1);
  if (!allowed || blocked || fault() != core::HardwareFault::None)
  {
    return IoResult::Skipped;
  }
  const char * name = write ? "write" : "read";
  const auto start = Clock::now();
  return_type result = return_type::ERROR;
  bool threw = false;
  try
  {
    // RT runs the IO of a sync component, only the owner names its call
    const InCall in_call(owner_call_, is_async() || through_block ? name : nullptr);
    result = write ? plugin_->write(time, period) : plugin_->read(time, period);
  }
  catch (const std::exception & e)
  {
    threw = true;
    raise(io_fault_, io_reason_, Fault::Error, {name, " threw: ", e.what()});
  }
  catch (...)
  {
    threw = true;
    raise(io_fault_, io_reason_, Fault::Error, {name, " threw an exception of unknown type"});
  }
  (write ? write_seconds_ : read_seconds_).store(seconds_since(start), std::memory_order_relaxed);
  if (threw)
  {
    return IoResult::Error;
  }
  if (result == return_type::OK)
  {
    return IoResult::Ok;
  }
  const bool deactivate = result == return_type::DEACTIVATE;
  raise(
    io_fault_, io_reason_, deactivate ? Fault::Deactivate : Fault::Error,
    {name, deactivate ? " returned DEACTIVATE" : " returned ERROR"});
  return deactivate ? IoResult::Deactivate : IoResult::Error;
}

IoResult HardwareRuntime::rt_read(
  const rclcpp::Time & time, const rclcpp::Duration & period) noexcept
{
  if (!gate_.rt_enter())
  {
    skipped_.fetch_add(1, std::memory_order_relaxed);
    if (rt_.stalled_since == Clock::time_point{})
    {
      rt_.stalled_since = Clock::now();
    }
    return IoResult::Skipped;
  }
  rt_.stalled_since = {};
  const IoResult result = io(false, time, period);
  gate_.rt_leave();
  return result;
}

IoResult HardwareRuntime::rt_write(
  const rclcpp::Time & time, const rclcpp::Duration & period) noexcept
{
  // Stops RT asked for must land before any further write
  if (stop_total_.load(std::memory_order_relaxed) != stops_done_.load(std::memory_order_acquire))
  {
    return IoResult::Skipped;
  }
  trigger_time_ns_.store(time.nanoseconds(), std::memory_order_relaxed);
  trigger_period_ns_.store(period.nanoseconds(), std::memory_order_relaxed);
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
  trigger_time_ns_.store(time.nanoseconds(), std::memory_order_relaxed);
  trigger_period_ns_.store(period.nanoseconds(), std::memory_order_relaxed);
  if (served_.load(std::memory_order_acquire) != triggers_.load(std::memory_order_relaxed))
  {
    skipped_.fetch_add(1, std::memory_order_relaxed);
  }
  rt_track_async_progress(Clock::now());
  triggers_.fetch_add(1, std::memory_order_release);
  owner_->wake();
}

void HardwareRuntime::rt_request_stop(uint32_t interface) noexcept
{
  stop_requests_[interface].fetch_add(1, std::memory_order_release);
  stop_total_.fetch_add(1, std::memory_order_release);
  owner_->wake();
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

void HardwareRuntime::rt_track_async_progress(Clock::time_point now) noexcept
{
  const uint64_t served = served_.load(std::memory_order_acquire);
  if (served == triggers_.load(std::memory_order_relaxed))
  {
    // Idle, a late check only measured a stall of RT itself
    rt_.stalled_since = {};
  }
  else if (served != rt_.last_served || rt_.stalled_since == Clock::time_point{})
  {
    // A slow owner that still completes cycles is not hung, the window starts at its last one
    rt_.stalled_since = now;
  }
  rt_.last_served = served;
}

bool HardwareRuntime::rt_check_hang(Clock::time_point now, Clock::duration timeout) noexcept
{
  if (is_async())
  {
    rt_track_async_progress(now);
  }
  const bool stalled =
    rt_.stalled_since != Clock::time_point{} && now - rt_.stalled_since > timeout;
  if (!stalled || fault() != core::HardwareFault::None)
  {
    return false;
  }
  const char * call = owner_call_.load(std::memory_order_relaxed);
  raise(
    hang_fault_, hang_reason_, Fault::Error,
    {"no read or write ran for ", Millis(now - rt_.stalled_since),
     " ms, longer than the hang timeout of ", Millis(timeout), " ms, while its owner ",
     call ? "was in " : "ran no plugin call", call ? call : ""});
  return true;
}

}  // namespace hector_controller_manager::runtime
