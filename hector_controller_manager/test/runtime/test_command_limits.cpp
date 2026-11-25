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

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "allocation_counter.hpp"
#include "controller_interface/controller_interface.hpp"
#include "fakes.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/runtime/rt_loop.hpp"
#include "hector_controller_manager/supervisor.hpp"
#include "supervisor_rig.hpp"
#include "urdf.hpp"

namespace hector_controller_manager
{
namespace
{
using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::return_type;
using testing::eventually;
using testing::HeldLock;

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
/// The rig's period and the URDF velocity limit.
constexpr double PERIOD = 0.001;
constexpr double VELOCITY = 2.0;
constexpr double STEP = VELOCITY * PERIOD * (1 + 1e-9);

std::string urdf(bool async, const std::string & command = "position")
{
  return testing::urdf(
    {{"j1", {command}, {"position", "velocity"}, true}}, async ? R"(is_async="true")" : "");
}

/// What the hardware wrote, tracked from the first command after a restart. An async write may
/// follow a cycle its owner was busy in, so steps count per period since the write before.
struct Observed
{
  std::atomic<double> actual{0.0};
  std::atomic<double> goal{0.9};
  std::atomic<bool> restart{true};
  std::atomic<double> first{NaN};
  std::atomic<double> first_periods{1.0};
  std::atomic<double> last{NaN};
  std::atomic<double> largest_step{0.0};
  std::atomic<int> writes{0};
  std::atomic<int> nans{0};
  std::atomic<int64_t> write_ns{0};
  /// Counts the writes and their commands from this time on.
  std::atomic<int64_t> count_from_ns{std::numeric_limits<int64_t>::max()};
  std::atomic<int> counted_writes{0};
  std::atomic<int> counted{0};
  /// Holds a write before it reads the command.
  testing::Gate write_gate;
  /// The next write records the command it read.
  std::atomic<bool> record_next{false};
  std::atomic<double> recorded{NaN};
  testing::Gate deactivate_gate;
};

class System : public hardware_interface::SystemInterface
{
public:
  explicit System(Observed & observed) : observed_(observed) {}

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    set_state("j1/position", observed_.actual.load());
    set_state("j1/velocity", 0.0);
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    const bool record = observed_.record_next.exchange(false);
    observed_.write_gate.pass();
    const double periods = std::max(
      1.0, std::round(
             1e-9 * static_cast<double>(time.nanoseconds() - observed_.write_ns.load()) / PERIOD));
    observed_.write_ns = time.nanoseconds();
    if (observed_.restart.exchange(false))
    {
      observed_.first = NaN;
      observed_.last = NaN;
      observed_.largest_step = 0.0;
      observed_.writes = 0;
    }
    const bool counting = time.nanoseconds() >= observed_.count_from_ns.load();
    if (counting)
    {
      observed_.counted_writes.fetch_add(1);
    }
    const double value = get_command<double>("j1/position");
    if (record)
    {
      observed_.recorded = value;
    }
    if (std::isnan(value))
    {
      observed_.nans.fetch_add(1);
      return return_type::OK;
    }
    if (counting)
    {
      observed_.counted.fetch_add(1);
    }
    if (std::isnan(observed_.first.load()))
    {
      observed_.first = value;
      observed_.first_periods = periods;
    }
    else
    {
      observed_.largest_step =
        std::max(observed_.largest_step.load(), std::abs(value - observed_.last.load()) / periods);
    }
    observed_.last = value;
    observed_.writes.fetch_add(1);
    return return_type::OK;
  }

private:
  Observed & observed_;
};

/// Commands the goal in every update, or nothing if quiet.
class Controller : public controller_interface::ControllerInterface
{
public:
  Controller(Observed & observed, bool quiet) : observed_(observed), quiet_(quiet) {}

  InterfaceConfiguration command_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, {"j1/position"}};
  }
  InterfaceConfiguration state_interface_configuration() const override
  {
    return {interface_configuration_type::NONE, {}};
  }
  CallbackReturn on_init() override { return CallbackReturn::SUCCESS; }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    observed_.deactivate_gate.pass();
    return CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!quiet_)
    {
      (void)command_interfaces_[0].set_value(observed_.goal.load());
    }
    return controller_interface::return_type::OK;
  }

private:
  Observed & observed_;
  bool quiet_;
};

/// On sync and async hardware by the parameter.
class CommandLimitsTest : public testing::SupervisorRig<::testing::TestWithParam<bool>>
{
protected:
  void start_with(const SupervisorConfig & config)
  {
    start(config, host_, urdf(GetParam()));
    load_and_configure("controller", "test/Controller");
  }

  void cycle(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    testing::CountAllocations scope(counting_.load());
    supervisor_->rt().read(time, period);
    supervisor_->rt().update(time, period);
    if (hold_after_update_.exchange(false))
    {
      after_update_.pass();
    }
    supervisor_->rt().write(time, period);
  }

  /// Moves the joint and waits until the hardware read it, an async one reads after its write.
  bool move_to(double actual)
  {
    observed_.actual = actual;
    const auto status = supervisor_->status();
    for (const auto & state : status->hardware.front().runtime->state_interfaces())
    {
      if (state->get_name() == "j1/position")
      {
        return eventually([&] { return state->get_optional() == actual; });
      }
    }
    return false;
  }

  /// Activates the controller and waits for `count` written commands of it.
  bool activate_and_write(int count)
  {
    observed_.restart = true;
    return switch_controllers({"controller"}, {}).ok &&
           eventually([&] { return !observed_.restart && observed_.writes >= count; });
  }

  /// The commands the hardware applies while the controller's on_deactivate is held. Its shadow
  /// still holds the goal then, RT already runs with the claims of the detach.
  int commands_during_a_slow_deactivate(const SupervisorConfig & config)
  {
    start_with(config);
    EXPECT_TRUE(activate_and_write(5));
    observed_.deactivate_gate.close();
    const testing::OnExit open([&] { observed_.deactivate_gate.open(); });
    auto stop =
      std::async(std::launch::async, [&] { return switch_controllers({}, {"controller"}).ok; });
    EXPECT_TRUE(observed_.deactivate_gate.holds());
    // The detach waited for a write with its claims, so every later cycle runs with them
    observed_.count_from_ns =
      (loop_.cycles() + 1) * rclcpp::Duration::from_seconds(PERIOD).nanoseconds();
    EXPECT_TRUE(cycles(20));
    EXPECT_GT(observed_.counted_writes.load(), 0) << "precondition: the hardware wrote meanwhile";
    observed_.deactivate_gate.open();
    EXPECT_TRUE(stop.get());
    return observed_.counted;
  }

  Observed observed_;
  testing::FakeHost host_{
    [this](const std::string & name, const std::string &)
    { return std::make_shared<Controller>(observed_, name.rfind("quiet", 0) == 0); },
    [this] { return std::make_unique<System>(observed_); }};
  std::atomic<bool> counting_{false};
  std::atomic<bool> hold_after_update_{false};
  testing::Gate after_update_;
};

TEST_P(CommandLimitsTest, a_position_goal_jump_reaches_the_hardware_at_the_velocity_limit)
{
  start_with({});
  ASSERT_TRUE(activate_and_write(50));
  EXPECT_LE(std::abs(observed_.first.load()), STEP * observed_.first_periods.load());
  EXPECT_LE(observed_.largest_step.load(), STEP);
  EXPECT_GT(observed_.largest_step.load(), STEP / 2) << "precondition: the joint moved";
  EXPECT_GT(supervisor_->status()->hardware.front().runtime->stats().clamped(), 0u);
}

TEST_P(CommandLimitsTest, an_acceleration_limit_from_the_parameters_is_enforced)
{
  SupervisorConfig config;
  config.limit_parameters["j1"].has_acceleration_limits = true;
  config.limit_parameters["j1"].max_acceleration = 10.0;
  start_with(config);
  ASSERT_TRUE(activate_and_write(5));
  const double seconds = PERIOD * observed_.first_periods.load();
  EXPECT_LE(std::abs(observed_.first.load()), 10.0 * seconds * seconds * (1 + 1e-9));
}

TEST_P(CommandLimitsTest, a_reactivation_starts_from_the_actual_position_without_a_jump)
{
  start_with({});
  ASSERT_TRUE(activate_and_write(50));
  ASSERT_TRUE(switch_controllers({}, {"controller"}).ok);
  ASSERT_TRUE(move_to(-0.5));
  observed_.goal = -0.5;
  ASSERT_TRUE(activate_and_write(5));
  EXPECT_NEAR(observed_.first.load(), -0.5, STEP * observed_.first_periods.load());
  EXPECT_LE(observed_.largest_step.load(), STEP);
}

TEST_P(CommandLimitsTest, a_restart_after_an_estop_starts_from_the_actual_position)
{
  start_with({});
  ASSERT_TRUE(activate_and_write(50));
  supervisor_->rt().set_estop(true);
  ASSERT_TRUE(
    eventually([&] { return controller("controller")->state == core::Lifecycle::Inactive; }));
  supervisor_->rt().set_estop(false);
  ASSERT_TRUE(move_to(0.7));
  observed_.goal = 0.7;
  ASSERT_TRUE(activate_and_write(5));
  EXPECT_NEAR(observed_.first.load(), 0.7, STEP * observed_.first_periods.load());
}

// The controller wrote a raw goal in the update, the write after it already runs with the plan
// that detached the controller
TEST_P(CommandLimitsTest, a_detach_between_update_and_write_sends_no_raw_command)
{
  start_with({});
  ASSERT_TRUE(activate_and_write(5));
  after_update_.close();
  const testing::OnExit open([&] { after_update_.open(); });
  hold_after_update_ = true;
  ASSERT_TRUE(after_update_.holds());
  auto stop =
    std::async(std::launch::async, [&] { return switch_controllers({}, {"controller"}).ok; });
  // The detach publishes its plan and then waits for this write
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(stop.wait_for(std::chrono::seconds(0)), std::future_status::timeout)
    << "nothing is released before the write with the claims of the update";
  after_update_.open();
  EXPECT_TRUE(stop.get());
  EXPECT_LE(observed_.largest_step.load(), STEP);
}

TEST_P(CommandLimitsTest, a_slow_deactivate_sends_no_raw_command)
{
  EXPECT_EQ(commands_during_a_slow_deactivate({}), 0) << "applied " << observed_.last.load();
}

TEST_P(CommandLimitsTest, a_slow_deactivate_without_limits_sends_no_raw_command)
{
  SupervisorConfig config;
  config.enforce_command_limits = false;
  EXPECT_EQ(commands_during_a_slow_deactivate(config), 0) << "applied " << observed_.last.load();
}

TEST_P(CommandLimitsTest, disabled_limits_pass_the_command_unchanged)
{
  SupervisorConfig config;
  config.enforce_command_limits = false;
  start_with(config);
  ASSERT_TRUE(activate_and_write(5));
  EXPECT_DOUBLE_EQ(observed_.first.load(), 0.9);
}

TEST_P(CommandLimitsTest, excluded_hardware_passes_the_command_unchanged)
{
  SupervisorConfig config;
  config.command_limits_excluded = {"system"};
  start_with(config);
  ASSERT_TRUE(activate_and_write(5));
  EXPECT_DOUBLE_EQ(observed_.first.load(), 0.9);
}

TEST_P(CommandLimitsTest, limits_for_an_unknown_joint_are_rejected)
{
  SupervisorConfig config;
  config.limit_parameters["j2"].has_acceleration_limits = true;
  config.limit_parameters["j2"].max_acceleration = 1.0;
  supervisor_ = std::make_unique<Supervisor>(config, host_, rclcpp::get_logger("test"), nullptr);
  const auto response = supervisor_->initialize(urdf(GetParam()), {});
  EXPECT_FALSE(response.ok);
  EXPECT_NE(response.message.find("joint_limits.j2"), std::string::npos) << response.message;
  supervisor_->shutdown(testing::FINISH_TIMEOUT);
  // Without the loop of start, which stop would join
  supervisor_.reset();
}

TEST_P(CommandLimitsTest, limiting_does_not_allocate_in_the_control_loop)
{
  if (!testing::allocation_counting_available())
  {
    GTEST_SKIP() << "Sanitizers replace operator new";
  }
  start_with({});
  ASSERT_TRUE(activate_and_write(5));
  const uint64_t before = testing::counted_allocations();
  const uint64_t clamped = supervisor_->status()->hardware.front().runtime->stats().clamped();
  counting_ = true;
  EXPECT_TRUE(cycles(100));
  observed_.actual = 1.05;
  ASSERT_TRUE(switch_controllers({}, {"controller"}).ok);
  ASSERT_TRUE(switch_controllers({"controller"}, {}).ok);
  EXPECT_TRUE(cycles(100));
  counting_ = false;
  EXPECT_EQ(testing::counted_allocations() - before, 0u);
  const auto & runtime = *supervisor_->status()->hardware.front().runtime;
  EXPECT_GT(runtime.stats().clamped(), clamped) << "precondition: it clamped";
  EXPECT_GT(runtime.stats().out_of_range_cycles(), 0u)
    << "precondition: it saw the joint out of range";
}

// RT pauses between an update and its write while a stop detaches the controller, and that write
// stalls in the plugin while the stop releases the joint. The release's NaN reaches the hardware,
// so a claimer that commands nothing never applies the old goal.
TEST_P(CommandLimitsTest, a_quiet_claimer_after_a_stop_never_applies_the_old_goal)
{
  start_with({});
  load_and_configure("quiet", "test/Controller");
  ASSERT_TRUE(activate_and_write(5));
  after_update_.close();
  const testing::OnExit open(
    [&]
    {
      after_update_.open();
      observed_.write_gate.open();
    });
  hold_after_update_ = true;
  ASSERT_TRUE(after_update_.holds());
  auto stop =
    std::async(std::launch::async, [&] { return switch_controllers({}, {"controller"}).ok; });
  // The stop holds the joint before RT writes the command of its update
  ASSERT_TRUE(eventually([&] { return !hardware().live.count("j1/position"); }));
  observed_.record_next = true;
  observed_.write_gate.close();
  after_update_.open();
  ASSERT_TRUE(observed_.write_gate.holds());
  // The stalled write of an async owner does not hold the stop, which releases the joint meanwhile
  if (GetParam())
  {
    ASSERT_TRUE(
      eventually([&] { return controller("controller")->state != core::Lifecycle::Active; }));
  }
  observed_.write_gate.open();
  ASSERT_TRUE(stop.get());
  // An async owner finishes the stalled write after the stop returned
  EXPECT_TRUE(eventually([&] { return !std::isnan(observed_.recorded.load()); }))
    << "precondition: the stalled write carries the old goal";
  ASSERT_TRUE(switch_controllers({"quiet"}, {}).ok);
  observed_.restart = true;
  ASSERT_TRUE(eventually([&] { return !observed_.restart.load(); }));
  ASSERT_TRUE(cycles(50));
  EXPECT_EQ(observed_.writes.load(), 0) << "applied " << observed_.last.load();
}

INSTANTIATE_TEST_SUITE_P(
  Hardware, CommandLimitsTest, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "async" : "sync"; });

/// A joint commanded at 1 rad/s toward its upper limit of 1.
struct Driven
{
  std::atomic<double> position{0.8};
  std::atomic<double> speed{0.0};
  std::atomic<double> highest{0.8};
  std::atomic<int> stopped_writes{0};
};

/// Moves the joint at the last written speed between the times of its calls, so an async read
/// right after the write does not see the motion of that write yet.
class VelocitySystem : public hardware_interface::SystemInterface
{
public:
  explicit VelocitySystem(Driven & driven) : driven_(driven) {}

  return_type read(const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    move_until(time);
    set_state("j1/position", driven_.position.load());
    set_state("j1/velocity", driven_.speed.load());
    return return_type::OK;
  }
  return_type write(const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    move_until(time);
    const double speed = get_command<double>("j1/velocity");
    if (std::isnan(speed))
    {
      return return_type::OK;
    }
    driven_.speed = speed;
    if (std::abs(speed) < 1e-9)
    {
      driven_.stopped_writes.fetch_add(1);
    }
    return return_type::OK;
  }

private:
  void move_until(const rclcpp::Time & time)
  {
    const int64_t now = time.nanoseconds();
    driven_.position =
      driven_.position + driven_.speed * 1e-9 * static_cast<double>(now - last_ns_);
    driven_.highest = std::max(driven_.highest.load(), driven_.position.load());
    last_ns_ = now;
  }

  Driven & driven_;
  int64_t last_ns_ = 0;
};

class VelocityController : public controller_interface::ControllerInterface
{
public:
  InterfaceConfiguration command_interface_configuration() const override
  {
    return {interface_configuration_type::INDIVIDUAL, {"j1/velocity"}};
  }
  InterfaceConfiguration state_interface_configuration() const override
  {
    return {interface_configuration_type::NONE, {}};
  }
  CallbackReturn on_init() override { return CallbackReturn::SUCCESS; }
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    (void)command_interfaces_[0].set_value(1.0);
    return controller_interface::return_type::OK;
  }
};

/// A velocity command at a position limit, on sync and async hardware by the parameter.
class VelocityLimitsTest : public testing::SupervisorRig<::testing::TestWithParam<bool>>
{
protected:
  /// Drives the joint into its limit and returns the highest position it reached.
  double drive(const SupervisorConfig & config)
  {
    start(config, host_, urdf(GetParam(), "velocity"));
    load_and_configure("controller", "test/Velocity");
    EXPECT_TRUE(switch_controllers({"controller"}, {}).ok);
    EXPECT_TRUE(eventually([&] { return driven_.stopped_writes.load() >= 50; }))
      << "it reached " << driven_.highest.load();
    return driven_.highest.load();
  }

  Driven driven_;
  testing::FakeHost host_{
    [](const std::string &, const std::string &) { return std::make_shared<VelocityController>(); },
    [this] { return std::make_unique<VelocitySystem>(driven_); }};
};

// Braking at the acceleration limit stops it at the limit. At 100 Hz an async owner moves the joint
// a long period past the position the limiter saw.
TEST_P(VelocityLimitsTest, a_velocity_command_brakes_at_the_acceleration_limit_before_the_limit)
{
  SupervisorConfig config;
  config.limit_parameters["j1"].has_acceleration_limits = true;
  config.limit_parameters["j1"].max_acceleration = 10.0;
  period_ = 0.01;
  driven_.position = 0.0;
  const double highest = drive(config);
  EXPECT_LE(highest, 1.0 + 1e-9);
  EXPECT_GT(highest, 0.99) << "precondition: it reached the limit";
}

TEST_P(VelocityLimitsTest, without_an_acceleration_limit_a_velocity_command_stops_at_the_limit)
{
  // Half a step off the grid of the steps, so a full last step would pass the limit
  driven_.position = 0.8005;
  const double highest = drive({});
  EXPECT_LE(highest, 1.0 + 1e-9);
  EXPECT_GT(highest, 0.99) << "precondition: it reached the limit";
}

INSTANTIATE_TEST_SUITE_P(
  Hardware, VelocityLimitsTest, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) { return info.param ? "async" : "sync"; });

using runtime::IoResult;

/// A component with the limited position command of j1, brought up and claimed, at 1000 Hz.
struct LimitedRuntime
{
  explicit LimitedRuntime(bool async = false, unsigned int update_rate = 1000)
  {
    auto info = testing::hardware_info("system", async);
    info.rw_rate = update_rate;
    auto & joint = info.joints.front();
    joint.command_interfaces.front().enable_limits = true;
    hardware_interface::InterfaceInfo velocity{};
    velocity.name = "velocity";
    joint.state_interfaces.push_back(velocity);
    info.limits["j1"].has_velocity_limits = true;
    info.limits["j1"].max_velocity = VELOCITY;
    hardware = std::make_unique<runtime::HardwareRuntime>(
      info, [](const std::string &, std::vector<std::string>) {}, std::chrono::seconds(1), true);
    EXPECT_TRUE(
      testing::bring_up(*hardware, [this] { return std::make_unique<System>(observed); }));
    hardware->rt_note_claims({{nullptr, 1}});
  }
  /// One cycle, an update in which a controller commands 0.9 and then `writes` writes a step
  /// apart, each with the period since the update as gz passes it.
  IoResult write(double step = PERIOD, int writes = 1)
  {
    hardware->rt_note_claims({{nullptr, 1}});
    EXPECT_TRUE(hardware->command_interfaces().front()->set_value(0.9));
    hardware->rt_read(rclcpp::Time(now_ns, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(step));
    IoResult result = IoResult::Skipped;
    for (int i = 1; i <= writes; ++i)
    {
      now_ns += rclcpp::Duration::from_seconds(step).nanoseconds();
      result = hardware->rt_write(
        rclcpp::Time(now_ns, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(i * step));
    }
    return result;
  }

  int64_t now_ns = 0;

  Observed observed;
  std::unique_ptr<runtime::HardwareRuntime> hardware;
};

TEST(CommandLimitsRuntime, a_clamp_names_the_command_interface_it_clamped)
{
  LimitedRuntime rig;
  EXPECT_EQ(rig.hardware->last_clamped(), "");
  ASSERT_EQ(rig.write(), IoResult::Ok);
  EXPECT_EQ(rig.hardware->stats().clamped(), 1u);
  EXPECT_EQ(rig.hardware->last_clamped(), "j1/position");
}

TEST(CommandLimitsRuntime, a_skipped_write_does_not_advance_the_limits)
{
  LimitedRuntime rig;
  ASSERT_EQ(rig.write(), IoResult::Ok);
  rig.hardware->block_writes();
  for (int i = 0; i < 50; ++i)
  {
    ASSERT_EQ(rig.write(), IoResult::Skipped);
  }
  rig.hardware->unblock_writes();
  ASSERT_EQ(rig.write(), IoResult::Ok);
  EXPECT_LE(rig.observed.largest_step.load(), STEP);
  EXPECT_NEAR(rig.observed.last.load(), 2 * VELOCITY * PERIOD, 1e-9);
}

TEST(CommandLimitsRuntime, a_write_after_a_detach_limits_with_the_claims_of_the_update)
{
  LimitedRuntime rig;
  runtime::Doorbell bell;
  runtime::PlanSlot slot(bell);
  runtime::RtLoop loop(slot, bell, std::chrono::seconds(1));
  runtime::Plan claimed;
  claimed.generation = 1;
  claimed.hardware.push_back({rig.hardware.get(), true, {}, {{nullptr, 1}}});
  runtime::Plan released;
  released.generation = 2;
  released.hardware.push_back({rig.hardware.get(), false, {}, {{}}});
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  const auto period = rclcpp::Duration::from_seconds(PERIOD);
  slot.publish(&claimed);
  ASSERT_TRUE(rig.hardware->command_interfaces().front()->set_value(0.9));
  loop.read(time, period);
  loop.update(time, period);
  slot.publish(&released);
  loop.write(time, period);
  EXPECT_LE(std::abs(rig.observed.last.load()), STEP);
  slot.publish(nullptr);
  loop.read(time, period);
}

// It writes once more within the period of the cycle's write, after the next update
TEST(CommandLimitsRuntime, the_write_through_a_block_adds_no_step)
{
  LimitedRuntime rig;
  ASSERT_EQ(rig.write(), IoResult::Ok);
  ASSERT_TRUE(rig.hardware->command_interfaces().front()->set_value(0.9));
  rig.hardware->rt_note_claims({{nullptr, 1}});
  auto & owner = rig.hardware->owner();
  ASSERT_TRUE(owner.wait(
    owner.submit([&] { rig.hardware->write_through_block(); }),
    Clock::now() + std::chrono::seconds(5)));
  EXPECT_EQ(rig.observed.writes.load(), 2);
  EXPECT_NEAR(rig.observed.last.load(), VELOCITY * PERIOD, 1e-9);
  const auto period = rclcpp::Duration::from_seconds(PERIOD);
  ASSERT_EQ(
    rig.hardware->rt_write(rclcpp::Time(rig.now_ns, RCL_STEADY_TIME) + period, period),
    IoResult::Ok);
  EXPECT_NEAR(rig.observed.last.load(), 2 * VELOCITY * PERIOD, 1e-9)
    << "the write of the update still takes its step";
}

// RT limits the copy for an asynchronous write, with the claims it recorded
TEST(CommandLimitsRuntime, an_asynchronous_write_is_limited_with_the_claims_rt_recorded)
{
  LimitedRuntime rig(true);
  const rclcpp::Time time(0, 0, RCL_STEADY_TIME);
  const auto period = rclcpp::Duration::from_seconds(PERIOD);
  // The owner reads after it writes, so the first cycle holds until the position is known
  const auto trigger = [&](double command, const std::atomic<int> & count, int at_least)
  {
    return testing::eventually(
      [&]
      {
        EXPECT_TRUE(rig.hardware->command_interfaces().front()->set_value(command));
        rig.hardware->rt_trigger(time, period);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        return count >= at_least;
      });
  };
  ASSERT_TRUE(trigger(0.9, rig.observed.writes, 1));
  EXPECT_LE(std::abs(rig.observed.first.load()), STEP);
  rig.hardware->rt_note_claims({{}});
  ASSERT_TRUE(trigger(0.7, rig.observed.nans, rig.observed.nans + 1))
    << "an unclaimed command reaches the hardware as NaN, it applied " << rig.observed.last.load();
  EXPECT_NE(rig.observed.last.load(), 0.7);
}

// gz writes on every physics step of 1 ms but updates only at the control rate of 50 Hz
TEST(CommandLimitsRuntime, several_writes_per_update_move_at_most_one_update_period)
{
  LimitedRuntime rig(false, 50);
  ASSERT_EQ(rig.write(0.001, 20), IoResult::Ok);
  const double start = rig.observed.last.load();
  ASSERT_EQ(rig.write(0.001, 20), IoResult::Ok);
  EXPECT_NEAR(rig.observed.last.load() - start, VELOCITY * 0.02, 1e-9)
    << "one update period at the velocity limit, not more and not less";
}

TEST(CommandLimitsRuntime, an_overrun_moves_no_further_than_one_update_period)
{
  LimitedRuntime rig;
  ASSERT_EQ(rig.write(0.5), IoResult::Ok);
  EXPECT_LE(std::abs(rig.observed.first.load()), STEP);
}

TEST(CommandLimitsRuntime, a_busy_command_skips_the_write_and_adds_no_step)
{
  LimitedRuntime rig;
  ASSERT_EQ(rig.write(), IoResult::Ok);
  const double written = rig.observed.last.load();
  const uint64_t skipped = rig.hardware->stats().skipped();
  {
    const HeldLock busy(rig.hardware->command_interfaces().front()->get_mutex());
    EXPECT_EQ(
      rig.hardware->rt_write(
        rclcpp::Time(rig.now_ns, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(PERIOD)),
      IoResult::Skipped);
  }
  EXPECT_EQ(rig.hardware->stats().skipped(), skipped + 1);
  EXPECT_EQ(rig.observed.writes.load(), 1);
  ASSERT_EQ(rig.write(), IoResult::Ok);
  EXPECT_NEAR(rig.observed.last.load() - written, VELOCITY * PERIOD, 1e-9);
}

/// What the writes of three joints applied.
struct Applied
{
  std::vector<std::array<double, 3>> all()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return writes;
  }

  testing::Gate between_joints;
  std::mutex mutex;
  std::vector<std::array<double, 3>> writes;
};

/// Reads the command of j1, waits at the gate and then reads j2 and j3, like a bus that transfers
/// one joint after the other.
class ThreeJointSystem : public hardware_interface::SystemInterface
{
public:
  explicit ThreeJointSystem(Applied & applied) : applied_(applied) {}

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (const char * name : {"j1/position", "j2/position", "j3/position"})
    {
      set_state(name, 0.0);
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    std::array<double, 3> values{};
    values[0] = get_command<double>("j1/position");
    applied_.between_joints.pass();
    values[1] = get_command<double>("j2/position");
    values[2] = get_command<double>("j3/position");
    std::lock_guard<std::mutex> lock(applied_.mutex);
    applied_.writes.push_back(values);
    return return_type::OK;
  }

private:
  Applied & applied_;
};

/// An asynchronous component of three joints that one controller claims and RT updates every
/// millisecond. The controller commands a jump on j2, which has a velocity limit, and the number
/// of its update on j1 and j3.
struct ThreeJointRig
{
  ThreeJointRig()
  {
    auto info = testing::hardware_info("system", true);
    info.rw_rate = 1000;
    info.joints.clear();
    for (const char * name : {"j1", "j2", "j3"})
    {
      hardware_interface::ComponentInfo joint;
      joint.name = name;
      joint.type = "joint";
      hardware_interface::InterfaceInfo position{};
      position.name = "position";
      position.enable_limits = true;
      joint.command_interfaces.push_back(position);
      joint.state_interfaces.push_back(position);
      info.joints.push_back(joint);
    }
    info.limits["j2"].has_velocity_limits = true;
    info.limits["j2"].max_velocity = VELOCITY;
    hardware = std::make_unique<runtime::HardwareRuntime>(
      info, [](const std::string &, std::vector<std::string>) {}, std::chrono::seconds(1), true);
    EXPECT_TRUE(
      testing::bring_up(
        *hardware,
        [this]
        {
          auto created = std::make_unique<ThreeJointSystem>(applied);
          plugin = created.get();
          return created;
        }));
    plan.generation = 1;
    plan.hardware.push_back({hardware.get(), true, {}, {{nullptr, 1}, {nullptr, 1}, {nullptr, 1}}});
    slot.publish(&plan);
  }

  ~ThreeJointRig()
  {
    applied.between_joints.open();
    slot.publish(nullptr);
  }

  void cycle()
  {
    ++update;
    const rclcpp::Time time(static_cast<int64_t>(update) * 1000000, RCL_STEADY_TIME);
    const auto period = rclcpp::Duration::from_seconds(PERIOD);
    loop.read(time, period);
    loop.update(time, period);
    const auto & commands = hardware->command_interfaces();
    EXPECT_TRUE(commands[0]->set_value(static_cast<double>(update)));
    EXPECT_TRUE(commands[1]->set_value(0.9));
    EXPECT_TRUE(commands[2]->set_value(static_cast<double>(update)));
    loop.write(time, period);
  }

  /// Cycles every millisecond until the owner applied `count` more writes.
  bool cycle_until_written(size_t count)
  {
    const size_t target = applied.all().size() + count;
    return eventually(
      [&]
      {
        cycle();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return applied.all().size() >= target;
      });
  }

  /// Lets the owner serve what RT triggered last. It polls before each job, and the poll before
  /// the second job runs after the first was queued, so after the trigger RT ran on this thread.
  void idle()
  {
    auto & owner = hardware->owner();
    owner.submit([] {});
    EXPECT_TRUE(owner.wait(owner.submit([] {}), Clock::now() + std::chrono::seconds(5)));
  }

  /// Every write applied j1 and j3 of one update and moved j2 by at most one step.
  void expect_one_step_of_one_update()
  {
    const auto writes = applied.all();
    size_t steps = 0;
    for (size_t i = 0; i < writes.size(); ++i)
    {
      EXPECT_EQ(writes[i][0], writes[i][2]) << "write " << i << " mixes two updates";
      if (i > 0 && !std::isnan(writes[i - 1][1]) && !std::isnan(writes[i][1]))
      {
        ++steps;
        EXPECT_LE(std::abs(writes[i][1] - writes[i - 1][1]), STEP) << "write " << i;
      }
    }
    EXPECT_GT(steps, 0u) << "precondition: j2 was written twice in a row";
  }

  Applied applied;
  runtime::Doorbell bell;
  runtime::PlanSlot slot{bell};
  runtime::RtLoop loop{slot, bell, std::chrono::seconds(1)};
  std::unique_ptr<runtime::HardwareRuntime> hardware;
  /// Owned by hardware, read between cycles while the owner is idle.
  ThreeJointSystem * plugin = nullptr;
  runtime::Plan plan;
  uint64_t update = 0;
};

// The owner's write overlaps two updates, which change none of the commands it applies
TEST(CommandLimitsRuntime, updates_during_an_asynchronous_write_neither_tear_nor_skip_the_limits)
{
  ThreeJointRig rig;
  ASSERT_TRUE(rig.cycle_until_written(5));
  rig.applied.between_joints.close();
  ASSERT_TRUE(eventually(
    [&]
    {
      rig.cycle();
      return rig.applied.between_joints.holds(std::chrono::seconds(0));
    }));
  rig.cycle();
  rig.cycle();
  rig.applied.between_joints.open();
  ASSERT_TRUE(rig.cycle_until_written(5));
  rig.expect_one_step_of_one_update();
}

// The owner serves a trigger late, RT copies again only after its write
TEST(CommandLimitsRuntime, an_owner_that_serves_late_writes_the_first_copy)
{
  ThreeJointRig rig;
  ASSERT_TRUE(rig.cycle_until_written(5));
  rig.idle();
  testing::Gate busy;
  busy.close();
  auto & owner = rig.hardware->owner();
  auto job = owner.submit([&] { busy.pass(); });
  ASSERT_TRUE(busy.holds());
  const size_t before = rig.applied.all().size();
  const double copied = static_cast<double>(rig.update + 1);
  for (int i = 0; i < 5; ++i)
  {
    rig.cycle();
  }
  busy.open();
  ASSERT_TRUE(owner.wait(job, Clock::now() + std::chrono::seconds(5)));
  ASSERT_TRUE(eventually([&] { return rig.applied.all().size() > before; }));
  EXPECT_EQ(rig.applied.all()[before][0], copied) << "RT copied again before the owner wrote";
  ASSERT_TRUE(rig.cycle_until_written(5));
  rig.expect_one_step_of_one_update();
  EXPECT_GE(rig.hardware->stats().skipped(), 4u) << "precondition: RT found the owner busy";
}

// The owner skipped the write of a copy, because a block came after it. The next copy limits
// from the last command that reached the hardware.
TEST(CommandLimitsRuntime, a_copy_the_owner_did_not_write_adds_no_step)
{
  ThreeJointRig rig;
  ASSERT_TRUE(rig.cycle_until_written(5));
  rig.idle();
  testing::Gate busy;
  busy.close();
  auto & owner = rig.hardware->owner();
  auto job = owner.submit([&] { busy.pass(); });
  ASSERT_TRUE(busy.holds());
  const size_t before = rig.applied.all().size();
  rig.cycle();
  rig.hardware->block_writes();
  busy.open();
  ASSERT_TRUE(owner.wait(job, Clock::now() + std::chrono::seconds(5)));
  rig.idle();
  ASSERT_EQ(rig.applied.all().size(), before) << "precondition: the owner skipped the write";
  rig.hardware->unblock_writes();
  ASSERT_TRUE(rig.cycle_until_written(3));
  rig.expect_one_step_of_one_update();
}

// A trigger that lands after the owner polled and before a mode switch is served after the
// perform. Its copy holds commands of the old modes, so the cycle only reads.
TEST(CommandLimitsRuntime, a_copy_pending_across_a_mode_switch_is_not_written)
{
  ThreeJointRig rig;
  ASSERT_TRUE(rig.cycle_until_written(5));
  rig.idle();
  testing::Gate busy;
  busy.close();
  auto & owner = rig.hardware->owner();
  std::string failure = "not run";
  auto job = owner.submit(
    [&]
    {
      busy.pass();
      failure = rig.hardware->switch_modes({"j1/position"}, {});
    });
  ASSERT_TRUE(busy.holds());
  const size_t before = rig.applied.all().size();
  rig.cycle();
  busy.open();
  ASSERT_TRUE(owner.wait(job, Clock::now() + std::chrono::seconds(5)));
  EXPECT_EQ(failure, "");
  rig.idle();
  EXPECT_EQ(rig.applied.all().size(), before) << "the copy from before the switch was written";
  ASSERT_TRUE(rig.cycle_until_written(3));
  rig.expect_one_step_of_one_update();
}

// The plugin, or a thread of it, holds one of its command handles while RT copies
TEST(CommandLimitsRuntime, a_busy_plugin_handle_skips_the_whole_copy)
{
  ThreeJointRig rig;
  const std::array<std::string, 3> names = {"j1/position", "j2/position", "j3/position"};
  for (const auto & busy_name : names)
  {
    ASSERT_TRUE(rig.cycle_until_written(5));
    rig.idle();
    std::array<double, 3> before{};
    for (size_t i = 0; i < names.size(); ++i)
    {
      before[i] = rig.plugin->get_command<double>(names[i]);
    }
    const uint64_t skipped = rig.hardware->stats().skipped();
    {
      const HeldLock busy(rig.plugin->get_command_interface_handle(busy_name)->get_mutex());
      rig.cycle();
      rig.idle();
    }
    EXPECT_EQ(rig.hardware->stats().skipped(), skipped + 1);
    for (size_t i = 0; i < names.size(); ++i)
    {
      EXPECT_EQ(rig.plugin->get_command<double>(names[i]), before[i])
        << names[i] << " changed while " << busy_name << " was busy";
    }
  }
}

TEST(CommandLimitsRuntime, rt_copies_nothing_into_a_blocked_component)
{
  ThreeJointRig rig;
  ASSERT_TRUE(rig.cycle_until_written(5));
  rig.idle();
  const double before = rig.plugin->get_command<double>("j1/position");
  rig.hardware->block_writes();
  for (int i = 0; i < 5; ++i)
  {
    rig.cycle();
    rig.idle();
  }
  EXPECT_EQ(rig.plugin->get_command<double>("j1/position"), before);
  rig.hardware->unblock_writes();
  rig.cycle();
  rig.idle();
  EXPECT_NE(rig.plugin->get_command<double>("j1/position"), before)
    << "precondition: an unblocked cycle copies";
}

// The controller writes its goal once, RT limits only the copy for the plugin
TEST(CommandLimitsRuntime, a_goal_written_once_stays_in_the_controller_handle_and_is_reached)
{
  LimitedRuntime rig;
  auto & handle = *rig.hardware->command_interfaces().front();
  ASSERT_TRUE(handle.set_value(0.9));
  int changed = 0;
  for (int i = 0; i < 600; ++i)
  {
    rig.hardware->rt_note_claims({{nullptr, 1}});
    rig.hardware->rt_read(
      rclcpp::Time(rig.now_ns, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(PERIOD));
    rig.now_ns += rclcpp::Duration::from_seconds(PERIOD).nanoseconds();
    rig.hardware->rt_write(
      rclcpp::Time(rig.now_ns, RCL_STEADY_TIME), rclcpp::Duration::from_seconds(PERIOD));
    changed += handle.get_optional() == 0.9 ? 0 : 1;
  }
  EXPECT_EQ(changed, 0);
  EXPECT_DOUBLE_EQ(rig.observed.last.load(), 0.9);
}

}  // namespace
}  // namespace hector_controller_manager
