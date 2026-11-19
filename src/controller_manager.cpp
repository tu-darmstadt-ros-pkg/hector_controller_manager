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

#include "hector_controller_manager/controller_manager.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>

#include "hardware_interface/introspection.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hector_controller_manager/core/names.hpp"
#include "hector_controller_manager/entities.hpp"
#include "hector_controller_manager/runtime/lifecycle_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rcl/arguments.h"

namespace hector_controller_manager
{
namespace
{
using core::Lifecycle;
using StateMsg = lifecycle_msgs::msg::State;
namespace names = hardware_interface::lifecycle_state_names;

constexpr std::chrono::seconds BARRIER_TIMEOUT{1};
constexpr std::chrono::milliseconds RETRY_PAUSE{100};
constexpr std::chrono::milliseconds STATISTICS_PERIOD{50};

/// The registry controller_interface registers the controllers' introspection in, published on
/// `/ros2_control/introspection_data` by a node of its own, `/ros2_control`, which the first
/// manager creates and which lives as long as the process, so that no manager's end leaves the
/// registry without its publisher. Never destroyed, as its publishers would go after the
/// middleware at exit.
class Introspection
{
public:
  static Introspection & instance()
  {
    static Introspection * const introspection = new Introspection();
    return *introspection;
  }

  /// Once per statistics period, whichever managers ask, as their timers tick.
  void publish()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    // A tick a little early still publishes, so that one manager's ticks all do
    if (now + STATISTICS_PERIOD / 2 < due_)
    {
      return;
    }
    due_ = std::max(due_, now) + STATISTICS_PERIOD;
    registry_->publishAsync();
  }

private:
  Introspection()
  : node_(std::make_shared<rclcpp::Node>(
      hardware_interface::DEFAULT_REGISTRY_KEY,
      rclcpp::NodeOptions().use_global_arguments(false).start_parameter_services(false))),
    registry_(pal_statistics::getOrCreateRegistry(
      node_, hardware_interface::DEFAULT_INTROSPECTION_TOPIC,
      hardware_interface::DEFAULT_REGISTRY_KEY))
  {
    registry_->startPublishThread();
    // Its node and publish thread live to the end, which static destructors must not precede
    record_leak();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<pal_statistics::StatisticsRegistry> registry_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point due_;
};

lifecycle_msgs::msg::State state_message(Lifecycle state, bool quarantined)
{
  lifecycle_msgs::msg::State message;
  if (quarantined)
  {
    // A callback still runs, the entity is in the transition it started
    switch (state)
    {
      case Lifecycle::Active:
        message.id = StateMsg::TRANSITION_STATE_DEACTIVATING;
        message.label = "deactivating";
        return message;
      case Lifecycle::Inactive:
        message.id = StateMsg::TRANSITION_STATE_ACTIVATING;
        message.label = "activating";
        return message;
      case Lifecycle::Unconfigured:
      case Lifecycle::Finalized:
        message.id = StateMsg::TRANSITION_STATE_CONFIGURING;
        message.label = "configuring";
        return message;
    }
  }
  const auto primary = runtime::primary_state(state);
  message.id = primary.id();
  message.label = primary.label();
  return message;
}

std::optional<Lifecycle> lifecycle_of(const lifecycle_msgs::msg::State & state)
{
  if (state.id == StateMsg::PRIMARY_STATE_ACTIVE || state.label == names::ACTIVE)
  {
    return Lifecycle::Active;
  }
  if (state.id == StateMsg::PRIMARY_STATE_INACTIVE || state.label == names::INACTIVE)
  {
    return Lifecycle::Inactive;
  }
  return std::nullopt;
}

template <typename T>
T parameter(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name))
  {
    return node.declare_parameter<T>(name, default_value);
  }
  return node.get_parameter(name).get_value<T>();
}

std::vector<std::string> string_list(rclcpp::Node & node, const std::string & name)
{
  return parameter<std::vector<std::string>>(node, name, {});
}

int64_t at_least(
  rclcpp::Node & node, const std::string & name, int64_t default_value, int64_t minimum)
{
  const int64_t value = parameter<int64_t>(node, name, default_value);
  if (value < minimum)
  {
    throw std::runtime_error(
      name + " must be at least " + std::to_string(minimum) + ", it is " + std::to_string(value));
  }
  return value;
}

std::chrono::milliseconds duration_ms(
  rclcpp::Node & node, const std::string & name, Clock::duration default_value, int64_t minimum = 1)
{
  const auto value = std::chrono::duration_cast<std::chrono::milliseconds>(default_value);
  return std::chrono::milliseconds(at_least(node, name, value.count(), minimum));
}

controller_manager_msgs::msg::HardwareInterface interface_entry(
  const std::string & name, bool available, bool claimed)
{
  controller_manager_msgs::msg::HardwareInterface entry;
  entry.name = name;
  entry.data_type = "double";
  entry.is_available = available;
  entry.is_claimed = claimed;
  return entry;
}

template <typename ServiceResponse>
void answer(const rclcpp::Logger & logger, const Response & result, ServiceResponse & response)
{
  RCLCPP_ERROR_EXPRESSION(logger, !result.ok, "%s", result.message.c_str());
  response.ok = result.ok;
}

bool is_claimed(const Status & status, const std::string & interface)
{
  for (const auto & controller : status.controllers)
  {
    const auto & claimed = controller.model.claims.command;
    if (
      controller.model.state == Lifecycle::Active &&
      std::find(claimed.begin(), claimed.end(), interface) != claimed.end())
    {
      return true;
    }
  }
  return false;
}
}  // namespace

rclcpp::NodeOptions get_cm_node_options()
{
  rclcpp::NodeOptions options;
  // Controller types arrive as parameters set by the spawner
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.enable_logger_service(true);
  return options;
}

ControllerManager::ControllerManager(
  std::shared_ptr<rclcpp::Executor> executor, const std::string & name,
  const std::string & node_namespace, const rclcpp::NodeOptions & options,
  runtime::HardwareFactory hardware_factory)
: rclcpp::Node(name, node_namespace, options),
  executor_(std::move(executor)),
  hardware_factory_(std::move(hardware_factory)),
  loader_("controller_interface", "controller_interface::ControllerInterface"),
  chainable_loader_("controller_interface", "controller_interface::ChainableControllerInterface"),
  system_loader_("hardware_interface", "hardware_interface::SystemInterface"),
  actuator_loader_("hardware_interface", "hardware_interface::ActuatorInterface"),
  sensor_loader_("hardware_interface", "hardware_interface::SensorInterface"),
  diagnostics_(this)
{
  if (!executor_)
  {
    executor_ = own_executor();
  }
  else if (has_parameter("executor_threads"))
  {
    RCLCPP_WARN(get_logger(), "executor_threads is ignored, the caller provides the executor");
  }
  EStopTopics estop_topics({}, {});
  const SupervisorConfig config = read_parameters(estop_topics);
  activity_ = create_publisher<controller_manager_msgs::msg::ControllerManagerActivity>(
    "~/activity", rclcpp::QoS(1).reliable().transient_local());
  // A registry per manager, keyed by its topic, so that managers in one process stay apart
  statistics_key_ =
    get_node_topics_interface()->resolve_topic_name(hardware_interface::CM_STATISTICS_TOPIC);
  // One left by a manager of this name before holds that manager's publisher and clock
  pal_statistics::deleteRegistry(statistics_key_);
  statistics_registry_ = pal_statistics::getOrCreateRegistry(
    this, hardware_interface::CM_STATISTICS_TOPIC, statistics_key_);
  statistics_registry_->startPublishThread();
  // Before any controller registers its introspection
  Introspection::instance();
  supervisor_ = std::make_unique<Supervisor>(
    config, *this, get_logger(),
    [this](std::shared_ptr<const Status> status) { on_status(status); });
  register_statistics(*supervisor_->status());
  rcl_interfaces::msg::ParameterDescriptor capabilities;
  capabilities.read_only = true;
  // A parameter file for every node may set it, which must neither stop the manager nor change it
  if (has_parameter("capabilities"))
  {
    RCLCPP_WARN(get_logger(), "capabilities is read only, its override is ignored");
    undeclare_parameter("capabilities");
  }
  // hector_controller_orchestrator sends FORCE_AUTO instead of planning a STRICT switch itself
  declare_parameter("capabilities", std::vector<std::string>{"force_auto"}, capabilities, true);
  estop_ = std::make_unique<EStopMonitor>(
    *this, std::move(estop_topics), [this](bool engaged, const std::string & cause, bool changed)
    { on_estop(engaged, cause, changed); });
  // Sampled off RT from atomics, so the statistics never race with or block the control loop
  statistics_timer_ = create_wall_timer(
    STATISTICS_PERIOD,
    [this]
    {
      statistics_registry_->publishAsync();
      Introspection::instance().publish();
    });
  diagnostics_.setHardwareID("ros2_control");
  diagnostics_.add(
    "Controller Manager Activity",
    [this](diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
      const auto status = supervisor_->status();
      if (status->initialized)
      {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
      }
      else if (initialization_failed_)
      {
        stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "Initialization failed, the node needs a restart");
      }
      else
      {
        stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for the robot description");
      }
      if (supervisor_->rt().estop())
      {
        stat.mergeSummary(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "E-stop engaged by " + supervisor_->estop_cause());
      }
      stat.add("cycle_time_s", supervisor_->rt().cycle_seconds());
      for (const auto & controller : status->controllers)
      {
        stat.add(controller.name, to_string(controller.model.state));
      }
      for (const auto & hardware : status->hardware)
      {
        stat.add(hardware.name, to_string(hardware.model.state));
        stat.add(hardware.name + ".skipped_cycles", hardware.runtime->skipped_cycles());
      }
    });
  list_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  barrier_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Request callbacks only start a thread, the supervisor serializes the requests
  request_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  robot_description_semantic_ = parameter<std::string>(*this, "robot_description_semantic", "");
  if (robot_description_semantic_.empty())
  {
    semantic_subscription_ = create_subscription<std_msgs::msg::String>(
      "robot_description_semantic", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String & message)
      {
        // The first wins, so every controller gets the same one
        std::lock_guard<std::mutex> lock(semantic_mutex_);
        robot_description_semantic_ = message.data;
        semantic_subscription_.reset();
      });
  }
  const std::string description = parameter<std::string>(*this, "robot_description", "");
  if (!description.empty())
  {
    // Only once the executor spins, so no hardware factory runs before its owner holds this object
    startup_timer_ = create_wall_timer(
      std::chrono::nanoseconds(0),
      [this, description]
      {
        startup_timer_->cancel();
        start_initialize(description);
      });
  }
  else
  {
    robot_description_subscription_ = create_subscription<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String & message)
      {
        // The first wins, a changing description is not supported
        robot_description_subscription_.reset();
        start_initialize(message.data);
      });
  }
  estop_->start();
  // After the context ends no controller can transition. Last, so a throw above leaves no callback.
  pre_shutdown_ =
    get_node_base_interface()->get_context()->add_pre_shutdown_callback([this] { shutdown(); });
}

ControllerManager::~ControllerManager()
{
  // Waits for a callback that already runs
  get_node_base_interface()->get_context()->remove_pre_shutdown_callback(pre_shutdown_);
  if (startup_.joinable())
  {
    startup_.join();
  }
  shutdown();
  // After shutdown the supervisor rejects every queued request at once, so these end soon
  std::vector<RequestThread> requests;
  {
    std::lock_guard<std::mutex> lock(request_threads_mutex_);
    requests.swap(request_threads_);
  }
  for (auto & request : requests)
  {
    request.thread.join();
  }
  statistics_timer_.reset();
  {
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    for (const auto & name : registered_statistics_)
    {
      statistics_registry_->unregisterVariable(name);
    }
  }
  supervisor_.reset();
}

std::shared_ptr<rclcpp::Executor> ControllerManager::own_executor()
{
  // It serves only services, topics and plugin nodes, one thread per core would idle
  const int64_t threads = at_least(*this, "executor_threads", 2, 1);
  return std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), static_cast<size_t>(threads));
}

SupervisorConfig ControllerManager::read_parameters(EStopTopics & estop_topics)
{
  use_sim_time_ = get_parameter("use_sim_time").as_bool();
  trigger_clock_ = use_sim_time_ ? get_clock() : std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  if (has_parameter("enforce_command_limits") && get_parameter("enforce_command_limits").as_bool())
  {
    throw std::runtime_error(
      "enforce_command_limits is not supported by hector_controller_manager, joint limits belong "
      "into the controllers");
  }
  // Before the supervisor starts, so a rejected value leaves nothing running
  SupervisorConfig config;
  config.update_rate =
    static_cast<unsigned int>(at_least(*this, "update_rate", config.update_rate, 1));
  config.hang_timeout = duration_ms(*this, "hardware_hang_timeout_ms", config.hang_timeout);
  config.default_timeout = duration_ms(*this, "default_timeout_ms", config.default_timeout);
  config.hardware_state_timeout =
    duration_ms(*this, "hardware_state_timeout_ms", config.hardware_state_timeout);
  config.hardware_fault_timeout =
    duration_ms(*this, "hardware_fault_timeout_ms", config.hardware_fault_timeout);
  const auto grace = duration_ms(*this, "estop_publisher_grace_ms", std::chrono::seconds(1));
  const auto deadline = duration_ms(*this, "estop_deadline_ms", Clock::duration::zero(), 0);
  if (deadline.count() != 0 && deadline.count() < 100)
  {
    throw std::runtime_error("estop_deadline_ms must be 0 (off) or at least 100");
  }
  estop_topics = EStopTopics(string_list(*this, "estop_topics"), grace, deadline);
  autostart_cooldown_ = duration_ms(*this, "autostart_cooldown_ms", std::chrono::seconds(2), 0);
  autostart_attempts_ = at_least(*this, "autostart_attempts", 3, 1);
  return config;
}

bool ControllerManager::is_initialized() const { return supervisor_->status()->initialized; }

void ControllerManager::start_initialize(const std::string & robot_description)
{
  startup_ = std::thread([this, robot_description] { initialize(robot_description); });
}

void ControllerManager::initialize(const std::string & robot_description)
{
  std::map<std::string, Lifecycle> initial;
  for (const auto & name : string_list(*this, "hardware_components_initial_state.unconfigured"))
  {
    initial[name] = Lifecycle::Unconfigured;
  }
  for (const auto & name : string_list(*this, "hardware_components_initial_state.inactive"))
  {
    initial[name] = Lifecycle::Inactive;
  }
  for (const auto & name : string_list(*this, "startup.hardware_active"))
  {
    initial[name] = Lifecycle::Active;
  }
  robot_description_ = robot_description;
  startup_hardware_ = initial;
  const Response response = supervisor_->initialize(robot_description, initial);
  if (!response.ok && !is_initialized())
  {
    RCLCPP_ERROR(
      get_logger(), "Initialization failed, the node needs a restart: %s",
      response.message.c_str());
    initialization_failed_ = true;
    return;
  }
  if (!response.ok)
  {
    RCLCPP_ERROR(get_logger(), "Bring up incomplete: %s", response.message.c_str());
  }
  // As upstream, so clients wait for the services instead of being refused
  init_services();
  apply_startup();
}

void ControllerManager::apply_startup()
{
  for (const auto & name : string_list(*this, "startup.controllers_loaded"))
  {
    const Response loaded = load_controller(name);
    const Response configured = loaded.ok ? supervisor_->configure(name) : loaded;
    if (!configured.ok)
    {
      RCLCPP_ERROR(
        get_logger(), "Startup: '%s' failed: %s", name.c_str(), configured.message.c_str());
    }
  }
  startup_switch(false);
}

void ControllerManager::startup_switch(bool restore)
{
  const auto status = supervisor_->status();
  core::Model model;
  for (const auto & hardware : status->hardware)
  {
    model.hardware.emplace(hardware.name, hardware.model);
  }
  for (const auto & controller : status->controllers)
  {
    model.controllers.emplace(controller.name, controller.model);
  }
  const auto selection = core::select_startup(
    model, string_list(*this, "startup.controllers_active"), restore,
    estop_->engaged() ? std::optional<std::string>(supervisor_->estop_cause()) : std::nullopt);
  for (const auto & entry : selection.skipped)
  {
    RCLCPP_WARN(get_logger(), "Startup skips %s", entry.c_str());
  }
  if (selection.activate.empty())
  {
    return;
  }
  const core::SwitchRequest request{selection.activate, {}, core::Strictness::Auto};
  const Response response =
    supervisor_->switch_controllers(request, supervisor_->config().default_timeout);
  if (response.ok)
  {
    RCLCPP_INFO(get_logger(), "Startup activated %s", core::text(request.activate).c_str());
  }
  else
  {
    // The whole startup set is rejected, the process keeps running
    RCLCPP_ERROR(get_logger(), "Startup switch rejected: %s", response.message.c_str());
  }
}

void ControllerManager::autostart(uint64_t release)
{
  // Initialization runs the startup itself once the description arrived
  if (!is_initialized())
  {
    return;
  }
  // False once the e-stop engaged, the controller manager stops or a newer release won
  const auto pause = [&](std::chrono::milliseconds time)
  {
    std::unique_lock<std::mutex> lock(autostart_mutex_);
    if (!estop_changed_.wait_for(
          lock, time, [&] { return estop_->engaged() || stopping_ || releases_ != release; }))
    {
      return true;
    }
    RCLCPP_WARN(get_logger(), "Autostart aborted, the e-stop engaged or a newer release won");
    return false;
  };
  for (int64_t attempt = 1;; ++attempt)
  {
    // Hardware that lost power needs a moment to boot
    if (!pause(autostart_cooldown_))
    {
      return;
    }
    std::string failures;
    const auto status = supervisor_->status();
    for (const auto & hardware : status->hardware)
    {
      const auto target = startup_hardware_.find(hardware.name);
      if (
        core::commandable(hardware.model) ||
        (target != startup_hardware_.end() && target->second != Lifecycle::Active))
      {
        continue;
      }
      // A request that came first is no failed attempt, ask again within the state timeout
      const auto give_up = Clock::now() + supervisor_->config().hardware_state_timeout;
      Response response = supervisor_->set_hardware_state(
        hardware.name, Lifecycle::Active, supervisor_->config().hardware_state_timeout);
      RCLCPP_INFO_EXPRESSION(
        get_logger(), !response.ok && response.try_again, "Autostart asks again for '%s': %s",
        hardware.name.c_str(), response.message.c_str());
      while (!response.ok && response.try_again && Clock::now() < give_up)
      {
        if (!pause(RETRY_PAUSE))
        {
          return;
        }
        response = supervisor_->set_hardware_state(
          hardware.name, Lifecycle::Active, supervisor_->config().hardware_state_timeout);
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
    if (attempt == autostart_attempts_)
    {
      RCLCPP_ERROR(get_logger(), "Autostart gave up on the hardware:%s", failures.c_str());
      return;
    }
    RCLCPP_WARN(get_logger(), "Autostart attempt %ld failed:%s", attempt, failures.c_str());
  }
  if (!pause(std::chrono::milliseconds::zero()))
  {
    return;
  }
  startup_switch(true);
}

bool ControllerManager::shutdown()
{
  if (!supervisor_)
  {
    return true;
  }
  {
    std::lock_guard<std::mutex> lock(autostart_mutex_);
    stopping_ = true;
    estop_changed_.notify_all();
  }
  const Response response = supervisor_->shutdown(supervisor_->config().default_timeout);
  // Only now, a busy or incomplete shutdown may leave controllers running
  estop_->stop();
  if (!response.ok)
  {
    RCLCPP_ERROR(get_logger(), "Shutdown incomplete: %s", response.message.c_str());
    stopped_cleanly_ = false;
  }
  return stopped_cleanly_;
}

void ControllerManager::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  supervisor_->rt().read(time, period);
}

void ControllerManager::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  supervisor_->rt().update(time, period);
}

void ControllerManager::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  supervisor_->rt().write(time, period);
}

Response ControllerManager::load_controller(const std::string & name)
{
  // A controller loaded earlier would get an empty robot description
  if (!is_initialized())
  {
    return {
      false, initialization_failed_ ? "Initialization failed, the node needs a restart"
                                    : "Not initialized yet, waiting for the robot description"};
  }
  const std::string type = parameter<std::string>(*this, name + ".type", "");
  if (
    has_parameter(name + ".fallback_controllers") &&
    !get_parameter(name + ".fallback_controllers").as_string_array().empty())
  {
    return {
      false, "Fallback controllers are not supported, remove '" + name + ".fallback_controllers'"};
  }
  return supervisor_->load(name, type, parameter<bool>(*this, name + ".protected", false));
}

rclcpp::NodeOptions ControllerManager::controller_node_options(const std::string & name)
{
  std::vector<std::string> arguments = {RCL_ROS_ARGS_FLAG};
  if (has_parameter(name + ".params_file"))
  {
    const auto files = get_parameter(name + ".params_file");
    const auto list = files.get_type() == rclcpp::ParameterType::PARAMETER_STRING
                        ? std::vector<std::string>{files.as_string()}
                        : files.as_string_array();
    for (const auto & file : list)
    {
      arguments.push_back(RCL_PARAM_FILE_FLAG);
      arguments.push_back(file);
    }
  }
  if (use_sim_time_)
  {
    arguments.push_back(RCL_PARAM_FLAG);
    arguments.push_back("use_sim_time:=true");
  }
  for (const auto & argument : string_list(*this, name + ".node_options_args"))
  {
    if (argument != RCL_ROS_ARGS_FLAG)
    {
      arguments.push_back(argument);
    }
  }
  rclcpp::NodeOptions options;
  options.arguments(arguments);
  options.use_global_arguments(false);
  return options;
}

runtime::ControllerRuntime::Plugin ControllerManager::create_controller(
  const std::string & name, const std::string & type)
{
  runtime::ControllerRuntime::Plugin plugin;
  std::string semantic;
  {
    std::lock_guard<std::mutex> lock(semantic_mutex_);
    semantic = robot_description_semantic_;
  }
  {
    std::lock_guard<std::mutex> lock(loader_mutex_);
    if (chainable_loader_.isClassAvailable(type))
    {
      plugin = chainable_loader_.createSharedInstance(type);
    }
    else if (loader_.isClassAvailable(type))
    {
      plugin = loader_.createSharedInstance(type);
    }
  }
  if (!plugin)
  {
    throw std::runtime_error("Controller type '" + type + "' is not available");
  }
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = name;
  params.robot_description = robot_description_;
  params.update_rate = get_update_rate();
  params.controller_manager_update_rate = get_update_rate();
  params.node_namespace = get_namespace();
  auto options = plugin->define_custom_node_options();
  const auto own = controller_node_options(name);
  auto arguments = options.arguments();
  arguments.insert(arguments.end(), own.arguments().begin(), own.arguments().end());
  params.node_options = options.arguments(arguments).use_global_arguments(false);
  // An override, not an argument, so the XML is not parsed as one. The controller declares it.
  if (!semantic.empty())
  {
    params.node_options.append_parameter_override("robot_description_semantic", semantic);
  }
  if (plugin->init(params) != controller_interface::return_type::OK)
  {
    throw std::runtime_error("on_init failed");
  }
  executor_->add_node(plugin->get_node()->get_node_base_interface());
  return plugin;
}

bool ControllerManager::forget_controller(const runtime::ControllerRuntime::Plugin & plugin)
{
  executor_->remove_node(plugin->get_node()->get_node_base_interface());
  if (!executor_->is_spinning())
  {
    return true;
  }
  // The node may be destroyed only after the executor rebuilt its entities without it, which a
  // timer created after the removal proves. Each call has its own flag, so another call's timer
  // confirms nothing.
  auto fired = std::make_shared<std::atomic<bool>>(false);
  const auto deadline = Clock::now() + BARRIER_TIMEOUT;
  auto barrier = create_wall_timer(
    std::chrono::nanoseconds(0),
    [this, fired]
    {
      fired->store(true);
      barrier_bell_.ring();
    },
    barrier_group_);
  // The bell is read before the flag, so a ring between the two is not missed
  for (uint32_t rung = barrier_bell_.value();
       !fired->load() && barrier_bell_.wait_until(rung, deadline); rung = barrier_bell_.value())
  {
  }
  const bool confirmed = fired->load();
  barrier->cancel();
  {
    // An executor thread may hold the last reference otherwise, and destroy the timer while another
    // barrier's creation frees what is left of it
    std::lock_guard<std::mutex> lock(barriers_mutex_);
    barriers_.push_back(std::move(barrier));
  }
  RCLCPP_ERROR_EXPRESSION(
    get_logger(), !confirmed,
    "The executor did not confirm the removal of a controller node, its plugin is abandoned");
  return confirmed;
}

std::unique_ptr<hardware_interface::HardwareComponentInterface> ControllerManager::create_hardware(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_factory_)
  {
    return hardware_factory_(info);
  }
  std::lock_guard<std::mutex> lock(loader_mutex_);
  const auto & plugin = info.hardware_plugin_name;
  if (info.type == "system")
  {
    return std::unique_ptr<hardware_interface::HardwareComponentInterface>(
      system_loader_.createUnmanagedInstance(plugin));
  }
  if (info.type == "actuator")
  {
    return std::unique_ptr<hardware_interface::HardwareComponentInterface>(
      actuator_loader_.createUnmanagedInstance(plugin));
  }
  if (info.type == "sensor")
  {
    return std::unique_ptr<hardware_interface::HardwareComponentInterface>(
      sensor_loader_.createUnmanagedInstance(plugin));
  }
  throw std::runtime_error("Unknown hardware type '" + info.type + "'");
}

hardware_interface::HardwareComponentParams ControllerManager::hardware_params(
  const hardware_interface::HardwareInfo & info)
{
  hardware_interface::HardwareComponentParams params;
  params.hardware_info = info;
  params.clock = trigger_clock_;
  params.logger = get_logger().get_child("hardware_component." + info.type + "." + info.name);
  params.executor = executor_;
  params.node_namespace = get_namespace();
  return params;
}

void ControllerManager::on_status(std::shared_ptr<const Status> status)
{
  controller_manager_msgs::msg::ControllerManagerActivity message;
  for (const auto & controller : status->controllers)
  {
    controller_manager_msgs::msg::NamedLifecycleState named;
    named.name = controller.name;
    named.state = state_message(controller.model.state, controller.model.quarantined);
    message.controllers.push_back(named);
  }
  for (const auto & hardware : status->hardware)
  {
    controller_manager_msgs::msg::NamedLifecycleState named;
    named.name = hardware.name;
    named.state = state_message(hardware.model.state, hardware.model.quarantined);
    message.hardware_components.push_back(named);
  }
  register_statistics(*status);
  std::lock_guard<std::mutex> lock(statistics_mutex_);
  if (message == last_activity_)
  {
    return;
  }
  last_activity_ = message;
  message.header.stamp = now();
  activity_->publish(message);
}

void ControllerManager::register_statistics(const Status & status)
{
  std::lock_guard<std::mutex> lock(statistics_mutex_);
  // Microseconds like upstream ros2_control, which the rqml statistics panel expects
  const auto add = [&](const std::string & name, std::function<double()> seconds)
  {
    if (registered_statistics_.insert(name).second)
    {
      statistics_registry_->registerFunction(
        name, [seconds = std::move(seconds)] { return seconds() * 1e6; });
    }
  };
  // The runtime may be unloaded before a sample
  const auto sampled = [](auto runtime, auto seconds) -> std::function<double()>
  {
    return [weak = std::weak_ptr(runtime), seconds]
    {
      const auto locked = weak.lock();
      return locked ? ((*locked).*seconds)() : 0.0;
    };
  };
  add(
    std::string(get_name()) + ".total_time", [this] { return supervisor_->rt().cycle_seconds(); });
  for (const auto & hardware : status.hardware)
  {
    add(
      hardware.name + ".stats/read_cycle/execution_time/current_value",
      sampled(hardware.runtime, &runtime::HardwareRuntime::read_seconds));
    add(
      hardware.name + ".stats/write_cycle/execution_time/current_value",
      sampled(hardware.runtime, &runtime::HardwareRuntime::write_seconds));
  }
  for (const auto & controller : status.controllers)
  {
    add(
      controller.name + ".stats/execution_time/current_value",
      sampled(controller.runtime, &runtime::ControllerRuntime::update_seconds));
  }
}

Clock::duration ControllerManager::timeout_of(
  const builtin_interfaces::msg::Duration & timeout) const
{
  const auto requested =
    std::chrono::seconds(timeout.sec) + std::chrono::nanoseconds(timeout.nanosec);
  return requested > Clock::duration::zero() ? Clock::duration(requested)
                                             : supervisor_->config().default_timeout;
}

template <typename ServiceT>
rclcpp::ServiceBase::SharedPtr ControllerManager::create_request_service(
  const std::string & name,
  void (ControllerManager::*callback)(
    const typename ServiceT::Request::SharedPtr, typename ServiceT::Response::SharedPtr))
{
  return create_service<ServiceT>(
    name,
    [this, name, callback](
      std::shared_ptr<rclcpp::Service<ServiceT>> service, std::shared_ptr<rmw_request_id_t> header,
      std::shared_ptr<typename ServiceT::Request> request)
    {
      run_request(
        [this, name, callback, service, header, request]
        {
          auto response = std::make_shared<typename ServiceT::Response>();
          (this->*callback)(request, response);
          try
          {
            service->send_response(*header, *response);
          }
          catch (const std::exception & e)
          {
            RCLCPP_ERROR(get_logger(), "Could not answer %s: %s", name.c_str(), e.what());
          }
        });
    },
    rclcpp::ServicesQoS(), request_group_);
}

void ControllerManager::run_request(std::function<void()> work)
{
  std::lock_guard<std::mutex> lock(request_threads_mutex_);
  std::erase_if(
    request_threads_,
    [](RequestThread & request)
    {
      if (!request.done->load())
      {
        return false;
      }
      request.thread.join();
      return true;
    });
  auto done = std::make_shared<std::atomic<bool>>(false);
  request_threads_.push_back(
    {std::thread(
       [work = std::move(work), done]
       {
         work();
         done->store(true);
       }),
     done});
}

void ControllerManager::on_estop(bool engaged, const std::string & cause, bool changed)
{
  supervisor_->set_estop(engaged, cause);
  if (!changed)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(autostart_mutex_);
  estop_changed_.notify_all();
  if (engaged)
  {
    RCLCPP_WARN(get_logger(), "E-stop engaged by %s", cause.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "E-stop released, autostart follows");
  run_request([this, release = ++releases_] { autostart(release); });
}

void ControllerManager::init_services()
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const auto qos = rclcpp::ServicesQoS();
  services_ = {
    create_service<ListControllers>(
      "~/list_controllers", std::bind(&ControllerManager::list_controllers_srv_cb, this, _1, _2),
      qos, list_group_),
    create_service<ListControllerTypes>(
      "~/list_controller_types",
      std::bind(&ControllerManager::list_controller_types_srv_cb, this, _1, _2), qos, list_group_),
    create_service<ListHardwareComponents>(
      "~/list_hardware_components",
      std::bind(&ControllerManager::list_hardware_components_srv_cb, this, _1, _2), qos,
      list_group_),
    create_service<ListHardwareInterfaces>(
      "~/list_hardware_interfaces",
      std::bind(&ControllerManager::list_hardware_interfaces_srv_cb, this, _1, _2), qos,
      list_group_),
    create_request_service<LoadController>(
      "~/load_controller", &ControllerManager::load_controller_service_cb),
    create_request_service<ConfigureController>(
      "~/configure_controller", &ControllerManager::configure_controller_service_cb),
    create_request_service<CleanupController>(
      "~/cleanup_controller", &ControllerManager::cleanup_controller_service_cb),
    create_request_service<UnloadController>(
      "~/unload_controller", &ControllerManager::unload_controller_service_cb),
    create_request_service<SwitchController>(
      "~/switch_controller", &ControllerManager::switch_controller_service_cb),
    create_request_service<ReloadControllerLibraries>(
      "~/reload_controller_libraries", &ControllerManager::reload_controller_libraries_service_cb),
    create_request_service<SetHardwareComponentState>(
      "~/set_hardware_component_state", &ControllerManager::set_hardware_component_state_srv_cb),
  };
}

std::vector<std::string> ControllerManager::active_controllers() const
{
  std::vector<std::string> active;
  for (const auto & controller : supervisor_->status()->controllers)
  {
    if (controller.model.state == Lifecycle::Active)
    {
      active.push_back(controller.name);
    }
  }
  return active;
}

void ControllerManager::list_controllers_srv_cb(
  const ListControllers::Request::SharedPtr, ListControllers::Response::SharedPtr response)
{
  const auto status = supervisor_->status();
  std::map<std::string, std::string> reference_owner;
  for (const auto & controller : status->controllers)
  {
    for (const auto & reference : controller.model.references)
    {
      reference_owner.emplace(reference, controller.name);
    }
  }
  for (const auto & controller : status->controllers)
  {
    const auto & model = controller.model;
    controller_manager_msgs::msg::ControllerState state;
    state.name = controller.name;
    state.type = controller.type;
    state.state = state_message(model.state, model.quarantined).label;
    state.is_async = false;
    state.update_rate = static_cast<uint16_t>(get_update_rate());
    state.claimed_interfaces = model.claims.command;
    state.required_command_interfaces = model.interfaces.command;
    state.required_state_interfaces = model.interfaces.state;
    state.is_chainable = model.chainable;
    state.is_chained = model.chained;
    state.reference_interfaces = model.references;
    state.exported_state_interfaces = model.exported_states;
    std::map<std::string, std::vector<std::string>> connections;
    for (const auto & interface : model.interfaces.command)
    {
      if (const auto it = reference_owner.find(interface); it != reference_owner.end())
      {
        connections[it->second].push_back(interface);
      }
    }
    for (auto & [name, references] : connections)
    {
      controller_manager_msgs::msg::ChainConnection connection;
      connection.name = name;
      connection.reference_interfaces = references;
      state.chain_connections.push_back(connection);
    }
    response->controller.push_back(state);
  }
}

void ControllerManager::list_controller_types_srv_cb(
  const ListControllerTypes::Request::SharedPtr, ListControllerTypes::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(loader_mutex_);
  for (const auto & type : loader_.getDeclaredClasses())
  {
    response->types.push_back(type);
    response->base_classes.push_back("controller_interface::ControllerInterface");
  }
  for (const auto & type : chainable_loader_.getDeclaredClasses())
  {
    response->types.push_back(type);
    response->base_classes.push_back("controller_interface::ChainableControllerInterface");
  }
}

void ControllerManager::list_hardware_components_srv_cb(
  const ListHardwareComponents::Request::SharedPtr,
  ListHardwareComponents::Response::SharedPtr response)
{
  const auto status = supervisor_->status();
  for (const auto & hardware : status->hardware)
  {
    controller_manager_msgs::msg::HardwareComponentState component;
    component.name = hardware.name;
    component.type = hardware.info.type;
    component.is_async = hardware.info.is_async;
    component.rw_rate = static_cast<uint16_t>(get_update_rate());
    component.plugin_name = hardware.info.hardware_plugin_name;
    component.state = state_message(hardware.model.state, hardware.model.quarantined);
    const bool active = hardware.model.state == Lifecycle::Active;
    const bool readable = active || hardware.model.state == Lifecycle::Inactive;
    for (const auto & interface : hardware.model.command_interfaces)
    {
      component.command_interfaces.push_back(
        interface_entry(interface.name, active, is_claimed(*status, interface.name)));
    }
    for (const auto & interface : hardware.model.state_interfaces)
    {
      component.state_interfaces.push_back(interface_entry(interface, readable, false));
    }
    response->component.push_back(component);
  }
}

void ControllerManager::list_hardware_interfaces_srv_cb(
  const ListHardwareInterfaces::Request::SharedPtr,
  ListHardwareInterfaces::Response::SharedPtr response)
{
  const auto status = supervisor_->status();
  auto & commands = response->command_interfaces;
  auto & states = response->state_interfaces;
  for (const auto & hardware : status->hardware)
  {
    const bool active = hardware.model.state == Lifecycle::Active;
    const bool readable = active || hardware.model.state == Lifecycle::Inactive;
    for (const auto & interface : hardware.model.command_interfaces)
    {
      commands.push_back(
        interface_entry(interface.name, active, is_claimed(*status, interface.name)));
    }
    for (const auto & interface : hardware.model.state_interfaces)
    {
      states.push_back(interface_entry(interface, readable, false));
    }
  }
  for (const auto & controller : status->controllers)
  {
    const bool active = controller.model.state == Lifecycle::Active;
    for (const auto & reference : controller.model.references)
    {
      commands.push_back(interface_entry(reference, active, is_claimed(*status, reference)));
    }
    for (const auto & exported : controller.model.exported_states)
    {
      states.push_back(interface_entry(exported, active, false));
    }
  }
}

void ControllerManager::load_controller_service_cb(
  const LoadController::Request::SharedPtr request, LoadController::Response::SharedPtr response)
{
  answer(get_logger(), load_controller(request->name), *response);
}

void ControllerManager::configure_controller_service_cb(
  const ConfigureController::Request::SharedPtr request,
  ConfigureController::Response::SharedPtr response)
{
  answer(get_logger(), supervisor_->configure(request->name), *response);
}

void ControllerManager::cleanup_controller_service_cb(
  const CleanupController::Request::SharedPtr request,
  CleanupController::Response::SharedPtr response)
{
  answer(get_logger(), supervisor_->cleanup(request->name), *response);
}

void ControllerManager::unload_controller_service_cb(
  const UnloadController::Request::SharedPtr request,
  UnloadController::Response::SharedPtr response)
{
  answer(get_logger(), supervisor_->unload(request->name), *response);
}

void ControllerManager::switch_controller_service_cb(
  const SwitchController::Request::SharedPtr request,
  SwitchController::Response::SharedPtr response)
{
  core::SwitchRequest switch_request;
  switch_request.activate =
    core::Names(request->activate_controllers.begin(), request->activate_controllers.end());
  switch_request.deactivate =
    core::Names(request->deactivate_controllers.begin(), request->deactivate_controllers.end());
  switch (request->strictness)
  {
    case SwitchController::Request::AUTO:
      switch_request.strictness = core::Strictness::Auto;
      break;
    case SwitchController::Request::FORCE_AUTO:
      switch_request.strictness = core::Strictness::ForceAuto;
      break;
    case SwitchController::Request::BEST_EFFORT:
      RCLCPP_WARN(get_logger(), "BEST_EFFORT is not supported, the switch runs as STRICT");
      switch_request.strictness = core::Strictness::Strict;
      break;
    default:
      switch_request.strictness = core::Strictness::Strict;
      break;
  }
  const Response result =
    supervisor_->switch_controllers(switch_request, timeout_of(request->timeout));
  // A warning, the supervisor logs the faults behind a rolled back switch as errors
  RCLCPP_WARN_EXPRESSION(get_logger(), !result.ok, "Switch rejected: %s", result.message.c_str());
  response->ok = result.ok;
  response->message = result.message;
}

void ControllerManager::reload_controller_libraries_service_cb(
  const ReloadControllerLibraries::Request::SharedPtr,
  ReloadControllerLibraries::Response::SharedPtr response)
{
  RCLCPP_ERROR(get_logger(), "reload_controller_libraries is not supported");
  response->ok = false;
}

void ControllerManager::set_hardware_component_state_srv_cb(
  const SetHardwareComponentState::Request::SharedPtr request,
  SetHardwareComponentState::Response::SharedPtr response)
{
  const auto target = lifecycle_of(request->target_state);
  bool was_up = false;
  // The snapshot must outlive the loop, a temporary would die before its body runs
  const auto before = supervisor_->status();
  for (const auto & hardware : before->hardware)
  {
    if (hardware.name == request->name)
    {
      was_up = core::commandable(hardware.model);
    }
  }
  Response result{false, "Only the target states active and inactive are supported"};
  if (target)
  {
    result = supervisor_->set_hardware_state(
      request->name, *target, supervisor_->config().hardware_state_timeout);
  }
  answer(get_logger(), result, *response);
  // An idempotent activation must not restart what the operator stopped
  if (result.ok && *target == Lifecycle::Active && !was_up)
  {
    startup_switch(true);
  }
  const auto status = supervisor_->status();
  for (const auto & hardware : status->hardware)
  {
    if (hardware.name == request->name)
    {
      response->state = state_message(hardware.model.state, hardware.model.quarantined);
    }
  }
}

}  // namespace hector_controller_manager
