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

#include <array>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include "hector_controller_manager/core/names.hpp"
#include "hector_controller_manager/parameters.hpp"
#include "hector_controller_manager/runtime/thrown.hpp"
#include "hector_controller_manager/status_messages.hpp"
#include "joint_limits/joint_limits_rosparam.hpp"
#include "rcl/arguments.h"

namespace hector_controller_manager
{
namespace
{
using core::Lifecycle;

constexpr std::chrono::seconds BARRIER_TIMEOUT{1};
constexpr std::array STARTUP_PARAMETERS = {
  "startup.hardware_active", "startup.controllers_loaded", "startup.controllers_active",
  "hardware_components_initial_state.unconfigured", "hardware_components_initial_state.inactive"};

template <typename T>
T parameter(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name))
  {
    return node.declare_parameter<T>(name, default_value);
  }
  return node.get_parameter(name).get_value<T>();
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

/// One core or a list of them, as upstream. Empty without the parameter.
std::vector<int> cores(rclcpp::Node & node, const std::string & name)
{
  if (!node.has_parameter(name))
  {
    return {};
  }
  const auto parameter = node.get_parameter(name);
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    return {static_cast<int>(parameter.as_int())};
  }
  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
  {
    throw std::runtime_error(
      name + " must be a core or a list of cores, it is " + parameter.value_to_string());
  }
  const auto list = parameter.as_integer_array();
  return std::vector<int>(list.begin(), list.end());
}

template <typename ServiceResponse>
void answer(const rclcpp::Logger & logger, const Response & result, ServiceResponse & response)
{
  RCLCPP_ERROR_EXPRESSION(logger, !result.ok, "%s", result.message.c_str());
  response.ok = result.ok;
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
  execution_times_(
    *this, std::string(get_name()) + ".total_time",
    [this] { return supervisor_->rt().cycle_seconds(); })
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
  Startup::Settings startup{};
  const SupervisorConfig config = read_parameters(estop_topics, startup);
  activity_ = create_publisher<controller_manager_msgs::msg::ControllerManagerActivity>(
    "~/activity", rclcpp::QoS(1).reliable().transient_local());
  // Before the supervisor, whose control loop samples it
  introspection_ = Introspection::join(*this);
  supervisor_ = std::make_unique<Supervisor>(
    config, *this, get_logger(),
    [this](std::shared_ptr<const Status> status) { on_status(status); });
  execution_times_.add(*supervisor_->status());
  startup_ = std::make_unique<Startup>(
    *this, *supervisor_, requests_, startup, [this] { return estop_->engaged(); },
    [this](const std::string & name) { return load_controller(name); });
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
  statistics_timer_ =
    create_wall_timer(std::chrono::milliseconds(50), [this] { execution_times_.publish(); });
  diagnostics_ = std::make_unique<Diagnostics>(
    *this, *supervisor_, callbacks_mutex_, closed_, initialization_failed_);
  list_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  removal_barrier_ = std::make_unique<ExecutorBarrier>(*this);
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
  {
    // An embedding executor may still spin, its callbacks must leave the supervisor alone now
    const std::unique_lock lock(callbacks_mutex_);
    closed_ = true;
  }
  if (startup_thread_.joinable())
  {
    startup_thread_.join();
  }
  shutdown();
  // After shutdown the supervisor rejects every queued request at once, so these end soon
  requests_.join_all();
  statistics_timer_.reset();
  execution_times_.remove_all();
  supervisor_.reset();
  // The plugins are gone, so another manager's RT may sample again
  introspection_.reset();
}

std::shared_ptr<rclcpp::Executor> ControllerManager::own_executor()
{
  // It serves only services, topics and plugin nodes, one thread per core would idle
  const int64_t threads = at_least(*this, "executor_threads", 2, 1);
  return std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), static_cast<size_t>(threads));
}

SupervisorConfig ControllerManager::read_parameters(
  EStopTopics & estop_topics, Startup::Settings & startup)
{
  use_sim_time_ = get_parameter("use_sim_time").as_bool();
  trigger_clock_ = use_sim_time_ ? get_clock() : std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  // Before the supervisor starts, so a rejected value leaves nothing running
  SupervisorConfig config;
  config.enforce_command_limits =
    parameter<bool>(*this, "enforce_command_limits", config.enforce_command_limits);
  for (const auto & name :
       parameter<std::vector<std::string>>(*this, "command_limits_excluded_hardware", {}))
  {
    config.command_limits_excluded.insert(name);
  }
  constexpr std::string_view LIMITS_PREFIX = "joint_limits.";
  for (const auto & name : list_parameters({"joint_limits"}, 0).names)
  {
    const auto end = name.find('.', LIMITS_PREFIX.size());
    const std::string joint = name.substr(LIMITS_PREFIX.size(), end - LIMITS_PREFIX.size());
    if (end != std::string::npos && !config.limit_parameters.count(joint))
    {
      joint_limits::get_joint_limits(
        joint, get_node_parameters_interface(), get_node_logging_interface(),
        config.limit_parameters[joint]);
    }
  }
  const int64_t update_rate = at_least(*this, "update_rate", config.update_rate, 1);
  // The list services report it as uint16
  if (update_rate > std::numeric_limits<uint16_t>::max())
  {
    throw std::runtime_error(
      "update_rate must be at most 65535, it is " + std::to_string(update_rate));
  }
  config.update_rate = static_cast<unsigned int>(update_rate);
  config.thread_priority =
    static_cast<int>(parameter<int64_t>(*this, "thread_priority", config.thread_priority));
  config.lock_memory = parameter<bool>(*this, "lock_memory", config.scheduler.realtime_kernel());
  config.cpu_affinity = cores(*this, "cpu_affinity");
  config.hang_timeout = duration_ms(*this, "hardware_hang_timeout_ms", config.hang_timeout);
  config.default_timeout = duration_ms(*this, "default_timeout_ms", config.default_timeout);
  config.hardware_state_timeout =
    duration_ms(*this, "hardware_state_timeout_ms", config.hardware_state_timeout);
  config.hardware_fault_timeout =
    duration_ms(*this, "hardware_fault_timeout_ms", config.hardware_fault_timeout);
  config.mode_switch_timeout =
    duration_ms(*this, "mode_switch_timeout_ms", config.mode_switch_timeout);
  const auto grace = duration_ms(*this, "estop_publisher_grace_ms", std::chrono::seconds(1));
  const auto deadline = duration_ms(*this, "estop_deadline_ms", Clock::duration::zero(), 0);
  if (deadline.count() != 0 && deadline < EStopMonitor::CHECK_PERIOD)
  {
    throw std::runtime_error(
      "estop_deadline_ms must be 0 (off) or at least " +
      std::to_string(EStopMonitor::CHECK_PERIOD.count()));
  }
  estop_topics =
    EStopTopics(parameter<std::vector<std::string>>(*this, "estop_topics", {}), grace, deadline);
  startup.cooldown = duration_ms(*this, "autostart_cooldown_ms", std::chrono::seconds(2), 0);
  startup.attempts = at_least(*this, "autostart_attempts", 3, 1);
  // Read again at startup and after a release, declared here so a wrong type stops the node now
  for (const char * name : STARTUP_PARAMETERS)
  {
    parameter<std::vector<std::string>>(*this, name, {});
  }
  return config;
}

bool ControllerManager::is_initialized() const { return supervisor_->status()->initialized; }

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

void ControllerManager::start_initialize(const std::string & robot_description)
{
  const std::shared_lock lock(callbacks_mutex_);
  if (closed_)
  {
    return;
  }
  // A second description can arrive before the subscription is gone, a second thread would abort
  if (startup_thread_.joinable())
  {
    RCLCPP_WARN(get_logger(), "A further robot description is ignored, the first one is used");
    return;
  }
  startup_thread_ = std::thread(
    [this, robot_description]
    {
      if (const auto error = runtime::thrown_by([&] { initialize(robot_description); }))
      {
        RCLCPP_ERROR(get_logger(), "Startup failed: %s", error->c_str());
        initialization_failed_ = !is_initialized();
      }
    });
}

void ControllerManager::initialize(const std::string & robot_description)
{
  const auto initial = startup_->read_hardware_targets();
  robot_description_ = robot_description;
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
  startup_->apply();
}

bool ControllerManager::shutdown()
{
  startup_->stop();
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
  std::string type;
  bool is_protected = false;
  std::vector<std::string> fallback;
  try
  {
    type = get_parameter_or<std::string>(name + ".type", "");
    is_protected = get_parameter_or(name + ".protected", false);
    fallback = string_list(*this, name + ".fallback_controllers");
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException & e)
  {
    return {false, "Controller '" + name + "' cannot be loaded, " + e.what()};
  }
  if (!fallback.empty())
  {
    return {
      false, "Fallback controllers are not supported, remove '" + name + ".fallback_controllers'"};
  }
  return supervisor_->load(name, type, is_protected);
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
  std::string semantic;
  {
    std::lock_guard<std::mutex> lock(semantic_mutex_);
    semantic = robot_description_semantic_;
  }
  const auto plugin = plugins_.create_controller(type);
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
  // The node may be destroyed only after the executor rebuilt its entities without it
  const bool confirmed = removal_barrier_->pass(Clock::now() + BARRIER_TIMEOUT);
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
  return plugins_.create_hardware(info);
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
  auto message = activity_message(*status);
  execution_times_.add(*status);
  std::lock_guard<std::mutex> lock(activity_mutex_);
  if (message == last_activity_)
  {
    return;
  }
  last_activity_ = message;
  message.header.stamp = now();
  activity_->publish(message);
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
      const std::shared_lock lock(callbacks_mutex_);
      if (closed_)
      {
        return;
      }
      requests_.run(
        [this, name, callback, service, header, request]
        {
          auto response = std::make_shared<typename ServiceT::Response>();
          if (const auto error = runtime::thrown_by([&] { (this->*callback)(request, response); }))
          {
            RCLCPP_ERROR(get_logger(), "%s failed: %s", name.c_str(), error->c_str());
            response->ok = false;
            if constexpr (requires { response->message; })
            {
              response->message = *error;
            }
          }
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

void ControllerManager::on_estop(bool engaged, const std::string & cause, bool changed)
{
  supervisor_->set_estop(engaged, cause);
  if (changed)
  {
    startup_->on_change(engaged, cause);
  }
}

template <typename ServiceT>
rclcpp::ServiceBase::SharedPtr ControllerManager::create_list_service(
  const std::string & name,
  void (ControllerManager::*callback)(
    const typename ServiceT::Request::SharedPtr, typename ServiceT::Response::SharedPtr))
{
  return create_service<ServiceT>(
    name,
    [this, callback](
      const typename ServiceT::Request::SharedPtr request,
      typename ServiceT::Response::SharedPtr response)
    {
      const std::shared_lock lock(callbacks_mutex_);
      if (!closed_)
      {
        (this->*callback)(request, response);
      }
    },
    rclcpp::ServicesQoS(), list_group_);
}

void ControllerManager::init_services()
{
  services_ = {
    create_list_service<ListControllers>(
      "~/list_controllers", &ControllerManager::list_controllers_srv_cb),
    create_list_service<ListControllerTypes>(
      "~/list_controller_types", &ControllerManager::list_controller_types_srv_cb),
    create_list_service<ListHardwareComponents>(
      "~/list_hardware_components", &ControllerManager::list_hardware_components_srv_cb),
    create_list_service<ListHardwareInterfaces>(
      "~/list_hardware_interfaces", &ControllerManager::list_hardware_interfaces_srv_cb),
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

void ControllerManager::list_controllers_srv_cb(
  const ListControllers::Request::SharedPtr, ListControllers::Response::SharedPtr response)
{
  response->controller =
    controller_states(*supervisor_->status(), static_cast<uint16_t>(get_update_rate()));
}

void ControllerManager::list_controller_types_srv_cb(
  const ListControllerTypes::Request::SharedPtr, ListControllerTypes::Response::SharedPtr response)
{
  for (auto & [type, base_class] : plugins_.controller_types())
  {
    response->types.push_back(std::move(type));
    response->base_classes.push_back(std::move(base_class));
  }
}

void ControllerManager::list_hardware_components_srv_cb(
  const ListHardwareComponents::Request::SharedPtr,
  ListHardwareComponents::Response::SharedPtr response)
{
  response->component =
    hardware_component_states(*supervisor_->status(), static_cast<uint16_t>(get_update_rate()));
}

void ControllerManager::list_hardware_interfaces_srv_cb(
  const ListHardwareInterfaces::Request::SharedPtr,
  ListHardwareInterfaces::Response::SharedPtr response)
{
  hardware_interfaces(
    *supervisor_->status(), response->command_interfaces, response->state_interfaces);
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
    case 0:
    case SwitchController::Request::BEST_EFFORT:
      RCLCPP_WARN(
        get_logger(), "BEST_EFFORT and strictness 0 are not supported, the switch runs as STRICT");
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
  // The snapshot must outlive the pointer into it
  const auto before = supervisor_->status();
  const auto * known = find_hardware(*before, request->name);
  const bool was_up = known && core::commandable(known->model);
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
    startup_->restore();
  }
  const auto status = supervisor_->status();
  if (const auto * hardware = find_hardware(*status, request->name))
  {
    response->state = state_message(*hardware);
  }
}

}  // namespace hector_controller_manager
