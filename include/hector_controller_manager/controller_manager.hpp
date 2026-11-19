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

#ifndef HECTOR_CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
#define HECTOR_CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "controller_manager_msgs/srv/cleanup_controller.hpp"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/list_controller_types.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "controller_manager_msgs/srv/list_hardware_interfaces.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/reload_controller_libraries.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"
#include "hector_controller_manager/diagnostics.hpp"
#include "hector_controller_manager/estop_monitor.hpp"
#include "hector_controller_manager/execution_time_registry.hpp"
#include "hector_controller_manager/executor_barrier.hpp"
#include "hector_controller_manager/plugin_loader.hpp"
#include "hector_controller_manager/request_threads.hpp"
#include "hector_controller_manager/startup.hpp"
#include "hector_controller_manager/supervisor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace hector_controller_manager
{
rclcpp::NodeOptions get_cm_node_options();

/// The ROS shell with node, parameters, services, topics and plugin loading. Every request goes to
/// the supervisor, the list services and topics read its status snapshot.
class ControllerManager : public rclcpp::Node, public PluginHost
{
public:
  /// `hardware_factory` replaces pluginlib for hardware, gz_ros2_control uses it. Without an
  /// `executor` it creates a MultiThreadedExecutor with the parameter `executor_threads` threads
  /// (default 2). With one the parameter is ignored, which it logs.
  ControllerManager(
    std::shared_ptr<rclcpp::Executor> executor, const std::string & name = "controller_manager",
    const std::string & node_namespace = "",
    const rclcpp::NodeOptions & options = get_cm_node_options(),
    runtime::HardwareFactory hardware_factory = {});
  ~ControllerManager() override;

  /// The control cycle, on the RT thread.
  void read(const rclcpp::Time & time, const rclcpp::Duration & period);
  void update(const rclcpp::Time & time, const rclcpp::Duration & period);
  void write(const rclcpp::Time & time, const rclcpp::Duration & period);

  unsigned int get_update_rate() const { return supervisor_->config().update_rate; }
  /// Call on the control loop thread before its first cycle, as ros2_control_node does, so it runs
  /// with SCHED_FIFO at `thread_priority` on a realtime kernel and the diagnostics report it.
  void schedule_control_loop() { supervisor_->schedule_control_loop(); }
  const std::shared_ptr<rclcpp::Executor> & executor() const { return executor_; }
  /// The services may still be in creation when this turns true.
  bool is_initialized() const;
  /// Brings everything down as on process exit. Returns false if something did not stop cleanly,
  /// also in an earlier call.
  bool shutdown();

  /// Service callbacks, public so tests read exactly what the services return.
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using ListHardwareComponents = controller_manager_msgs::srv::ListHardwareComponents;
  using ListHardwareInterfaces = controller_manager_msgs::srv::ListHardwareInterfaces;
  using ListControllerTypes = controller_manager_msgs::srv::ListControllerTypes;
  using LoadController = controller_manager_msgs::srv::LoadController;
  using ConfigureController = controller_manager_msgs::srv::ConfigureController;
  using CleanupController = controller_manager_msgs::srv::CleanupController;
  using UnloadController = controller_manager_msgs::srv::UnloadController;
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using ReloadControllerLibraries = controller_manager_msgs::srv::ReloadControllerLibraries;
  using SetHardwareComponentState = controller_manager_msgs::srv::SetHardwareComponentState;

  void list_controllers_srv_cb(
    const ListControllers::Request::SharedPtr request,
    ListControllers::Response::SharedPtr response);
  void list_hardware_components_srv_cb(
    const ListHardwareComponents::Request::SharedPtr request,
    ListHardwareComponents::Response::SharedPtr response);
  void list_hardware_interfaces_srv_cb(
    const ListHardwareInterfaces::Request::SharedPtr request,
    ListHardwareInterfaces::Response::SharedPtr response);
  void list_controller_types_srv_cb(
    const ListControllerTypes::Request::SharedPtr request,
    ListControllerTypes::Response::SharedPtr response);
  void load_controller_service_cb(
    const LoadController::Request::SharedPtr request, LoadController::Response::SharedPtr response);
  void configure_controller_service_cb(
    const ConfigureController::Request::SharedPtr request,
    ConfigureController::Response::SharedPtr response);
  void cleanup_controller_service_cb(
    const CleanupController::Request::SharedPtr request,
    CleanupController::Response::SharedPtr response);
  void unload_controller_service_cb(
    const UnloadController::Request::SharedPtr request,
    UnloadController::Response::SharedPtr response);
  void switch_controller_service_cb(
    const SwitchController::Request::SharedPtr request,
    SwitchController::Response::SharedPtr response);
  void reload_controller_libraries_service_cb(
    const ReloadControllerLibraries::Request::SharedPtr request,
    ReloadControllerLibraries::Response::SharedPtr response);
  void set_hardware_component_state_srv_cb(
    const SetHardwareComponentState::Request::SharedPtr request,
    SetHardwareComponentState::Response::SharedPtr response);

  /// Loads a controller by the type in the `<name>.type` parameter, refused until initialized.
  Response load_controller(const std::string & name);

private:
  runtime::ControllerRuntime::Plugin create_controller(
    const std::string & name, const std::string & type) override;
  bool forget_controller(const runtime::ControllerRuntime::Plugin & plugin) override;
  std::unique_ptr<hardware_interface::HardwareComponentInterface> create_hardware(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::HardwareComponentParams hardware_params(
    const hardware_interface::HardwareInfo & info) override;

  std::shared_ptr<rclcpp::Executor> own_executor();
  SupervisorConfig read_parameters(EStopTopics & estop_topics, Startup::Settings & startup);
  /// Starts the one initialization. Activating the startup controllers needs control cycles, so it
  /// holds no executor thread and runs on `startup_thread_`.
  void start_initialize(const std::string & robot_description);
  void initialize(const std::string & robot_description);
  void init_services();
  /// A list service that answers nothing once the destructor began, an embedding executor may still
  /// call it then.
  template <typename ServiceT>
  rclcpp::ServiceBase::SharedPtr create_list_service(
    const std::string & name,
    void (ControllerManager::*callback)(
      const typename ServiceT::Request::SharedPtr, typename ServiceT::Response::SharedPtr));
  /// A request service whose callback hands the request to a thread of its own and returns. The
  /// supervisor may need an executor thread to finish it (the removal barrier of an unload), so no
  /// request may hold one while it waits.
  template <typename ServiceT>
  rclcpp::ServiceBase::SharedPtr create_request_service(
    const std::string & name,
    void (ControllerManager::*callback)(
      const typename ServiceT::Request::SharedPtr, typename ServiceT::Response::SharedPtr));
  /// The EStopMonitor's listener, it hands the e-stop to the supervisor and the startup.
  void on_estop(bool engaged, const std::string & cause, bool changed);
  void on_status(std::shared_ptr<const Status> status);
  rclcpp::NodeOptions controller_node_options(const std::string & name);
  Clock::duration timeout_of(const builtin_interfaces::msg::Duration & timeout) const;

  /// Held shared by the executor callbacks that use the supervisor or start a thread. The
  /// destructor takes it to set `closed_`, after which they return at once. First, so it outlives
  /// every callback.
  std::shared_mutex callbacks_mutex_;
  bool closed_ = false;
  std::shared_ptr<rclcpp::Executor> executor_;
  runtime::HardwareFactory hardware_factory_;
  PluginLoader plugins_;
  ExecutionTimeRegistry execution_times_;
  /// Set once at initialization, read by plugin creation afterwards.
  std::string robot_description_;
  /// From the parameter, else the first on the topic. Forwarded to controllers loaded later.
  std::mutex semantic_mutex_;
  std::string robot_description_semantic_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr semantic_subscription_;
  bool use_sim_time_ = false;
  rclcpp::Clock::SharedPtr trigger_clock_;

  rclcpp::Publisher<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr activity_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
  rclcpp::CallbackGroup::SharedPtr list_group_;
  rclcpp::CallbackGroup::SharedPtr request_group_;
  std::unique_ptr<ExecutorBarrier> removal_barrier_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
  std::mutex activity_mutex_;
  controller_manager_msgs::msg::ControllerManagerActivity last_activity_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::thread startup_thread_;
  /// Written by the startup thread, read by the diagnostics and the load service.
  std::atomic<bool> initialization_failed_{false};
  /// Cleared by an incomplete shutdown, which may run on several threads.
  std::atomic<bool> stopped_cleanly_{true};
  RequestThreads requests_;
  std::unique_ptr<Startup> startup_;
  /// Its thread runs until shutdown.
  std::unique_ptr<EStopMonitor> estop_;
  std::unique_ptr<Diagnostics> diagnostics_;
  /// Shuts the controller manager down while the context is still valid, removed on destruction.
  rclcpp::PreShutdownCallbackHandle pre_shutdown_;

  /// Last, it runs threads that use the members above.
  std::unique_ptr<Supervisor> supervisor_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
