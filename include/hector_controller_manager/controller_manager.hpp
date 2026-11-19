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
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
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
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hector_controller_manager/estop_monitor.hpp"
#include "hector_controller_manager/supervisor.hpp"
#include "pal_statistics/pal_statistics.hpp"
#include "pluginlib/class_loader.hpp"
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
  const std::shared_ptr<rclcpp::Executor> & executor() const { return executor_; }
  bool is_initialized() const;
  /// Brings everything down as on process exit. Returns false if something did not stop cleanly,
  /// also in an earlier call.
  bool shutdown();
  /// The controllers that are active, also those whose call still runs past its deadline, which
  /// the list services report as deactivating.
  std::vector<std::string> active_controllers() const;

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
  SupervisorConfig read_parameters(EStopTopics & estop_topics);
  /// Starts the one initialization. Activating the startup controllers needs control cycles, so it
  /// holds no executor thread and runs on `startup_`.
  void start_initialize(const std::string & robot_description);
  void initialize(const std::string & robot_description);
  void apply_startup();
  void init_services();
  /// A request service whose callback hands the request to a thread of its own and returns. The
  /// supervisor may need an executor thread to finish it (the removal barrier of an unload), so no
  /// request may hold one while it waits.
  template <typename ServiceT>
  rclcpp::ServiceBase::SharedPtr create_request_service(
    const std::string & name,
    void (ControllerManager::*callback)(
      const typename ServiceT::Request::SharedPtr, typename ServiceT::Response::SharedPtr));
  void run_request(std::function<void()> work);
  /// The EStopMonitor's listener, it hands the e-stop to the supervisor and starts the autostart.
  void on_estop(bool engaged, const std::string & cause, bool changed);
  /// The startup set again after a release, see "Autostart on release" in the design.
  void autostart(uint64_t release);
  /// One AUTO switch to the startup controllers that are not active. `restore` skips protected
  /// ones, only an explicit request brings those back after a fault.
  void startup_switch(bool restore);
  void on_status(std::shared_ptr<const Status> status);
  void register_statistics(const Status & status);
  rclcpp::NodeOptions controller_node_options(const std::string & name);
  Clock::duration timeout_of(const builtin_interfaces::msg::Duration & timeout) const;

  std::shared_ptr<rclcpp::Executor> executor_;
  runtime::HardwareFactory hardware_factory_;
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader_;
  pluginlib::ClassLoader<controller_interface::ChainableControllerInterface> chainable_loader_;
  pluginlib::ClassLoader<hardware_interface::SystemInterface> system_loader_;
  pluginlib::ClassLoader<hardware_interface::ActuatorInterface> actuator_loader_;
  pluginlib::ClassLoader<hardware_interface::SensorInterface> sensor_loader_;
  /// Guards the loaders.
  std::mutex loader_mutex_;
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
  rclcpp::CallbackGroup::SharedPtr barrier_group_;
  /// Rung by removal barrier timers on an executor thread.
  runtime::Doorbell barrier_bell_;
  /// Cancelled barrier timers, destroyed with the controller manager after its executor stopped.
  std::mutex barriers_mutex_;
  std::vector<rclcpp::TimerBase::SharedPtr> barriers_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
  diagnostic_updater::Updater diagnostics_;
  std::mutex statistics_mutex_;
  /// Of this manager's statistics registry: the resolved name of its statistics topic. The
  /// registry outlives the manager, as deleting it can leave DDS types behind at exit, until a
  /// later manager of the same name replaces it.
  std::string statistics_key_;
  std::shared_ptr<pal_statistics::StatisticsRegistry> statistics_registry_;
  std::set<std::string> registered_statistics_;
  controller_manager_msgs::msg::ControllerManagerActivity last_activity_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::thread startup_;
  std::atomic<bool> initialization_failed_{false};
  std::atomic<bool> stopped_cleanly_{true};
  struct RequestThread
  {
    std::thread thread;
    std::shared_ptr<std::atomic<bool>> done;
  };
  std::mutex request_threads_mutex_;
  std::vector<RequestThread> request_threads_;
  /// Taken after the EStopMonitor's lock, guards the members below up to the condition variable.
  std::mutex autostart_mutex_;
  /// Counts releases, a newer one supersedes the autostart of an older one.
  uint64_t releases_ = 0;
  bool stopping_ = false;
  std::condition_variable estop_changed_;
  std::chrono::milliseconds autostart_cooldown_;
  int64_t autostart_attempts_;
  /// Target state of every component the startup configuration names, the others go active.
  std::map<std::string, core::Lifecycle> startup_hardware_;
  /// Its thread runs until shutdown.
  std::unique_ptr<EStopMonitor> estop_;
  /// Shuts the controller manager down while the context is still valid, removed on destruction.
  rclcpp::PreShutdownCallbackHandle pre_shutdown_;

  /// Last, it runs threads that use the members above.
  std::unique_ptr<Supervisor> supervisor_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
