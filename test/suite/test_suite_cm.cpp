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
#include <unistd.h>

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "cm_test_suite/suite.hpp"
#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "hector_controller_manager/controller_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"

namespace
{
using cm_test_suite::Lifecycle;
using hector_controller_manager::ControllerManager;
using SwitchRequest = controller_manager_msgs::srv::SwitchController::Request;

/// Primary lifecycle states. Transition states such as deactivating have none.
std::optional<Lifecycle> lifecycle_from_label(const std::string & label)
{
  static const std::map<std::string, Lifecycle> primary = {
    {"unconfigured", Lifecycle::Unconfigured},
    {"inactive", Lifecycle::Inactive},
    {"active", Lifecycle::Active},
    {"finalized", Lifecycle::Finalized}};
  static const std::set<std::string> transitions = {
    "configuring", "cleaningup", "activating", "deactivating", "shuttingdown", "errorprocessing"};
  if (const auto it = primary.find(label); it != primary.end())
  {
    return it->second;
  }
  if (transitions.count(label) == 0)
  {
    throw std::runtime_error("Unknown lifecycle label '" + label + "'");
  }
  return std::nullopt;
}

void add_entry(
  std::map<std::string, Lifecycle> & entries, cm_test_suite::ReportedState & state,
  const std::string & name, const std::string & label)
{
  if (const auto lifecycle = lifecycle_from_label(label))
  {
    entries[name] = *lifecycle;
  }
  else
  {
    state.transitioning.insert(name);
  }
}

int strictness_of(cm_test_suite::Strictness strictness)
{
  switch (strictness)
  {
    case cm_test_suite::Strictness::Strict:
      return SwitchRequest::STRICT;
    case cm_test_suite::Strictness::Auto:
      return SwitchRequest::AUTO;
    case cm_test_suite::Strictness::ForceAuto:
      return SwitchRequest::FORCE_AUTO;
  }
  throw std::logic_error("Unknown strictness");
}

std::vector<std::string> sorted(const std::set<std::string> & names)
{
  return std::vector<std::string>(names.begin(), names.end());
}

#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
// Sanitizers slow every thread about tenfold, like the suite's cycle limit the hang rule scales
constexpr int64_t HANG_TIMEOUT_MS = 1000;
#else
constexpr int64_t HANG_TIMEOUT_MS = 100;
#endif

rclcpp::NodeOptions options_for(const cm_test_suite::CmConfig & config)
{
  std::vector<rclcpp::Parameter> parameters = {
    {"robot_description", config.robot_description},
    {"update_rate", 100},
    {"default_timeout_ms", static_cast<int64_t>(config.switch_timeout.count())},
    {"hardware_hang_timeout_ms", HANG_TIMEOUT_MS},
    {"startup.hardware_active", sorted(config.startup.hardware_active)},
    {"startup.controllers_loaded", sorted(config.startup.controllers_loaded)},
    {"startup.controllers_active", sorted(config.startup.controllers_active)},
  };
  for (const auto & controller : cm_test_suite::rig::controllers())
  {
    parameters.emplace_back(controller.name + ".type", cm_test_suite::rig::plugin_type(controller));
  }
  auto options = hector_controller_manager::get_cm_node_options();
  options.parameter_overrides(parameters);
  return options;
}

/// hector_controller_manager under the common test suite. Every operation goes through the service
/// callbacks, so the suite sees what spawner and ros2 control see.
class HectorCm : public cm_test_suite::CmUnderTest
{
public:
  explicit HectorCm(const cm_test_suite::CmConfig & config)
  : executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()),
    // A namespace of its own per instance and run, so a transient local activity message of the
    // previous test's or a concurrent run's controller manager never reaches this listener
    name_space_("/hector_cm_" + std::to_string(::getpid()) + "_" + std::to_string(instances_++)),
    cm_(
      std::make_shared<ControllerManager>(
        executor_, "cm_under_test", name_space_, options_for(config))),
    listener_(std::make_shared<rclcpp::Node>("cm_under_test_listener", name_space_)),
    time_(0, 0, RCL_STEADY_TIME),
    timeout_(config.switch_timeout)
  {
    activity_subscription_ =
      listener_->create_subscription<controller_manager_msgs::msg::ControllerManagerActivity>(
        name_space_ + "/cm_under_test/activity", rclcpp::QoS(1).reliable().transient_local(),
        [this](const controller_manager_msgs::msg::ControllerManagerActivity & message)
        {
          cm_test_suite::ReportedState state;
          for (const auto & named : message.controllers)
          {
            add_entry(state.controllers, state, named.name, named.state.label);
          }
          for (const auto & named : message.hardware_components)
          {
            add_entry(state.hardware, state, named.name, named.state.label);
          }
          std::lock_guard<std::mutex> lock(activity_mutex_);
          activity_ = std::move(state);
        });
    const char * verbose = std::getenv("CM_FUZZ_VERBOSE");
    if (
      verbose && std::string(verbose) == "steps" &&
      rcutils_logging_set_logger_level(cm_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG) !=
        RCUTILS_RET_OK)
    {
      throw std::runtime_error("Could not enable step logging");
    }
    executor_->add_node(cm_);
    executor_->add_node(listener_);
    spinner_ = std::thread([this] { executor_->spin(); });
  }

  ~HectorCm() override
  {
    cm_->shutdown();
    executor_->cancel();
    spinner_.join();
  }

  void cycle() override
  {
    cm_->read(time_, PERIOD);
    cm_->update(time_, PERIOD);
    cm_->write(time_, PERIOD);
    time_ += PERIOD;
  }

  template <typename Service>
  typename Service::Response::SharedPtr call(
    void (ControllerManager::*callback)(
      const typename Service::Request::SharedPtr, typename Service::Response::SharedPtr),
    typename Service::Request::SharedPtr request)
  {
    auto response = std::make_shared<typename Service::Response>();
    ((*cm_).*callback)(request, response);
    return response;
  }

  template <typename Service>
  bool by_name(
    void (ControllerManager::*callback)(
      const typename Service::Request::SharedPtr, typename Service::Response::SharedPtr),
    const std::string & name)
  {
    auto request = std::make_shared<typename Service::Request>();
    request->name = name;
    return call<Service>(callback, request)->ok;
  }

  bool load(const std::string & controller) override
  {
    return by_name<ControllerManager::LoadController>(
      &ControllerManager::load_controller_service_cb, controller);
  }

  bool configure(const std::string & controller) override
  {
    return by_name<ControllerManager::ConfigureController>(
      &ControllerManager::configure_controller_service_cb, controller);
  }

  bool cleanup(const std::string & controller) override
  {
    return by_name<ControllerManager::CleanupController>(
      &ControllerManager::cleanup_controller_service_cb, controller);
  }

  bool unload(const std::string & controller) override
  {
    return by_name<ControllerManager::UnloadController>(
      &ControllerManager::unload_controller_service_cb, controller);
  }

  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate,
    cm_test_suite::Strictness strictness) override
  {
    auto request = std::make_shared<SwitchRequest>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    request->strictness = strictness_of(strictness);
    request->timeout = rclcpp::Duration(timeout_);
    return call<ControllerManager::SwitchController>(
             &ControllerManager::switch_controller_service_cb, request)
      ->ok;
  }

  bool recover_hardware(const std::string & hardware) override
  {
    auto request = std::make_shared<ControllerManager::SetHardwareComponentState::Request>();
    request->name = hardware;
    request->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    request->target_state.label = "active";
    return call<ControllerManager::SetHardwareComponentState>(
             &ControllerManager::set_hardware_component_state_srv_cb, request)
      ->ok;
  }

  cm_test_suite::ReportedState reported() override
  {
    cm_test_suite::ReportedState state;
    const auto controllers = call<ControllerManager::ListControllers>(
      &ControllerManager::list_controllers_srv_cb,
      std::make_shared<ControllerManager::ListControllers::Request>());
    for (const auto & controller : controllers->controller)
    {
      add_entry(state.controllers, state, controller.name, controller.state);
    }
    const auto hardware = call<ControllerManager::ListHardwareComponents>(
      &ControllerManager::list_hardware_components_srv_cb,
      std::make_shared<ControllerManager::ListHardwareComponents::Request>());
    for (const auto & component : hardware->component)
    {
      add_entry(state.hardware, state, component.name, component.state.label);
    }
    return state;
  }

  cm_test_suite::ReportedState activity() override
  {
    std::lock_guard<std::mutex> lock(activity_mutex_);
    return activity_;
  }

  void shutdown() override { cm_->shutdown(); }

private:
  static inline const rclcpp::Duration PERIOD = rclcpp::Duration::from_seconds(0.01);

  static inline int instances_ = 0;
  std::shared_ptr<rclcpp::Executor> executor_;
  const std::string name_space_;
  std::shared_ptr<ControllerManager> cm_;
  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr
    activity_subscription_;
  std::mutex activity_mutex_;
  cm_test_suite::ReportedState activity_;
  std::thread spinner_;
  rclcpp::Time time_;
  const std::chrono::milliseconds timeout_;
};
}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  if (!std::getenv("CM_FUZZ_VERBOSE"))
  {
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
  }
  cm_test_suite::Capabilities capabilities;
  capabilities.auto_switch = true;
  capabilities.force_auto_switch = true;
  capabilities.recover_hardware = true;
  capabilities.startup = true;
  cm_test_suite::register_suite(
    "Hector", capabilities,
    [](const cm_test_suite::CmConfig & config) { return std::make_unique<HectorCm>(config); });
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
