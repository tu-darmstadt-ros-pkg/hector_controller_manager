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

#ifndef SUITE__SERVICES_RIG_HPP_
#define SUITE__SERVICES_RIG_HPP_

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../runtime/gate.hpp"
#include "../runtime/rclcpp_environment.hpp"
#include "../runtime/rt_thread.hpp"
#include "cm_test_suite/faults.hpp"
#include "cm_test_suite/rig.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"
#include "hector_controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace hector_controller_manager::testing::services
{
using namespace std::chrono_literals;
using Load = controller_manager_msgs::srv::LoadController;
using Unload = controller_manager_msgs::srv::UnloadController;

/// The namespace of this test run, so concurrent runs in one domain do not cross-talk.
inline const std::string RUN_NAMESPACE = "/hcm_services_" + std::to_string(::getpid());

/// Spins the controller manager's own executor with the given threads, as ros2_control_node does.
class Services : public ::testing::TestWithParam<size_t>
{
protected:
  void SetUp() override
  {
    std::vector<rclcpp::Parameter> parameters = {
      {"robot_description", cm_test_suite::rig::urdf()},
      {"executor_threads", static_cast<int64_t>(GetParam())},
      {"arm_vel.protected", true}};
    for (const auto & controller : cm_test_suite::rig::controllers())
    {
      parameters.emplace_back(
        controller.name + ".type", cm_test_suite::rig::plugin_type(controller));
    }
    for (auto & parameter : extra_parameters())
    {
      parameters.push_back(std::move(parameter));
    }
    auto options = get_cm_node_options();
    options.parameter_overrides(parameters);
    const std::string name = "cm_services_" + std::to_string(GetParam());
    cm_ = std::make_shared<ControllerManager>(nullptr, name, RUN_NAMESPACE, options);
    executor_ = cm_->executor();
    executor_->add_node(cm_);
    spinner_ = std::thread([this] { executor_->spin(); });
    loop_.start(
      [this, scheduled = !schedules_loop()](
        const rclcpp::Time & time, const rclcpp::Duration & period) mutable
      {
        if (!scheduled)
        {
          cm_->schedule_control_loop();
          scheduled = true;
        }
        cm_->read(time, period);
        while (stall_)
        {
          std::this_thread::sleep_for(1ms);
        }
        cm_->update(time, period);
        cm_->write(time, period);
      });
    client_node_ = std::make_shared<rclcpp::Node>(name + "_client", RUN_NAMESPACE);
    client_executor_.add_node(client_node_);
    const std::string cm_name = cm_->get_fully_qualified_name();
    load_ = client_node_->create_client<Load>(cm_name + "/load_controller");
    unload_ = client_node_->create_client<Unload>(cm_name + "/unload_controller");
    ASSERT_TRUE(load_->wait_for_service(10s));
    ASSERT_TRUE(unload_->wait_for_service(10s));
  }

  virtual std::vector<rclcpp::Parameter> extra_parameters() const { return {}; }
  /// Whether the loop schedules itself as ros2_control_node does, else it is an embedding loop.
  virtual bool schedules_loop() const { return false; }

  void TearDown() override
  {
    cm_test_suite::faults().clear();
    cm_->shutdown();
    stall_ = false;
    loop_.stop();
    executor_->cancel();
    spinner_.join();
  }

  template <typename Client>
  auto send(Client & client, const std::string & name)
  {
    auto request = std::make_shared<typename Client::element_type::Request>();
    request->name = name;
    return client->async_send_request(request);
  }

  template <typename Future>
  bool answered(Future & future)
  {
    return client_executor_.spin_until_future_complete(future, 10s) ==
             rclcpp::FutureReturnCode::SUCCESS &&
           future.get()->ok;
  }

  std::shared_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<ControllerManager> cm_;
  std::thread spinner_;
  /// Holds the loop between a read and its update.
  std::atomic<bool> stall_{false};
  RtThread loop_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::SingleThreadedExecutor client_executor_;
  rclcpp::Client<Load>::SharedPtr load_;
  rclcpp::Client<Unload>::SharedPtr unload_;
};

inline bool configure(ControllerManager & cm, const std::string & name)
{
  auto request = std::make_shared<ControllerManager::ConfigureController::Request>();
  request->name = name;
  auto response = std::make_shared<ControllerManager::ConfigureController::Response>();
  cm.configure_controller_service_cb(request, response);
  return response->ok;
}

inline std::shared_ptr<ControllerManager::SwitchController::Response> switch_controllers(
  ControllerManager & cm, std::vector<std::string> activate, std::vector<std::string> deactivate,
  int32_t strictness = ControllerManager::SwitchController::Request::STRICT)
{
  auto request = std::make_shared<ControllerManager::SwitchController::Request>();
  request->activate_controllers = std::move(activate);
  request->deactivate_controllers = std::move(deactivate);
  request->strictness = strictness;
  auto response = std::make_shared<ControllerManager::SwitchController::Response>();
  cm.switch_controller_service_cb(request, response);
  return response;
}

inline std::string state_of(ControllerManager & cm, const std::string & name)
{
  auto response = std::make_shared<ControllerManager::ListControllers::Response>();
  cm.list_controllers_srv_cb(
    std::make_shared<ControllerManager::ListControllers::Request>(), response);
  for (const auto & controller : response->controller)
  {
    if (controller.name == name)
    {
      return controller.state;
    }
  }
  return "";
}

inline std::string hardware_state_of(ControllerManager & cm, const std::string & name)
{
  auto response = std::make_shared<ControllerManager::ListHardwareComponents::Response>();
  cm.list_hardware_components_srv_cb(
    std::make_shared<ControllerManager::ListHardwareComponents::Request>(), response);
  for (const auto & component : response->component)
  {
    if (component.name == name)
    {
      return component.state.label;
    }
  }
  return "";
}

/// Two e-stop topics, which deliver nothing until the test publishes.
class EStop : public Services
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    return {{"estop_topics", std::vector<std::string>{topic("a"), topic("b")}}};
  }

  std::string topic(const std::string & which) const
  {
    return RUN_NAMESPACE + "/estop_" + which + "_" + std::to_string(GetParam());
  }

  void publish(const std::string & which, bool engaged)
  {
    auto & publisher = publishers_[which];
    if (!publisher)
    {
      publisher = client_node_->create_publisher<std_msgs::msg::Bool>(
        topic(which), rclcpp::QoS(1).reliable().transient_local());
    }
    std_msgs::msg::Bool message;
    message.data = engaged;
    publisher->publish(message);
  }

  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> publishers_;
};

}  // namespace hector_controller_manager::testing::services

#endif  // SUITE__SERVICES_RIG_HPP_
