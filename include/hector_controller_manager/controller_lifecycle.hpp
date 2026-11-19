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

#ifndef HECTOR_CONTROLLER_MANAGER__CONTROLLER_LIFECYCLE_HPP_
#define HECTOR_CONTROLLER_MANAGER__CONTROLLER_LIFECYCLE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/entities.hpp"
#include "hector_controller_manager/inbox.hpp"
#include "hector_controller_manager/plugin_host.hpp"
#include "hector_controller_manager/request_queue.hpp"
#include "hector_controller_manager/runtime/controller_runtime.hpp"
#include "hector_controller_manager/step_executor.hpp"
#include "rclcpp/logger.hpp"

namespace hector_controller_manager
{
/// Loads, configures, cleans up and unloads controllers on their owner threads. Supervisor thread
/// only. A call it gives up on quarantines the controller, its late return arrives through the
/// inbox.
class ControllerLifecycle
{
public:
  struct Dependencies
  {
    std::shared_ptr<HostAccess> host_access;
    std::shared_ptr<Inbox> inbox;
    rclcpp::Logger logger;
    /// Called after every change of the model that clients should see.
    std::function<void()> publish_status;
    unsigned int update_rate;
    /// Deadline of the cleanup that undoes a configure which returned after its request failed.
    Clock::duration default_timeout;
    std::shared_ptr<runtime::IoGate> introspection_gate;
  };

  ControllerLifecycle(
    const runtime::ThreadRole & supervisor, core::Model & model, Entities & entities,
    StepExecutor & executor, RequestQueue & requests, Dependencies dependencies);

  Response load(
    const std::string & name, const std::string & type, bool is_protected,
    Clock::time_point deadline);
  Response configure(const std::string & name, Clock::time_point deadline);
  Response cleanup(const std::string & name, Clock::time_point deadline);
  Response unload(const std::string & name, Clock::time_point deadline);

private:
  /// Gives up on a call that did not return and appends that to `what`.
  std::string quarantine(const std::string & name, const std::string & what);
  /// Takes a configured controller's interfaces into the model. False if they collide.
  bool register_configured(const std::string & name, const runtime::ControllerExports & exports);
  void unregister_exports(const std::string & name);

  const runtime::ThreadRole & supervisor_;
  core::Model & model_;
  Entities & entities_;
  StepExecutor & executor_;
  RequestQueue & requests_;
  std::shared_ptr<HostAccess> host_access_;
  std::shared_ptr<Inbox> inbox_;
  rclcpp::Logger logger_;
  std::function<void()> publish_status_;
  unsigned int update_rate_;
  Clock::duration default_timeout_;
  std::shared_ptr<runtime::IoGate> introspection_gate_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__CONTROLLER_LIFECYCLE_HPP_
