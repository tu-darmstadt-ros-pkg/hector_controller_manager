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

#include "hector_controller_manager/controller_lifecycle.hpp"

#include <utility>

#include "hector_controller_manager/leak.hpp"
#include "hector_controller_manager/runtime/thrown.hpp"
#include "rclcpp/logging.hpp"

namespace hector_controller_manager
{
namespace
{
using core::CallResult;
using core::Lifecycle;
using runtime::ControllerRuntime;
}  // namespace

ControllerLifecycle::ControllerLifecycle(
  const runtime::ThreadRole & supervisor, core::Model & model, Entities & entities,
  StepExecutor & executor, RequestQueue & requests, Dependencies dependencies)
: supervisor_(supervisor),
  model_(model),
  entities_(entities),
  executor_(executor),
  requests_(requests),
  host_access_(std::move(dependencies.host_access)),
  inbox_(std::move(dependencies.inbox)),
  logger_(std::move(dependencies.logger)),
  publish_status_(std::move(dependencies.publish_status)),
  update_rate_(dependencies.update_rate),
  default_timeout_(dependencies.default_timeout),
  introspection_gate_(std::move(dependencies.introspection_gate))
{
}

Response ControllerLifecycle::load(
  const std::string & name, const std::string & type, bool is_protected, Clock::time_point deadline)
{
  supervisor_.expect();
  if (model_.controllers.count(name))
  {
    return {false, "A controller named '" + name + "' is already loaded"};
  }
  if (type.empty())
  {
    return {false, "No type is set for controller '" + name + "'"};
  }
  auto controller = std::make_shared<ControllerRuntime>(name, type, introspection_gate_);
  auto error = std::make_shared<std::string>();
  auto job = controller->owner().submit(
    [access = host_access_, controller = controller.get(), error, gate = introspection_gate_]
    {
      // on_init may register a value enabled and write it right after
      const runtime::OwnerHold hold(*gate);
      *error = runtime::thrown_by(
                 [&]
                 {
                   access->with_host(
                     [&](PluginHost & host)
                     {
                       controller->set_plugin(
                         host.create_controller(controller->name(), controller->type()));
                     });
                 })
                 .value_or("");
    });
  // A load that returns late created a plugin nobody owns, its node leaves the executor
  const auto late = [access = host_access_, controller = controller.get()]
  {
    if (controller->plugin())
    {
      access->with_host([&](PluginHost & host) { host.forget_controller(controller->plugin()); });
    }
  };
  if (!controller->owner().wait(job, deadline, late))
  {
    leak(controller);
    return {false, "Loading '" + name + "' did not finish in time, it is abandoned"};
  }
  if (!error->empty())
  {
    return {false, "Loading '" + name + "' failed: " + *error};
  }
  entities_.controllers.emplace(name, controller);
  core::Controller model;
  model.is_protected = is_protected;
  model_.controllers.emplace(name, model);
  publish_status_();
  return {true, ""};
}

std::string ControllerLifecycle::quarantine(const std::string & name, const std::string & what)
{
  core::apply(model_, core::GivenUp{{name}, {}});
  publish_status_();
  return what + ", it is quarantined until it returns";
}

bool ControllerLifecycle::register_configured(
  const std::string & name, const runtime::ControllerExports & exports)
{
  auto & model = model_.controllers.at(name);
  model.interfaces = exports.interfaces;
  model.chainable = exports.chainable;
  model.references.clear();
  model.exported_states.clear();
  for (const auto & reference : exports.references)
  {
    model.references.push_back(reference->get_name());
  }
  for (const auto & exported : exports.exported_states)
  {
    model.exported_states.push_back(exported->get_name());
  }
  try
  {
    const core::InterfaceIndex index(model_);
  }
  catch (const std::invalid_argument & e)
  {
    RCLCPP_ERROR(logger_, "Controller '%s': %s", name.c_str(), e.what());
    model.references.clear();
    model.exported_states.clear();
    return false;
  }
  for (const auto & reference : exports.references)
  {
    entities_.command_handles[reference->get_name()] = reference;
  }
  for (const auto & exported : exports.exported_states)
  {
    entities_.state_handles[exported->get_name()] = exported;
  }
  return true;
}

void ControllerLifecycle::unregister_exports(const std::string & name)
{
  auto & model = model_.controllers.at(name);
  for (const auto & reference : model.references)
  {
    entities_.command_handles.erase(reference);
  }
  for (const auto & exported : model.exported_states)
  {
    entities_.state_handles.erase(exported);
  }
  model.references.clear();
  model.exported_states.clear();
  model.interfaces = {};
  model.chainable = false;
  model.chained = false;
}

Response ControllerLifecycle::configure(const std::string & name, Clock::time_point deadline)
{
  supervisor_.expect();
  const auto it = model_.controllers.find(name);
  if (it == model_.controllers.end())
  {
    return {false, "Controller '" + name + "' is not loaded"};
  }
  const auto & model = it->second;
  if (
    model.quarantined ||
    (model.state != Lifecycle::Unconfigured && model.state != Lifecycle::Inactive))
  {
    return {
      false, "Controller '" + name + "' cannot be configured, it is " + to_string(model.state)};
  }
  if (model.state == Lifecycle::Inactive)
  {
    // Configuring an inactive controller again cleans it up first
    if (auto cleaned = cleanup(name, deadline); !cleaned.ok)
    {
      return cleaned;
    }
  }
  auto * controller = entities_.controllers.at(name).get();
  auto exports = std::make_shared<runtime::ControllerExports>();
  const unsigned int rate = update_rate_;
  // False if the exports collide, the controller then has to be cleaned up again
  const auto settle = [this, name, exports](Lifecycle state)
  { return state != Lifecycle::Inactive || register_configured(name, *exports); };
  const core::Outcome outcome = executor_.lifecycle_call(
    controller->owner(),
    [controller, exports, rate]
    {
      Lifecycle state{};
      const auto error = runtime::thrown_by([&] { state = controller->configure(rate, *exports); });
      if (error)
      {
        return core::call_outcome(CallResult::Error, controller->state(), *error);
      }
      return core::call_outcome(
        state == Lifecycle::Inactive ? CallResult::Ok : CallResult::Failure, state);
    },
    deadline,
    [this, name, settle](Lifecycle state)
    {
      core::apply(model_, core::returned::ControllerCallback{name, state});
      settle(state);
      // The caller heard the configure failed, so it ends where it started. Queued like a
      // request, so it never holds up one that is already waiting.
      if (state == Lifecycle::Inactive)
      {
        requests_.push(
          [this, name](Clock::time_point deadline) { return cleanup(name, deadline); },
          default_timeout_);
      }
    });
  if (outcome.result == CallResult::TimedOut)
  {
    return {false, quarantine(name, "Configuring '" + name + "' timed out")};
  }
  model_.controllers.at(name).state = outcome.state;
  if (!settle(outcome.state))
  {
    // On the owner with the request's deadline, like every plugin call
    const auto cleaned = cleanup(name, deadline);
    return {false, "Configuring '" + name + "' failed, its exports collide. " + cleaned.message};
  }
  publish_status_();
  if (model_.controllers.at(name).state != Lifecycle::Inactive)
  {
    return {
      false,
      "Configuring '" + name + "' failed" + (outcome.reason.empty() ? "" : ": " + outcome.reason)};
  }
  return {true, ""};
}

Response ControllerLifecycle::cleanup(const std::string & name, Clock::time_point deadline)
{
  supervisor_.expect();
  const auto it = model_.controllers.find(name);
  if (it == model_.controllers.end())
  {
    return {false, "Controller '" + name + "' is not loaded"};
  }
  if (it->second.state == Lifecycle::Unconfigured && !it->second.quarantined)
  {
    return {true, ""};
  }
  if (it->second.quarantined || it->second.state != Lifecycle::Inactive)
  {
    return {
      false,
      "Controller '" + name + "' cannot be cleaned up, it is " + to_string(it->second.state)};
  }
  auto * controller = entities_.controllers.at(name).get();
  const auto settle = [this, name](Lifecycle state)
  {
    if (state != Lifecycle::Inactive)
    {
      unregister_exports(name);
    }
  };
  const core::Outcome outcome = executor_.lifecycle_call(
    controller->owner(),
    [controller] { return core::call_outcome(CallResult::Ok, controller->cleanup()); }, deadline,
    [this, name, settle](Lifecycle state)
    {
      core::apply(model_, core::returned::ControllerCallback{name, state});
      settle(state);
    });
  if (outcome.result == CallResult::TimedOut)
  {
    return {false, quarantine(name, "Cleaning up '" + name + "' timed out")};
  }
  model_.controllers.at(name).state = outcome.state;
  settle(outcome.state);
  publish_status_();
  if (outcome.state != Lifecycle::Unconfigured)
  {
    return {false, "Cleaning up '" + name + "' failed, it is " + to_string(outcome.state)};
  }
  return {true, ""};
}

Response ControllerLifecycle::unload(const std::string & name, Clock::time_point deadline)
{
  supervisor_.expect();
  const auto it = model_.controllers.find(name);
  if (it == model_.controllers.end())
  {
    return {false, "Controller '" + name + "' is not loaded"};
  }
  if (it->second.quarantined || it->second.state == Lifecycle::Active)
  {
    return {
      false, "Controller '" + name + "' cannot be unloaded, it is " + to_string(it->second.state)};
  }
  // An older plan may still keep the runtime alive, the plugin has to go with the unload
  if (!executor_.release_old_plans(deadline))
  {
    return {false, "RT is still in a call that may use '" + name + "', try again"};
  }
  auto controller = entities_.controllers.at(name);
  auto forgotten = std::make_shared<bool>(false);
  auto job = controller->owner().submit(
    [access = host_access_, controller = controller.get(), forgotten]
    {
      // Like upstream, so a plugin releases in on_cleanup what on_configure took
      if (controller->state() == Lifecycle::Inactive)
      {
        controller->cleanup();
      }
      controller->shutdown();
      access->with_host([&](PluginHost & host)
                        { *forgotten = host.forget_controller(controller->plugin()); });
    });
  unregister_exports(name);
  // The entry stays until the shutdown returned, so the name cannot be loaded twice
  const auto remove = [this, name, forgotten]
  {
    auto runtime = std::move(entities_.controllers.at(name));
    entities_.controllers.erase(name);
    model_.controllers.erase(name);
    if (!*forgotten)
    {
      leak(std::move(runtime));
    }
  };
  auto inbox = inbox_;
  if (!controller->owner().wait(job, deadline, [inbox, remove] { inbox->post(remove); }))
  {
    return {false, quarantine(name, "Shutting down '" + name + "' did not finish")};
  }
  controller.reset();
  remove();
  publish_status_();
  return {true, ""};
}

}  // namespace hector_controller_manager
