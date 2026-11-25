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

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fakes.hpp"
#include "gate.hpp"
#include "hector_controller_manager/controller_lifecycle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_environment.hpp"

namespace hector_controller_manager
{
namespace
{
using controller_interface::CallbackReturn;
using core::Lifecycle;
using namespace std::chrono_literals;

struct Delays
{
  std::atomic<int> configure_ms{0};
  std::atomic<int> shutdown_ms{0};
  /// on_cleanup calls, and how many of them came before on_shutdown.
  std::atomic<int> cleanups{0};
  std::atomic<int> cleanups_before_shutdown{-1};
};

/// Exports the reference "<name>/x" and the state "<name>/position".
class Exporter : public testing::NoInterfacesChainable
{
public:
  explicit Exporter(Delays & delays) : delays_(delays) {}
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(delays_.configure_ms.load()));
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    delays_.cleanups.fetch_add(1);
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    delays_.cleanups_before_shutdown = delays_.cleanups.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(delays_.shutdown_ms.load()));
    return CallbackReturn::SUCCESS;
  }

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list()
    override
  {
    hardware_interface::InterfaceInfo info{};
    info.name = "x";
    return {std::make_shared<hardware_interface::CommandInterface>(
      hardware_interface::InterfaceDescription(get_node()->get_name(), info))};
  }
  std::vector<hardware_interface::StateInterface::SharedPtr> on_export_state_interfaces_list()
    override
  {
    hardware_interface::InterfaceInfo info{};
    info.name = "position";
    return {std::make_shared<hardware_interface::StateInterface>(
      hardware_interface::InterfaceDescription(get_node()->get_name(), info))};
  }

private:
  Delays & delays_;
};

class ControllerLifecycleTest : public ::testing::Test
{
protected:
  ControllerLifecycleTest() { access_->open(host_); }
  ~ControllerLifecycleTest() override
  {
    inbox_->close();
    EXPECT_TRUE(access_->close(Clock::now() + 5s));
  }

  Clock::time_point in(Clock::duration timeout) const { return Clock::now() + timeout; }

  /// Runs the events of late returns, as the supervisor thread does.
  void absorb_late_return()
  {
    std::vector<std::function<void()>> events;
    ASSERT_TRUE(
      testing::eventually(
        [&]
        {
          for (auto & event : inbox_->take())
          {
            events.push_back(std::move(event));
          }
          return !events.empty();
        }));
    for (auto & event : events)
    {
      event();
    }
  }

  ControllerLifecycle::Dependencies dependencies()
  {
    return {
      access_,
      inbox_,
      rclcpp::get_logger("lifecycle"),
      [] {},
      100,
      testing::FINISH_TIMEOUT,
      std::make_shared<runtime::IoGate>()};
  }

  Delays delays_;
  testing::FakeHost host_{
    [this](const std::string &, const std::string & type) -> runtime::ControllerRuntime::Plugin
    {
      if (type != "test/Exporter")
      {
        throw std::runtime_error("unknown type 4711");
      }
      last_ = std::make_shared<Exporter>(delays_);
      return last_;
    },
    nullptr};
  /// The plugin the host created last.
  std::shared_ptr<Exporter> last_;
  std::shared_ptr<HostAccess> access_ = std::make_shared<HostAccess>();
  runtime::Doorbell wake_;
  runtime::PlanSlot slot_{wake_};
  std::shared_ptr<Inbox> inbox_ = std::make_shared<Inbox>(wake_);
  core::Model model_;
  Entities entities_;
  runtime::ThreadRole role_{"supervisor"};
  StepExecutor executor_{role_, model_, entities_, slot_, inbox_};
  RequestQueue requests_{wake_};
  ControllerLifecycle lifecycle_{role_, model_, entities_, executor_, requests_, dependencies()};
};

TEST_F(ControllerLifecycleTest, configure_registers_the_exports_and_cleanup_removes_them)
{
  ASSERT_TRUE(lifecycle_.load("e", "test/Exporter", false, in(testing::FINISH_TIMEOUT)).ok);
  ASSERT_TRUE(lifecycle_.configure("e", in(testing::FINISH_TIMEOUT)).ok);
  EXPECT_EQ(model_.controllers.at("e").state, Lifecycle::Inactive);
  EXPECT_EQ(model_.controllers.at("e").references, std::vector<std::string>{"e/x"});
  EXPECT_EQ(entities_.command_handles.count("e/x"), 1u);
  EXPECT_EQ(entities_.state_handles.count("e/position"), 1u);
  ASSERT_TRUE(lifecycle_.cleanup("e", in(testing::FINISH_TIMEOUT)).ok);
  EXPECT_EQ(model_.controllers.at("e").state, Lifecycle::Unconfigured);
  EXPECT_TRUE(model_.controllers.at("e").references.empty());
  EXPECT_EQ(entities_.command_handles.count("e/x"), 0u);
  EXPECT_EQ(entities_.state_handles.count("e/position"), 0u);
  ASSERT_TRUE(lifecycle_.unload("e", in(testing::FINISH_TIMEOUT)).ok);
  EXPECT_EQ(model_.controllers.count("e"), 0u);
  EXPECT_EQ(entities_.controllers.count("e"), 0u);
  EXPECT_EQ(host_.forgotten.load(), 1);
}

// Like upstream, an inactive controller is cleaned up before its shutdown
TEST_F(ControllerLifecycleTest, unloading_an_inactive_controller_cleans_it_up_first)
{
  ASSERT_TRUE(lifecycle_.load("e", "test/Exporter", false, in(testing::FINISH_TIMEOUT)).ok);
  ASSERT_TRUE(lifecycle_.configure("e", in(testing::FINISH_TIMEOUT)).ok);
  ASSERT_TRUE(lifecycle_.unload("e", in(testing::FINISH_TIMEOUT)).ok);
  EXPECT_EQ(delays_.cleanups_before_shutdown.load(), 1);
}

TEST_F(ControllerLifecycleTest, a_failed_load_keeps_its_reason_and_adds_nothing)
{
  const auto response = lifecycle_.load("u", "test/Unknown", false, in(testing::FINISH_TIMEOUT));
  EXPECT_FALSE(response.ok);
  EXPECT_NE(response.message.find("unknown type 4711"), std::string::npos) << response.message;
  EXPECT_EQ(model_.controllers.count("u"), 0u);
  EXPECT_EQ(entities_.controllers.count("u"), 0u);
}

TEST_F(ControllerLifecycleTest, colliding_exports_clean_the_controller_up_again)
{
  model_.hardware["hw"].state_interfaces = {"e/position"};
  ASSERT_TRUE(lifecycle_.load("e", "test/Exporter", false, in(testing::FINISH_TIMEOUT)).ok);
  const auto response = lifecycle_.configure("e", in(testing::FINISH_TIMEOUT));
  EXPECT_FALSE(response.ok);
  EXPECT_NE(response.message.find("collide"), std::string::npos) << response.message;
  EXPECT_EQ(model_.controllers.at("e").state, Lifecycle::Unconfigured);
  EXPECT_EQ(entities_.command_handles.count("e/x"), 0u);
}

TEST_F(ControllerLifecycleTest, a_controller_asking_for_its_own_update_rate_or_async_is_refused)
{
  const std::vector<std::pair<rclcpp::Parameter, std::string>> cases = {
    {rclcpp::Parameter("is_async", true), "Asynchronous controllers are not supported"},
    {rclcpp::Parameter("update_rate", 50), "Per controller update rates are not supported"},
  };
  for (const auto & [parameter, refusal] : cases)
  {
    SCOPED_TRACE(parameter.get_name());
    ASSERT_TRUE(lifecycle_.load("e", "test/Exporter", false, in(testing::FINISH_TIMEOUT)).ok);
    ASSERT_TRUE(last_->get_node()->set_parameter(parameter).successful);
    const auto response = lifecycle_.configure("e", in(testing::FINISH_TIMEOUT));
    EXPECT_FALSE(response.ok);
    EXPECT_NE(response.message.find(refusal), std::string::npos) << response.message;
    EXPECT_EQ(model_.controllers.at("e").state, Lifecycle::Unconfigured);
    ASSERT_TRUE(lifecycle_.unload("e", in(testing::FINISH_TIMEOUT)).ok);
  }
}

// The caller heard the configure failed, so the late success is undone by a queued cleanup
TEST_F(ControllerLifecycleTest, a_configure_that_returns_late_is_cleaned_up_by_a_queued_request)
{
  ASSERT_TRUE(lifecycle_.load("e", "test/Exporter", false, in(testing::FINISH_TIMEOUT)).ok);
  delays_.configure_ms = 200;
  EXPECT_FALSE(lifecycle_.configure("e", in(20ms)).ok);
  EXPECT_TRUE(model_.controllers.at("e").quarantined);
  EXPECT_FALSE(requests_.waiting());
  absorb_late_return();
  EXPECT_FALSE(model_.controllers.at("e").quarantined);
  EXPECT_EQ(model_.controllers.at("e").state, Lifecycle::Inactive);
  const auto queued = requests_.pop();
  ASSERT_TRUE(queued.request);
  EXPECT_TRUE(queued.request->work(queued.request->deadline).ok);
  EXPECT_EQ(model_.controllers.at("e").state, Lifecycle::Unconfigured);
}

// The name stays taken until the abandoned shutdown returned
TEST_F(ControllerLifecycleTest, an_unload_that_outlives_its_deadline_removes_the_entry_on_return)
{
  ASSERT_TRUE(lifecycle_.load("e", "test/Exporter", false, in(testing::FINISH_TIMEOUT)).ok);
  delays_.shutdown_ms = 200;
  EXPECT_FALSE(lifecycle_.unload("e", in(20ms)).ok);
  EXPECT_TRUE(model_.controllers.at("e").quarantined);
  EXPECT_FALSE(lifecycle_.load("e", "test/Exporter", false, in(testing::FINISH_TIMEOUT)).ok);
  absorb_late_return();
  EXPECT_EQ(model_.controllers.count("e"), 0u);
  EXPECT_EQ(entities_.controllers.count("e"), 0u);
  EXPECT_EQ(host_.forgotten.load(), 1);
}

TEST_F(ControllerLifecycleTest, requests_run_only_on_the_supervisor_thread)
{
  if (!runtime::CHECKED_BUILD)
  {
    GTEST_SKIP() << "thread roles are checked in debug and sanitizer builds";
  }
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  std::thread([this] { role_.bind(); }).join();
  EXPECT_DEATH(
    lifecycle_.load("e", "test/Exporter", false, in(testing::FINISH_TIMEOUT)),
    "Called off the supervisor thread");
}

}  // namespace
}  // namespace hector_controller_manager
