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
#include <thread>

#include "fakes.hpp"
#include "gate.hpp"
#include "hector_controller_manager/plugin_host.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace std::chrono_literals;

TEST(HostAccess, a_closed_access_never_calls_the_host)
{
  testing::FakeHost host{nullptr, nullptr};
  HostAccess access;
  access.open(host);
  PluginHost * used = nullptr;
  access.with_host([&](PluginHost & h) { used = &h; });
  EXPECT_EQ(used, &host);
  EXPECT_TRUE(access.close(Clock::now()));
  used = nullptr;
  access.with_host([&](PluginHost & h) { used = &h; });
  EXPECT_EQ(used, nullptr);
}

TEST(HostAccess, close_waits_for_a_call_in_flight_until_its_deadline)
{
  testing::FakeHost host{nullptr, nullptr};
  HostAccess access;
  access.open(host);
  testing::Gate call;
  call.close();
  std::thread job([&] { access.with_host([&](PluginHost &) { call.pass(); }); });
  EXPECT_TRUE(call.holds());
  EXPECT_FALSE(access.close(Clock::now() + 20ms));
  std::atomic<bool> released{false};
  std::thread releaser(
    [&]
    {
      std::this_thread::sleep_for(20ms);
      released = true;
      call.open();
    });
  EXPECT_TRUE(access.close(Clock::now() + 5s));
  EXPECT_TRUE(released.load());
  releaser.join();
  job.join();
}

}  // namespace
}  // namespace hector_controller_manager
