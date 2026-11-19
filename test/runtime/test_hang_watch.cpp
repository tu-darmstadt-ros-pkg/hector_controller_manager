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

#include <chrono>
#include <optional>

#include "hector_controller_manager/runtime/hang_watch.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
using std::chrono::milliseconds;
constexpr Clock::duration TIMEOUT = milliseconds(100);

TEST(HangWatch, a_busy_owner_stalls_once_it_stayed_busy_longer_than_the_timeout)
{
  HangWatch watch;
  const auto begin = Clock::now();
  EXPECT_EQ(watch.rt_check(begin, TIMEOUT, true), std::nullopt);
  EXPECT_EQ(watch.rt_check(begin + TIMEOUT, TIMEOUT, true), std::nullopt);
  EXPECT_EQ(watch.rt_check(begin + milliseconds(101), TIMEOUT, true), milliseconds(101));
}

TEST(HangWatch, a_mode_switch_ends_the_window)
{
  HangWatch watch;
  const auto begin = Clock::now();
  EXPECT_EQ(watch.rt_check(begin, TIMEOUT, true), std::nullopt);
  watch.owner_switching(true);
  EXPECT_EQ(watch.rt_check(begin + milliseconds(50), TIMEOUT, true), std::nullopt);
  watch.owner_switching(false);
  EXPECT_EQ(watch.rt_check(begin + milliseconds(120), TIMEOUT, true), std::nullopt)
    << "the window starts again after the switch";
  EXPECT_EQ(watch.rt_check(begin + milliseconds(221), TIMEOUT, true), milliseconds(101));
}

TEST(HangWatch, an_idle_owner_ends_the_window)
{
  HangWatch watch;
  const auto begin = Clock::now();
  EXPECT_EQ(watch.rt_check(begin, TIMEOUT, true), std::nullopt);
  EXPECT_EQ(watch.rt_check(begin + milliseconds(50), TIMEOUT, false), std::nullopt);
  EXPECT_EQ(watch.rt_check(begin + milliseconds(120), TIMEOUT, true), std::nullopt);
  EXPECT_EQ(watch.rt_check(begin + milliseconds(221), TIMEOUT, true), milliseconds(101));
}

TEST(HangWatch, ending_the_window_restarts_it_at_the_next_check)
{
  HangWatch watch;
  const auto begin = Clock::now();
  EXPECT_EQ(watch.rt_check(begin, TIMEOUT, true), std::nullopt);
  watch.rt_end_window();
  EXPECT_EQ(watch.rt_check(begin + milliseconds(120), TIMEOUT, true), std::nullopt);
  EXPECT_EQ(watch.rt_check(begin + milliseconds(221), TIMEOUT, true), milliseconds(101));
}

TEST(HangWatch, the_owner_names_its_plugin_call_while_it_runs)
{
  HangWatch watch;
  EXPECT_EQ(watch.call(), nullptr);
  {
    const HangWatch::InCall in_call(watch, "on_activate");
    EXPECT_STREQ(watch.call(), "on_activate");
    {
      const HangWatch::InCall unnamed(watch, nullptr);
      EXPECT_STREQ(watch.call(), "on_activate") << "a call without a name names nothing";
    }
    {
      const HangWatch::InCall nested(watch, "perform_command_mode_switch");
      EXPECT_STREQ(watch.call(), "perform_command_mode_switch");
    }
    EXPECT_STREQ(watch.call(), "on_activate") << "a nested call gives the name back";
  }
  EXPECT_EQ(watch.call(), nullptr);
}

}  // namespace
}  // namespace hector_controller_manager::runtime
