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

#include "hector_controller_manager/estop_topics.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace std::chrono_literals;
// A topic that never saw a message or publisher counts as this old
const Clock::time_point T0 = Clock::time_point{} + std::chrono::hours(1);

TEST(EStopTopics, every_topic_must_release_and_an_unknown_one_is_engaged)
{
  EStopTopics topics({"a", "b"}, 1s);
  EXPECT_TRUE(topics.engaged());
  topics.message("a", false, T0);
  EXPECT_TRUE(topics.engaged());
  topics.message("b", false, T0);
  EXPECT_FALSE(topics.engaged());
  topics.message("b", true, T0);
  EXPECT_TRUE(topics.engaged());
}

TEST(EStopTopics, the_cause_names_an_engaged_topic)
{
  EStopTopics topics({"a", "b"}, 1s);
  EXPECT_EQ(topics.cause(), "'a', it sent no message yet");
  topics.message("a", false, T0);
  EXPECT_EQ(topics.cause(), "'b', it sent no message yet");
  topics.message("a", true, T0);
  EXPECT_EQ(topics.cause(), "'b', it sent no message yet") << "b still engages";
  topics.message("b", false, T0);
  EXPECT_EQ(topics.cause(), "'a', it says engaged");
  topics.message("a", false, T0);
  EXPECT_EQ(topics.cause(), "'a', it says engaged") << "kept after the release";
  EXPECT_TRUE(topics.publishers("b", 0, T0 + 2s));
  EXPECT_EQ(topics.cause(), "'b', it has no publisher any more");
}

TEST(EStopTopics, losing_the_publisher_engages_after_the_grace_until_a_new_release)
{
  EStopTopics topics({"a"}, 1s);
  topics.message("a", false, T0);
  EXPECT_FALSE(topics.publishers("a", 1, T0 + 500ms));
  // Past the grace of the message, within the grace of the count
  EXPECT_FALSE(topics.publishers("a", 0, T0 + 1300ms));
  EXPECT_FALSE(topics.engaged()) << "within the grace of the last count";
  EXPECT_TRUE(topics.publishers("a", 0, T0 + 1600ms));
  EXPECT_TRUE(topics.engaged());
  EXPECT_FALSE(topics.publishers("a", 0, T0 + 1700ms)) << "reported once";
  topics.message("a", false, T0 + 2s);
  EXPECT_FALSE(topics.engaged());
}

// rmw_zenoh may deliver a new publisher's latched message before its graph counts the publisher
TEST(EStopTopics, a_message_before_its_publisher_is_counted_is_kept)
{
  EStopTopics topics({"a"}, 1s);
  topics.message("a", false, T0);
  EXPECT_FALSE(topics.publishers("a", 0, T0 + 100ms));
  EXPECT_FALSE(topics.publishers("a", 1, T0 + 300ms));
  EXPECT_FALSE(topics.publishers("a", 1, T0 + 5s));
  EXPECT_FALSE(topics.engaged());
}

// ros2 topic pub --once releases and leaves before a check counted the publisher
TEST(EStopTopics, a_release_by_a_publisher_never_counted_ends_after_the_grace)
{
  EStopTopics topics({"a"}, 1s);
  topics.message("a", false, T0);
  EXPECT_FALSE(topics.publishers("a", 0, T0 + 900ms));
  EXPECT_FALSE(topics.engaged());
  EXPECT_TRUE(topics.publishers("a", 0, T0 + 1100ms));
  EXPECT_TRUE(topics.engaged());
}

TEST(EStopTopics, a_released_topic_without_a_message_within_the_deadline_engages)
{
  EStopTopics topics({"a"}, 1s, 300ms);
  topics.message("a", false, T0);
  // A publisher alone does not keep it released
  EXPECT_FALSE(topics.publishers("a", 1, T0 + 250ms));
  EXPECT_FALSE(topics.deadline_missed("a", T0 + 300ms));
  EXPECT_FALSE(topics.engaged());
  EXPECT_TRUE(topics.deadline_missed("a", T0 + 350ms));
  EXPECT_TRUE(topics.engaged());
  EXPECT_FALSE(topics.deadline_missed("a", T0 + 400ms)) << "reported once";
  topics.message("a", true, T0 + 500ms);
  EXPECT_FALSE(topics.deadline_missed("a", T0 + 2s)) << "already engaged";
  topics.message("a", false, T0 + 2s);
  EXPECT_FALSE(topics.engaged());
}

TEST(EStopTopics, a_periodic_release_within_the_deadline_stays_released)
{
  EStopTopics topics({"a"}, 1s, 300ms);
  for (auto at = T0; at < T0 + 3s; at += 200ms)
  {
    topics.message("a", false, at);
    EXPECT_FALSE(topics.deadline_missed("a", at + 150ms));
    EXPECT_FALSE(topics.publishers("a", 1, at + 150ms));
  }
  EXPECT_FALSE(topics.engaged());
}

TEST(EStopTopics, without_a_deadline_a_silent_released_topic_stays_released)
{
  EStopTopics topics({"a"}, 1s);
  topics.message("a", false, T0);
  EXPECT_FALSE(topics.publishers("a", 1, T0 + 1h));
  EXPECT_FALSE(topics.deadline_missed("a", T0 + 1h));
  EXPECT_FALSE(topics.engaged());
}

}  // namespace
}  // namespace hector_controller_manager
