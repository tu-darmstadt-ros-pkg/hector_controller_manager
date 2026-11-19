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

#ifndef HECTOR_CONTROLLER_MANAGER__ESTOP_TOPICS_HPP_
#define HECTOR_CONTROLLER_MANAGER__ESTOP_TOPICS_HPP_

#include <algorithm>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "hector_controller_manager/clock.hpp"

namespace hector_controller_manager
{
/// The state of the e-stop topics, not thread safe. A topic is engaged until a message says
/// false and while the last one says true. It is also engaged once it had no publisher for longer
/// than the grace since its last message or the last time a publisher was counted, until a message
/// says false again. The grace keeps a latched release that an RMW delivers before it counts the
/// publisher (rmw_zenoh), and it still catches a publisher that releases and leaves unseen. With a
/// deadline, a released topic without a message for longer than it is engaged as well, until a
/// message says false again. A deadline of zero is off. The cause is the topic that engaged the
/// e-stop, until it releases while another topic keeps the e-stop engaged.
class EStopTopics
{
public:
  EStopTopics(
    const std::vector<std::string> & names, Clock::duration grace,
    Clock::duration deadline = Clock::duration::zero())
  : grace_(grace), deadline_(deadline)
  {
    for (const auto & name : names)
    {
      topics_.emplace(name, Topic{});
    }
    if (!topics_.empty())
    {
      cause_ = topics_.begin()->first;
    }
  }

  bool empty() const { return topics_.empty(); }

  std::vector<std::string> names() const
  {
    std::vector<std::string> result;
    for (const auto & [name, unused] : topics_)
    {
      result.push_back(name);
    }
    return result;
  }

  void message(const std::string & name, bool engaged, Clock::time_point now)
  {
    auto & topic = topics_.at(name);
    topic.engaged = engaged;
    topic.alive = now;
    topic.message = now;
    if (engaged)
    {
      topic.reason = "it says engaged";
    }
    update_cause();
  }

  /// True if the topic just counts as without publisher.
  bool publishers(const std::string & name, size_t count, Clock::time_point now)
  {
    auto & topic = topics_.at(name);
    if (count > 0)
    {
      topic.alive = now;
      return false;
    }
    if (!topic.engaged || now - topic.alive <= grace_)
    {
      return false;
    }
    topic.engaged.reset();
    topic.reason = "it has no publisher any more";
    update_cause();
    return true;
  }

  /// True if the released topic just missed its deadline.
  bool deadline_missed(const std::string & name, Clock::time_point now)
  {
    auto & topic = topics_.at(name);
    if (
      deadline_ == Clock::duration::zero() || topic.engaged.value_or(true) ||
      now - topic.message <= deadline_)
    {
      return false;
    }
    topic.engaged.reset();
    topic.reason = "it sent no message within estop_deadline_ms";
    update_cause();
    return true;
  }

  bool engaged() const
  {
    return std::any_of(
      topics_.begin(), topics_.end(),
      [](const auto & entry) { return entry.second.engaged.value_or(true); });
  }

  /// The cause topic and why it engaged, the last one after a release.
  std::string cause() const
  {
    return topics_.empty() ? std::string() : "'" + cause_ + "', " + topics_.at(cause_).reason;
  }

private:
  struct Topic
  {
    std::optional<bool> engaged;
    /// Last message or last count of a publisher.
    Clock::time_point alive{};
    Clock::time_point message{};
    const char * reason = "it sent no message yet";
  };

  void update_cause()
  {
    if (topics_.at(cause_).engaged.value_or(true))
    {
      return;
    }
    const auto engaged = std::find_if(
      topics_.begin(), topics_.end(),
      [](const auto & entry) { return entry.second.engaged.value_or(true); });
    if (engaged != topics_.end())
    {
      cause_ = engaged->first;
    }
  }

  Clock::duration grace_;
  Clock::duration deadline_;
  std::map<std::string, Topic> topics_;
  std::string cause_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__ESTOP_TOPICS_HPP_
