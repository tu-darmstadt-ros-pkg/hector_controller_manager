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

#include <future>
#include <thread>

#include "hector_controller_manager/request_queue.hpp"

namespace hector_controller_manager
{
namespace
{
using namespace std::chrono_literals;

RequestQueue::Work answer(const std::string & message)
{
  return [message](Clock::time_point) { return Response{true, message}; };
}

TEST(RequestQueue, pops_in_fifo_order_and_rejects_requests_after_close)
{
  runtime::Doorbell wake;
  RequestQueue queue(wake);
  const uint32_t seen = wake.value();
  ASSERT_TRUE(queue.push(answer("a"), 1s));
  ASSERT_TRUE(queue.push(answer("b"), 1s));
  EXPECT_NE(wake.value(), seen);
  queue.close();
  EXPECT_FALSE(queue.push(answer("c"), 1s));
  EXPECT_EQ(queue.pop().request->work({}).message, "a");
  const auto last = queue.pop();
  EXPECT_EQ(last.request->work({}).message, "b");
  EXPECT_FALSE(last.closed);
  EXPECT_TRUE(queue.empty());
  const auto end = queue.pop();
  EXPECT_FALSE(end.request);
  EXPECT_TRUE(end.closed);
}

TEST(RequestQueue, a_request_that_does_not_start_in_time_is_answered_busy_and_dropped)
{
  runtime::Doorbell wake;
  RequestQueue queue(wake);
  const auto response = queue.call(answer("late"), 10ms, 10ms);
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.message, RequestQueue::busy().message);
  EXPECT_TRUE(queue.empty());
}

TEST(RequestQueue, a_started_request_answers_with_its_result)
{
  runtime::Doorbell wake;
  RequestQueue queue(wake);
  std::thread worker(
    [&]
    {
      RequestQueue::Next next;
      while (!next.request)
      {
        const uint32_t seen = wake.value();
        next = queue.pop();
        if (!next.request)
        {
          wake.wait_until(seen, Clock::now() + 1s);
        }
      }
      next.request->promise.set_value(next.request->work(next.request->deadline));
    });
  const auto response = queue.call(answer("done"), 5s, 1s);
  worker.join();
  EXPECT_TRUE(response.ok);
  EXPECT_EQ(response.message, "done");
}

TEST(RequestQueue, a_first_call_goes_ahead_and_waits_past_any_deadline_until_answered)
{
  runtime::Doorbell wake;
  RequestQueue queue(wake);
  ASSERT_TRUE(queue.push(answer("waiting"), 1ms));
  const uint32_t seen = wake.value();
  auto first = std::async(std::launch::async, [&] { return queue.call_first(answer("first")); });
  ASSERT_TRUE(wake.wait_until(seen, Clock::now() + 5s));
  const auto next = queue.pop();
  const auto then = queue.pop();
  EXPECT_EQ(first.wait_for(50ms), std::future_status::timeout);
  for (const auto & popped : {next, then})
  {
    popped.request->promise.set_value(popped.request->work({}));
  }
  EXPECT_EQ(next.request->work({}).message, "first");
  EXPECT_EQ(first.get().message, "first");
}

}  // namespace
}  // namespace hector_controller_manager
