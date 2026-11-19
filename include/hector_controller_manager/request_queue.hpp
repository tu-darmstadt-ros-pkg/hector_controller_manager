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

#ifndef HECTOR_CONTROLLER_MANAGER__REQUEST_QUEUE_HPP_
#define HECTOR_CONTROLLER_MANAGER__REQUEST_QUEUE_HPP_

#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include "hector_controller_manager/runtime/doorbell.hpp"

namespace hector_controller_manager
{
struct Response
{
  bool ok = false;
  std::string message;
  /// Another request came first, the same request may succeed when asked again.
  bool try_again = false;
};

/// The requests waiting for the supervisor thread, in FIFO order. Safe from any thread except RT.
class RequestQueue
{
public:
  using Work = std::function<Response(Clock::time_point)>;
  struct Request
  {
    Work work;
    Clock::time_point deadline;
    std::promise<Response> promise;
  };
  struct Next
  {
    /// Null while none waits.
    std::shared_ptr<Request> request;
    /// No request comes any more.
    bool closed = false;
  };

  /// Rings `wake` for every queued request.
  explicit RequestQueue(runtime::Doorbell & wake) : wake_(wake) {}

  /// Queues work with a deadline `timeout` from now. Null once closed.
  std::shared_ptr<Request> push(Work work, Clock::duration timeout);
  /// Queues work and blocks until it was answered. A request that has not started `grace` after
  /// its deadline is answered busy, so the caller hears back in time.
  Response call(Work work, Clock::duration timeout, Clock::duration grace);
  /// Queues work ahead of every waiting request and blocks until it was answered. It is never
  /// answered busy, so it waits for the request in flight, whose own deadline bounds that. The work
  /// gets no deadline and sets its own from its start.
  Response call_first(Work work);
  Next pop();
  /// True while a request or the close waits.
  bool waiting();
  /// Rejects every later push.
  void close();

  static Response busy()
  {
    return {false, "Busy, the request could not start before its timeout", true};
  }

private:
  runtime::Doorbell & wake_;
  std::mutex mutex_;
  std::deque<std::shared_ptr<Request>> requests_;
  bool closed_ = false;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__REQUEST_QUEUE_HPP_
