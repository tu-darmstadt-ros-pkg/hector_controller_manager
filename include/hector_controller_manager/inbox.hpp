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

#ifndef HECTOR_CONTROLLER_MANAGER__INBOX_HPP_
#define HECTOR_CONTROLLER_MANAGER__INBOX_HPP_

#include <functional>
#include <mutex>
#include <utility>
#include <vector>

#include "hector_controller_manager/runtime/doorbell.hpp"

namespace hector_controller_manager
{
/// Events from owner threads and late callbacks for the supervisor thread, which drains them.
/// Closed when the supervisor ends, so a late callback never touches a destroyed supervisor.
class Inbox
{
public:
  explicit Inbox(runtime::Doorbell & wake) : wake_(wake) {}

  void post(std::function<void()> event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!closed_)
    {
      events_.push_back(std::move(event));
      wake_.ring();
    }
  }

  std::vector<std::function<void()>> take()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::exchange(events_, {});
  }

  void close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
  }

private:
  std::mutex mutex_;
  std::vector<std::function<void()>> events_;
  bool closed_ = false;
  runtime::Doorbell & wake_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__INBOX_HPP_
