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

#include "hector_controller_manager/request_queue.hpp"

#include <algorithm>
#include <utility>

namespace hector_controller_manager
{
std::shared_ptr<RequestQueue::Request> RequestQueue::push(Work work, Clock::duration timeout)
{
  auto request = std::make_shared<Request>();
  request->work = std::move(work);
  request->deadline = Clock::now() + timeout;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_)
    {
      return nullptr;
    }
    requests_.push_back(request);
  }
  wake_.ring();
  return request;
}

Response RequestQueue::call(Work work, Clock::duration timeout, Clock::duration grace)
{
  const auto request = push(std::move(work), timeout);
  if (!request)
  {
    return {false, "The controller manager is stopping"};
  }
  auto future = request->promise.get_future();
  if (future.wait_until(request->deadline + grace) == std::future_status::timeout)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (
      const auto it = std::find(requests_.begin(), requests_.end(), request); it != requests_.end())
    {
      requests_.erase(it);
      return busy();
    }
  }
  return future.get();
}

Response RequestQueue::call_first(Work work)
{
  auto request = std::make_shared<Request>();
  request->work = std::move(work);
  request->deadline = Clock::time_point::max();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_)
    {
      return {false, "The controller manager is stopping"};
    }
    requests_.push_front(request);
  }
  wake_.ring();
  return request->promise.get_future().get();
}

RequestQueue::Next RequestQueue::pop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (requests_.empty())
  {
    return {nullptr, closed_};
  }
  auto request = std::move(requests_.front());
  requests_.pop_front();
  return {std::move(request), false};
}

bool RequestQueue::waiting()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !requests_.empty() || closed_;
}

void RequestQueue::close()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
  }
  wake_.ring();
}

}  // namespace hector_controller_manager
