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

#include "hector_controller_manager/request_threads.hpp"

#include <utility>

namespace hector_controller_manager
{
void RequestThreads::run(std::function<void()> work)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::erase_if(
    threads_,
    [](Thread & request)
    {
      if (!request.done->load())
      {
        return false;
      }
      request.thread.join();
      return true;
    });
  auto done = std::make_shared<std::atomic<bool>>(false);
  threads_.push_back(
    {std::thread(
       [work = std::move(work), done]
       {
         work();
         done->store(true);
       }),
     done});
}

void RequestThreads::join_all()
{
  std::vector<Thread> threads;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    threads.swap(threads_);
  }
  for (auto & request : threads)
  {
    request.thread.join();
  }
}

}  // namespace hector_controller_manager
