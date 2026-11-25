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

#ifndef HECTOR_CONTROLLER_MANAGER__REQUEST_THREADS_HPP_
#define HECTOR_CONTROLLER_MANAGER__REQUEST_THREADS_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace hector_controller_manager
{
/// One thread per job, safe from any thread. A finished thread is joined when the next one starts.
class RequestThreads
{
public:
  void run(std::function<void()> work);
  /// Joins the threads started so far.
  void join_all();

private:
  struct Thread
  {
    std::thread thread;
    /// Set by the thread when its job returned, read by the next `run` to join it.
    std::shared_ptr<std::atomic<bool>> done;
  };
  std::mutex mutex_;
  std::vector<Thread> threads_;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__REQUEST_THREADS_HPP_
