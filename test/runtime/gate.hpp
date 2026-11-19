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

#ifndef RUNTIME__GATE_HPP_
#define RUNTIME__GATE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

namespace hector_controller_manager::testing
{
/// Holds a fake plugin callback until the test opens it. A held callback outlives its deadline for
/// as long as the test needs, where a sleep might return early under load.
class Gate
{
public:
  void close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = false;
  }

  void open()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = true;
    changed_.notify_all();
  }

  /// Called in a callback, returns once the gate is open.
  void pass()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++held_;
    changed_.notify_all();
    changed_.wait(lock, [this] { return open_; });
    --held_;
  }

  /// Waits until a callback is held. The timeout only bounds a failing test.
  bool holds(std::chrono::seconds timeout = std::chrono::seconds(10))
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, timeout, [this] { return held_ > 0; });
  }

private:
  std::mutex mutex_;
  std::condition_variable changed_;
  bool open_ = true;
  int held_ = 0;
};

/// Counts a fake plugin callback while it runs. The controller manager leaks a plugin whose call it
/// gave up on, so a test waits for the count to drop before the state the callback uses goes away.
class InFlight
{
public:
  explicit InFlight(std::atomic<int> & count) : count_(count) { count_.fetch_add(1); }
  ~InFlight() { count_.fetch_sub(1); }
  InFlight(const InFlight &) = delete;
  InFlight & operator=(const InFlight &) = delete;

private:
  std::atomic<int> & count_;
};

/// Timeout of a request the test lets finish. Creating a controller node alone took over two
/// seconds on a loaded CI runner.
constexpr std::chrono::seconds FINISH_TIMEOUT{5};

/// Polls the condition until it held once. The timeout only bounds a failing test.
template <typename Condition>
bool eventually(Condition condition, std::chrono::milliseconds timeout = FINISH_TIMEOUT)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!condition())
  {
    if (std::chrono::steady_clock::now() >= deadline)
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

/// For the message of a failed time bound.
inline int64_t elapsed_ms(std::chrono::steady_clock::time_point begin)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - begin)
    .count();
}

}  // namespace hector_controller_manager::testing

#endif  // RUNTIME__GATE_HPP_
