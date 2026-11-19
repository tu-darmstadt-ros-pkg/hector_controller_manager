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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__HANG_WATCH_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__HANG_WATCH_HPP_

#include <atomic>
#include <optional>

#include "hector_controller_manager/clock.hpp"

namespace hector_controller_manager::runtime
{
/// Tells RT when the owner of a component in use stays busy for longer than the hang timeout, and
/// in which plugin call.
///
/// - The owner names its plugin call with relaxed stores, since the name only goes into the reason.
/// - The owner marks a mode switch with release stores that RT acquires. A switch has a deadline of
///   its own, so it ends the window.
/// - The window is RT's alone and sits off the cache line the owner writes.
class HangWatch
{
public:
  /// Owner. Names the plugin call while it lives, a null name leaves the current one.
  class InCall
  {
  public:
    InCall(HangWatch & watch, const char * name) noexcept : call_(name ? &watch.call_ : nullptr)
    {
      if (call_)
      {
        previous_ = call_->load(std::memory_order_relaxed);
        call_->store(name, std::memory_order_relaxed);
      }
    }
    ~InCall()
    {
      if (call_)
      {
        call_->store(previous_, std::memory_order_relaxed);
      }
    }
    InCall(const InCall &) = delete;
    InCall & operator=(const InCall &) = delete;

  private:
    std::atomic<const char *> * call_;
    const char * previous_ = nullptr;
  };

  void owner_switching(bool switching) noexcept
  {
    switching_.store(switching, std::memory_order_release);
  }

  /// RT. The stall once a busy owner stayed so for longer than the timeout, measured from the first
  /// check that found it busy. A check of an idle or switching owner ends the window.
  std::optional<Clock::duration> rt_check(
    Clock::time_point now, Clock::duration timeout, bool busy) noexcept
  {
    if (!busy || switching_.load(std::memory_order_acquire))
    {
      rt_end_window();
      return std::nullopt;
    }
    if (rt_stalled_since_ == Clock::time_point{})
    {
      rt_stalled_since_ = now;
    }
    if (now - rt_stalled_since_ <= timeout)
    {
      return std::nullopt;
    }
    return now - rt_stalled_since_;
  }
  /// RT. Ends the window without a check, for an owner RT just found free.
  void rt_end_window() noexcept { rt_stalled_since_ = {}; }
  /// The plugin call the owner is in, null outside one.
  const char * call() const noexcept { return call_.load(std::memory_order_relaxed); }

private:
  std::atomic<const char *> call_{nullptr};
  std::atomic<bool> switching_{false};
  /// Since when the checks find the owner busy, zero outside a window.
  alignas(64) Clock::time_point rt_stalled_since_{};
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__HANG_WATCH_HPP_
