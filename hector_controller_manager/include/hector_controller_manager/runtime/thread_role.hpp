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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__THREAD_ROLE_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__THREAD_ROLE_HPP_

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace hector_controller_manager::runtime
{
/// Debug and sanitizer builds abort on a broken invariant and on a call off its thread.
constexpr bool CHECKED_BUILD =
#if !defined(NDEBUG) || defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
  true;
#else
  false;
#endif

/// Aborts in checked builds unless the caller runs on the thread of `role`.
inline void expect_thread(bool on_role_thread, const char * role) noexcept
{
  if (CHECKED_BUILD && !on_role_thread)
  {
    std::fprintf(stderr, "Called off the %s thread\n", role);
    std::abort();
  }
}

/// Aborts in checked builds unless `holds()`. Release builds never call it, so a check of an atomic
/// costs nothing there.
template <typename Condition>
void expect_precondition(Condition holds, const char * what) noexcept
{
  if constexpr (CHECKED_BUILD)
  {
    if (!holds())
    {
      std::fprintf(stderr, "Broken precondition: %s\n", what);
      std::abort();
    }
  }
}

/// The one thread that acts in a role, such as the supervisor or RT. While no thread holds it,
/// for example before the supervisor thread starts, any thread may act.
class ThreadRole
{
public:
  explicit ThreadRole(const char * name) : name_(name) {}

  void bind() noexcept { id_.store(std::this_thread::get_id()); }
  void unbind() noexcept { id_.store(std::thread::id()); }
  /// Holds the role for one call. Calls may come from changing threads, a simulator runs the
  /// control cycle from two of its own, but they never overlap.
  class Call
  {
  public:
    explicit Call(ThreadRole & role) noexcept : role_(role)
    {
      if constexpr (CHECKED_BUILD)
      {
        std::thread::id none;
        expect_thread(
          role_.id_.compare_exchange_strong(none, std::this_thread::get_id()), role_.name_);
      }
    }
    ~Call()
    {
      if constexpr (CHECKED_BUILD)
      {
        role_.unbind();
      }
    }
    Call(const Call &) = delete;
    Call & operator=(const Call &) = delete;

  private:
    ThreadRole & role_;
  };
  void expect() const noexcept
  {
    if constexpr (CHECKED_BUILD)
    {
      const std::thread::id holder = id_.load();
      expect_thread(holder == std::thread::id() || holder == std::this_thread::get_id(), name_);
    }
  }

private:
  const char * name_;
  std::atomic<std::thread::id> id_{};
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__THREAD_ROLE_HPP_
