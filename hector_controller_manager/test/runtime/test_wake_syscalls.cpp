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

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <linux/futex.h>
#include <sys/syscall.h>

#include <atomic>
#include <cstdarg>
#include <thread>

#include "gate.hpp"
#include "hector_controller_manager/runtime/doorbell.hpp"
#include "hector_controller_manager/runtime/owner_thread.hpp"

namespace
{
std::atomic<int> futex_waits{0};
std::atomic<int> futex_wakes{0};
}  // namespace

// Counts the futex calls the library makes. A definition in the executable takes precedence over
// libc's for the shared library too.
extern "C" long syscall(long number, ...)  // NOLINT(runtime/int)
{
  va_list list;
  va_start(list, number);
  long args[6];  // NOLINT(runtime/int)
  for (auto & arg : args)
  {
    arg = va_arg(list, long);  // NOLINT(runtime/int)
  }
  va_end(list);
  if (number == SYS_futex)
  {
    const auto operation = args[1] & FUTEX_CMD_MASK;
    (operation == FUTEX_WAKE ? futex_wakes : futex_waits).fetch_add(1);
  }
  using Real = long (*)(long, ...);  // NOLINT(runtime/int)
  static const auto real = reinterpret_cast<Real>(dlsym(RTLD_NEXT, "syscall"));
  return real(number, args[0], args[1], args[2], args[3], args[4], args[5]);
}

namespace hector_controller_manager::runtime
{
namespace
{
TEST(WakeSyscalls, a_ring_without_a_waiter_makes_no_syscall)
{
  Doorbell bell;
  const int before = futex_wakes.load();
  bell.ring();
  bell.ring();
  EXPECT_EQ(futex_wakes.load(), before);
}

TEST(WakeSyscalls, a_ring_with_a_waiter_wakes_it_with_one_syscall)
{
  Doorbell bell;
  const uint32_t seen = bell.value();
  const int waits = futex_waits.load();
  std::thread waiter(
    [&] { EXPECT_TRUE(bell.wait_until(seen, Clock::now() + std::chrono::seconds(5))); });
  // Sleeps make no futex call, a Gate would
  ASSERT_TRUE(testing::eventually([&] { return futex_waits.load() != waits; }));
  const int before = futex_wakes.load();
  bell.ring();
  waiter.join();
  EXPECT_EQ(futex_wakes.load(), before + 1);
}

TEST(WakeSyscalls, waking_a_busy_owner_makes_no_syscall)
{
  OwnerThread owner;
  std::atomic<bool> running{false};
  std::atomic<bool> release{false};
  const auto job = owner.submit(
    [&]
    {
      running = true;
      testing::eventually([&] { return release.load(); });
    });
  ASSERT_TRUE(testing::eventually([&] { return running.load(); }));
  const int before = futex_wakes.load();
  owner.wake();
  owner.wake();
  EXPECT_EQ(futex_wakes.load(), before);
  release = true;
  EXPECT_TRUE(owner.wait(job, Clock::now() + std::chrono::seconds(5)));
}

}  // namespace
}  // namespace hector_controller_manager::runtime
