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

#include "hector_controller_manager/runtime/doorbell.hpp"

#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <climits>
#include <ctime>

namespace hector_controller_manager::runtime
{
static_assert(std::atomic<uint32_t>::is_always_lock_free);
static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t));

namespace
{
uint32_t * address(std::atomic<uint32_t> & word) { return reinterpret_cast<uint32_t *>(&word); }
}  // namespace

void Doorbell::ring() noexcept
{
  sequence_.fetch_add(1, std::memory_order_seq_cst);
  if (waiters_.load(std::memory_order_seq_cst) > 0)
  {
    syscall(SYS_futex, address(sequence_), FUTEX_WAKE_PRIVATE, INT_MAX, nullptr, nullptr, 0);
  }
}

bool Doorbell::wait_until(uint32_t seen, Clock::time_point deadline) noexcept
{
  waiters_.fetch_add(1, std::memory_order_seq_cst);
  while (sequence_.load(std::memory_order_seq_cst) == seen)
  {
    const auto left = deadline - Clock::now();
    if (left <= Clock::duration::zero())
    {
      break;
    }
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(left);
    const timespec timeout{
      static_cast<time_t>(seconds.count()),
      static_cast<long>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(left - seconds).count())};
    syscall(SYS_futex, address(sequence_), FUTEX_WAIT_PRIVATE, seen, &timeout, nullptr, 0);
  }
  waiters_.fetch_sub(1, std::memory_order_seq_cst);
  return sequence_.load(std::memory_order_seq_cst) != seen;
}

}  // namespace hector_controller_manager::runtime
