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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__DOORBELL_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__DOORBELL_HPP_

#include <atomic>
#include <cstdint>

#include "hector_controller_manager/clock.hpp"

namespace hector_controller_manager::runtime
{
/// A wakeup counter on a futex. Ringing is safe on RT, it is one atomic increment and a non
/// blocking wake syscall only while somebody waits.
class Doorbell
{
public:
  /// Read before checking the condition, then wait with it, so no ring is lost.
  uint32_t value() const noexcept { return sequence_.load(std::memory_order_seq_cst); }

  void ring() noexcept;

  /// Waits until the bell rang after `seen` was read. Returns false at the deadline.
  bool wait_until(uint32_t seen, Clock::time_point deadline) noexcept;

private:
  /// Bumped by every ring. Sequentially consistent with `waiters_`, so a ringer either sees a
  /// waiter or the waiter sees the bump before it sleeps.
  std::atomic<uint32_t> sequence_{0};
  /// Threads in wait_until, a ring without one skips the syscall.
  std::atomic<uint32_t> waiters_{0};
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__DOORBELL_HPP_
