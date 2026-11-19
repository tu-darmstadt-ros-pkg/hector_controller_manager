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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__IO_GATE_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__IO_GATE_HPP_

#include <atomic>
#include <cstdint>

#include "hector_controller_manager/runtime/doorbell.hpp"
#include "hector_controller_manager/runtime/thread_role.hpp"

namespace hector_controller_manager::runtime
{
/// Mutual exclusion between RT's read, write and handle copies of a component and every plugin call
/// on its owner thread. RT never waits. While the owner holds the gate, RT skips the component, so
/// a blocking plugin call never stalls the control loop. The owner waits until RT left. Owners
/// count, so the owner threads of all controllers can share one gate.
///
/// Both sides change one word with read modify write operations, so their order is total. Either
/// RT entered first and the owner waits for it, or the owner announced itself first and RT skips.
class IoGate
{
public:
  /// Called by RT, true if the read or write may run. Every true return needs an rt_leave.
  [[nodiscard]] bool rt_enter() noexcept
  {
    const uint32_t before = bits_.fetch_or(RT_IN_IO, std::memory_order_acq_rel);
    if (before >= OWNER)
    {
      rt_leave();
      return false;
    }
    return true;
  }

  void rt_leave() noexcept
  {
    const uint32_t before = bits_.fetch_and(~RT_IN_IO, std::memory_order_acq_rel);
    expect_precondition([before] { return before & RT_IN_IO; }, "IoGate::rt_leave without RT in");
    if (before >= OWNER)
    {
      left_.ring();
    }
  }

  /// For the owner, waits until RT left. Returns false at the deadline, then the gate is not held.
  [[nodiscard]] bool owner_acquire(Clock::time_point deadline) noexcept
  {
    bits_.fetch_add(OWNER, std::memory_order_acq_rel);
    while (true)
    {
      const uint32_t seen = left_.value();
      if (!(bits_.load(std::memory_order_acquire) & RT_IN_IO))
      {
        return true;
      }
      if (!left_.wait_until(seen, deadline) && (bits_.load(std::memory_order_acquire) & RT_IN_IO))
      {
        owner_release();
        return false;
      }
    }
  }

  void owner_release() noexcept
  {
    const uint32_t before = bits_.fetch_sub(OWNER, std::memory_order_acq_rel);
    expect_precondition(
      [before] { return before >= OWNER; }, "IoGate::owner_release without an owner in");
  }

  /// Whether an owner holds or waits for the gate.
  bool owner_in() const noexcept { return bits_.load(std::memory_order_acquire) >= OWNER; }

private:
  static constexpr uint32_t RT_IN_IO = 1;
  /// One owner, the count of owners above RT's bit.
  static constexpr uint32_t OWNER = 2;

  std::atomic<uint32_t> bits_{0};
  Doorbell left_;
};

/// Holds a gate on an owner thread while it lives, however long RT stays inside.
class OwnerHold
{
public:
  explicit OwnerHold(IoGate & gate) noexcept : gate_(gate)
  {
    (void)gate_.owner_acquire(Clock::time_point::max());
  }
  ~OwnerHold() { gate_.owner_release(); }
  OwnerHold(const OwnerHold &) = delete;
  OwnerHold & operator=(const OwnerHold &) = delete;

private:
  IoGate & gate_;
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__IO_GATE_HPP_
