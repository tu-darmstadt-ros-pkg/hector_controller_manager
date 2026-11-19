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

namespace hector_controller_manager::runtime
{
/// Mutual exclusion between RT read and write of a sync component and every other call of it on
/// its owner thread. RT never waits. While the owner holds the gate, RT skips the component, so a
/// blocking plugin call never stalls the control loop. The owner waits until RT left a running
/// read or write.
///
/// Both sides change one word with read modify write operations, so their order is total. Either
/// RT entered first and the owner waits for it, or the owner announced itself first and RT skips.
class IoGate
{
public:
  /// Called by RT, true if the read or write may run. Every true return needs an rt_leave.
  bool rt_enter() noexcept
  {
    const uint32_t before = bits_.fetch_or(RT_IN_IO, std::memory_order_acq_rel);
    if (before & OWNER)
    {
      rt_leave();
      return false;
    }
    return true;
  }

  void rt_leave() noexcept
  {
    const uint32_t before = bits_.fetch_and(~RT_IN_IO, std::memory_order_acq_rel);
    if (before & OWNER)
    {
      left_.ring();
    }
  }

  /// For the owner, waits until RT left. Returns false at the deadline, then the gate is not held.
  bool owner_acquire(Clock::time_point deadline) noexcept
  {
    bits_.fetch_or(OWNER, std::memory_order_acq_rel);
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

  void owner_release() noexcept { bits_.fetch_and(~OWNER, std::memory_order_acq_rel); }

private:
  static constexpr uint32_t RT_IN_IO = 1;
  static constexpr uint32_t OWNER = 2;

  std::atomic<uint32_t> bits_{0};
  Doorbell left_;
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__IO_GATE_HPP_
