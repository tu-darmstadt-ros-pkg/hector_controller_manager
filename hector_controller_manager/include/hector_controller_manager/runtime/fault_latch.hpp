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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__FAULT_LATCH_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__FAULT_LATCH_HPP_

#include <atomic>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <string_view>

#include "hector_controller_manager/core/model.hpp"
#include "hector_controller_manager/runtime/fixed_text.hpp"

namespace hector_controller_manager::runtime
{
/// The first fault of a lifecycle generation and its reason.
///
/// One thread raises at a time, any thread reads. The IO fault of sync hardware is raised by RT and
/// by the owner's write through a block, which the IoGate keeps apart. The raise records the
/// reason, then publishes the fault with a release store of the word, which a reader acquires
/// before it reads the reason. The word holds the generation, so a fault of an older one reads as
/// none and a transition clears the latch.
class FaultLatch
{
public:
  /// The writer. Keeps the first fault of the generation and its reason, allocates nothing.
  void raise(
    uint64_t generation, core::HardwareFault kind,
    std::initializer_list<std::string_view> parts) noexcept
  {
    if (in(word_.load(std::memory_order_relaxed), generation) != core::HardwareFault::None)
    {
      return;
    }
    reason_.set(parts);
    word_.store((generation << KIND_BITS) | static_cast<uint64_t>(kind), std::memory_order_release);
  }

  core::HardwareFault get(uint64_t generation) const noexcept
  {
    return in(word_.load(std::memory_order_acquire), generation);
  }

  /// The reason of the last raise, complete once get found its fault.
  std::string reason() const { return reason_.get(); }

private:
  static constexpr int KIND_BITS = 2;
  static_assert(static_cast<int>(core::HardwareFault::Deactivate) < (1 << KIND_BITS));

  static core::HardwareFault in(uint64_t word, uint64_t generation) noexcept
  {
    return (word >> KIND_BITS) == generation
             ? static_cast<core::HardwareFault>(word & ((1u << KIND_BITS) - 1))
             : core::HardwareFault::None;
  }

  std::atomic<uint64_t> word_{0};
  FixedText reason_;
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__FAULT_LATCH_HPP_
