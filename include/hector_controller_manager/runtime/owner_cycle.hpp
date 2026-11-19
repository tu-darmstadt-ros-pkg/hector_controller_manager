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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__OWNER_CYCLE_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__OWNER_CYCLE_HPP_

#include <atomic>
#include <cstdint>

#include "hector_controller_manager/runtime/thread_role.hpp"

namespace hector_controller_manager::runtime
{
/// The cycle RT triggers on the owner of an async component, at most one outstanding.
///
/// - RT starts a cycle with a release store, only while none is outstanding.
/// - The owner acquires the cycle, runs it and completes it with a release store.
/// - RT acquires the completion before it touches what the cycle used.
/// - A call of the owner under the gate may turn a pending write into a read. The commands RT
///   copied are as old as the call then.
class OwnerCycle
{
public:
  enum class Kind : uint8_t
  {
    None,
    Read,
    /// Writes the commands RT copied, then reads.
    Write,
  };
  /// RT, only while idle.
  void rt_start(bool write) noexcept
  {
    expect_precondition(
      [this] { return kind_.load(std::memory_order_relaxed) == Kind::None; },
      "OwnerCycle::rt_start while a cycle is outstanding");
    kind_.store(write ? Kind::Write : Kind::Read, std::memory_order_release);
  }
  /// RT and owner. The outstanding cycle, None while RT may start one.
  Kind pending() const noexcept { return kind_.load(std::memory_order_acquire); }

  /// Owner.
  void owner_complete() noexcept { kind_.store(Kind::None, std::memory_order_release); }
  void owner_drop_write() noexcept
  {
    Kind pending = Kind::Write;
    kind_.compare_exchange_strong(pending, Kind::Read, std::memory_order_relaxed);
  }

private:
  std::atomic<Kind> kind_{Kind::None};
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__OWNER_CYCLE_HPP_
