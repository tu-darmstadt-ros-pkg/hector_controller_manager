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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__CYCLE_STAMP_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__CYCLE_STAMP_HPP_

#include <atomic>
#include <cstdint>

namespace hector_controller_manager::runtime
{
/// Time and period of RT's last write of a sync component or trigger of an async one.
///
/// - RT stores both with relaxed stores.
/// - An async cycle reads them after it acquired its start, which publishes them.
/// - The owner's write through a block reads them without any order, so time and period may come
///   from different writes there.
class CycleStamp
{
public:
  struct Value
  {
    int64_t time_ns;
    int64_t period_ns;
  };

  /// RT.
  void rt_store(int64_t time_ns, int64_t period_ns) noexcept
  {
    time_ns_.store(time_ns, std::memory_order_relaxed);
    period_ns_.store(period_ns, std::memory_order_relaxed);
  }
  /// Owner.
  Value load() const noexcept
  {
    return {time_ns_.load(std::memory_order_relaxed), period_ns_.load(std::memory_order_relaxed)};
  }

private:
  std::atomic<int64_t> time_ns_{0};
  std::atomic<int64_t> period_ns_{0};
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__CYCLE_STAMP_HPP_
