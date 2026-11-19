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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__WRITE_BLOCK_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__WRITE_BLOCK_HPP_

#include <atomic>
#include <cstdint>

namespace hector_controller_manager::runtime
{
/// Blocks the writes of a component for one lifecycle generation, so the block expires at the next
/// transition. The supervisor blocks and unblocks with release stores, RT and an async owner
/// acquire the word before every write.
class WriteBlock
{
public:
  void block(uint64_t generation) noexcept
  {
    generation_.store(generation, std::memory_order_release);
  }
  void unblock() noexcept { generation_.store(0, std::memory_order_release); }
  bool blocks(uint64_t generation) const noexcept
  {
    return generation_.load(std::memory_order_acquire) == generation;
  }

private:
  /// 0 while unblocked, generations start at 1.
  std::atomic<uint64_t> generation_{0};
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__WRITE_BLOCK_HPP_
