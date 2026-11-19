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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__STOP_BATCH_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__STOP_BATCH_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace hector_controller_manager::runtime
{
/// Stop requests of RT to the owner of a component, per command interface and in batches, so the
/// owner holds the stops of one failure together.
///
/// - RT counts the requests of each interface and publishes the batch with a release store of the
///   total.
/// - `begun` is the total of the batch RT writes. A counter's release publishes it.
/// - The owner acquires the total, then the counters. It takes the batch only once `begun` shows
///   that RT is not writing a newer one.
/// - The owner acknowledges with a release store of the total it held, which RT acquires before a
///   write.
class StopBatch
{
public:
  /// Before RT and the owner use it.
  void reset(size_t interfaces);

  /// RT.
  void rt_request(uint32_t interface) noexcept;
  /// True if it published new requests, then the owner needs a wake.
  bool rt_publish() noexcept;
  /// A published batch the owner has not held yet.
  bool rt_pending() const noexcept
  {
    return total_.load(std::memory_order_relaxed) != done_.load(std::memory_order_acquire);
  }

  /// Any thread.
  bool pending() const noexcept
  {
    return total_.load(std::memory_order_acquire) != done_.load(std::memory_order_acquire);
  }

  /// Owner. The total of the published batch to hold, with the interfaces requested since the last
  /// one in `requested`. None while nothing is pending or RT is writing a newer batch, whose
  /// publish wakes the owner again.
  std::optional<uint32_t> owner_take(std::vector<uint32_t> & requested);
  /// Once the owner held the batch.
  void owner_done(uint32_t total) noexcept { done_.store(total, std::memory_order_release); }

private:
  std::unique_ptr<std::atomic<uint32_t>[]> requests_;
  size_t size_ = 0;
  std::atomic<uint32_t> total_{0};
  /// The total of the batch RT writes, ahead of total_ until it is published.
  std::atomic<uint32_t> begun_{0};
  std::atomic<uint32_t> done_{0};
  /// Owner only, the counters of the last batch it took.
  std::vector<uint32_t> seen_;
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__STOP_BATCH_HPP_
