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

#include "hector_controller_manager/runtime/stop_batch.hpp"

#include "hector_controller_manager/runtime/thread_role.hpp"

namespace hector_controller_manager::runtime
{
void StopBatch::reset(size_t interfaces)
{
  expect_precondition(
    [this] { return total_.load() == 0 && begun_.load() == 0 && done_.load() == 0; },
    "StopBatch::reset after RT or the owner used it");
  requests_ = std::make_unique<std::atomic<uint32_t>[]>(interfaces);
  size_ = interfaces;
  seen_.assign(interfaces, 0);
}

void StopBatch::rt_request(uint32_t interface) noexcept
{
  const uint32_t total = total_.load(std::memory_order_relaxed);
  if (begun_.load(std::memory_order_relaxed) == total)
  {
    begun_.store(total + 1, std::memory_order_relaxed);
  }
  // Releases the begun total, so an owner that sees this request also sees its open batch
  requests_[interface].fetch_add(1, std::memory_order_release);
}

bool StopBatch::rt_publish() noexcept
{
  const uint32_t begun = begun_.load(std::memory_order_relaxed);
  if (begun == total_.load(std::memory_order_relaxed))
  {
    return false;
  }
  total_.store(begun, std::memory_order_release);
  return true;
}

std::optional<uint32_t> StopBatch::owner_take(std::vector<uint32_t> & requested)
{
  const uint32_t total = total_.load(std::memory_order_acquire);
  if (total == done_.load(std::memory_order_relaxed))
  {
    return std::nullopt;
  }
  std::vector<uint32_t> requests(size_);
  for (size_t i = 0; i < size_; ++i)
  {
    // Acquires the begun total RT stored before it bumped the counter
    requests[i] = requests_[i].load(std::memory_order_acquire);
  }
  if (begun_.load(std::memory_order_relaxed) != total)
  {
    return std::nullopt;
  }
  for (uint32_t i = 0; i < size_; ++i)
  {
    if (requests[i] != seen_[i])
    {
      seen_[i] = requests[i];
      requested.push_back(i);
    }
  }
  return total;
}

}  // namespace hector_controller_manager::runtime
