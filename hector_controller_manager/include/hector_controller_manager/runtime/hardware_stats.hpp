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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__HARDWARE_STATS_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__HARDWARE_STATS_HPP_

#include <atomic>
#include <cstdint>
#include <optional>

namespace hector_controller_manager::runtime
{
/// Timings and counts of a component. The IO and copying threads write them, diagnostics samples
/// them. Relaxed, a sample is ordered with nothing else, except that the release of the clamp count
/// publishes the interface clamped last.
class HardwareStats
{
public:
  void record_io(bool write, double seconds) noexcept
  {
    (write ? write_seconds_ : read_seconds_).store(seconds, std::memory_order_relaxed);
  }
  void skip() noexcept { skipped_.fetch_add(1, std::memory_order_relaxed); }
  void out_of_range() noexcept { out_of_range_.fetch_add(1, std::memory_order_relaxed); }
  void clamp(uint32_t interface) noexcept
  {
    last_clamped_.store(interface, std::memory_order_relaxed);
    clamped_.fetch_add(1, std::memory_order_release);
  }

  double read_seconds() const noexcept { return read_seconds_.load(std::memory_order_relaxed); }
  double write_seconds() const noexcept { return write_seconds_.load(std::memory_order_relaxed); }
  uint64_t skipped() const noexcept { return skipped_.load(std::memory_order_relaxed); }
  /// Cycles in which a joint was outside its position limits.
  uint64_t out_of_range_cycles() const noexcept
  {
    return out_of_range_.load(std::memory_order_relaxed);
  }
  /// Cycles in which the command limits clamped a command.
  uint64_t clamped() const noexcept { return clamped_.load(std::memory_order_acquire); }
  /// None before the first clamp.
  std::optional<uint32_t> last_clamped() const noexcept
  {
    return clamped() == 0 ? std::nullopt
                          : std::optional(last_clamped_.load(std::memory_order_relaxed));
  }

private:
  std::atomic<double> read_seconds_{0.0};
  std::atomic<double> write_seconds_{0.0};
  std::atomic<uint64_t> skipped_{0};
  std::atomic<uint64_t> clamped_{0};
  std::atomic<uint32_t> last_clamped_{0};
  std::atomic<uint64_t> out_of_range_{0};
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__HARDWARE_STATS_HPP_
