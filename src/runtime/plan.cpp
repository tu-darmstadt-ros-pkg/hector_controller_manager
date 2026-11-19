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

#include "hector_controller_manager/runtime/plan.hpp"

namespace hector_controller_manager::runtime
{
void PlanSlot::publish(const Plan * plan) noexcept
{
  published_.store(plan, std::memory_order_seq_cst);
}

template <typename Reached>
bool PlanSlot::wait_for(Reached reached, uint32_t seen, Clock::time_point deadline) noexcept
{
  waiting_.fetch_add(1, std::memory_order_seq_cst);
  bool result = reached() || (progress_.wait_until(seen, deadline), reached());
  waiting_.fetch_sub(1, std::memory_order_seq_cst);
  return result;
}

bool PlanSlot::quiescent(uint64_t generation) const noexcept
{
  const uint64_t state = rt_state_.load(std::memory_order_seq_cst);
  return !(state & 1) || (state >> 1) >= generation;
}

bool PlanSlot::wait_quiescent(uint64_t generation, Clock::time_point deadline) noexcept
{
  while (!wait_for([&] { return quiescent(generation); }, progress_.value(), deadline))
  {
    if (Clock::now() >= deadline)
    {
      return false;
    }
  }
  return true;
}

bool PlanSlot::wait_updated(uint64_t generation, Clock::time_point deadline) noexcept
{
  return wait_for(
    [&] { return rt_updated_.load(std::memory_order_seq_cst) >= generation; }, progress_.value(),
    deadline);
}

void PlanSlot::wait_quiescent_or_ring(
  uint64_t generation, uint32_t seen, Clock::time_point deadline) noexcept
{
  wait_for([&] { return quiescent(generation); }, seen, deadline);
}

void PlanSlot::ring_if_waiting() noexcept
{
  if (waiting_.load(std::memory_order_seq_cst) > 0)
  {
    progress_.ring();
  }
}

const Plan * PlanSlot::rt_enter() noexcept
{
  rt_state_.store((rt_generation_ << 1) | 1, std::memory_order_seq_cst);
  const Plan * published = published_.load(std::memory_order_seq_cst);
  // Compared by generation, a new plan may reuse the address of a freed one
  const uint64_t generation = published ? published->generation : 0;
  if (generation != rt_generation_)
  {
    rt_adopted_ = published;
    rt_generation_ = generation;
    rt_state_.store((rt_generation_ << 1) | 1, std::memory_order_seq_cst);
    ring_if_waiting();
  }
  return rt_adopted_;
}

void PlanSlot::rt_leave() noexcept
{
  rt_state_.store(rt_generation_ << 1, std::memory_order_seq_cst);
  ring_if_waiting();
}

void PlanSlot::rt_updated() noexcept
{
  if (rt_updated_.load(std::memory_order_relaxed) != rt_generation_)
  {
    rt_updated_.store(rt_generation_, std::memory_order_seq_cst);
    ring_if_waiting();
  }
}

}  // namespace hector_controller_manager::runtime
