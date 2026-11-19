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

#include "allocation_counter.hpp"

#include <atomic>
#include <cstdlib>
#include <new>

#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define COUNT_ALLOCATIONS 0
#else
#define COUNT_ALLOCATIONS 1
#endif

namespace hector_controller_manager::testing
{
namespace
{
thread_local int depth = 0;
std::atomic<uint64_t> allocations{0};
}  // namespace

CountAllocations::CountAllocations(bool enabled) : enabled_(enabled) { depth += enabled_ ? 1 : 0; }

CountAllocations::~CountAllocations() { depth -= enabled_ ? 1 : 0; }

uint64_t counted_allocations() { return allocations.load(); }

bool allocation_counting_available() { return COUNT_ALLOCATIONS != 0; }

void count_allocation()
{
  if (depth > 0)
  {
    allocations.fetch_add(1, std::memory_order_relaxed);
  }
}

}  // namespace hector_controller_manager::testing

#if COUNT_ALLOCATIONS

void * operator new(std::size_t size)
{
  hector_controller_manager::testing::count_allocation();
  if (void * memory = std::malloc(size == 0 ? 1 : size))
  {
    return memory;
  }
  throw std::bad_alloc();
}

void operator delete(void * memory) noexcept { std::free(memory); }

void operator delete(void * memory, std::size_t) noexcept { std::free(memory); }

#endif
