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
#include <cstddef>
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

namespace
{
void * allocate(std::size_t size, std::size_t alignment)
{
  hector_controller_manager::testing::count_allocation();
  void * memory = nullptr;
  if (alignment <= alignof(std::max_align_t))
  {
    memory = std::malloc(size == 0 ? 1 : size);
  }
  else if (posix_memalign(&memory, alignment, size == 0 ? 1 : size) != 0)
  {
    memory = nullptr;
  }
  if (!memory)
  {
    throw std::bad_alloc();
  }
  return memory;
}

void * allocate_nothrow(std::size_t size, std::size_t alignment) noexcept
{
  try
  {
    return allocate(size, alignment);
  }
  catch (const std::bad_alloc &)
  {
    return nullptr;
  }
}

constexpr std::size_t DEFAULT = alignof(std::max_align_t);
}  // namespace

void * operator new(std::size_t size) { return allocate(size, DEFAULT); }
void * operator new[](std::size_t size) { return allocate(size, DEFAULT); }
void * operator new(std::size_t size, const std::nothrow_t &) noexcept
{
  return allocate_nothrow(size, DEFAULT);
}
void * operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
  return allocate_nothrow(size, DEFAULT);
}
void * operator new(std::size_t size, std::align_val_t alignment)
{
  return allocate(size, static_cast<std::size_t>(alignment));
}
void * operator new[](std::size_t size, std::align_val_t alignment)
{
  return allocate(size, static_cast<std::size_t>(alignment));
}
void * operator new(std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  return allocate_nothrow(size, static_cast<std::size_t>(alignment));
}
void * operator new[](std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  return allocate_nothrow(size, static_cast<std::size_t>(alignment));
}

void operator delete(void * memory) noexcept { std::free(memory); }
void operator delete[](void * memory) noexcept { std::free(memory); }
void operator delete(void * memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void * memory, std::size_t) noexcept { std::free(memory); }
void operator delete(void * memory, std::align_val_t) noexcept { std::free(memory); }
void operator delete[](void * memory, std::align_val_t) noexcept { std::free(memory); }
void operator delete(void * memory, std::size_t, std::align_val_t) noexcept { std::free(memory); }
void operator delete[](void * memory, std::size_t, std::align_val_t) noexcept { std::free(memory); }

#endif
