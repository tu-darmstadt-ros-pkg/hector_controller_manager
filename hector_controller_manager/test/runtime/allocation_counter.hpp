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

#ifndef RUNTIME__ALLOCATION_COUNTER_HPP_
#define RUNTIME__ALLOCATION_COUNTER_HPP_

#include <cstdint>

namespace hector_controller_manager::testing
{
/// Counts every form of operator new called by this thread while it lives and `enabled` is set.
class CountAllocations
{
public:
  explicit CountAllocations(bool enabled);
  ~CountAllocations();

private:
  bool enabled_;
};

uint64_t counted_allocations();
bool allocation_counting_available();

}  // namespace hector_controller_manager::testing

#endif  // RUNTIME__ALLOCATION_COUNTER_HPP_
