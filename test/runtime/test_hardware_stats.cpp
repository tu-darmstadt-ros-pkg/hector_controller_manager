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

#include <gtest/gtest.h>

#include "hector_controller_manager/runtime/hardware_stats.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
TEST(HardwareStats, read_and_write_times_go_to_their_own_statistic)
{
  HardwareStats stats;
  stats.record_io(false, 0.25);
  stats.record_io(true, 0.5);
  EXPECT_EQ(stats.read_seconds(), 0.25);
  EXPECT_EQ(stats.write_seconds(), 0.5);
}

}  // namespace
}  // namespace hector_controller_manager::runtime
