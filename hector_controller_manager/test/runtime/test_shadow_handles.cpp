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

#include <memory>

#include "gate.hpp"
#include "hector_controller_manager/runtime/shadow_handles.hpp"

namespace hector_controller_manager::runtime
{
namespace
{
// A plugin thread that holds a state handle, such as a driver writing it, keeps the shadows as they
// were, so a controller never sees part of a read
TEST(ShadowHandles, a_busy_plugin_state_copies_no_state)
{
  auto j1 = std::make_shared<hardware_interface::StateInterface>("j1", "position");
  auto j2 = std::make_shared<hardware_interface::StateInterface>("j2", "position");
  ASSERT_TRUE(j1->set_value(1.0, true));
  ASSERT_TRUE(j2->set_value(1.0, true));
  hardware_interface::HardwareInfo info;
  info.name = "bus";
  info.is_async = true;
  info.rw_rate = 0;
  HardwareStats stats;
  ShadowHandles shadows(stats);
  shadows.init(info, {j1, j2}, {}, false);
  ASSERT_TRUE(j1->set_value(2.0, true));
  ASSERT_TRUE(j2->set_value(2.0, true));
  {
    const testing::HeldLock busy(j2->get_mutex());
    EXPECT_FALSE(shadows.copy_states());
  }
  EXPECT_EQ(shadows.states()[0]->get_optional(), 1.0);
  EXPECT_EQ(shadows.states()[1]->get_optional(), 1.0);
  EXPECT_TRUE(shadows.copy_states());
  EXPECT_EQ(shadows.states()[0]->get_optional(), 2.0);
}

}  // namespace
}  // namespace hector_controller_manager::runtime
