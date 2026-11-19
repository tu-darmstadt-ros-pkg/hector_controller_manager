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

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "hector_controller_manager/entities.hpp"

namespace hector_controller_manager
{
namespace
{
// A plugin library loads on another thread while a leak is recorded, as when a load is abandoned
// during its dlopen. Lock order inversion in the loader would hang both threads.
TEST(RecordLeak, does_not_deadlock_with_a_concurrent_dlopen)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      // Bounds a hang, SIGALRM ends the child
      ::alarm(30);
      // Each copy is a new object, so its dlopen takes the loader's locks as a plugin's first load
      // does. The directory is unique, other jobs on the machine run this test too.
      std::string dir = ::testing::TempDir() + "hcm_record_leak_XXXXXX";
      if (!::mkdtemp(dir.data()))
      {
        std::perror("mkdtemp");
        std::_Exit(2);
      }
      std::vector<std::string> copies;
      for (int i = 0; i < 100; ++i)
      {
        copies.push_back(dir + "/probe_" + std::to_string(i) + ".so");
        std::filesystem::copy_file(LEAK_PROBE_PATH, copies.back());
      }
      std::atomic<bool> done{false};
      std::thread loader(
        [&]
        {
          for (const auto & copy : copies)
          {
            if (!dlopen(copy.c_str(), RTLD_NOW | RTLD_LOCAL))
            {
              std::fprintf(stderr, "%s\n", dlerror());
              std::_Exit(2);
            }
          }
          done = true;
        });
      while (!done)
      {
        record_leak();
      }
      loader.join();
      std::filesystem::remove_all(dir);
      std::_Exit(0);
    },
    ::testing::ExitedWithCode(0), "");
}

}  // namespace
}  // namespace hector_controller_manager
