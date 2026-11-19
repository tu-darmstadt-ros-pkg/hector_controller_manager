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
#include <link.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "hector_controller_manager/entities.hpp"

namespace hector_controller_manager
{
namespace
{
std::atomic<bool> leaked{false};
}  // namespace

void record_leak()
{
  leaked = true;
  std::vector<std::string> libraries;
  dl_iterate_phdr(
    [](dl_phdr_info * info, size_t, void * names)
    {
      // The executable has an empty name, it is never unloaded anyway
      if (info->dlpi_name && info->dlpi_name[0] != '\0')
      {
        static_cast<std::vector<std::string> *>(names)->emplace_back(info->dlpi_name);
      }
      return 0;
    },
    &libraries);
  // A dlopen inside the iteration takes the loader's locks in the order opposite to a dlopen on
  // another thread, both would hang
  for (const auto & library : libraries)
  {
    dlopen(library.c_str(), RTLD_LAZY | RTLD_NOLOAD | RTLD_NODELETE);
  }
}

void exit_process(int code)
{
  if (leaked)
  {
    std::fflush(nullptr);
    std::quick_exit(code);
  }
  std::exit(code);
}

}  // namespace hector_controller_manager
