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

#ifndef HECTOR_CONTROLLER_MANAGER__LEAK_HPP_
#define HECTOR_CONTROLLER_MANAGER__LEAK_HPP_

#include <memory>
#include <string>
#include <utility>

namespace hector_controller_manager
{
/// Marks the loaded library at `path` as never unloaded, so its dependencies stay mapped too. False
/// if it is not loaded. Not inside dl_iterate_phdr, see record_leak.
bool pin_library(const std::string & path);

/// Marks every library loaded now as never unloaded and the process as leaking. A leaked plugin may
/// run code of any of them until the process ends, its own library and the middleware its node uses
/// included. Blocks while an abandoned load hangs in a static constructor of its library, because
/// dlopen holds the loader lock during constructors.
void record_leak();

/// The runtime of an entity whose callback still runs is never destroyed under its thread.
template <typename T>
void leak(std::shared_ptr<T> runtime)
{
  record_leak();
  new std::shared_ptr<T>(std::move(runtime));
}

/// Ends the process with `code`. After a leak it skips static destructors, which would destroy
/// what the leaked plugin's threads still use.
[[noreturn]] void exit_process(int code);

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__LEAK_HPP_
