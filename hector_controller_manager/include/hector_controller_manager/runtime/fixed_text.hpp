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

#ifndef HECTOR_CONTROLLER_MANAGER__RUNTIME__FIXED_TEXT_HPP_
#define HECTOR_CONTROLLER_MANAGER__RUNTIME__FIXED_TEXT_HPP_

#include <array>
#include <atomic>
#include <initializer_list>
#include <string>
#include <string_view>

namespace hector_controller_manager::runtime
{
/// Text in fixed storage, so RT records it without allocating. Its writer publishes it with a
/// release store of its own, a reader acquires that store first. Longer text is cut.
class FixedText
{
public:
  void set(std::initializer_list<std::string_view> parts) noexcept
  {
    size_t size = 0;
    for (const auto part : parts)
    {
      for (const char c : part)
      {
        if (size + 1 < text_.size())
        {
          text_[size++].store(c, std::memory_order_relaxed);
        }
      }
    }
    text_[size].store('\0', std::memory_order_relaxed);
  }

  std::string get() const
  {
    std::string text;
    for (const auto & c : text_)
    {
      const char value = c.load(std::memory_order_relaxed);
      if (value == '\0')
      {
        break;
      }
      text += value;
    }
    return text;
  }

private:
  std::array<std::atomic<char>, 256> text_{};
};

}  // namespace hector_controller_manager::runtime

#endif  // HECTOR_CONTROLLER_MANAGER__RUNTIME__FIXED_TEXT_HPP_
