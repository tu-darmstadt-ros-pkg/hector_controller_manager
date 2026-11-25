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

#ifndef RUNTIME__LOG_CAPTURE_HPP_
#define RUNTIME__LOG_CAPTURE_HPP_

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include "rcutils/logging.h"

namespace hector_controller_manager::testing
{
/// Collects every log line of the process while it lives, instead of printing it.
class LogCapture
{
public:
  LogCapture() : previous_(rcutils_logging_get_output_handler())
  {
    {
      std::lock_guard<std::mutex> lock(mutex());
      lines().clear();
    }
    rcutils_logging_set_output_handler(&LogCapture::capture);
  }
  ~LogCapture() { rcutils_logging_set_output_handler(previous_); }
  LogCapture(const LogCapture &) = delete;
  LogCapture & operator=(const LogCapture &) = delete;

  bool contains(const std::string & text) const
  {
    std::lock_guard<std::mutex> lock(mutex());
    for (const auto & line : lines())
    {
      if (line.find(text) != std::string::npos)
      {
        return true;
      }
    }
    return false;
  }

  size_t count(const std::string & text) const
  {
    std::lock_guard<std::mutex> lock(mutex());
    return std::count_if(
      lines().begin(), lines().end(),
      [&](const std::string & line) { return line.find(text) != std::string::npos; });
  }

  /// The first captured line that contains `text`, empty if none does.
  std::string first(const std::string & text) const
  {
    std::lock_guard<std::mutex> lock(mutex());
    const auto line = std::find_if(
      lines().begin(), lines().end(),
      [&](const std::string & candidate) { return candidate.find(text) != std::string::npos; });
    return line == lines().end() ? std::string() : *line;
  }

  /// Every captured line, for the message of a failed expectation.
  std::string text() const
  {
    std::lock_guard<std::mutex> lock(mutex());
    std::string result;
    for (const auto & line : lines())
    {
      result += line + "\n";
    }
    return result;
  }

private:
  static std::mutex & mutex()
  {
    static std::mutex instance;
    return instance;
  }
  static std::vector<std::string> & lines()
  {
    static std::vector<std::string> instance;
    return instance;
  }
  static void capture(
    const rcutils_log_location_t *, int, const char * name, rcutils_time_point_value_t,
    const char * format, va_list * args)
  {
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, *args);
    std::lock_guard<std::mutex> lock(mutex());
    lines().emplace_back(std::string(name) + ": " + buffer);
  }

  rcutils_logging_output_handler_t previous_;
};

}  // namespace hector_controller_manager::testing

#endif  // RUNTIME__LOG_CAPTURE_HPP_
