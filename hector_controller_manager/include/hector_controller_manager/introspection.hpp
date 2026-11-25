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

#ifndef HECTOR_CONTROLLER_MANAGER__INTROSPECTION_HPP_
#define HECTOR_CONTROLLER_MANAGER__INTROSPECTION_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include "hector_controller_manager/runtime/io_gate.hpp"
#include "pal_statistics/pal_statistics.hpp"
#include "rclcpp/node.hpp"

namespace hector_controller_manager
{
/// The registry that controllers and hardware register their introspection in. hardware_interface
/// fixes its key, so it is one per process, shared by every controller manager of the process.
///
/// - A node of its own publishes it, `ros2_control` in the namespace of the first manager, so no
///   manager's end leaves the registry without its publisher.
/// - Never destroyed, as its publish thread would race the static destructors. The process
///   therefore counts as leaking and `exit_process` skips them.
/// - A sample reads the values of every manager, and the RT loops of two managers cannot hold off
///   each other's writers without waiting. While more than one manager lives, `gate` is held as
///   owner, so RT samples nothing.
class Introspection
{
public:
  /// Counts its manager as one of the process while it lives.
  class Member
  {
  public:
    explicit Member(Introspection & introspection) : introspection_(introspection) {}
    ~Member() { introspection_.leave(); }
    Member(const Member &) = delete;
    Member & operator=(const Member &) = delete;

  private:
    Introspection & introspection_;
  };

  /// Off RT, before `manager` creates a plugin. The first creates the registry. Waits for a sample
  /// that runs.
  static std::unique_ptr<Member> join(rclcpp::Node & manager);
  /// Null before the first join.
  static Introspection * get() noexcept;

  pal_statistics::StatisticsRegistry & registry() { return *registry_; }
  /// The resolved topic it is published on.
  std::string topic() const;
  /// A sample enters it as RT, so it skips while an owner holds it.
  runtime::IoGate & gate() { return gate_; }

private:
  explicit Introspection(rclcpp::Node & manager);
  void leave();

  const rclcpp::Node::SharedPtr node_;
  const std::shared_ptr<pal_statistics::StatisticsRegistry> registry_;
  runtime::IoGate gate_;
  std::mutex mutex_;
  size_t members_ = 0;
};

}  // namespace hector_controller_manager

#endif  // HECTOR_CONTROLLER_MANAGER__INTROSPECTION_HPP_
