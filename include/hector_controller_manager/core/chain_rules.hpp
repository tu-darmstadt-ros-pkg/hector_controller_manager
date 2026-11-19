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

#ifndef HECTOR_CONTROLLER_MANAGER__CORE__CHAIN_RULES_HPP_
#define HECTOR_CONTROLLER_MANAGER__CORE__CHAIN_RULES_HPP_

#include <map>
#include <string>
#include <vector>

#include "hector_controller_manager/core/model.hpp"

namespace hector_controller_manager::core
{
/// Chain edges between a set of controllers with given claims.
class ChainGraph
{
public:
  ChainGraph(const InterfaceIndex & index, const std::map<std::string, Claims> & claims);

  /// Controllers whose references the controller claims.
  const Names & written_by(const std::string & controller) const;
  /// Controllers claiming references or exported states of the controller.
  Names users_of(const std::string & controller) const;
  /// Controllers claiming references of the controller. Its chained mode follows them.
  Names writers_of(const std::string & controller) const;
  /// Throws on a cycle through written or read providers.
  void check_acyclic() const;
  /// The order RT updates in, a writer before the provider it writes, so references arrive in the
  /// same cycle, and a provider before its readers, so exported states do. Throws on a cycle.
  std::vector<std::string> update_order() const;

private:
  /// Controllers whose exported states the controller claims.
  const Names & read_by(const std::string & controller) const;

  std::map<std::string, Names> writes_;
  std::map<std::string, Names> reads_;
};

/// Everything that cannot keep running once the seeds stop, the seeds, every writer and reader of a
/// stopped controller, and every provider a stopped controller writes. The same rule decides the
/// failure scope and the restart scope.
Names stop_closure(const ChainGraph & graph, const Names & seeds);

/// Calls `visit(interface, owner)` for every claimed interface the index knows.
template <typename Visit>
void for_each_claim(const Claims & claims, const InterfaceIndex & index, Visit visit)
{
  for (const auto & interface : claims.command)
  {
    if (const auto * owner = index.command(interface))
    {
      visit(interface, *owner);
    }
  }
  for (const auto & interface : claims.state)
  {
    if (const auto * owner = index.state(interface))
    {
      visit(interface, *owner);
    }
  }
}

/// Controllers whose references the claims write.
Names providers_written(const InterfaceIndex & index, const Claims & claims);

/// The claims include a command or state interface of the hardware.
bool claims_hardware(const InterfaceIndex & index, const Claims & claims, const std::string & hw);

/// Controllers of the model that claim an interface of the hardware.
Names claimers_of_hardware(
  const Model & model, const InterfaceIndex & index, const std::string & hw);

}  // namespace hector_controller_manager::core

#endif  // HECTOR_CONTROLLER_MANAGER__CORE__CHAIN_RULES_HPP_
