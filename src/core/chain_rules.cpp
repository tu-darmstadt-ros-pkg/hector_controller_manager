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

#include "hector_controller_manager/core/chain_rules.hpp"

#include <algorithm>
#include <stdexcept>

#include "hector_controller_manager/core/names.hpp"

namespace hector_controller_manager::core
{
namespace
{
const Names EMPTY;

const Names & find_or_empty(const std::map<std::string, Names> & edges, const std::string & key)
{
  const auto it = edges.find(key);
  return it == edges.end() ? EMPTY : it->second;
}
}  // namespace

ChainGraph::ChainGraph(const InterfaceIndex & index, const std::map<std::string, Claims> & claims)
{
  for (const auto & [name, claim] : claims)
  {
    writes_[name] = providers_written(index, claim);
    auto & reads = reads_[name];
    for (const auto & interface : claim.state)
    {
      const auto * owner = index.state(interface);
      if (owner && owner->kind == InterfaceOwner::Kind::ExportedState)
      {
        reads.insert(owner->owner);
      }
    }
  }
}

const Names & ChainGraph::written_by(const std::string & controller) const
{
  return find_or_empty(writes_, controller);
}

const Names & ChainGraph::read_by(const std::string & controller) const
{
  return find_or_empty(reads_, controller);
}

Names ChainGraph::users_of(const std::string & controller) const
{
  Names users = writers_of(controller);
  for (const auto & [name, providers] : reads_)
  {
    if (providers.count(controller))
    {
      users.insert(name);
    }
  }
  return users;
}

Names ChainGraph::writers_of(const std::string & controller) const
{
  Names writers;
  for (const auto & [name, providers] : writes_)
  {
    if (providers.count(controller))
    {
      writers.insert(name);
    }
  }
  return writers;
}

void ChainGraph::check_acyclic() const
{
  Names done;
  Names visiting;
  const auto visit = [&](const std::string & name, const auto & self) -> void
  {
    if (done.count(name))
    {
      return;
    }
    if (!visiting.insert(name).second)
    {
      throw std::invalid_argument("Controller chain has a cycle through '" + name + "'");
    }
    for (const auto * edges : {&written_by(name), &read_by(name)})
    {
      for (const auto & provider : *edges)
      {
        if (writes_.count(provider))
        {
          self(provider, self);
        }
      }
    }
    visiting.erase(name);
    done.insert(name);
  };
  for (const auto & [name, unused] : writes_)
  {
    visit(name, visit);
  }
}

std::vector<std::string> ChainGraph::update_order() const
{
  // A writer must update before the provider it writes. A provider should update before its
  // readers, but a writer reading a provider down its chain gets that state from the last cycle.
  std::vector<std::string> order;
  Names done;
  Names pending;
  for (const auto & [name, unused] : writes_)
  {
    pending.insert(name);
  }
  const auto all_done = [&](const Names & names)
  {
    return std::all_of(
      names.begin(), names.end(),
      [&](const auto & n) { return done.count(n) || !writes_.count(n); });
  };
  while (!pending.empty())
  {
    const std::string * next = nullptr;
    for (const bool strict : {true, false})
    {
      for (const auto & name : pending)
      {
        if (all_done(writers_of(name)) && (!strict || all_done(read_by(name))))
        {
          next = &name;
          break;
        }
      }
      if (next)
      {
        break;
      }
    }
    if (!next)
    {
      throw std::invalid_argument(
        "Controller chain has a cycle through '" + *pending.begin() + "'");
    }
    order.push_back(*next);
    done.insert(*next);
    pending.erase(*next);
  }
  return order;
}

Names stop_closure(const ChainGraph & graph, const Names & seeds)
{
  Names stopped;
  std::vector<std::string> pending(seeds.begin(), seeds.end());
  while (!pending.empty())
  {
    const std::string name = pending.back();
    pending.pop_back();
    if (!stopped.insert(name).second)
    {
      continue;
    }
    for (const auto & user : graph.users_of(name))
    {
      pending.push_back(user);
    }
    for (const auto & provider : graph.written_by(name))
    {
      pending.push_back(provider);
    }
  }
  return stopped;
}

Names providers_written(const InterfaceIndex & index, const Claims & claims)
{
  Names providers;
  for (const auto & interface : claims.command)
  {
    if (const auto * owner = index.command(interface);
        owner && owner->kind == InterfaceOwner::Kind::Reference)
    {
      providers.insert(owner->owner);
    }
  }
  return providers;
}

bool claims_hardware(const InterfaceIndex & index, const Claims & claims, const std::string & hw)
{
  bool claimed = false;
  for_each_claim(
    claims, index, [&](const std::string &, const InterfaceOwner & owner)
    { claimed = claimed || (is_hardware_kind(owner.kind) && owner.owner == hw); });
  return claimed;
}

bool commands_hardware(const InterfaceIndex & index, const Claims & claims, const std::string & hw)
{
  return std::any_of(
    claims.command.begin(), claims.command.end(),
    [&](const std::string & interface)
    {
      const auto * owner = index.hardware_command(interface);
      return owner && owner->owner == hw;
    });
}

Names claimers_of_hardware(
  const Model & model, const InterfaceIndex & index, const std::string & hw)
{
  Names claimers;
  for (const auto & [name, controller] : model.controllers)
  {
    if (claims_hardware(index, controller.claims, hw))
    {
      claimers.insert(name);
    }
  }
  return claimers;
}

Names claimers_holding(const Model & model, const InterfaceIndex & index, const std::string & hw)
{
  Names claimers;
  for (const auto & name : claimers_of_hardware(model, index, hw))
  {
    if (holds_claims(model.controllers.at(name)))
    {
      claimers.insert(name);
    }
  }
  return claimers;
}

}  // namespace hector_controller_manager::core
