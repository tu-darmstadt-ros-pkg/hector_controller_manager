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

#ifndef RUNTIME__URDF_HPP_
#define RUNTIME__URDF_HPP_

#include <set>
#include <string>
#include <vector>

namespace hector_controller_manager::testing
{
/// A joint and the interfaces its ros2_control entry exports.
struct UrdfJoint
{
  std::string name;
  std::vector<std::string> commands;
  std::vector<std::string> states = {"position"};
  /// Revolute within -1 and 1 at 2 rad/s, else continuous.
  bool limited = false;
};

/// A ros2_control system of the plugin test/System.
struct UrdfSystem
{
  std::string name;
  std::vector<UrdfJoint> joints;
  /// More attributes of the ros2_control tag, such as is_async="true".
  std::string attributes = "";
  /// The content of its properties tag.
  std::string properties = "";
};

/// A description of the systems, each joint on a link of its own below one base.
inline std::string urdf_of(const std::vector<UrdfSystem> & systems)
{
  std::string links = "  <link name=\"base\"/>\n";
  std::string joints;
  std::string controls;
  std::set<std::string> described;
  for (const auto & system : systems)
  {
    controls += "  <ros2_control name=\"" + system.name + "\" type=\"system\" " +
                system.attributes + ">\n    <properties>" + system.properties +
                "</properties>\n    <hardware><plugin>test/System</plugin></hardware>\n";
    for (const auto & joint : system.joints)
    {
      controls += "    <joint name=\"" + joint.name + "\">";
      for (const auto & command : joint.commands)
      {
        controls += "<command_interface name=\"" + command + "\"/>";
      }
      for (const auto & state : joint.states)
      {
        controls += "<state_interface name=\"" + state + "\"/>";
      }
      controls += "</joint>\n";
      if (described.insert(joint.name).second)
      {
        links += "  <link name=\"" + joint.name + "_link\"/>\n";
        joints +=
          "  <joint name=\"" + joint.name + "\" type=\"" +
          (joint.limited ? "revolute" : "continuous") + "\"><parent link=\"base\"/><child link=\"" +
          joint.name + "_link\"/>" +
          (joint.limited ? "<limit lower=\"-1\" upper=\"1\" velocity=\"2\" effort=\"10\"/>" : "") +
          "</joint>\n";
      }
    }
    controls += "  </ros2_control>\n";
  }
  return "<?xml version=\"1.0\"?>\n<robot name=\"test\">\n" + links + joints + controls +
         "</robot>";
}

/// A description with one system named "system".
inline std::string urdf(
  const std::vector<UrdfJoint> & joints, const std::string & attributes = "",
  const std::string & properties = "")
{
  return urdf_of({{"system", joints, attributes, properties}});
}

}  // namespace hector_controller_manager::testing

#endif  // RUNTIME__URDF_HPP_
