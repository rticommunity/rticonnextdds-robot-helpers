// Copyright 2021 Real-Time Innovations, Inc. (RTI)
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

#include <string>

#include "ros2dds/resolve.hpp"

namespace ros2dds
{
std::string
resolve_topic_name(
  const std::string & node_name,
  const std::string & node_namespace,
  const std::string & topic_name,
  const TopicKind topic_kind,
  const bool use_ros_conventions)
{
  static const char * topic_prefixes[] = {
    "rt",  // RosTopic
    "rq",  // RosRequest
    "rr"   // RosReply
  };
  static const char * topic_suffixes[] = {
    "",         // RosTopic
    "Request",  // RosRequest
    "Reply"     // RosReply
  };
  if (topic_name.length() == 0) {
    throw rclcpp::exceptions::InvalidTopicNameError(
            topic_name.c_str(), "topic name cannot be empty", 0);
  }
  std::string suffix = topic_suffixes[static_cast<int>(topic_kind)];
  if (!use_ros_conventions) {
    return topic_name + suffix;
  } else {
    const bool is_service = topic_kind != TopicKind::Topic;
    std::string prefix = topic_prefixes[static_cast<int>(topic_kind)];
    std::string fq_name = rclcpp::expand_topic_or_service_name(
      topic_name, node_name, node_namespace, is_service);
    return prefix + fq_name + suffix;
  }
}

std::string
resolve_type_name(
  const std::string & type_name,
  const bool use_ros_conventions)
{
  if (!use_ros_conventions) {
    return type_name;
  }
  std::ostringstream ss;
  const size_t base_name_pos = type_name.rfind("::");
  std::string type_base_name;
  std::string type_ns;
  if (base_name_pos != std::string::npos) {
    type_base_name = type_name.substr(base_name_pos + 2);
    type_ns = type_name.substr(0, base_name_pos);
  } else {
    type_base_name = type_name;
  }
  const std::string dds_ns = "dds_";
  const std::string dds_ns_sfx = "::" + dds_ns;
  if (!type_ns.empty()) {
    if (type_ns == dds_ns) {
      ss << type_ns;
    } else if (type_ns.length() > dds_ns.length()) {
      ss << type_ns;
      const size_t dds_ns_pos = type_ns.rfind(dds_ns_sfx);
      if (dds_ns_pos != std::string::npos) {
        if (type_ns.substr(dds_ns_pos).length() != dds_ns_sfx.length()) {
          throw std::runtime_error(
                  "invalid type name: `dds_` should appear last in namespace");
        }
      } else {
        ss << dds_ns_sfx;
      }
    } else {
      ss << type_ns << dds_ns_sfx;
    }
  } else {
    ss << dds_ns;
  }

  ss << "::" << type_base_name;
  if (type_base_name[type_base_name.length() - 1] != '_') {
    ss << "_";
  }

  return ss.str();
}

template<>
std::string
resolve_topic_name(
  rclcpp::Node & node,
  const std::string & topic_name,
  const TopicKind topic_kind,
  const bool use_ros_conventions)
{
  if (!use_ros_conventions) {
    return topic_name;
  }
  if (topic_kind == TopicKind::Request || topic_kind == TopicKind::Reply)
  {
    std::string sub_namespace = node.get_sub_namespace();
    std::string res_topic_name(topic_name);
    if (sub_namespace != "" && topic_name.front() != '/' && topic_name.front() != '~') {
      res_topic_name = sub_namespace + "/" + topic_name;
    }

    return resolve_topic_name(
      node_name(node), node_namespace(node), res_topic_name, topic_kind, true);
  }
  auto node_topics_interface = rclcpp::node_interfaces::get_node_topics_interface(node);
  return node_topics_interface->resolve_topic_name(topic_name);
}

}  // namespace ros2dds
