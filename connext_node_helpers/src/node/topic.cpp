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

#include <rti/ros2/node/dds.hpp>

#include <rclcpp/expand_topic_or_service_name.hpp>

#include <string>

/* *INDENT-OFF* */
namespace rti { namespace ros2 { namespace node {
/* *INDENT-ON* */
std::string
resolve_topic_name(
  const std::string & node_name,
  const std::string & node_namespace,
  const std::string & topic_name,
  const NodeTopicKind topic_kind,
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
    const bool is_service = topic_kind != NodeTopicKind::Topic;
    std::string prefix = topic_prefixes[static_cast<int>(topic_kind)];
    std::string fq_name = rclcpp::expand_topic_or_service_name(
      topic_name, node_name, node_namespace, is_service);
    return prefix + fq_name + suffix;
  }
}
}  // namespace node
}  // namespace ros2
}  // namespace rti
