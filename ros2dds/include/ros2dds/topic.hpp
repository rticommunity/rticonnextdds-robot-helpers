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

#ifndef ROS2DDS__TOPIC_HPP_
#define ROS2DDS__TOPIC_HPP_

#include <cstring>
#include <string>
// #include <exception>
#include <vector>

#include "ros2dds/domain.hpp"
#include "ros2dds/resolve.hpp"

namespace ros2dds
{
template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
std::string
topic_type_name(
  NodeT & node,
  const bool use_ros_conventions = true,
  const std::string & type_name = std::string())
{
  (void)node;
  if (type_name.length() == 0) {
    const std::string default_type_name = dds::topic::topic_type_name<MessageT>::value();
    return resolve_type_name(default_type_name, use_ros_conventions);
  } else {
    return resolve_type_name(type_name, use_ros_conventions);
  }
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::topic::Topic<MessageT>
create_topic(
  NodeT & node,
  const std::string & topic_name,
  const TopicKind topic_kind = TopicKind::Topic,
  const bool use_ros_conventions = true,
  const std::string & type_name = std::string())
{
  auto participant = domain_participant<NodeT>(node);
  std::string fq_topic_name =
    resolve_topic_name<NodeT>(node, topic_name, topic_kind, use_ros_conventions);

  auto fq_type_name = topic_type_name<MessageT>(node, use_ros_conventions, type_name);

  return dds::topic::Topic<MessageT>(participant, fq_topic_name, fq_type_name);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::topic::Topic<MessageT>
lookup_topic(
  NodeT & node,
  const std::string & topic_name,
  const TopicKind topic_kind = TopicKind::Topic,
  const bool use_ros_conventions = true)
{
  auto participant = domain_participant<NodeT>(node);
  std::string fq_topic_name =
    resolve_topic_name<NodeT>(node, topic_name, topic_kind, use_ros_conventions);
  return dds::topic::find<dds::topic::Topic<MessageT>>(participant, fq_topic_name);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::topic::Topic<MessageT>
assert_topic(
  NodeT & node,
  const std::string & topic_name,
  const TopicKind topic_kind = TopicKind::Topic,
  const bool use_ros_conventions = true,
  const std::string & type_name = std::string())
{
  auto participant = domain_participant<NodeT>(node);
  auto existing_topic =
    lookup_topic<MessageT, NodeT>(node, topic_name, topic_kind, use_ros_conventions);
  if (dds::core::null != existing_topic) {
    // Check that the type name matches, if one was specified.
    auto fq_type_name = topic_type_name<MessageT>(node, use_ros_conventions, type_name);
    if (fq_type_name != existing_topic.type_name()) {
      rclcpp::exceptions::throw_from_rcl_error(
        RCL_RET_INVALID_ARGUMENT, "Existing topic has mismatching type name");
    }
    return existing_topic;
  }
  return create_topic<MessageT, NodeT>(
    node, topic_name, topic_kind, use_ros_conventions, type_name);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::topic::ContentFilteredTopic<MessageT>
create_content_filtered_topic(
  NodeT & node,
  dds::topic::Topic<MessageT> & topic,
  const std::string & filter_name,
  const std::string & query_expression,
  const std::vector<std::string> & query_parameters = std::vector<std::string>())
{
  (void)node;
  dds::topic::Filter filter(query_expression, query_parameters);
  return dds::topic::ContentFilteredTopic<MessageT>(topic, filter_name, filter);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::topic::ContentFilteredTopic<MessageT>
lookup_content_filtered_topic(
  NodeT & node,
  const std::string & filter_name)
{
  auto participant = domain_participant<NodeT>(node);
  return dds::topic::find<dds::topic::ContentFilteredTopic<MessageT>>(participant, filter_name);
}

}  // namespace ros2dds

#endif  // ROS2DDS__TOPIC_HPP_
