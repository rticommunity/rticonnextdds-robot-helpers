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

#ifndef ROS2DDS__PUB_HPP_
#define ROS2DDS__PUB_HPP_

#include <string>

#include "ros2dds/domain.hpp"
#include "ros2dds/topic.hpp"
#include "ros2dds/qos.hpp"

namespace ros2dds
{
template<typename NodeT = rclcpp::Node>
dds::pub::Publisher
default_publisher(NodeT & node)
{
  return rti::pub::implicit_publisher(domain_participant<NodeT>(node));
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::pub::DataWriter<MessageT>
create_datawriter(
  NodeT & node,
  const std::string & topic_name,
  const TopicKind topic_kind = TopicKind::Topic,
  const bool use_ros_conventions = true,
  const std::string & type_name = std::string())
{
  auto publisher = default_publisher<NodeT>(node);
  auto topic = assert_topic<MessageT, NodeT>(
    node, topic_name, topic_kind, use_ros_conventions, type_name);
  auto qos = default_datawriter_qos<MessageT, NodeT>(node, topic);
  return dds::pub::DataWriter<MessageT>(publisher, topic, qos);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::pub::DataWriter<MessageT>
create_datawriter(
  NodeT & node,
  const std::string & topic_name,
  const dds::pub::qos::DataWriterQos & qos,
  const TopicKind topic_kind = TopicKind::Topic,
  const bool use_ros_conventions = true,
  const std::string & type_name = std::string())
{
  auto publisher = default_publisher<NodeT>(node);
  auto topic = assert_topic<MessageT, NodeT>(
    node, topic_name, topic_kind, use_ros_conventions, type_name);
  return dds::pub::DataWriter<MessageT>(publisher, topic, qos);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::pub::DataWriter<MessageT>
create_datawriter(
  NodeT & node,
  dds::topic::Topic<MessageT> & topic)
{
  auto publisher = default_publisher<NodeT>(node);
  auto qos = default_datawriter_qos<MessageT, NodeT>(node, topic);
  return dds::pub::DataWriter<MessageT>(publisher, topic, qos);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::pub::DataWriter<MessageT>
create_datawriter(
  NodeT & node,
  dds::topic::Topic<MessageT> & topic,
  const dds::pub::qos::DataWriterQos & qos)
{
  auto publisher = default_publisher<NodeT>(node);
  return dds::pub::DataWriter<MessageT>(publisher, topic, qos);
}
}  // namespace ros2dds

#endif  // ROS2DDS__PUB_HPP_
