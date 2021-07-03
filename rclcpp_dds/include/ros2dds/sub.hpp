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

#ifndef ROS2DDS__SUB_HPP_
#define ROS2DDS__SUB_HPP_

#include <string>

#include "ros2dds/domain.hpp"
#include "ros2dds/topic.hpp"
#include "ros2dds/qos.hpp"

namespace ros2dds
{
template<typename NodeT = rclcpp::Node>
dds::sub::Subscriber
default_subscriber(NodeT & node)
{
  return rti::sub::implicit_subscriber(domain_participant<NodeT>(node));
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::sub::DataReader<MessageT>
create_datareader(
  NodeT & node,
  const std::string & topic_name,
  const TopicKind topic_kind = TopicKind::Topic,
  const bool use_ros_conventions = true,
  const std::string & type_name = std::string())
{
  auto subscriber = default_subscriber<NodeT>(node);
  auto topic = assert_topic<MessageT, NodeT>(
    node, topic_name, topic_kind, use_ros_conventions, type_name);
  auto qos = default_datareader_qos<MessageT, NodeT>(node, topic);
  return dds::sub::DataReader<MessageT>(subscriber, topic, qos);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::sub::DataReader<MessageT>
create_datareader(
  NodeT & node,
  const std::string & topic_name,
  const dds::sub::qos::DataReaderQos & qos,
  const TopicKind topic_kind = TopicKind::Topic,
  const bool use_ros_conventions = true,
  const std::string & type_name = std::string())
{
  auto subscriber = default_subscriber<NodeT>(node);
  auto topic = assert_topic<MessageT, NodeT>(
    node, topic_name, topic_kind, use_ros_conventions, type_name);
  return dds::sub::DataReader<MessageT>(subscriber, topic, qos);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::sub::DataReader<MessageT>
create_datareader(
  NodeT & node,
  dds::topic::Topic<MessageT> & topic)
{
  auto subscriber = default_subscriber<NodeT>(node);
  auto qos = default_datareader_qos<MessageT, NodeT>(node, topic);
  return dds::sub::DataReader<MessageT>(subscriber, topic, qos);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::sub::DataReader<MessageT>
create_datareader(
  NodeT & node,
  dds::topic::Topic<MessageT> & topic,
  const dds::sub::qos::DataReaderQos & qos)
{
  auto subscriber = default_subscriber<NodeT>(node);
  return dds::sub::DataReader<MessageT>(subscriber, topic, qos);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::sub::DataReader<MessageT>
create_datareader(
  NodeT & node,
  dds::topic::ContentFilteredTopic<MessageT> & topic)
{
  auto subscriber = default_subscriber<NodeT>(node);
  auto qos = default_datareader_qos<MessageT, NodeT>(node, topic);
  return dds::sub::DataReader<MessageT>(subscriber, topic, qos);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::sub::DataReader<MessageT>
create_datareader(
  NodeT & node,
  dds::topic::ContentFilteredTopic<MessageT> & topic,
  const dds::sub::qos::DataReaderQos & qos)
{
  auto subscriber = default_subscriber<NodeT>(node);
  return dds::sub::DataReader<MessageT>(subscriber, topic, qos);
}
}  // namespace ros2dds

#endif  // ROS2DDS__SUB_HPP_
