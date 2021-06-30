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

#ifndef RTI__ROS2__NODE__QOS_HPP_
#define RTI__ROS2__NODE__QOS_HPP_

#include <rti/ros2/node/domain.hpp>

#include <string>

/* *INDENT-OFF* */
namespace rti { namespace ros2 { namespace node {
/* *INDENT-ON* */
dds::pub::qos::DataWriterQos
default_datawriter_qos(rclcpp::Node & node);

dds::pub::qos::DataWriterQos
default_datawriter_qos(
  rclcpp::Node & node,
  const std::string & topic_name,
  const bool expand_topic_name = true,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true);

template<typename T>
dds::pub::qos::DataWriterQos
default_datawriter_qos(
  rclcpp::Node & node,
  dds::topic::Topic<T> & topic)
{
  return default_datawriter_qos(node, topic.name(), false /* resolve_topic_name */);
}

dds::sub::qos::DataReaderQos
default_datareader_qos(rclcpp::Node & node);

dds::sub::qos::DataReaderQos
default_datareader_qos(
  rclcpp::Node & node,
  const std::string & topic_name,
  const bool expand_topic_name = true,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true);

template<typename T>
dds::sub::qos::DataReaderQos
default_datareader_qos(
  rclcpp::Node & node,
  dds::topic::Topic<T> & topic)
{
  return default_datareader_qos(node, topic->name(), false /* resolve_topic_name */);
}

inline
dds::pub::qos::DataWriterQos
datawriter_qos_profile(
  rclcpp::Node & node,
  const std::string & qos_profile)
{
  (void)node;
  auto qos_provider = dds::core::QosProvider::Default();
  return qos_provider.datawriter_qos(qos_profile);
}

inline
dds::sub::qos::DataReaderQos
datareader_qos_profile(
  rclcpp::Node & node,
  const std::string & qos_profile)
{
  (void)node;
  auto qos_provider = dds::core::QosProvider::Default();
  return qos_provider.datareader_qos(qos_profile);
}


}  // namespace node
}  // namespace ros2
}  // namespace rti

#endif  // RTI__ROS2__NODE__QOS_HPP_
