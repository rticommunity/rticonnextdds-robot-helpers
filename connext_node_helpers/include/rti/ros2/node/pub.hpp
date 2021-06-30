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

#ifndef RTI__ROS2__NODE__PUB_HPP_
#define RTI__ROS2__NODE__PUB_HPP_

#include <rti/ros2/node/domain.hpp>
#include <rti/ros2/node/topic.hpp>
#include <rti/ros2/node/qos.hpp>

#include <string>

/* *INDENT-OFF* */
namespace rti { namespace ros2 { namespace node {
/* *INDENT-ON* */
/**
 * @brief Create a DataWriter with default Quality of Service.
 *
 * @tparam T
 * @param node
 * @param topic_name
 * @param topic_kind
 * @param use_ros_conventions
 * @param type_name
 * @return dds::pub::DataWriter<T>
 */
template<typename T>
dds::pub::DataWriter<T>
create_datawriter(
  rclcpp::Node & node,
  const std::string & topic_name,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true,
  const char * const type_name = nullptr,
  dds::pub::Publisher * const custom_publisher = nullptr)
{
  auto publisher = (custom_publisher == nullptr)?
    rti::pub::implicit_publisher(dds_domain_participant(node))
    : *custom_publisher;
  dds::topic::Topic<T> topic = assert_topic<T>(
    node, topic_name, topic_kind, use_ros_conventions, type_name);
  auto qos = default_datawriter_qos(node, topic);
  return dds::pub::DataWriter<T>(publisher, topic, qos);
}

/**
 * @brief Create a DataWriter with custom Quality of Service.
 *
 * @tparam T
 * @param node
 * @param topic_name
 * @param qos
 * @param topic_kind
 * @param use_ros_conventions
 * @param type_name
 * @return dds::pub::DataWriter<T>
 */
template<typename T>
dds::pub::DataWriter<T>
create_datawriter(
  rclcpp::Node & node,
  const std::string & topic_name,
  const dds::pub::qos::DataWriterQos & qos,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true,
  const char * const type_name = nullptr,
  dds::pub::Publisher * const custom_publisher = nullptr)
{
  auto publisher = (custom_publisher == nullptr)?
    rti::pub::implicit_publisher(dds_domain_participant(node))
    : *custom_publisher;
  dds::topic::Topic<T> topic = assert_topic<T>(
    node, topic_name, topic_kind, use_ros_conventions, type_name);
  return dds::pub::DataWriter<T>(publisher, topic, qos);
}

/**
 * @brief Create a DataWriter for an existing Topic with default QoS.
 *
 * @tparam T
 * @param node
 * @param topic
 * @return dds::pub::DataWriter<T>
 */
template<typename T>
dds::pub::DataWriter<T>
create_datawriter(
  rclcpp::Node & node,
  dds::topic::Topic<T> & topic,
  dds::pub::Publisher * const custom_publisher = nullptr)
{
  auto publisher = (custom_publisher == nullptr)?
    rti::pub::implicit_publisher(dds_domain_participant(node))
    : *custom_publisher;
  auto qos = default_datawriter_qos<T>(node, topic);
  return dds::pub::DataWriter<T>(publisher, topic, qos);
}

/**
 * @brief Create a DataWriter for an existing Topic with custom QoS.
 *
 * @tparam T
 * @param node
 * @param topic
 * @param qos
 * @return dds::pub::DataWriter<T>
 */
template<typename T>
dds::pub::DataWriter<T>
create_datareader(
  rclcpp::Node & node,
  dds::topic::Topic<T> & topic,
  const dds::pub::qos::DataWriterQos & qos,
  dds::pub::Publisher * const custom_publisher = nullptr)
{
  auto publisher = (custom_publisher == nullptr)?
    rti::pub::implicit_publisher(dds_domain_participant(node))
    : *custom_publisher;
  return dds::pub::DataWriter<T>(publisher, topic, qos);
}

}  // namespace node
}  // namespace ros2
}  // namespace rti

#endif  // RTI__ROS2__NODE__PUB_HPP_
