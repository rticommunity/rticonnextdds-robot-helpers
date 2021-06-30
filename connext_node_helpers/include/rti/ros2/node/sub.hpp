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

#ifndef RTI__ROS2__NODE__SUB_HPP_
#define RTI__ROS2__NODE__SUB_HPP_

#include <rti/ros2/node/domain.hpp>
#include <rti/ros2/node/topic.hpp>
#include <rti/ros2/node/qos.hpp>

#include <string>

/* *INDENT-OFF* */
namespace rti { namespace ros2 { namespace node {
/* *INDENT-ON* */
/**
 * @brief Create a DataReader with default Quality of Service.
 *
 * @tparam T
 * @param node
 * @param topic_name
 * @param topic_kind
 * @param use_ros_conventions
 * @param type_name
 * @return dds::sub::DataReader<T>
 */
template<typename T>
dds::sub::DataReader<T>
create_datareader(
  rclcpp::Node & node,
  const std::string & topic_name,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true,
  const char * const type_name = nullptr,
  dds::sub::Subscriber * const custom_subscriber = nullptr)
{
  auto subscriber = (custom_subscriber == nullptr)?
    rti::sub::implicit_subscriber(dds_domain_participant(node))
    : *custom_subscriber;
  dds::topic::Topic<T> topic = assert_topic<T>(
    node, topic_name, topic_kind, use_ros_conventions, type_name);
  auto qos = default_datareader_qos<T>(node, topic);
  return dds::sub::DataReader<T>(subscriber, topic, qos);
}

/**
 * @brief Create a DataReader with custom Quality of Service.
 *
 * @tparam T
 * @param node
 * @param topic_name
 * @param qos
 * @param topic_kind
 * @param use_ros_conventions
 * @param type_name
 * @return dds::sub::DataReader<T>
 */
template<typename T>
dds::sub::DataReader<T>
create_datareader(
  rclcpp::Node & node,
  const std::string & topic_name,
  const dds::sub::qos::DataReaderQos & qos,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true,
  const char * const type_name = nullptr,
  dds::sub::Subscriber * const custom_subscriber = nullptr)
{
  auto subscriber = (custom_subscriber == nullptr)?
    rti::sub::implicit_subscriber(dds_domain_participant(node))
    : *custom_subscriber;
  dds::topic::Topic<T> topic = assert_topic<T>(
    node, topic_name, topic_kind, use_ros_conventions, type_name);
  return dds::sub::DataReader<T>(subscriber, topic, qos);
}


/**
 * @brief Create a DataReader for an existing Topic with default QoS.
 *
 * @tparam T
 * @param node
 * @param topic
 * @return dds::sub::DataReader<T>
 */
template<typename T>
dds::sub::DataReader<T>
create_datareader(
  rclcpp::Node & node,
  dds::topic::Topic<T> & topic,
  dds::sub::Subscriber * const custom_subscriber = nullptr)
{
  auto subscriber = (custom_subscriber == nullptr)?
    rti::sub::implicit_subscriber(dds_domain_participant(node))
    : *custom_subscriber;
  auto qos = default_datareader_qos(node, topic);
  return dds::sub::DataReader<T>(subscriber, topic, qos);
}

/**
 * @brief Create a DataReader for an existing ContentFilteredTopic with default QoS.
 *
 * @tparam T
 * @param node
 * @param topic
 * @return dds::sub::DataReader<T>
 */
template<typename T>
dds::sub::DataReader<T>
create_datareader(
  rclcpp::Node & node,
  dds::topic::ContentFilteredTopic<T> & topic,
  dds::sub::Subscriber * const custom_subscriber = nullptr)
{
  auto subscriber = (custom_subscriber == nullptr)?
    rti::sub::implicit_subscriber(dds_domain_participant(node))
    : *custom_subscriber;
  auto qos = default_datareader_qos(node, topic);
  return dds::sub::DataReader<T>(subscriber, topic, qos);
}

/**
 * @brief Create a DataReader for an existing Topic with custom QoS.
 *
 * @tparam T
 * @param node
 * @param topic
 * @param qos
 * @return dds::sub::DataReader<T>
 */
template<typename T>
dds::sub::DataReader<T>
create_datareader(
  rclcpp::Node & node,
  dds::topic::Topic<T> & topic,
  const dds::sub::qos::DataReaderQos & qos,
  dds::sub::Subscriber * const custom_subscriber = nullptr)
{
  auto subscriber = (custom_subscriber == nullptr)?
    rti::sub::implicit_subscriber(dds_domain_participant(node))
    : *custom_subscriber;
  return dds::sub::DataReader<T>(subscriber, topic, qos);
}

/**
 * @brief Create a DataReader for an existing ContentFilteredTopic with custom QoS.
 *
 * @tparam T
 * @param node
 * @param topic
 * @param qos
 * @return dds::sub::DataReader<T>
 */
template<typename T>
dds::sub::DataReader<T>
create_datareader(
  rclcpp::Node & node,
  dds::topic::ContentFilteredTopic<T> & topic,
  const dds::sub::qos::DataReaderQos & qos,
  dds::sub::Subscriber * const custom_subscriber = nullptr)
{
  auto subscriber = (custom_subscriber == nullptr)?
    rti::sub::implicit_subscriber(dds_domain_participant(node))
    : *custom_subscriber;
  return dds::sub::DataReader<T>(subscriber, topic, qos);
}

}  // namespace node
}  // namespace ros2
}  // namespace rti

#endif  // RTI__ROS2__NODE__SUB_HPP_
