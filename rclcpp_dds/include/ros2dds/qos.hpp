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

#ifndef ROS2DDS__QOS_HPP_
#define ROS2DDS__QOS_HPP_

#include <string>

#include "ros2dds/domain.hpp"

namespace ros2dds
{
inline
void set_default_qos_properties(dds::pub::qos::DataWriterQos & qos)
{
  rti::core::policy::Property props;
  props.set(
  {
    "dds.data_writer.history.memory_manager.fast_pool.pool_buffer_max_size",
    "0"
  }, false);
  qos << props;
}

inline
void set_default_qos_properties(dds::sub::qos::DataReaderQos & qos)
{
  rti::core::policy::Property props;
  props.set(
  {
    "dds.data_reader.history.memory_manager.fast_pool.pool_buffer_max_size",
    "0"
  }, false);
  qos << props;
}

template<typename NodeT = rclcpp::Node>
dds::pub::qos::DataWriterQos
default_datawriter_qos(NodeT & node)
{
  auto participant = domain_participant<NodeT>(node);
  auto publisher = rti::pub::implicit_publisher(participant);
  auto qos = publisher.default_datawriter_qos();
  set_default_qos_properties(qos);
  return qos;
}

template<typename NodeT = rclcpp::Node>
dds::pub::qos::DataWriterQos
default_datawriter_qos(
  NodeT & node,
  const std::string & topic_name,
  const bool expand_topic_name = true,
  const TopicKind topic_kind = TopicKind::Topic,
  const bool use_ros_conventions = true)
{
  auto participant = domain_participant<NodeT>(node);
  auto publisher = rti::pub::implicit_publisher(participant);
  std::string fq_topic_name = topic_name;
  if (expand_topic_name) {
    fq_topic_name = resolve_topic_name<NodeT>(
      node, topic_name, topic_kind, use_ros_conventions);
  }
  dds::pub::qos::DataWriterQos dw_qos;
  if (DDS_RETCODE_OK !=
    DDS_Publisher_get_default_datawriter_qos_w_topic_name(
      publisher->native_publisher(), dw_qos->native(), fq_topic_name.c_str()))
  {
    throw std::runtime_error("failed to get C writer qos");
  }
  set_default_qos_properties(dw_qos);
  return dw_qos;
  // auto qos_provider = dds::core::QosProvider::Default();
  // return qos_provider->datawriter_qos_w_topic_name(topic_name);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::pub::qos::DataWriterQos
default_datawriter_qos(
  NodeT & node,
  const dds::topic::TopicDescription<MessageT> & topic)
{
  return default_datawriter_qos<NodeT>(
    node, topic.name(), false /* expand_topic_name */);
}

template<typename NodeT = rclcpp::Node>
dds::sub::qos::DataReaderQos
default_datareader_qos(NodeT & node)
{
  auto participant = domain_participant<NodeT>(node);
  auto subscriber = rti::sub::implicit_subscriber(participant);
  auto qos = subscriber.default_datareader_qos();
  set_default_qos_properties(qos);
  return qos;
}

template<typename NodeT = rclcpp::Node>
dds::sub::qos::DataReaderQos
default_datareader_qos(
  NodeT & node,
  const std::string & topic_name,
  const bool expand_topic_name = true,
  const TopicKind topic_kind = TopicKind::Topic,
  const bool use_ros_conventions = true)
{
  auto participant = domain_participant<NodeT>(node);
  auto subscriber = rti::sub::implicit_subscriber(participant);
  std::string fq_topic_name = topic_name;
  if (expand_topic_name) {
    fq_topic_name = resolve_topic_name<NodeT>(
      node, topic_name, topic_kind, use_ros_conventions);
  }
  dds::sub::qos::DataReaderQos dr_qos;
  if (DDS_RETCODE_OK !=
    DDS_Subscriber_get_default_datareader_qos_w_topic_name(
      subscriber->native_subscriber(), dr_qos->native(), fq_topic_name.c_str()))
  {
    throw std::runtime_error("failed to get C reader qos");
  }
  set_default_qos_properties(dr_qos);
  return dr_qos;
  // (void)node;
  // auto qos_provider = dds::core::QosProvider::Default();
  // return qos_provider.delegate()->datareader_qos_w_topic_name(topic_name);
}

template<
  typename MessageT,
  typename NodeT = rclcpp::Node>
dds::sub::qos::DataReaderQos
default_datareader_qos(
  NodeT & node,
  const dds::topic::TopicDescription<MessageT> & topic)
{
  return default_datareader_qos<NodeT>(
    node, topic->name(), false /* resolve_topic_name */);
}

template<typename NodeT = rclcpp::Node>
dds::pub::qos::DataWriterQos
datawriter_qos_profile(
  NodeT & node,
  const std::string & qos_profile)
{
  (void)node;
  auto qos_provider = dds::core::QosProvider::Default();
  return qos_provider.datawriter_qos(qos_profile);
}

template<typename NodeT = rclcpp::Node>
dds::sub::qos::DataReaderQos
datareader_qos_profile(
  NodeT & node,
  const std::string & qos_profile)
{
  (void)node;
  auto qos_provider = dds::core::QosProvider::Default();
  return qos_provider.datareader_qos(qos_profile);
}
}  // namespace ros2dds

#endif  // ROS2DDS__QOS_HPP_
