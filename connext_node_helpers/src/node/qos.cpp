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

#include <string>

#include "rclcpp/scope_exit.hpp"

/* *INDENT-OFF* */
namespace rti { namespace ros2 { namespace node {

static
void configure_default_properties(dds::pub::qos::DataWriterQos & qos)
{
  rti::core::policy::Property props;
  props.set(
  {
    "dds.data_writer.history.memory_manager.fast_pool.pool_buffer_max_size",
    "0"
  }, false);
  qos << props;
}

static
void configure_default_properties(dds::sub::qos::DataReaderQos & qos)
{
  rti::core::policy::Property props;
  props.set(
  {
    "dds.data_reader.history.memory_manager.fast_pool.pool_buffer_max_size",
    "0"
  }, false);
  qos << props;
}

/* *INDENT-ON* */
/**
 * @brief
 *
 * @param node
 * @param topic_name
 * @return dds::pub::qos::DataWriterQos
 */
dds::pub::qos::DataWriterQos
default_datawriter_qos(rclcpp::Node & node)
{
  auto participant = dds_domain_participant(node);
  auto publisher = rti::pub::implicit_publisher(participant);
  auto qos = publisher.default_datawriter_qos();
  configure_default_properties(qos);
  return qos;
}

dds::pub::qos::DataWriterQos
default_datawriter_qos(
  rclcpp::Node & node,
  const std::string & topic_name,
  const bool resolve_topic_name,
  const NodeTopicKind topic_kind,
  const bool use_ros_conventions)
{
  auto participant = dds_domain_participant(node);
  auto publisher = rti::pub::implicit_publisher(participant);
  std::string fq_topic_name = topic_name;
  if (resolve_topic_name) {
    fq_topic_name = rti::ros2::node::resolve_topic_name(
      node, topic_name, topic_kind, use_ros_conventions);
  }
  dds::pub::qos::DataWriterQos dw_qos;
  if (DDS_RETCODE_OK !=
    DDS_Publisher_get_default_datawriter_qos_w_topic_name(
      publisher->native_publisher(), dw_qos->native(), fq_topic_name.c_str()))
  {
    throw std::runtime_error("failed to get C writer qos");
  }
  configure_default_properties(dw_qos);
  return dw_qos;
  // auto qos_provider = dds::core::QosProvider::Default();
  // return qos_provider->datawriter_qos_w_topic_name(topic_name);
}

/**
 * @brief
 *
 * @param node
 * @param topic_name
 * @return dds::pub::qos::DataReaderQos
 */
dds::sub::qos::DataReaderQos
default_datareader_qos(rclcpp::Node & node)
{
  auto participant = dds_domain_participant(node);
  auto subscriber = rti::sub::implicit_subscriber(participant);
  auto qos = subscriber.default_datareader_qos();
  configure_default_properties(qos);
  return qos;
}

dds::sub::qos::DataReaderQos
default_datareader_qos(
  rclcpp::Node & node,
  const std::string & topic_name,
  const bool resolve_topic_name,
  const NodeTopicKind topic_kind,
  const bool use_ros_conventions)
{
  auto participant = dds_domain_participant(node);
  auto subscriber = rti::sub::implicit_subscriber(participant);
  std::string fq_topic_name = topic_name;
  if (resolve_topic_name) {
    fq_topic_name = rti::ros2::node::resolve_topic_name(
      node, topic_name, topic_kind, use_ros_conventions);
  }
  dds::sub::qos::DataReaderQos dr_qos;
  if (DDS_RETCODE_OK !=
    DDS_Subscriber_get_default_datareader_qos_w_topic_name(
      subscriber->native_subscriber(), dr_qos->native(), fq_topic_name.c_str()))
  {
    throw std::runtime_error("failed to get C reader qos");
  }
  configure_default_properties(dr_qos);
  return dr_qos;
  // (void)node;
  // auto qos_provider = dds::core::QosProvider::Default();
  // return qos_provider.delegate()->datareader_qos_w_topic_name(topic_name);
}

}  // namespace node
}  // namespace ros2
}  // namespace rti
