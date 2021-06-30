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

#ifndef RTI__ROS2__NODE__TOPIC_HPP_
#define RTI__ROS2__NODE__TOPIC_HPP_

#include <rti/ros2/node/domain.hpp>

#include <cstring>
#include <string>
#include <exception>

#include "rcpputils/find_and_replace.hpp"

/* *INDENT-OFF* */
namespace rti { namespace ros2 { namespace node {
/* *INDENT-ON* */
/**
 * @brief Enumeration describing the purpose of a certain topic.
 *
 * DDS topics are used for various purposes:
 *
 * - Distribute sample of a certain type.
 * - Send a request payload to a service.
 * - Receive a reply payload from a service.
 *
 */
enum class NodeTopicKind
{
  Topic = 0,
  Request = 1,
  Reply = 2
};

/**
 * @brief
 *
 * @param node_name
 * @param namespace
 * @param topic_name
 * @param topic_kind
 * @param use_ros_conventions
 * @return std::string
 */
std::string
resolve_topic_name(
  const std::string & node_name,
  const std::string & node_namespace,
  const std::string & topic_name,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true);

/**
 * @brief
 *
 * @param node
 * @param topic_name
 * @param topic_kind
 * @param use_ros_conventions
 * @return std::string
 */
inline
std::string
resolve_topic_name(
  rclcpp::Node & node,
  const std::string & topic_name,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true)
{
  return resolve_topic_name(
    node.get_name(), node.get_namespace(),
    topic_name, topic_kind, use_ros_conventions);
}

inline
std::string
resolve_topic_name(
  rclcpp::Node::SharedPtr & node,
  const std::string & topic_name,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true)
{
  return resolve_topic_name(
    *node, topic_name, topic_kind, use_ros_conventions);
}

inline
std::string
resolve_type_name(
  const std::string & type_name,
  const bool use_ros_conventions = true)
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
    type_ns = type_name.substr(0, base_name_pos + 2);
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
  }

  ss << "::" << type_base_name;
  if (type_base_name[type_base_name.length() - 1] != '_') {
    ss << "_";
  }
  
  return ss.str();
}

/**
 * @brief Create a topic object
 *
 * @tparam T
 * @param node
 * @param topic_name
 * @param topic_kind
 * @param use_ros_conventions
 * @param type_name
 * @return dds::topic::Topic<T>
 */
template<typename T>
dds::topic::Topic<T>
create_topic(
  rclcpp::Node & node,
  const std::string & topic_name,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true,
  const char * const type_name = nullptr)
{
  auto participant = dds_domain_participant(node);
  std::string fq_topic_name =
    resolve_topic_name(node, topic_name, topic_kind, use_ros_conventions);

  std::string fq_type_name;
  if (nullptr == type_name) {
    const std::string default_type_name = dds::topic::topic_type_name<T>::value();
    fq_type_name = resolve_type_name(default_type_name, use_ros_conventions);
  } else {
    const std::string user_type_name = type_name;
    fq_type_name = resolve_type_name(user_type_name, use_ros_conventions);
  }

  return dds::topic::Topic<T>(participant, fq_topic_name, fq_type_name);
}

/**
 * @brief
 *
 * @tparam T
 * @param node
 * @param topic_name
 * @param topic_kind
 * @param use_ros_conventions
 * @return dds::topic::Topic<T>
 */
template<typename T>
dds::topic::Topic<T>
lookup_topic(
  rclcpp::Node & node,
  const std::string & topic_name,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true)
{
  auto participant = dds_domain_participant(node);
  std::string fq_topic_name =
    resolve_topic_name(node, topic_name, topic_kind, use_ros_conventions);
  return dds::topic::find<dds::topic::Topic<T>>(participant, fq_topic_name);
}

/**
 * @brief
 *
 * @tparam T
 * @param node
 * @param topic_name
 * @param topic_kind
 * @param use_ros_conventions
 * @param type_name
 * @return dds::topic::Topic<T>
 */
template<typename T>
dds::topic::Topic<T>
assert_topic(
  rclcpp::Node & node,
  const std::string & topic_name,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true,
  const char * const type_name = nullptr)
{
  auto participant = dds_domain_participant(node);
  auto existing_topic = lookup_topic<T>(node, topic_name, topic_kind, use_ros_conventions);
  if (dds::core::null != existing_topic) {
    // Check that the type name matches, if one was specified.
    if (nullptr != type_name && strcmp(type_name, existing_topic.name().c_str()) != 0) {
      throw std::runtime_error("existing topic has mismatching type name");
    }
    return existing_topic;
  }
  return create_topic<T>(node, topic_name, topic_kind, use_ros_conventions, type_name);
}

template<typename T>
bool
is_filtered(dds::topic::TopicDescription<T> topic_desc)
{
  try {
    dds::core::polymorphic_cast<dds::topic::ContentFilteredTopic<T>>(topic_desc);
    return true;
  } catch (std::exception & e) {
    return false;
  }
}

template<typename T>
dds::topic::ContentFilteredTopic<T>
create_filtered_topic(
  rclcpp::Node & node,
  dds::topic::Topic<T> & topic,
  const std::string & filter_name,
  const std::string & query_expression)
{
  dds::topic::Filter filter(query_expression);
  return dds::topic::ContentFilteredTopic<T>(topic, filter_name, filter);
}

template<typename T>
dds::topic::ContentFilteredTopic<T>
create_filtered_topic(
  rclcpp::Node & node,
  dds::topic::Topic<T> & topic,
  const std::string & filter_name,
  const std::string & query_expression,
  const std::vector<std::string> & query_parameters)
{
  dds::topic::Filter filter(query_expression, query_parameters);
  return dds::topic::ContentFilteredTopic<T>(topic, filter_name, filter);
}

template<typename T>
dds::topic::ContentFilteredTopic<T>
create_filtered_topic(
  rclcpp::Node & node,
  const std::string & topic_name,
  const std::string & filter_name,
  const std::string & query_expression,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true,
  const char * const type_name = nullptr)
{
  auto topic = assert_topic<T>(
    node, topic_name, topic_kind, use_ros_conventions, type_name);
  dds::topic::Filter filter(query_expression);
  return dds::topic::ContentFilteredTopic<T>(topic, filter_name, filter);
}

template<typename T>
dds::topic::ContentFilteredTopic<T>
create_filtered_topic(
  rclcpp::Node & node,
  const std::string & topic_name,
  const std::string & filter_name,
  const std::string & query_expression,
  const std::vector<std::string> & query_parameters,
  const NodeTopicKind topic_kind = NodeTopicKind::Topic,
  const bool use_ros_conventions = true,
  const char * const type_name = nullptr)
{
  auto topic = assert_topic<T>(
    node, topic_name, topic_kind, use_ros_conventions, type_name);
  dds::topic::Filter filter(query_expression, query_parameters);
  return dds::topic::ContentFilteredTopic<T>(topic, filter_name, filter);
}


}  // namespace node
}  // namespace ros2
}  // namespace rti

#endif  // RTI__ROS2__NODE__TOPIC_HPP_
