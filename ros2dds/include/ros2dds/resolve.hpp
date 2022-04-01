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
#ifndef ROS2DDS__RESOLVE_HPP_
#define ROS2DDS__RESOLVE_HPP_

#include <cstring>
#include <string>
#include <exception>

#include "rcpputils/find_and_replace.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ros2dds/domain.hpp"
#include "ros2dds/visibility_control.hpp"

#include "rclcpp/node_interfaces/node_topics_interface.hpp"

namespace ros2dds
{
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
enum class TopicKind
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
ROS2DDS_PUBLIC
std::string
resolve_topic_name(
  const std::string & node_name,
  const std::string & node_namespace,
  const std::string & topic_name,
  const TopicKind topic_kind = TopicKind::Topic,
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
template<typename NodeT>
std::string
resolve_topic_name(
  NodeT & node,
  const std::string & topic_name,
  const TopicKind topic_kind = TopicKind::Topic,
  const bool use_ros_conventions = true)
{
  return resolve_topic_name(
    node_name(node), node_namespace(node), topic_name, topic_kind, use_ros_conventions);
}

template<>
std::string
resolve_topic_name(
  rclcpp::Node & node,
  const std::string & topic_name,
  const TopicKind topic_kind,
  const bool use_ros_conventions);

ROS2DDS_PUBLIC
std::string
resolve_type_name(
  const std::string & type_name,
  const bool use_ros_conventions = true);

}  // namespace ros2dds

#endif  // ROS2DDS__RESOLVE_HPP_
