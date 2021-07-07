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

#ifndef ROS2DDS__REQUEST_HPP_
#define ROS2DDS__REQUEST_HPP_

#include <string>
#include <sstream>

#include "rti/request/rtirequest.hpp"

#include "ros2dds/domain.hpp"
#include "ros2dds/topic.hpp"
#include "ros2dds/qos.hpp"
#include "ros2dds/pub.hpp"
#include "ros2dds/sub.hpp"

namespace ros2dds
{
template<
  typename MessageReqT,
  typename MessageRepT,
  typename NodeT = rclcpp::Node>
rti::request::Requester<MessageReqT, MessageRepT>
create_requester(
  NodeT & node,
  const std::string & service_name,
  const bool use_ros_conventions = true)
{
  const std::string req_topic = resolve_topic_name(
    node, service_name, TopicKind::Request, use_ros_conventions);
  const std::string rep_topic = resolve_topic_name(
    node, service_name, TopicKind::Reply, use_ros_conventions);
  // TODO(asorbini) create resolve_type() to return a DynamicType for the params
  // const std::string req_type = resolve_type_name(req_topic, use_ros_conventions);
  // const std::string rep_type = resolve_type_name(rep_topic, use_ros_conventions);
  rti::request::RequesterParams params(domain_participant(node));
  params.request_topic_name(req_topic);
  params.reply_topic_name(rep_topic);
  params.datawriter_qos(default_datawriter_qos<NodeT>(node, req_topic, false));
  params.datareader_qos(default_datareader_qos<NodeT>(node, rep_topic, false));
  params.publisher(default_publisher<NodeT>(node));
  params.subscriber(default_subscriber<NodeT>(node));
  return rti::request::Requester<MessageReqT, MessageRepT>(params);
}

template<
  typename MessageReqT,
  typename MessageRepT,
  typename NodeT = rclcpp::Node>
rti::request::Replier<MessageReqT, MessageRepT>
create_replier(
  NodeT & node,
  const std::string & service_name,
  const bool use_ros_conventions = true)
{
  const std::string req_topic = resolve_topic_name(
    node, service_name, TopicKind::Request, use_ros_conventions);
  const std::string rep_topic = resolve_topic_name(
    node, service_name, TopicKind::Reply, use_ros_conventions);
  // TODO(asorbini) create resolve_type() to return a DynamicType for the params
  // const std::string req_type = resolve_type_name(req_topic, use_ros_conventions);
  // const std::string rep_type = resolve_type_name(rep_topic, use_ros_conventions);
  rti::request::ReplierParams params(domain_participant(node));
  params.request_topic_name(req_topic);
  params.reply_topic_name(rep_topic);
  params.datawriter_qos(default_datawriter_qos<NodeT>(node, rep_topic, false));
  params.datareader_qos(default_datareader_qos<NodeT>(node, req_topic, false));
  params.publisher(default_publisher<NodeT>(node));
  params.subscriber(default_subscriber<NodeT>(node));
  return rti::request::Replier<MessageReqT, MessageRepT>(params);
}

}  // namespace ros2dds
#endif  // ROS2DDS__REQUEST_HPP_
