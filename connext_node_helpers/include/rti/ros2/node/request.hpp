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

#ifndef RTI__ROS2__NODE__REQUEST_HPP_
#define RTI__ROS2__NODE__REQUEST_HPP_

#include <rti/request/rtirequest.hpp>

#include <rti/ros2/node/domain.hpp>
#include <rti/ros2/node/topic.hpp>
#include <rti/ros2/node/qos.hpp>

#include <string>
/* *INDENT-OFF* */
namespace rti { namespace ros2 { namespace node {
/* *INDENT-ON* */
template<typename Req, typename Rep>
rti::request::Requester<Req, Rep>
create_requester(
  rclcpp::Node & node,
  const std::string & service_name,
  const bool use_ros_conventions = true)
{
  std::string req_topic = resolve_topic_name(
    node, service_name, NodeTopicKind::Request, use_ros_conventions);
  std::string rep_topic = resolve_topic_name(
    node, service_name, NodeTopicKind::Reply, use_ros_conventions);

  auto participant = dds_domain_participant(node);
  rti::request::RequesterParams params(participant);
  params.request_topic_name(req_topic);
  params.reply_topic_name(rep_topic);
  params.datawriter_qos(default_datawriter_qos(node, req_topic.c_str()));
  params.datareader_qos(default_datareader_qos(node, rep_topic.c_str()));
  params.publisher(rti::pub::implicit_publisher(participant));
  params.subscriber(rti::sub::implicit_subscriber(participant));

  return rti::request::Requester<Req, Rep>(params);
}

template<typename Req, typename Rep>
rti::request::Replier<Req, Rep>
create_replier(
  rclcpp::Node & node,
  const std::string & service_name,
  const bool use_ros_conventions = true)
{
  std::string req_topic = resolve_topic_name(
    node, service_name, NodeTopicKind::Request, use_ros_conventions);
  std::string rep_topic = resolve_topic_name(
    node, service_name, NodeTopicKind::Reply, use_ros_conventions);

  auto participant = dds_domain_participant(node);
  rti::request::ReplierParams params(participant);
  params.request_topic_name(req_topic);
  params.reply_topic_name(rep_topic);
  params.datawriter_qos(default_datawriter_qos(node, rep_topic.c_str()));
  params.datareader_qos(default_datareader_qos(node, req_topic.c_str()));
  params.publisher(rti::pub::implicit_publisher(participant));
  params.subscriber(rti::sub::implicit_subscriber(participant));

  return rti::request::Replier<Req, Rep>(params);
}
}  // namespace node
}  // namespace ros2
}  // namespace rti
#endif  // RTI__ROS2__NODE__REQUEST_HPP_
