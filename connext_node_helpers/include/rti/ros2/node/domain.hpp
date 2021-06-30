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

#ifndef RTI__ROS2__NODE__DOMAIN_HPP_
#define RTI__ROS2__NODE__DOMAIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <dds/dds.hpp>

namespace rti
{
namespace ros2
{
namespace node
{
/**
 * @brief Get the id of the DDS domain joined by the DomainParticipant
 * associated with a ROS 2 Node.
 *
 * @param node The ROS 2 Node to inspect.
 * @return size_t An integer value representing the id of the DDS domain.
 */
size_t
dds_domain_id(rclcpp::Node & node);

inline
size_t
dds_domain_id(rclcpp::Node::SharedPtr & node)
{
  return dds_domain_id(*node);
}

/**
 * @brief Get the DDS DomainParticipant associated with a ROS 2 Node.
 *
 * The RMW layer automatically creates a DDS DomainParticipant for every ROS 2
 * context (typically one per process). This DomainParticipant is shared among
 * all Nodes associated with that Context.
 *
 * This operation requires the use of `rmw_connextdds` as the RMW layer.
 *
 * An `std::runtime_error` will be thrown if the DomainParticipant instance
 * cannot be retrieved (typically because the Node is not running on
 * `rmw_connextdds`).
 *
 * @param node The ROS 2 Node to inspect.
 * @return The DDS DomainParticipant instance associated with the node.
 */
dds::domain::DomainParticipant
dds_domain_participant(rclcpp::Node & node);

inline
dds::domain::DomainParticipant
dds_domain_participant(rclcpp::Node::SharedPtr & node)
{
  return dds_domain_participant(*node);
}


}  // namespace node
}  // namespace ros2
}  // namespace rti

#endif  // RTI__ROS2__NODE__DOMAIN_HPP_
