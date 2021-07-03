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

#ifndef ROS2DDS__DOMAIN_HPP_
#define ROS2DDS__DOMAIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <dds/dds.hpp>

namespace ros2dds
{
/**
 * @brief Get the id of the DDS domain joined by the DomainParticipant
 * associated with a ROS 2 Node.
 *
 * @param node The ROS 2 Node to inspect.
 * @return size_t An integer value representing the id of the DDS domain.
 */
template<typename NodeT = rclcpp::Node>
size_t
domain_id(NodeT & node)
{
  size_t id = 0;
  auto rcl_node = node.get_node_base_interface()->get_rcl_node_handle();
  rcl_ret_t rc = rcl_node_get_domain_id(rcl_node, &id);
  if (RCL_RET_OK != rc) {
    throw std::runtime_error("failed to retrieve DDS domain ID");
    rclcpp::exceptions::throw_from_rcl_error(rc, "Failed to retrieve DDS domain id");
  }
  return id;
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
 * @return The DDS DomainParticipant instance associated with this node.
 */
template<typename NodeT = rclcpp::Node>
dds::domain::DomainParticipant
domain_participant(NodeT & node)
{
  auto participant = dds::domain::find(domain_id(node));
  if (dds::core::null == participant) {
    rclcpp::exceptions::throw_from_rcl_error(
      RCL_RET_ERROR, "Failed to retrieve DDS DomainParticipant for node");
    
  }
  return participant;
}

template<typename NodeT = rclcpp::Node>
std::string
node_name(NodeT & node)
{
  return node.get_name();
}

template<typename NodeT = rclcpp::Node>
std::string
node_namespace(NodeT & node)
{
  return node.get_namespace();
}
}  // namespace ros2dds

#endif  // ROS2DDS__DOMAIN_HPP_
