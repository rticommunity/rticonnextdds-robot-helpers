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

/* *INDENT-OFF* */
namespace rti { namespace ros2 { namespace node {
/* *INDENT-ON* */
size_t
dds_domain_id(rclcpp::Node & node)
{
  size_t domain_id = 0;
  auto rcl_node = node.get_node_base_interface()->get_rcl_node_handle();
  if (RCL_RET_OK != rcl_node_get_domain_id(rcl_node, &domain_id)) {
    throw std::runtime_error("failed to retrieve DDS domain ID");
  }
  return domain_id;
}

dds::domain::DomainParticipant
dds_domain_participant(rclcpp::Node & node)
{
  size_t domain_id = dds_domain_id(node);
  auto participant = dds::domain::find(domain_id);
  if (dds::core::null == participant) {
    throw std::runtime_error("failed to lookup DomainParticipant");
  }
  return participant;
}
}  // namespace node
}  // namespace ros2
}  // namespace rti
