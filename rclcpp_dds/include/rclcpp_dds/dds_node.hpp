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

#ifndef RCLCPP_DDS__DDS_NODE_HPP_
#define RCLCPP_DDS__DDS_NODE_HPP_

#include <string>

#include "rclcpp/macros.hpp"

#include "rclcpp_dds/dds_node_mixin.hpp"

namespace rclcpp_dds
{
class DDSNode : public DDSNodeMixin<rclcpp::Node>
{
public:
  // NOTE: cppcheck on Foxy fails to correctly resolve the following macro's definition.
  // This is resolved in later versions, see https://github.com/ament/ament_lint/issues/116
  // The following #ifndef causes cppcheck to succeed even for Foxy.
#ifndef RCLCPP_SMART_PTR_DEFINITIONS
#error "RCLCPP_SMART_PTR_DEFINITIONS undefined"
#endif  // RCLCPP_SMART_PTR_DEFINITIONS
  RCLCPP_SMART_PTR_DEFINITIONS(DDSNode)

  RCLCPP_DDS_PUBLIC
  explicit DDSNode(
    const std::string & node_name,
    const DDSNodeOptions & options = DDSNodeOptions());

  RCLCPP_DDS_PUBLIC
  explicit DDSNode(
    const std::string & node_name,
    const std::string & namespace_,
    const DDSNodeOptions & options = DDSNodeOptions());

private:
  RCLCPP_DISABLE_COPY(DDSNode)
};

}  // namespace rclcpp_dds

#endif  // RCLCPP_DDS__DDS_NODE_HPP_
