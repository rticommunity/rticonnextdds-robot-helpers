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

#include "rclcpp_dds/dds_node_mixin.hpp"

namespace rclcpp_dds
{
class DdsNode : public DdsNodeMixin<rclcpp::Node>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(DdsNode)

  RCLCPP_DDS_PUBLIC
  explicit DdsNode(
    const std::string & node_name,
    const DdsNodeOptions & options = DdsNodeOptions());
  
  RCLCPP_DDS_PUBLIC
  explicit DdsNode(
    const std::string & node_name,
    const std::string & namespace_,
    const DdsNodeOptions & options = DdsNodeOptions());

private:
  RCLCPP_DISABLE_COPY(DdsNode)
};

}  // namespace rclcpp_dds

#endif  // RCLCPP_DDS__DDS_NODE_HPP_