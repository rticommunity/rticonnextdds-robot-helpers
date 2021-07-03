// (c) 2021 Copyright, Real-Time Innovations, Inc. (RTI)
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
#include "rclcpp_dds/dds_node.hpp"

namespace rclcpp_dds
{
DdsNode::DdsNode(
  const std::string & node_name,
  const DdsNodeOptions & options)
: DdsNodeMixin(node_name, options)
{}

DdsNode::DdsNode(
  const std::string & node_name,
  const std::string & namespace_,
  const DdsNodeOptions & options)
: DdsNodeMixin(node_name, namespace_, options)
{}
}  // namespace ros2dds
