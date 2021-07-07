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

#include <string>

#include "rclcpp_dds/dds_node.hpp"

namespace rclcpp_dds
{
DDSNode::DDSNode(
  const std::string & node_name,
  const DDSNodeOptions & options)
: DDSNodeMixin(node_name, options)
{}

DDSNode::DDSNode(
  const std::string & node_name,
  const std::string & namespace_,
  const DDSNodeOptions & options)
: DDSNodeMixin(node_name, namespace_, options)
{}
}  // namespace rclcpp_dds
