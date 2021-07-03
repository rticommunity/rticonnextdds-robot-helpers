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

#ifndef RCLCPP_DDS__DDS_NODE_OPTIONS_HPP_
#define RCLCPP_DDS__DDS_NODE_OPTIONS_HPP_

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_dds/visibility_control.hpp"

namespace rclcpp_dds
{
class DdsNodeOptions : public rclcpp::NodeOptions
{
public:
  RCLCPP_DDS_PUBLIC
  explicit DdsNodeOptions(rcl_allocator_t allocator = rcl_get_default_allocator())
  : NodeOptions(allocator)
  {}

  /// Destructor.
  RCLCPP_DDS_PUBLIC
  virtual
  ~DdsNodeOptions() = default;

  /// Copy constructor.
  RCLCPP_DDS_PUBLIC
  DdsNodeOptions(const DdsNodeOptions & other)
  : NodeOptions(other),
    use_ros_naming_conventions_(other.use_ros_naming_conventions_)
  {}

  /// Assignment operator.
  RCLCPP_DDS_PUBLIC
  DdsNodeOptions &
  operator=(const DdsNodeOptions & other)
  {
    // TODO(asorbini) implement me
    (void)other;
    return *this;
  }

  RCLCPP_DDS_PUBLIC
  bool
  use_ros_naming_conventions() const
  {
    return use_ros_naming_conventions_;
  }

  RCLCPP_DDS_PUBLIC
  DdsNodeOptions &
  use_ros_naming_conventions(const bool the_use_ros_naming_conventions)
  {
    use_ros_naming_conventions_ = the_use_ros_naming_conventions;
    return *this;
  }

private:
  bool use_ros_naming_conventions_{true};
};

}


#endif  // RCLCPP_DDS__DDS_NODE_OPTIONS_HPP_