// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include "rclcpp_dds/rclcpp_dds.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include "rclcpp_dds_examples/visibility_control.h"

#include "std_msgs/msg/String.hpp"

namespace rclcpp_dds_examples
{
// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class DdsListener : public rclcpp_dds::DdsNode
{
public:
  RCLCPP_DDS_EXAMPLES_PUBLIC
  explicit DdsListener(const rclcpp::NodeOptions & options)
  : DdsNode("dds_listener", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    using std_msgs::msg::String;
    auto reader_qos = this->get_default_datareader_qos();
    reader_qos << dds::core::policy::Reliability::Reliable();
    reader_qos << dds::core::policy::History::KeepLast(7);
    sub_ = this->create_datareader<String>("chatter", reader_qos);
    this->set_data_callback<String>(
      sub_, [this](const String & msg) {
        RCLCPP_INFO(this->get_logger(), "I heard from Connext: [%s]", msg.data().c_str());
      });
  }

private:
  dds::sub::DataReader<std_msgs::msg::String> sub_{nullptr};
};

}  // namespace rclcpp_dds_examples

RCLCPP_COMPONENTS_REGISTER_NODE(rclcpp_dds_examples::DdsListener)
