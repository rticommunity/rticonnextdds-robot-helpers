// Copyright 2016 Open Source Robotics Foundation, Inc.
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
//
// Copyright 2021 Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp_dds/rclcpp_dds.hpp"

#include "std_msgs/msg/String.hpp"

using namespace std::chrono_literals;
using namespace dds::core;

namespace rclcpp_dds_examples {

class DdsTalker : public rclcpp_dds::DdsNode
{
public:
  DdsTalker()
  : DdsNode("dds_talker"),
    count_(0)
  {
    // Create a function to send messages periodically.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto publish_message =
      [this]() -> void
      {
        std_msgs::msg::String msg("Hello World: " + std::to_string(count_++));
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data().c_str());
        writer_.write(msg);
      };
    auto writer_qos = this->get_default_datawriter_qos(); 
    writer_qos << policy::Reliability::Reliable();
    writer_qos << policy::History(policy::HistoryKind::KEEP_LAST, 7);
    writer_ = this->create_datawriter<std_msgs::msg::String>("chatter", writer_qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  size_t count_ = 1;
  dds::pub::DataWriter<std_msgs::msg::String> writer_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rclcpp_dds_examples

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rclcpp_dds_examples::DdsTalker>());
  rclcpp::shutdown();
  return 0;
}
