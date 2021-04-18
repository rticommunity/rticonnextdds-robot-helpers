// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CONNEXT_NODE_HELPERS__RTI_ROS2_PROCESSOR_BASE_HPP
#define CONNEXT_NODE_HELPERS__RTI_ROS2_PROCESSOR_BASE_HPP

#include "rclcpp/rclcpp.hpp"

#include <rti/ros2/processor/processor_base.hpp>

namespace rti { namespace ros2 { namespace processor {

template<typename T, typename V>
class BaseProcessorNode : public rclcpp::Node
{
public:
  BaseProcessorNode(
    const std::string & node_name,
    const std::string & topic_in,
    const std::string & topic_out)
  : Node(node_name),
    topic_in_(topic_in),
    topic_out_(topic_out)
  {}

protected:
  virtual void create_entities()
  {
    create_input();
    create_output();
  }

  virtual void publish_result(const V & msg) = 0;

  virtual void create_output() = 0;

  virtual void create_input() = 0;

  virtual V & result_message() = 0;

  virtual void process_message(const T & msg_in, V & msg_out) = 0;

  virtual void on_data(const T & msg) {
    V & result_msg = result_message();
    process_message(msg, result_msg);
    this->publish_result(result_msg);
  }

  std::string topic_in_;
  std::string topic_out_;
};

}  // namespace processor
}  // namespace ros2
}  // namespace rti

#endif  // CONNEXT_NODE_HELPERS__RTI_ROS2_PROCESSOR_BASE_HPP
