// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CONNEXT_NODE_HELPERS__RTI_ROS2_PROCESSOR_ROS_HPP
#define CONNEXT_NODE_HELPERS__RTI_ROS2_PROCESSOR_ROS_HPP

#include "rclcpp/rclcpp.hpp"

namespace rti { namespace ros2 { namespace processor {

template<typename T, typename V, typename MessageT = V>
class Ros2ProcessorNode : public BaseProcessorNode<T,V>
{
public:
  Ros2ProcessorNode(
    const std::string & node_name,
    const std::string & topic_in,
    const std::string & topic_out,
    const int32_t history_depth = 10)
  : BaseProcessorNode<T,V>(node_name, topic_in, topic_out),
    history_depth_(history_depth)
  {}

protected:
  virtual void publish_result(const V & msg)
  {
    pub_->publish(msg);
  }

  virtual void create_output() = 0;

  virtual void create_input() = 0;

  const int32_t history_depth_;
  typename rclcpp::Subscription<T>::SharedPtr sub_;
  typename rclcpp::Publisher<V>::SharedPtr pub_;
};

}  // namespace processor
}  // namespace ros2
}  // namespace rti

#endif  // CONNEXT_NODE_HELPERS__RTI_ROS2_PROCESSOR_ROS_HPP
