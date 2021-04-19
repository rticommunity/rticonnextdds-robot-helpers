// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CONNEXT_NODE_HELPERS__RTI_ROS2_PING_SUBSCRIBER_ROS_HPP
#define CONNEXT_NODE_HELPERS__RTI_ROS2_PING_SUBSCRIBER_ROS_HPP

#include <rti/ros2/ping/tester_ros.hpp>

namespace rti { namespace ros2 { namespace ping {

template<typename T>
class RosPingPongSubscriber : public RosPingPongTester<T>
{
public:
  RosPingPongSubscriber(
    const char * const name,
    const rclcpp::NodeOptions & options)
  : RosPingPongTester<T>(name, options, false /* pong */)
  {}

protected:
  // Helper function to fill in the contents of a pong
  virtual void prepare_pong(T & pong, const uint64_t ping_ts) = 0;

  // Process received ping sample and return the timestamp
  virtual void process_ping(T & ping, uint64_t & pong_timestamp) = 0;

  // Helper function to dump the contents of a received ping to a string
  virtual void dump_ping(T & ping, std::ostringstream & msg) = 0;

  virtual void init_test()
  {
    RosPingPongTester<T>::init_test();

    RCLCPP_INFO(this->get_logger(),
      "ping-pong subscriber ready, waiting for publisher...");
  }

  // Overload `on_message()` to propagate ping sample to the pong topic.
  virtual void on_message(const typename T::SharedPtr ping)
  {
    uint64_t ping_ts;
    process_ping(*ping, ping_ts);

    if (ping_ts == 0) {
      RCLCPP_INFO(this->get_logger(), "received end ping, exiting");
      this->shutdown();
      return;
    }

    if (this->test_options_.display_received) {
      std::ostringstream msg;

      msg << "[CameraImage] ";

      dump_ping(*ping, msg);

      RCLCPP_INFO(this->get_logger(), msg.str().c_str());
    }

    // Send back the timestamp to the writer.
    prepare_pong(*this->cached_sample_, ping_ts);
    
    this->writer_->publish(*this->cached_sample_);
  }
};

}  // namespace ping
}  // namespace ros2
}  // namespace rti

#endif  // CONNEXT_NODE_HELPERS__RTI_ROS2_PING_SUBSCRIBER_ROS_HPP
