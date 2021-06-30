// Copyright 2021 Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef RTI__ROS2__PING__SUBSCRIBER_HPP_
#define RTI__ROS2__PING__SUBSCRIBER_HPP_

#include <rti/ros2/ping/tester.hpp>

/* *INDENT-OFF* */
namespace rti { namespace ros2 { namespace ping {
/* *INDENT-ON* */
template<typename T>
class PingPongSubscriber : public PingPongTester<T>
{
public:
  PingPongSubscriber(
    const char * const name,
    const rclcpp::NodeOptions & options)
  : PingPongTester<T>(name, options, false /* pong */)
  {}

protected:
  // Helper function to fill in the contents of a pong
  virtual void prepare_pong(T * const pong, const uint64_t ping_ts) = 0;

  // Process received ping sample and return the timestamp
  virtual void process_ping(
    dds::sub::LoanedSamples<T> & ping_samples,
    uint64_t & pong_timestamp) = 0;

  // Helper function to dump the contents of a received ping to a string
  virtual void dump_ping(
    dds::sub::LoanedSamples<T> & ping_samples,
    std::ostringstream & msg) = 0;

  virtual void init_test()
  {
    PingPongTester<T>::init_test();

    RCLCPP_INFO(
      this->get_logger(),
      "ping-pong subscriber ready, waiting for publisher...");
  }

  // Overload `on_data()` to propagate ping sample to the pong topic.
  virtual void on_data(dds::sub::LoanedSamples<T> & ping_samples)
  {
    uint64_t ping_ts;
    process_ping(ping_samples, ping_ts);

    if (ping_ts == 0) {
      RCLCPP_INFO(this->get_logger(), "received end ping, exiting");
      this->shutdown();
      return;
    }

    if (this->test_options_.display_received) {
      std::ostringstream msg;

      msg << "[CameraImage] ";

      dump_ping(ping_samples, msg);

      RCLCPP_INFO(this->get_logger(), msg.str().c_str());
    }

    // Send back the timestamp to the writer.
    auto pong = this->alloc_sample();

    prepare_pong(pong, ping_ts);

    this->writer_.write(*pong);
  }
};
}  // namespace ping
}  // namespace ros2
}  // namespace rti

#endif  // RTI__ROS2__PING__SUBSCRIBER_HPP_
