// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CONNEXT_NODE_HELPERS__RTI_ROS2_PING_PUBLISHER_ROS_HPP
#define CONNEXT_NODE_HELPERS__RTI_ROS2_PING_PUBLISHER_ROS_HPP

#include <rti/ros2/ping/ros_tester.hpp>

namespace rti { namespace ros2 { namespace ping {

template<typename T>
class RosPingPongPublisher : public RosPingPongTester<T>
{
public:
  RosPingPongPublisher(
    const char * const name,
    const rclcpp::NodeOptions & options)
  : RosPingPongTester<T>(name, options, true /* ping */)
  {}

protected:
  // Helper function to fill in the contents of a sample
  virtual void prepare_ping(T & sample, const bool final) = 0;

  // Process received pong sample and return the timestamp
  virtual void process_pong(T & pong, uint64_t & pong_timestamp) = 0;
  
  virtual void init_test()
  {
    RosPingPongTester<T>::init_test();

    // Start timer to periodically check exit conditions
    this->start_exit_timer();
    
    RCLCPP_INFO(this->get_logger(),
      "ping-pong publisher ready, waiting for subscriber...");
  }

  // Overload `test_start()` to initialize test state and send an initial ping.
  virtual void test_start()
  {
    RosPingPongTester<T>::test_start();
    ping();
  }

  // Overload `test_stop()` to print the final computed latency
  virtual void test_stop()
  {
    if (this->test_complete_) {
      print_latency(true /* final */);
    }
    RosPingPongTester<T>::test_stop();
  }

  // Once the test is complete, we send a final ping with timestamp=0 to notify
  // that reader that the test is done. Once the reader is detected as offline,
  // the writer will also exit.
  virtual void test_complete()
  {
    RosPingPongTester<T>::test_complete();
    ping(true /* final */);
  }

  // Compute a running average of the latency measurements and log it to stdout.
  virtual void print_latency(const bool final_log = false)
  {
    std::ostringstream msg;

    if (this->count_ > 0) {
      const uint64_t avg_us = total_latency_ / (this->count_ * 2);
      const double avg_ms = static_cast<double>(avg_us) / 1000.0;
      msg << "Avg-Latency [" << this->count_ << "/" <<
        this->test_options_.max_samples << "] " <<
        std::fixed << std::setprecision(3) << avg_ms << " ms";
      if (final_log) {
        msg << " [FINAL]";
      }
    } else {
      msg << "Avg-Latency: no samples yet";
    }

    RCLCPP_INFO(this->get_logger(), msg.str().c_str());
  }

  // Send a ping message
  virtual void ping(const bool final = false)
  {
    prepare_ping(*this->cached_sample_, final);
    
    // Write the sample out
    this->writer_->publish(*this->cached_sample_);
  }

  // Overload on_message() to process the pong sample and compute the
  // round-trip latency. We store the value halved to compute a running average
  // of one-way latency.
  virtual void on_message(const typename T::SharedPtr pong)
  {
    if (!this->test_active_) {
      RCLCPP_ERROR(this->get_logger(), "pong received while test inactive");
      this->shutdown();
      return;
    }

    // Extract timestamp from pong sample.
    uint64_t pong_ts = 0;
    process_pong(*pong, pong_ts);

    uint64_t receive_ts = this->ts_now();
    // this test will not cope well with a drifting clock
    assert(receive_ts >= pong_ts);
    uint64_t latency = (receive_ts - pong_ts) / 2;

    // we ignore the first few samples to let things ramp up to steady state
    const bool ignored =
      static_cast<uint64_t>(this->test_options_.ignored_initial_samples) > this->count_ignored_;
    if (!ignored) {
      this->count_ += 1;
      total_latency_ += latency;

      if (last_print_ == 0 ||
        receive_ts - static_cast<uint64_t>(this->test_options_.print_interval) > last_print_) {
        print_latency();
        last_print_ = receive_ts;
      }
    } else {
      this->count_ignored_ += 1;
    }

    // Check if thest is complete, otherwise send another ping
    if (!this->check_test_complete())
    {
      ping();
    }
  }

  uint64_t total_latency_ = 0;
  uint64_t last_print_ = 0;
  uint64_t ping_ts_ = 0;
};

}  // namespace ping
}  // namespace ros2
}  // namespace rti

#endif  // CONNEXT_NODE_HELPERS__RTI_ROS2_PING_PUBLISHER_ROS_HPP
