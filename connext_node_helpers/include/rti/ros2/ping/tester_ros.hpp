// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CONNEXT_NODE_HELPERS__RTI_ROS2_PING_TESTER_ROS_HPP
#define CONNEXT_NODE_HELPERS__RTI_ROS2_PING_TESTER_ROS_HPP

#include <rti/ros2/ping/tester.hpp>

#include <rti/ros2/data/access.hpp>

namespace rti { namespace ros2 { namespace ping {

template<typename T>
class RosPingPongTester : public rclcpp::Node
{
protected:
  RosPingPongTester(
    const char * const name,
    const rclcpp::NodeOptions & options,
    const bool ping)
  : Node(name, options),
    ping_(ping)
  {
    PingPongTesterOptions::declare(this);
    rclcpp::Parameter p_ping("topic_name_ping", "ping");
    this->set_parameter(p_ping);
    rclcpp::Parameter p_pong("topic_name_pong", "pong");
    this->set_parameter(p_pong);
  }

  // Initialize DDS entities used by the tester. Each operation is delegated to
  // a virtual function, so that subclasses may override each step as needed.
  virtual void init_test()
  {
    // Load test configuration from ROS 2 parameters
    test_options_ = PingPongTesterOptions::load(this);

    const std::string * writer_topic,
                      * reader_topic;

    if (ping_) {
      writer_topic = &test_options_.topic_name_ping;
      reader_topic = &test_options_.topic_name_pong;
    } else {
      writer_topic = &test_options_.topic_name_pong;
      reader_topic = &test_options_.topic_name_ping;
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();

    writer_ = this->create_publisher<T>(*writer_topic, qos);

    reader_ = this->create_subscription<T>(
      *reader_topic, qos, [this](const typename T::SharedPtr msg){
        on_message(msg);
      });

    cached_sample_ = std::make_shared<T>();

    // Since we don't have a way to wait for matches in the ROS 2 API we just
    // call test_start() to begin the test immediately.
    test_start();
  }

  // Disable event notification and call `rclcpp::shutdown()` to stop process.
  virtual void shutdown()
  {
    test_active_ = false;
    rclcpp::shutdown();
  }

  virtual uint64_t ts_now()
  {
    uint64_t ts = 0;
    do {
      ts = this->now().nanoseconds() / 1000;
    } while (!ts);
    return ts;
  }

  // Start a timer to force the test to exit after the maximum execution time
  // has expired.
  virtual void start_exit_timer()
  {
    using namespace std::chrono_literals;

    if (test_options_.max_execution_time > 0) {
      exit_timer_ = this->create_wall_timer(1s, [this](){
        if (check_test_complete()) {
          exit_timer_->cancel();
          exit_timer_ = this->create_wall_timer(500ms, [this](){
            this->shutdown();
          });
        }
      });
    }
  }

  // Check if the test is complete, because either:
  // - The maximum number of samples were published/received.
  // - The maximum allowed run time was reached.
  virtual bool is_test_complete()
  {
    auto ts = this->ts_now();

    const bool time_expired =
      start_ts_ > 0 && test_options_.max_execution_time > 0 &&
        ts - static_cast<uint64_t>(test_options_.max_execution_time) > start_ts_;

    const bool max_samples_reached =
      test_options_.max_samples > 0 &&
        count_ >= static_cast<uint64_t>(test_options_.max_samples);
        
    return time_expired || max_samples_reached;
  }

  // Called before sending a new ping. Return true if the test is complete,
  // informing the caller not to start a new iteration of the test.
  virtual bool check_test_complete()
  {
    if (is_test_complete())
    {
      test_complete();
      return true;
    }
    return false;
  }

  virtual void on_message(const typename T::SharedPtr msg)
  {
    (void)msg;
  }

  virtual void test_start()
  {
    RCLCPP_INFO(this->get_logger(), "beginning test");
    test_active_ = true;
    test_complete_ = false;
    count_ignored_ = 0;
    count_ = 0;
    start_ts_ = this->ts_now();
  }

  // Mark test as inactive and call shutdown() to terminate process.
  virtual void test_stop()
  {
    auto stop_ts = this->ts_now();
    const uint64_t run_time = stop_ts - start_ts_;
    const double run_time_s = run_time / 1000000.0;
    RCLCPP_INFO(this->get_logger(), "test stopped after %lf s", run_time_s);
    
    test_active_ = false;

    if (test_complete_) {
      RCLCPP_INFO(this->get_logger(), "test completed");
    } else {
      RCLCPP_ERROR(this->get_logger(), "test interrupted before completion");
    }

    shutdown();
  }

  // Callback to perform custom operation when the test is considered complete.
  virtual void test_complete()
  {
    test_complete_ = true;
  }

  const bool ping_;
  PingPongTesterOptions test_options_;
  bool test_active_{false};
  bool test_complete_{false};
  uint64_t count_ = 0;
  uint64_t count_ignored_ = 0;
  uint64_t start_ts_ = 0;

  typename T::SharedPtr cached_sample_;
  typename rclcpp::Publisher<T>::SharedPtr writer_;
  typename rclcpp::Subscription<T>::SharedPtr reader_;
  rclcpp::TimerBase::SharedPtr exit_timer_;
};

}  // namespace ping
}  // namespace ros2
}  // namespace rti

#endif  // CONNEXT_NODE_HELPERS__RTI_ROS2_PING_TESTER_ROS_HPP
