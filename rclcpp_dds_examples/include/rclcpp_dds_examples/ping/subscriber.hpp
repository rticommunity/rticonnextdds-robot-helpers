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

#ifndef RCLCPP_DDS_EXAMPLES__PING__SUBSCRIBER_HPP_
#define RCLCPP_DDS_EXAMPLES__PING__SUBSCRIBER_HPP_

#include "rclcpp_dds_examples/ping/tester.hpp"

namespace rclcpp_dds_examples
{
template<typename T>
class PingPongSubscriber : public PingPongTester<T>
{
public:
  PingPongSubscriber(
    const char * const name,
    const rclcpp_dds::DDSNodeOptions & options)
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
}  // namespace rclcpp_dds_examples

#endif  // RCLCPP_DDS_EXAMPLES__PING__SUBSCRIBER_HPP_
