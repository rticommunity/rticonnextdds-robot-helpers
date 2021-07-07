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

#include <string>

#include "std_msgs/msg/String.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rclcpp_dds_examples/ping/publisher.hpp"
#include "rclcpp_dds_examples/ping/subscriber.hpp"
#include "rclcpp_dds_examples/visibility_control.h"

/******************************************************************************
 * PingPongPublisher implementation for std_msgs::msg::String
 ******************************************************************************/
class StringPingPublisher
  : public rclcpp_dds_examples::PingPongPublisher<std_msgs::msg::String>
{
public:
  RCLCPP_DDS_EXAMPLES_PUBLIC
  explicit StringPingPublisher(const rclcpp::NodeOptions & options)
  : PingPongPublisher("string_pub", options)
  {
    this->init_test();
  }

protected:
  virtual std_msgs::msg::String * alloc_sample()
  {
    return &msg_;
  }

  virtual void prepare_ping(std_msgs::msg::String & ping, const bool final)
  {
    if (final) {
      ping.data(std::to_string(0));
      return;
    }
    ping.data(std::to_string(this->ts_now()));
  }

  // Process received pong sample and return the timestamp
  virtual void process_pong(
    dds::sub::LoanedSamples<std_msgs::msg::String> & pong_samples,
    uint64_t & pong_timestamp)
  {
    pong_timestamp = std::stoull(pong_samples[0].data().data(), nullptr, 0);
  }

  std_msgs::msg::String msg_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(StringPingPublisher)

/******************************************************************************
 * PingPongSubscriber implementation for std_msgs::msg::String
 ******************************************************************************/
class StringPingSubscriber
  : public rclcpp_dds_examples::PingPongSubscriber<std_msgs::msg::String>
{
public:
  RCLCPP_DDS_EXAMPLES_PUBLIC
  explicit StringPingSubscriber(const rclcpp::NodeOptions & options)
  : PingPongSubscriber("string_sub", options)
  {
    this->init_test();
  }

protected:
  virtual std_msgs::msg::String * alloc_sample()
  {
    return &msg_;
  }

  virtual void prepare_pong(
    std_msgs::msg::String * const pong, const uint64_t ping_ts)
  {
    pong->data(std::to_string(ping_ts));
  }

  virtual void process_ping(
    dds::sub::LoanedSamples<std_msgs::msg::String> & ping_samples,
    uint64_t & ping_timestamp)
  {
    ping_timestamp = std::stoull(ping_samples[0].data().data(), nullptr, 0);
  }

  virtual void dump_ping(
    dds::sub::LoanedSamples<std_msgs::msg::String> & ping_samples,
    std::ostringstream & msg)
  {
    msg << ping_samples[0].data().data();
  }

  std_msgs::msg::String msg_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(StringPingSubscriber)
