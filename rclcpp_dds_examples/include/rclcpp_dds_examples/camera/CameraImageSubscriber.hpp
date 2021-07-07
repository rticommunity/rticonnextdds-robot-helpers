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

#ifndef RCLCPP_DDS_EXAMPLES__CAMERA__CAMERAIMAGESUBSCRIBER_HPP_
#define RCLCPP_DDS_EXAMPLES__CAMERA__CAMERAIMAGESUBSCRIBER_HPP_

#include <string>

#include "rclcpp_dds_examples/ping/subscriber.hpp"

#include "camera/CameraCommon.hpp"

namespace rclcpp_dds_examples
{

// This is a generic implementation of the CameraImageSubscriber classes, which
// can be instantiated independently of transfer method and memory binding
// through the use of metaprogramming.
template<typename T>
class CameraImageSubscriber : public rclcpp_dds_examples::PingPongSubscriber<T>
{
public:
  CameraImageSubscriber(
    const char * const name,
    const rclcpp::NodeOptions & options)
  : PingPongSubscriber<T>(name, options)
  {
    this->init_test();
  }

protected:
  virtual void create_endpoints(
    const std::string & writer_topic,
    const std::string & writer_profile,
    const std::string & reader_topic,
    const std::string & reader_profile)
  {
    rclcpp_dds_examples::PingPongSubscriber<T>::create_endpoints(
      writer_topic, writer_profile, reader_topic, reader_profile);
    cached_sample_ = A::prealloc(writer_);
  }

  virtual T * alloc_sample()
  {
    return A::alloc(writer_, cached_sample_);
  }

  virtual void prepare_pong(T * const pong, const uint64_t ping_ts)
  {
    M::get(*pong).timestamp(ping_ts);
  }

  virtual void process_ping(
    dds::sub::LoanedSamples<T> & ping_samples,
    uint64_t & ping_timestamp)
  {
    ping_timestamp = M::get(ping_samples[0].data()).timestamp();
  }

  virtual void dump_ping(
    dds::sub::LoanedSamples<T> & ping_samples,
    std::ostringstream & msg)
  {
    auto & sample = ping_samples[0].data();

    msg << "[" << M::get(sample).timestamp() << "] " << M::get(sample).format();

    for (int i = 0; i < 4; i++) {
      const uint8_t * el;
      M::array::ref(M::get(sample).data(), i, el);

      msg << "0x" <<
        std::hex << std::uppercase <<
        std::setfill('0') << std::setw(2) <<
        static_cast<int>(*el) <<
        std::nouppercase << std::dec <<
        " ";
    }
  }

  CameraImage * cached_sample_;
};

}  // namespace rclcpp_dds_examples

#endif  // RCLCPP_DDS_EXAMPLES__CAMERA__CAMERAIMAGESUBSCRIBER_HPP_
