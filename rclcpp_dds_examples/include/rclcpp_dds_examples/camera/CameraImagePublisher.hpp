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

#ifndef RCLCPP_DDS_EXAMPLES__CAMERA__CAMERAIMAGEPUBLISHER_HPP_
#define RCLCPP_DDS_EXAMPLES__CAMERA__CAMERAIMAGEPUBLISHER_HPP_

#include <string>

#include "rclcpp_dds_examples/ping/publisher.hpp"

#include "rti/ros2/data/access.hpp"
#include "rti/ros2/data/memory.hpp"

#include "camera/CameraCommon.hpp"

namespace rclcpp_dds_examples
{

// This is a generic implementation of the CameraImagePublisher classes, which
// can be instantiated independently of transfer method and memory binding
// through the use of metaprogramming.
template<typename T, typename A, typename M>
class CameraImagePublisher : public rclcpp_dds_examples::PingPongPublisher<T>
{
public:
  CameraImagePublisher(
    const char * const name,
    const rclcpp::NodeOptions & options)
  : rclcpp_dds_examples::PingPongPublisher<T>(name, options)
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
    rclcpp_dds_examples::PingPongPublisher<T>::create_endpoints(
      writer_topic, writer_profile, reader_topic, reader_profile);
    cached_sample_ = A::prealloc(writer_);
  }

  virtual T * alloc_sample()
  {
    return A::alloc(writer_, cached_sample_);
  }

  virtual void prepare_ping(T & ping, const bool final)
  {
    if (final) {
      M::get(ping).timestamp(0);
      return;
    }

    M::get(ping).format(rti::camera::common::Format::RGB);
    M::get(ping).resolution().height(rti::camera::common::CAMERA_HEIGHT_DEFAULT);
    M::get(ping).resolution().width(rti::camera::common::CAMERA_WIDTH_DEFAULT);

    // Just set the first 4 bytes
    for (int i = 0; i < 4; i++) {
      uint8_t image_value = (48 + this->count_) % 124;
      M::array::set(M::get(ping).data(), i, image_value);
    }

    // Update timestamp
    M::get(ping).timestamp(this->participant_->current_time().to_microsecs());
  }

  // Process received pong sample and return the timestamp
  virtual void process_pong(
    dds::sub::LoanedSamples<T> & pong_samples,
    uint64_t & pong_timestamp)
  {
    pong_timestamp = M::get(pong_samples[0].data()).timestamp();
  }

  CameraImage * cached_sample_;
};

}  // namespace rclcpp_dds_examples

#endif  // RCLCPP_DDS_EXAMPLES__CAMERA__CAMERAIMAGEPUBLISHER_HPP_
