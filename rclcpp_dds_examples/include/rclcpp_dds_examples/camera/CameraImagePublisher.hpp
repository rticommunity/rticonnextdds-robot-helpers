// Copyright 2021 Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef RCLCPP_DDS_EXAMPLES__CAMERA__CAMERA_IMAGE_PUBLISHER_HPP_
#define RCLCPP_DDS_EXAMPLES__CAMERA__CAMERA_IMAGE_PUBLISHER_HPP_

#include "rclcpp_dds_examples/ping/publisher.hpp"

#include "rti/ros2/data/access.hpp"
#include "rti/ros2/data/memory.hpp"

#include "camera/CameraCommon.hpp"

namespace rclcpp_dds_examples {

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

  virtual T * alloc_sample() {
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

#endif  // RCLCPP_DDS_EXAMPLES__CAMERA__CAMERA_IMAGE_PUBLISHER_HPP_
