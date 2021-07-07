// Copyright 2021 Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#include "rclcpp_dds_examples/visibility_control.h"

#include "rclcpp_dds_examples/ping/publisher.hpp"

#include "camera/CameraImageFlat.hpp"

#include "rclcpp_components/register_node_macro.hpp"

using namespace rti::camera::flat;

namespace rclcpp_dds_examples {

class CameraImagePublisherFlat :
  public rclcpp_dds_examples::PingPongPublisher<CameraImage>
{
public:
  RCLCPP_DDS_EXAMPLES_PUBLIC
  explicit CameraImagePublisherFlat(const rclcpp::NodeOptions & options)
  : PingPongPublisher("camera_pub_flat", options)
  {
    this->init_test();
  }

protected:
  virtual CameraImage * alloc_sample()
  {
    return writer_.extensions().get_loan();
  }

  virtual void prepare_ping(CameraImage & ping, const bool final)
  {
    auto sample = ping.root();

    if (final) {
      sample.timestamp(0);
      return;
    }

    sample.format(rti::camera::common::Format::RGB);
    sample.resolution().height(rti::camera::common::CAMERA_HEIGHT_DEFAULT);
    sample.resolution().width(rti::camera::common::CAMERA_WIDTH_DEFAULT);
    
    // Just set the first 4 bytes
    for (int i = 0; i < 4; i++) {
      uint8_t image_value = (48 + this->count_) % 124;
      sample.data().set_element(i, image_value);
    }
    
    // Update timestamp
    sample.timestamp(this->ts_now());
  }

  // Process received pong sample and return the timestamp
  virtual void process_pong(
    dds::sub::LoanedSamples<CameraImage> & pong_samples,
    uint64_t & pong_timestamp)
  {
    pong_timestamp = pong_samples[0].data().root().timestamp();
  }
};

}  // namespace rclcpp_dds_examples

RCLCPP_COMPONENTS_REGISTER_NODE(rclcpp_dds_examples::CameraImagePublisherFlat)
