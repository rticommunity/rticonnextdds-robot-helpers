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

#include "rclcpp_dds_examples/visibility_control.h"

#include "rclcpp_dds_examples/ping/publisher.hpp"

#include "camera/CameraImageFlat.hpp"

#include "rclcpp_components/register_node_macro.hpp"

using rti::camera::flat::CameraImage;

namespace rclcpp_dds_examples
{

class CameraImagePublisherFlat
  : public rclcpp_dds_examples::PingPongPublisher<CameraImage>
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
