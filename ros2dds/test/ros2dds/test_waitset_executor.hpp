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

#ifndef ROS2DDS__TEST_WAITSET_EXECUTOR_HPP_
#define ROS2DDS__TEST_WAITSET_EXECUTOR_HPP_

#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>
#include <sstream>

#include "std_msgs/msg/String.hpp"

#include "ros2dds/pub.hpp"
#include "ros2dds/sub.hpp"

// Some data structures use to keep track of callback events.
struct ReaderState
{
  const size_t n;
  size_t status_count{0};
  size_t data_count{0};
  size_t received_count{0};
  size_t received_invalid_count{0};
  dds::core::status::SubscriptionMatchedStatus matched_status;

  explicit ReaderState(const size_t id)
  : n(id)
  {}

  template<typename T>
  void on_status(dds::sub::DataReader<T> & reader)
  {
    status_count += 1;
    matched_status = reader.subscription_matched_status();
  }

  template<typename T, typename C>
  void on_data(
    dds::sub::DataReader<T> & reader,
    C & condition,
    const bool take = false)
  {
    data_count += 1;
    auto samples = (take) ?
      reader.select().condition(condition).take() :
      reader.select().condition(condition).read();
    for (const auto & sample : samples) {
      if (sample.info().valid()) {
        received_count += 1;
      } else {
        received_invalid_count += 1;
      }
    }
  }
  void on_data()
  {
    data_count += 1;
    received_count += 1;
  }
};

struct WriterState
{
  size_t status_count{0};
  dds::core::status::PublicationMatchedStatus matched_status;

  template<typename T>
  void on_status(dds::pub::DataWriter<T> & writer)
  {
    status_count += 1;
    matched_status = writer.publication_matched_status();
  }
};

#endif  // ROS2DDS__TEST_WAITSET_EXECUTOR_HPP_
