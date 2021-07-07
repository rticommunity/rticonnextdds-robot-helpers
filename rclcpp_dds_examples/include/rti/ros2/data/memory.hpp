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

#ifndef RTI__ROS2__DATA__MEMORY_HPP_
#define RTI__ROS2__DATA__MEMORY_HPP_

#include <dds/dds.hpp>

namespace rti
{
namespace ros2
{
namespace data
{
template<typename T>
class DataMemoryDynamic
{
public:
  // Dynamically pre-allocate a sample.
  static T * prealloc(
    dds::pub::DataWriter<T> & writer)
  {
    (void)writer;

    return new T();
  }

  static T * alloc(
    dds::pub::DataWriter<T> & writer,
    T * const preallocd_sample)
  {
    (void)writer;
    assert(preallocd_sample != nullptr);

    return preallocd_sample;
  }
};

template<typename T>
class DataMemoryLoan
{
public:
  // No pre-allocated samples, since we are going to loan them from the writer
  static T * prealloc(
    dds::pub::DataWriter<T> & writer)
  {
    (void)writer;

    return nullptr;
  }

  // Return a sample loaned from the DataWriter
  static T * alloc(
    dds::pub::DataWriter<T> & writer,
    T * const preallocd_sample)
  {
    (void)preallocd_sample;
    assert(preallocd_sample == nullptr);

    return writer.extensions().get_loan();
  }
};

}  // namespace data
}  // namespace ros2
}  // namespace rti

#endif  // RTI__ROS2__DATA__MEMORY_HPP_
