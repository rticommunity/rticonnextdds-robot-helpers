// (c) 2019-2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CONNEXT_NODE_HELPERS__RTI_ROS2_DATA_MEMORY_HPP
#define CONNEXT_NODE_HELPERS__RTI_ROS2_DATA_MEMORY_HPP

#include <dds/dds.hpp>

namespace rti { namespace ros2 { namespace data {

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

#endif  // CONNEXT_NODE_HELPERS__RTI_ROS2_DATA_MEMORY_HPP
