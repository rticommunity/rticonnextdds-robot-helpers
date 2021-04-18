// (c) 2019-2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CONNEXT_NODE_HELPERS__RTI_ROS2_DATA_ACCESS_HPP
#define CONNEXT_NODE_HELPERS__RTI_ROS2_DATA_ACCESS_HPP

#include <dds/dds.hpp>

#include "rti/topic/flat/FlatData.hpp"

namespace rti { namespace ros2 { namespace data {

template<typename T>
class DataAccessPlain
{
public:

  static T & get(T & sample) {
    return sample;
  }

  static const T & get(const T & sample) {
    return sample;
  }

  class array {
  public:
    template<typename A, typename E>
    static void ref(A & array, const size_t i, E & val)
    {
        val = array.data() + i;
    }

    template<typename A, typename E>
    static void set(A & array, const size_t i, E & val)
    {
        array[i] = val;
    }
  };
};

template<typename T>
class DataAccessFlat
{
public:

  static typename rti::flat::flat_type_traits<T>::offset get(T & sample) {
    return sample.root();
  }

  static typename rti::flat::flat_type_traits<T>::offset::ConstOffset get(const T & sample) {
    return sample.root();
  }

  class array {
  public:
    template<typename A, typename E>
    static void ref(A array, const size_t i, E & val)
    {
        val = array.get_elements() + i;
    }

    template<typename A, typename E>
    static void set(A array, const size_t i, E & val)
    {
        array.set_element(i, val);
    }
  };

};

}  // namespace data
}  // namespace ros2
}  // namespace rti

#endif  // CONNEXT_NODE_HELPERS__RTI_ROS2_DATA_ACCESS_HPP
