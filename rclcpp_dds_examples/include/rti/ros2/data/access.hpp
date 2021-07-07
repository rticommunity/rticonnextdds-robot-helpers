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

#ifndef RTI__ROS2__DATA__ACCESS_HPP_
#define RTI__ROS2__DATA__ACCESS_HPP_

#include <dds/dds.hpp>

#include "rti/topic/flat/FlatData.hpp"

namespace rti
{
namespace ros2
{
namespace data
{
template<typename T>
class DataAccessPlain
{
public:
  static T & get(T & sample)
  {
    return sample;
  }

  static const T & get(const T & sample)
  {
    return sample;
  }

  class array
  {
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
  static typename rti::flat::flat_type_traits<T>::offset get(T & sample)
  {
    return sample.root();
  }

  static typename rti::flat::flat_type_traits<T>::offset::ConstOffset get(const T & sample)
  {
    return sample.root();
  }

  class array
  {
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

#endif  // RTI__ROS2__DATA__ACCESS_HPP_
