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
#ifndef ROS2DDS__READ_OR_TAKE_HPP_
#define ROS2DDS__READ_OR_TAKE_HPP_

#include <chrono>
#include <string>

#include "ros2dds/notifier.hpp"

namespace ros2dds
{
template<
  typename MessageT,
  typename NotifierT = SyncConditionNotifier,
  bool Take = false,
  bool Sync = true>
NotifierT
read_or_take_sync_or_async_w_notifier(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  const std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0),
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
{
  NotifierT notifier;
  auto condition = (Take)?
    notifier.template take_data_callback<MessageT>(reader, cb, data_state)
    : notifier.template read_data_callback<MessageT>(reader, cb, data_state);
  if (Sync) {
    if (max_duration > std::chrono::nanoseconds(0)) {
      notifier.spin(max_duration);
    } else {
      condition->dispatch();
    }
  }
  return notifier;
}

template<
  typename MessageT,
  typename NotifierT = SyncConditionNotifier,
  typename NodeT = rclcpp::Node,
  bool Take = false,
  bool Sync = true>
NotifierT
read_or_take_sync_or_async_w_notifier(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  const std::string & query_expression,
  const std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0),
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
  const std::vector<std::string> & query_parameters = std::vector<std::string>())
{
  NotifierT notifier;
  auto condition = (Take)?
    notifier.template take_data_callback<MessageT>(reader, cb, data_state,
      query_expression, query_parameters)
    : notifier.template read_data_callback<MessageT>(reader, cb, data_state,
      query_expression, query_parameters);
  if (Sync) {
    if (max_duration > std::chrono::nanoseconds(0)) {
      notifier.spin(max_duration);
    } else {
      condition->dispatch();
    }
  }
  return notifier;
}

template<
  typename MessageT,
  typename NotifierT = SyncConditionNotifier,
  typename NodeT = rclcpp::Node>
NotifierT
read_sync(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  const std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0),
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
{
  return read_or_take_sync_or_async_w_notifier<MessageT, NotifierT>(
    reader, cb, max_duration, data_state);
}

template<
  typename MessageT,
  typename NotifierT = SyncConditionNotifier>
NotifierT
read_sync(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  const std::string & query_expression,
  const std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0),
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
  const std::vector<std::string> & query_parameters = std::vector<std::string>())
{
  return read_or_take_sync_or_async_w_notifier<MessageT, NotifierT>(
    reader, cb, query_expression, max_duration, data_state, query_parameters);
}

template<
  typename MessageT,
  typename NotifierT = SyncConditionNotifier>
NotifierT
take_sync(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  const std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0),
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
{
  return read_or_take_sync_or_async_w_notifier<MessageT, NotifierT, true>(
    reader, cb, max_duration, data_state);
}

template<
  typename MessageT,
  typename NotifierT = SyncConditionNotifier>
NotifierT
take_sync(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  const std::string & query_expression,
  const std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0),
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
  const std::vector<std::string> & query_parameters = std::vector<std::string>())
{
  return read_or_take_sync_or_async_w_notifier<MessageT, NotifierT, true>(
    reader, cb, query_expression, max_duration, data_state, query_parameters);
}


template<
  typename MessageT,
  typename NotifierT = AsyncConditionNotifier>
NotifierT
read_async(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  const std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0),
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
{
  return read_or_take_sync_or_async_w_notifier<MessageT, NotifierT, false, false>(
    reader, cb, max_duration, data_state);
}

template<
  typename MessageT,
  typename NotifierT = AsyncConditionNotifier>
NotifierT
read_async(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  const std::string & query_expression,
  const std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0),
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
  const std::vector<std::string> & query_parameters = std::vector<std::string>())
{
  read_or_take_sync_or_async_w_notifier<MessageT, NotifierT, false, false>(
    reader, cb, query_expression, max_duration, data_state, query_parameters);
}

template<
  typename MessageT,
  typename NotifierT = AsyncConditionNotifier>
NotifierT
take_async(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  const std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0),
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
{
  read_or_take_sync_or_async_w_notifier<MessageT, NotifierT, true, false>(
    reader, cb, max_duration, data_state);
}

template<
  typename MessageT,
  typename NotifierT = AsyncConditionNotifier>
NotifierT
take_async(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  const std::string & query_expression,
  const std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0),
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
  const std::vector<std::string> & query_parameters = std::vector<std::string>())
{
  read_or_take_sync_or_async_w_notifier<MessageT, NotifierT, true, false>(
    reader, cb, query_expression, max_duration, data_state, query_parameters);
}
}  // namespace ros2dds

#endif  // ROS2DDS__READ_OR_TAKE_HPP_
