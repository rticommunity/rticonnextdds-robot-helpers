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
#ifndef ROS2DDS__CALLBACK_HPP_
#define ROS2DDS__CALLBACK_HPP_

#include <condition_variable>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <limits>
#include <functional>

#include "rti/core/cond/AsyncWaitSet.hpp"

#include "rclcpp/scope_exit.hpp"

#include "ros2dds/domain.hpp"

namespace ros2dds
{
template<typename ConditionT>
using ConditionCallbackFunction = std::function<void(ConditionT&)>;

template<typename EntityT, typename ConditionT>
using EntityCallbackFunction = std::function<void(ConditionT&,EntityT&)>;

template<typename MessageT>
using DataCallbackFunction =
  EntityCallbackFunction<dds::sub::DataReader<MessageT>, dds::sub::cond::ReadCondition>;
  
template<typename MessageT>
using DataQueryCallbackFunction =
  EntityCallbackFunction<dds::sub::DataReader<MessageT>, dds::sub::cond::QueryCondition>;

template<typename MessageT>
using DataReaderStatusCallbackFunction =
  EntityCallbackFunction<dds::sub::DataReader<MessageT>, dds::core::cond::StatusCondition>;

template<typename MessageT>
using DataWriterStatusCallbackFunction =
  EntityCallbackFunction<dds::pub::DataWriter<MessageT>, dds::core::cond::StatusCondition>;

template<typename MessageT>
using MessageCallbackFunction = std::function<void(const MessageT&)>;

template<typename StatusT>
using EntityStatusCallbackFunction = std::function<void(const StatusT&)>;

template<typename EntityT, typename StatusT>
using StatusGetterFunction = std::function<StatusT(EntityT&)>;

using SubscriptionMatchedStatusCallbackFunction =
  EntityStatusCallbackFunction<dds::core::status::SubscriptionMatchedStatus>;

using PublicationMatchedStatusCallbackFunction =
  EntityStatusCallbackFunction<dds::core::status::PublicationMatchedStatus>;

using UserCallbackFunction = ConditionCallbackFunction<dds::core::cond::GuardCondition>;

using SimpleUserCallbackFunction = std::function<void()>;

template<typename MessageT>
dds::sub::cond::ReadCondition
data_callback(
  dds::sub::DataReader<MessageT> reader,
  DataCallbackFunction<MessageT> cb,
  dds::sub::status::DataState data_state)
{
  auto cond_ref = std::make_shared<dds::sub::cond::ReadCondition>(nullptr);
  auto reader_ref = std::make_shared<dds::sub::DataReader<MessageT>>(reader);
  dds::sub::cond::ReadCondition cond(reader, data_state,
    [cb, cond_ref, reader_ref](){
      cb(*cond_ref, *reader_ref);
    });
  *cond_ref = cond;
  return cond;
}

template<typename MessageT, bool Take>
dds::sub::cond::ReadCondition
read_or_take_data_callback(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
{
  auto cond_ref = std::make_shared<dds::sub::cond::ReadCondition>(nullptr);
  auto reader_ref = std::make_shared<dds::sub::DataReader<MessageT>>(reader);
  dds::sub::cond::ReadCondition cond(reader, data_state,
    [cb, cond_ref, reader_ref](){
      auto selector = reader_ref->select().condition(*cond_ref);
      auto samples = (Take)? selector.take() : selector.read();
      for (const auto& sample : samples) {
        cb(sample.data());
      }
    });
  *cond_ref = cond;
  return cond;
}

template<typename MessageT>
dds::sub::cond::QueryCondition
data_callback(
  dds::sub::DataReader<MessageT> reader,
  DataQueryCallbackFunction<MessageT> cb,
  const std::string & query_expression,
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
  const std::vector<std::string> & query_parameters = std::vector<std::string>())
{
  auto cond_ref = std::make_shared<dds::sub::cond::QueryCondition>(nullptr);
  auto reader_ref = std::make_shared<dds::sub::DataReader<MessageT>>(reader);
  dds::sub::Query query(reader, query_expression, query_parameters);
  dds::sub::cond::QueryCondition cond(query, data_state,
    [cb, cond_ref, reader_ref](){
      cb(*cond_ref, *reader_ref);
    });
  *cond_ref = cond;
  return cond;
}

template<typename MessageT, bool Take>
dds::sub::cond::QueryCondition
read_or_take_data_callback(
  dds::sub::DataReader<MessageT> reader,
  MessageCallbackFunction<MessageT> cb,
  const std::string & query_expression,
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
  const std::vector<std::string> & query_parameters = std::vector<std::string>())
{
  auto cond_ref = std::make_shared<dds::sub::cond::QueryCondition>(nullptr);
  auto reader_ref = std::make_shared<dds::sub::DataReader<MessageT>>(reader);
  dds::sub::Query query(reader, query_expression, query_parameters);
  dds::sub::cond::QueryCondition cond(query, data_state,
    [cb, cond_ref, reader_ref](){
      auto selector = reader_ref->select().condition(*cond_ref);
      auto samples = (Take)? selector.take() : selector.read();
      for (const auto& sample : samples) {
        cb(sample.data());
      }
    });
  *cond_ref = cond;
  return cond;
}

template<typename MessageT>
dds::core::cond::StatusCondition
status_callback(
  dds::sub::DataReader<MessageT> reader,
  DataReaderStatusCallbackFunction<MessageT> cb,
  const dds::core::status::StatusMask enabled_statuses)
{
  dds::core::cond::StatusCondition cond(reader);
  auto cond_ref = std::make_shared<dds::core::cond::StatusCondition>(cond);
  auto reader_ref = std::make_shared<dds::sub::DataReader<MessageT>>(reader);
  cond->enabled_statuses(enabled_statuses);
  cond->handler([cb, cond_ref, reader_ref](){
    cb(*cond_ref, *reader_ref);
  });
  *cond_ref = cond;
  return cond;
}

template<typename MessageT>
dds::core::cond::StatusCondition
status_callback(
  dds::pub::DataWriter<MessageT> writer,
  DataWriterStatusCallbackFunction<MessageT> cb,
  const dds::core::status::StatusMask enabled_statuses)
{
  dds::core::cond::StatusCondition cond(writer);
  auto cond_ref = std::make_shared<dds::core::cond::StatusCondition>(cond);
  auto writer_ref = std::make_shared<dds::pub::DataWriter<MessageT>>(writer);
  cond->enabled_statuses(enabled_statuses);
  cond->handler([cb, cond_ref, writer_ref](){
    cb(*cond_ref, *writer_ref);
  });
  *cond_ref = cond;
  return cond;
}

dds::core::cond::GuardCondition
user_callback(UserCallbackFunction cb)
{
  auto cond_ref = std::make_shared<dds::core::cond::GuardCondition>();
  dds::core::cond::GuardCondition cond;
  cond->handler([cb, cond_ref](){
    cb(*cond_ref);
  });
  *cond_ref = cond;
  return cond;
}

dds::core::cond::GuardCondition
user_callback(SimpleUserCallbackFunction cb)
{
  auto cond_ref = std::make_shared<dds::core::cond::GuardCondition>();
  dds::core::cond::GuardCondition cond;
  cond->handler([cb, cond_ref](){
    cb();
    cond_ref->trigger_value(false);
  });
  *cond_ref = cond;
  return cond;
}

}  // namespace ros2dds

#endif  // ROS2DDS__CALLBACK_HPP_
