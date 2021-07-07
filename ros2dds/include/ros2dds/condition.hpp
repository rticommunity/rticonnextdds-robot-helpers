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
#ifndef ROS2DDS__CONDITION_HPP_
#define ROS2DDS__CONDITION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ros2dds/domain.hpp"

namespace ros2dds
{
namespace condition
{
template<typename ConditionT>
using ConditionCallback = std::function<void (ConditionT &)>;

template<typename EntityT, typename ConditionT>
using EntityCallback = std::function<void (ConditionT &, EntityT &)>;

template<typename MessageT>
using DataCallback =
  EntityCallback<dds::sub::DataReader<MessageT>, dds::sub::cond::ReadCondition>;

template<typename MessageT>
using DataQueryCallback =
  EntityCallback<dds::sub::DataReader<MessageT>, dds::sub::cond::QueryCondition>;

template<typename MessageT>
using DataReaderStatusCallback =
  EntityCallback<dds::sub::DataReader<MessageT>, dds::core::cond::StatusCondition>;

template<typename MessageT>
using DataWriterStatusCallback =
  EntityCallback<dds::pub::DataWriter<MessageT>, dds::core::cond::StatusCondition>;

template<typename MessageT>
using MessageCallback = std::function<void (const MessageT &)>;

template<typename StatusT>
using EntityStatusCallback = std::function<void (const StatusT &)>;

template<typename EntityT, typename StatusT>
using StatusGetter = std::function<StatusT(EntityT &)>;

using SubscriptionMatchedStatusCallback =
  EntityStatusCallback<dds::core::status::SubscriptionMatchedStatus>;

using PublicationMatchedStatusCallback =
  EntityStatusCallback<dds::core::status::PublicationMatchedStatus>;

using UserCallback = ConditionCallback<dds::core::cond::GuardCondition>;

using SimpleUserCallback = std::function<void ()>;

template<typename MessageT>
dds::sub::cond::ReadCondition
data(
  dds::sub::DataReader<MessageT> reader,
  DataCallback<MessageT> cb,
  const dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
{
  auto cond_ref = std::make_shared<dds::sub::cond::ReadCondition>(nullptr);
  auto reader_ref = std::make_shared<dds::sub::DataReader<MessageT>>(reader);
  dds::sub::cond::ReadCondition cond(reader, data_state,
    [cb, cond_ref, reader_ref]() {
      cb(*cond_ref, *reader_ref);
    });
  *cond_ref = cond;
  return cond;
}

template<typename MessageT, bool Take = false>
dds::sub::cond::ReadCondition
read_data(
  dds::sub::DataReader<MessageT> reader,
  MessageCallback<MessageT> cb,
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
{
  auto cond_ref = std::make_shared<dds::sub::cond::ReadCondition>(nullptr);
  auto reader_ref = std::make_shared<dds::sub::DataReader<MessageT>>(reader);
  dds::sub::cond::ReadCondition cond(reader, data_state,
    [cb, cond_ref, reader_ref]() {
      auto selector = reader_ref->select().condition(*cond_ref);
      auto samples = (Take) ? selector.take() : selector.read();
      for (const auto & sample : samples) {
        if (sample.info().valid()) {
          cb(sample.data());
        }
      }
    });
  *cond_ref = cond;
  return cond;
}

template<typename MessageT>
dds::sub::cond::QueryCondition
query(
  dds::sub::DataReader<MessageT> reader,
  DataQueryCallback<MessageT> cb,
  const std::string & query_expression,
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
  const std::vector<std::string> & query_parameters = std::vector<std::string>())
{
  auto cond_ref = std::make_shared<dds::sub::cond::QueryCondition>(nullptr);
  auto reader_ref = std::make_shared<dds::sub::DataReader<MessageT>>(reader);
  dds::sub::Query query(reader, query_expression, query_parameters);
  dds::sub::cond::QueryCondition cond(query, data_state,
    [cb, cond_ref, reader_ref]() {
      cb(*cond_ref, *reader_ref);
    });
  *cond_ref = cond;
  return cond;
}

template<typename MessageT, bool Take = false>
dds::sub::cond::QueryCondition
read_query(
  dds::sub::DataReader<MessageT> reader,
  MessageCallback<MessageT> cb,
  const std::string & query_expression,
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
  const std::vector<std::string> & query_parameters = std::vector<std::string>())
{
  auto cond_ref = std::make_shared<dds::sub::cond::QueryCondition>(nullptr);
  auto reader_ref = std::make_shared<dds::sub::DataReader<MessageT>>(reader);
  dds::sub::Query query(reader, query_expression, query_parameters);
  dds::sub::cond::QueryCondition cond(query, data_state,
    [cb, cond_ref, reader_ref]() {
      auto selector = reader_ref->select().condition(*cond_ref);
      auto samples = (Take) ? selector.take() : selector.read();
      for (const auto & sample : samples) {
        if (sample.info().valid()) {
          cb(sample.data());
        }
      }
    });
  *cond_ref = cond;
  return cond;
}

template<typename MessageT>
dds::sub::cond::ReadCondition
take_data(
  dds::sub::DataReader<MessageT> reader,
  MessageCallback<MessageT> cb,
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
{
  return read_data<MessageT, true>(reader, cb, data_state);
}

template<typename MessageT>
dds::sub::cond::QueryCondition
take_query(
  dds::sub::DataReader<MessageT> reader,
  MessageCallback<MessageT> cb,
  const std::string & query_expression,
  dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
  const std::vector<std::string> & query_parameters = std::vector<std::string>())
{
  return read_query<MessageT, true>(
    reader, cb, query_expression, data_state, query_parameters);
}


template<typename MessageT>
dds::core::cond::StatusCondition
status(
  dds::sub::DataReader<MessageT> reader,
  DataReaderStatusCallback<MessageT> cb,
  const dds::core::status::StatusMask enabled_statuses)
{
  dds::core::cond::StatusCondition cond(reader);
  auto cond_ref = std::make_shared<dds::core::cond::StatusCondition>(cond);
  auto reader_ref = std::make_shared<dds::sub::DataReader<MessageT>>(reader);
  cond->enabled_statuses(enabled_statuses);
  cond->handler(
    [cb, cond_ref, reader_ref]() {
      cb(*cond_ref, *reader_ref);
    });
  *cond_ref = cond;
  return cond;
}

template<typename MessageT>
dds::core::cond::StatusCondition
status(
  dds::pub::DataWriter<MessageT> writer,
  DataWriterStatusCallback<MessageT> cb,
  const dds::core::status::StatusMask enabled_statuses)
{
  dds::core::cond::StatusCondition cond(writer);
  auto cond_ref = std::make_shared<dds::core::cond::StatusCondition>(cond);
  auto writer_ref = std::make_shared<dds::pub::DataWriter<MessageT>>(writer);
  cond->enabled_statuses(enabled_statuses);
  cond->handler(
    [cb, cond_ref, writer_ref]() {
      cb(*cond_ref, *writer_ref);
    });
  *cond_ref = cond;
  return cond;
}

inline
dds::core::cond::GuardCondition
guard(UserCallback cb)
{
  auto cond_ref = std::make_shared<dds::core::cond::GuardCondition>();
  dds::core::cond::GuardCondition cond;
  cond->handler(
    [cb, cond_ref]() {
      cb(*cond_ref);
    });
  *cond_ref = cond;
  return cond;
}

inline
dds::core::cond::GuardCondition
guard(SimpleUserCallback cb)
{
  auto cond_ref = std::make_shared<dds::core::cond::GuardCondition>();
  dds::core::cond::GuardCondition cond;
  cond->handler(
    [cb, cond_ref]() {
      cb();
      cond_ref->trigger_value(false);
    });
  *cond_ref = cond;
  return cond;
}

}  // namespace condition

}  // namespace ros2dds

#endif  // ROS2DDS__CONDITION_HPP_
