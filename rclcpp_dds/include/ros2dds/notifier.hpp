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
#ifndef ROS2DDS__NOTIFIER_HPP_
#define ROS2DDS__NOTIFIER_HPP_

#include <condition_variable>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <limits>
#include <functional>

#include "ros2dds/callback.hpp"
#include "ros2dds/waitset.hpp"

namespace ros2dds
{
template<typename WaitSetExecT>
class ConditionNotifier
{
public:
  template<typename MessageT>
  dds::sub::cond::ReadCondition
  data_callback(
    dds::sub::DataReader<MessageT> reader,
    DataCallbackFunction<MessageT> cb,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    assert_detached_data(reader);
    auto condition = ros2dds::data_callback<MessageT>(reader,
      [this, cb](
        dds::sub::cond::ReadCondition & condition,
        dds::sub::DataReader<MessageT> & reader)
      {
        cb(condition, reader);
        on_callback();
      },
      data_state);
    attach_condition(condition);
    return condition;
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
    assert_detached_data(reader);
    auto condition = ros2dds::data_callback<MessageT>(reader,
      [this, cb](
        dds::sub::cond::QueryCondition & condition,
        dds::sub::DataReader<MessageT> & reader)
      {
        cb(condition, reader);
        on_callback();
      },
      query_expression,
      data_state,
      query_parameters);
    attach_condition(condition);
    return condition;
  }

  template<typename MessageT>
  dds::sub::cond::ReadCondition
  read_data_callback(
    dds::sub::DataReader<MessageT> reader,
    MessageCallbackFunction<MessageT> cb,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    return attach_read_or_take_data_callback<MessageT, false>(reader, cb, data_state);
  }

  template<typename MessageT>
  dds::sub::cond::QueryCondition
  read_data_callback(
    dds::sub::DataReader<MessageT> reader,
    MessageCallbackFunction<MessageT> cb,
    const std::string & query_expression,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
    const std::vector<std::string> & query_parameters = std::vector<std::string>())
  {
    return attach_read_or_take_data_callback<MessageT, false>(
      reader, cb, query_expression, data_state, query_parameters);
  }

  template<typename MessageT>
  dds::sub::cond::ReadCondition
  take_data_callback(
    dds::sub::DataReader<MessageT> reader,
    MessageCallbackFunction<MessageT> cb,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    return attach_read_or_take_data_callback<MessageT, true>(reader, cb, data_state);
  }

  template<typename MessageT>
  dds::sub::cond::QueryCondition
  take_data_callback(
    dds::sub::DataReader<MessageT> reader,
    MessageCallbackFunction<MessageT> cb,
    const std::string & query_expression,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
    const std::vector<std::string> & query_parameters = std::vector<std::string>())
  {
    return attach_read_or_take_data_callback<MessageT, true>(
      reader, cb, query_expression, data_state, query_parameters);
  }

  template<typename MessageT>
  dds::core::cond::StatusCondition
  status_callback(
    dds::sub::DataReader<MessageT> reader,
    DataReaderStatusCallbackFunction<MessageT> cb,
    const dds::core::status::StatusMask enabled_statuses)
  {
    return attach_status_callback<
      MessageT,
      dds::sub::DataReader<MessageT>,
      DataReaderStatusCallbackFunction<MessageT>>(reader, cb, enabled_statuses);
  }

  template<typename MessageT>
  dds::core::cond::StatusCondition
  status_callback(
    dds::pub::DataWriter<MessageT> writer,
    DataWriterStatusCallbackFunction<MessageT> cb,
    const dds::core::status::StatusMask enabled_statuses)
  {
    return attach_status_callback<
      MessageT,
      dds::pub::DataWriter<MessageT>,
      DataWriterStatusCallbackFunction<MessageT>>(writer, cb, enabled_statuses);
  }

  dds::core::cond::GuardCondition
  user_callback(UserCallbackFunction cb)
  {
    auto condition = ros2dds::user_callback(
      [this, cb](dds::core::cond::GuardCondition & condition){
        cb(condition);
        on_callback();
      });
    attach_condition(condition);
    return condition;
  }

  dds::core::cond::GuardCondition
  user_callback(SimpleUserCallbackFunction cb)
  {
    auto condition = ros2dds::user_callback(
      [this, cb](){
        cb();
        on_callback();
      });
    attach_condition(condition);
    return condition;
  }

  void detach(const dds::sub::cond::ReadCondition & read_condition)
  {
    detach_condition(read_condition);
  }

  void detach(const dds::sub::cond::QueryCondition & query_condition)
  {
    detach_condition(query_condition);
  }

  void detach(const dds::core::cond::StatusCondition & status_condition)
  {

    detach_status_condition(status_condition);
  }

  void detach(const dds::core::cond::GuardCondition & guard_condition)
  {
    detach_condition(guard_condition);
  }

  virtual void
  spin(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0))
  {
    exec_.spin(max_duration);
  }

  virtual void
  shutdown()
  {
    exec_.shutdown();
  }

protected:
  template<typename MessageT, bool Take>
  dds::sub::cond::ReadCondition
  attach_read_or_take_data_callback(
    dds::sub::DataReader<MessageT> reader,
    MessageCallbackFunction<MessageT> cb,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    assert_detached_data(reader);
    auto condition = ros2dds::read_or_take_data_callback<MessageT, Take>(reader,
      [this, cb](const MessageT & msg)
      {
        cb(msg);
        on_callback();
      },
      data_state);
    attach_condition(condition);
    return condition;
  }

  template<typename MessageT, bool Take>
  dds::sub::cond::QueryCondition
  attach_read_or_take_data_callback(
    dds::sub::DataReader<MessageT> reader,
    MessageCallbackFunction<MessageT> cb,
    const std::string & query_expression,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data(),
    const std::vector<std::string> & query_parameters = std::vector<std::string>())
  {
    assert_detached_data(reader);
    auto condition = ros2dds::read_or_take_data_callback<MessageT, Take>(
      reader,
      [this, cb](const MessageT & msg)
      {
        cb(msg);
        on_callback();
      },
      query_expression,
      data_state,
      query_parameters);
    attach_condition(condition);
    return condition;
  }

  template<
    typename MessageT,
    typename EntityT,
    typename CallbackT>
  dds::core::cond::StatusCondition
  attach_status_callback(
    EntityT entity,
    CallbackT cb,
    const dds::core::status::StatusMask enabled_statuses)
  {
    assert_detached_status(entity);
    auto condition = ros2dds::status_callback<MessageT>(entity,
      [this, cb](
        dds::core::cond::StatusCondition & condition,
        EntityT & entity)
      {
        cb(condition, entity);
        on_callback();
      },
      enabled_statuses);
    attach_status_condition(condition);
    return condition;
  }

  void on_callback()
  {
    exec_.wake_up();
  }

  void assert_detached_status(dds::core::Entity entity)
  {
    for (auto &&e : entities_)
    {
      if (entity == e) {
        rclcpp::exceptions::throw_from_rcl_error(
            RCL_RET_INVALID_ARGUMENT, "entity already attached");
      }
    }
  }

  void assert_detached_data(dds::sub::AnyDataReader reader)
  {
    for (auto &&r : readers_)
    {
      if (reader == r) {
        rclcpp::exceptions::throw_from_rcl_error(
            RCL_RET_INVALID_ARGUMENT, "reader already attached");
      }
    }
  }

  void attach_condition(const dds::core::cond::Condition & condition)
  {
    conditions_.push_back(condition);
    exec_.attach(condition);
  }

  void attach_status_condition(dds::core::cond::StatusCondition & condition)
  {
    entities_.push_back(condition.entity());
    attach_condition(condition);
  }

  template<typename MessageT, typename ConditionT>
  void attach_data_condition(
    dds::sub::DataReader<MessageT> & reader,
    ConditionT & condition)
  {
    readers_.push_back(dds::sub::AnyDataReader(reader));
    attach_condition(condition);
  }

  void detach_condition(const dds::core::cond::Condition & condition)
  {
    for (auto it = conditions_.begin(); it != conditions_.end(); ++it) {
      if (*it == condition) {
        conditions_.erase(it);
        exec_.detach(condition);
        break;
      }
    }
  }

  void detach_status_condition(const dds::core::cond::StatusCondition & condition)
  {
    for (auto it = entities_.begin(); it != entities_.end(); it++)
    {
      if (*it == condition->entity()) {
        entities_.erase(it);
        break;
      }
    }
    detach_condition(condition);
  }

  template<typename MessageT, typename ConditionT>
  void detach_data_condition(const ConditionT & condition)
  {
    auto reader = dds::core::polymorphic_cast<dds::sub::DataReader<MessageT>>(condition.entity());
    auto any_reader = dds::sub::AnyDataReader(reader);
    for (auto it = readers_.begin(); it != readers_.end(); it++)
    {
      if (*it == any_reader) {
        readers_.erase(it);
        break;
      }
    }
    detach_condition(condition);
  }

  WaitSetExecT exec_;
  std::vector<dds::core::cond::Condition> conditions_;
  std::vector<dds::core::Entity> entities_;
  std::vector<dds::sub::AnyDataReader> readers_;
};

using AsyncConditionNotifier = ConditionNotifier<AsyncWaitSetExecutor>;

using SyncConditionNotifier = ConditionNotifier<SyncWaitSetExecutor>;

}  // namespace ros2dds

#endif  // ROS2DDS__NOTIFIER_HPP_
