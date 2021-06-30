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

#ifndef RTI__ROS2__NODE__EVENT_HPP_
#define RTI__ROS2__NODE__EVENT_HPP_

#include <rti/core/cond/AsyncWaitSet.hpp>

#include <rti/ros2/node/domain.hpp>
#include <rti/ros2/node/topic.hpp>

#include <condition_variable>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <limits>
#include <functional>

#include "rclcpp/scope_exit.hpp"

namespace rti
{
namespace ros2
{
namespace node
{
class EventNotifier;

class Event {
public:
  virtual void cancel() = 0;
  virtual dds::core::cond::Condition condition() = 0;
};

template<typename ENTITY>
class EntityEvent : public Event {
public:
  virtual ENTITY entity() = 0;
};

template<typename T>
class ReaderEvent : public EntityEvent<dds::sub::DataReader<T>> {};

template<typename T>
class WriterEvent : public EntityEvent<dds::pub::DataWriter<T>> {};

template<typename T>
class DataEvent : public ReaderEvent<T>
{
public:
  DataEvent(
    dds::sub::DataReader<T> reader,
    dds::sub::status::DataState data_state,
    std::function<void(dds::sub::DataReader<T>&,dds::sub::cond::ReadCondition&)> cb)
  : reader_(reader),
    condition_(reader_, data_state, [this](){
      cb_(reader_, condition_);
    }),
    cb_(cb)
  {}

  virtual void cancel() {
    cb_ = [](dds::sub::DataReader<T>&,dds::sub::cond::ReadCondition&){};
    reader_ = nullptr;
    condition_ = nullptr;
  }

  virtual dds::core::cond::Condition condition()
  {
    return condition_;
  }

  virtual dds::sub::DataReader<T> entity()
  {
    return reader_;
  }

  dds::sub::cond::ReadCondition read_condition()
  {
    return condition_;
  }

private:
  dds::sub::DataReader<T> reader_;
  dds::sub::cond::ReadCondition condition_;
  std::function<void(dds::sub::DataReader<T>&,dds::sub::cond::ReadCondition&)> cb_;
};


template<typename T>
class DataQueryEvent : public ReaderEvent<T>
{
public:
  DataQueryEvent(
    dds::sub::DataReader<T> reader,
    const std::string & query_expression,
    dds::sub::status::DataState data_state,
    std::function<void(dds::sub::DataReader<T>&,dds::sub::cond::QueryCondition&)> cb)
  : reader_(reader),
    query_(reader_, query_expression),
    condition_(query_, data_state, [this](){
      cb_(reader_, condition_);
    }),
    cb_(cb)
  {}

  DataQueryEvent(
    dds::sub::DataReader<T> reader,
    const std::string & query_expression,
    const std::vector<std::string> & query_parameters,
    dds::sub::status::DataState data_state,
    std::function<void(dds::sub::DataReader<T>&,dds::sub::cond::QueryCondition&)> cb)
  : reader_(reader),
    query_(reader_, query_expression, query_parameters),
    condition_(query_, data_state, [this](){
      cb_(reader_, condition_);
    }),
    cb_(cb)
  {}

  virtual void cancel() {
    cb_ = [](dds::sub::DataReader<T>&,dds::sub::cond::QueryCondition&){};
    reader_ = nullptr;
    condition_ = nullptr;
  }

  virtual dds::core::cond::Condition condition()
  {
    return condition_;
  }

  virtual dds::sub::DataReader<T> entity()
  {
    return reader_;
  }

  dds::sub::cond::QueryCondition query_condition()
  {
    return condition_;
  }

private:
  dds::sub::DataReader<T> reader_;
  dds::sub::Query query_;
  dds::sub::cond::QueryCondition condition_;
  std::function<void(dds::sub::DataReader<T>&,dds::sub::cond::QueryCondition&)> cb_;
};

template<typename T>
class ReaderStatusEvent : public ReaderEvent<T>
{
public:
  ReaderStatusEvent(
    dds::sub::DataReader<T> reader,
    dds::core::status::StatusMask statuses,
    std::function<void(dds::sub::DataReader<T>&)> cb)
  : reader_(reader),
    condition_(reader_),
    cb_(cb)
  {
    condition_->enabled_statuses(statuses);
    condition_->handler([this](){
      cb_(reader_);
    });
  }

  virtual void cancel() {
    cb_ = [](dds::sub::DataReader<T>&){};
    reader_ = nullptr;
    condition_ = nullptr;
  }

  virtual
  dds::core::cond::Condition condition()
  {
    return condition_;
  }

  virtual dds::sub::DataReader<T> entity()
  {
    return reader_;
  }

private:
  dds::sub::DataReader<T> reader_;
  dds::core::cond::StatusCondition condition_;
  std::function<void(dds::sub::DataReader<T>&)> cb_;
};

template<typename T>
class WriterStatusEvent : public WriterEvent<T>
{
public:
  WriterStatusEvent(
    dds::pub::DataWriter<T> writer,
    dds::core::status::StatusMask statuses,
    std::function<void(dds::pub::DataWriter<T>&)> cb)
  : writer_(writer),
    condition_(writer_),
    cb_(cb)
  {
    condition_->enabled_statuses(statuses);
    condition_->handler([this](){
      cb_(writer_);
    });
  }

  virtual void cancel() {
    cb_ = [](dds::pub::DataWriter<T>&){};
    writer_ = nullptr;
    condition_ = nullptr;
  }

  virtual
  dds::core::cond::Condition condition()
  {
    return condition_;
  }

  virtual dds::pub::DataWriter<T> entity()
  {
    return writer_;
  }

private:
  dds::pub::DataWriter<T> writer_;
  dds::core::cond::StatusCondition condition_;
  std::function<void(dds::pub::DataWriter<T>&)> cb_;
};

class UserEvent : public Event
{
public:
  explicit UserEvent(std::function<void(dds::core::cond::GuardCondition&)> cb)
  : condition_(),
    cb_(cb)
  {
    condition_->handler([this]() {
      cb_(condition_);
    });
  }

  virtual void cancel() {
    cb_ = [](dds::core::cond::GuardCondition&){};
    condition_ = nullptr;
  }

  virtual dds::core::cond::Condition condition()
  {
    return condition_;
  }

  dds::core::cond::GuardCondition guard_condition()
  {
    return condition_;
  }

private:
  dds::core::cond::GuardCondition condition_;
  std::function<void(dds::core::cond::GuardCondition&)> cb_;
};

class EventNotifier
{
public:
  explicit EventNotifier()
  : wait_mutex_(),
    wait_cond_()
  {}

  virtual ~EventNotifier()
  {}

  template<typename T>
  std::shared_ptr<DataEvent<T>>
  notify_data(
    dds::sub::DataReader<T> reader,
    std::function<void(dds::sub::DataReader<T>&,dds::sub::cond::ReadCondition&)> cb,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    auto event = std::make_shared<DataEvent<T>>(reader, data_state,
      [this, cb](
          dds::sub::DataReader<T> & reader,
          dds::sub::cond::ReadCondition & condition){
        cb(reader, condition);
        on_event();
      });
    notify(event);
    return event;
  }

  template<typename T>
  std::shared_ptr<DataQueryEvent<T>>
  notify_data(
    dds::sub::DataReader<T> reader,
    std::function<void(dds::sub::DataReader<T>&,dds::sub::cond::QueryCondition&)> cb,
    const std::string & query_expression,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    auto event = std::make_shared<DataQueryEvent<T>>(
      reader, query_expression, data_state,
      [this, cb](
          dds::sub::DataReader<T> & reader,
          dds::sub::cond::QueryCondition & condition){
        cb(reader, condition);
        on_event();
      });
    notify(event);
    return event;
  }

  template<typename T>
  std::shared_ptr<DataQueryEvent<T>>
  notify_data(
    dds::sub::DataReader<T> reader,
    std::function<void(dds::sub::DataReader<T>&,dds::sub::cond::QueryCondition&)> cb,
    const std::string & query_expression,
    const std::vector<std::string> & query_parameters,
    dds::sub::status::DataState data_state = dds::sub::status::DataState::new_data())
  {
    auto event = std::make_shared<DataQueryEvent<T>>(
      reader, query_expression, query_parameters, data_state,
      [this, cb](
          dds::sub::DataReader<T> & reader,
          dds::sub::cond::QueryCondition & condition){
        cb(reader, condition);
        on_event();
      });
    notify(event);
    return event;
  }

  template<typename T>
  std::shared_ptr<ReaderStatusEvent<T>>
  notify_status_changed(
    dds::sub::DataReader<T> reader,
    std::function<void(dds::sub::DataReader<T>&)> cb,
    dds::core::status::StatusMask statuses)
  {
    auto event = std::make_shared<ReaderStatusEvent<T>>(reader, statuses,
      [this, cb](dds::sub::DataReader<T> & reader){
        cb(reader);
        on_event();
      });
    notify(event);
    return event;
  }

  template<typename T>
  std::shared_ptr<WriterStatusEvent<T>>
  notify_status_changed(
    dds::pub::DataWriter<T> writer,
    std::function<void(dds::pub::DataWriter<T>&)> cb,
    dds::core::status::StatusMask statuses)
  {
    auto event = std::make_shared<WriterStatusEvent<T>>(writer, statuses,
      [this, cb](dds::pub::DataWriter<T> & writer){
        cb(writer);
        on_event();
      });
    notify(event);
    return event;
  }

  std::shared_ptr<UserEvent>
  notify_user_event(std::function<void(dds::core::cond::GuardCondition&)> cb)
  {
    auto event = std::make_shared<UserEvent>(
      [this, cb](dds::core::cond::GuardCondition & condition){
        cb(condition);
        on_event();
      });
    notify(event);
    return event;
  }

  void
  cancel(std::shared_ptr<Event> e)
  {
    bool removed = false;
    for (auto it = events_.begin(); it != events_.end() && !removed; ++it) {
      if (*it == e) {
        events_.erase(it);
        removed = true;
      }
    }
    if (!removed) {
      throw std::runtime_error("unknown event");
    }
    detach(*e);
    e->cancel();
  }

  virtual void
  spin(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0))
  {
    auto start = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(wait_mutex_);
    if (waiting_) {
      throw std::runtime_error("notifier already spinning");
    }
    waiting_ = true;
    auto wait_exit = rclcpp::make_scope_exit(
      [this]() {
        waiting_ = false;
      });
    wait_cond_.wait_for(lock,
      max_duration,
      [this, max_duration, start]() {
        // exit if notifier is stopped, or
        // if max_duration is not infinite and it has expired
        auto ts_now = std::chrono::steady_clock::now() - start;
        const bool duration_elapsed =
          (std::chrono::nanoseconds(0) != max_duration && ts_now >= max_duration);
        return !active_ || duration_elapsed;
      });
  }

  void stop()
  {
    std::lock_guard<std::mutex> lock(wait_mutex_);
    active_ = false;
  }

  void notify(std::shared_ptr<Event> e)
  {
    attach(*e);
    events_.push_back(e);
  }

protected:
  virtual void attach(Event & e) = 0;

  virtual void detach(Event & e) = 0;

  size_t available_events()
  {
    if (events_notified_ > events_triggered_) {
      // TODO(asorbini) keep track of "rollover epochs"? at the moment
      // we can only detect at most one roll over between spin_some()'s
      return std::numeric_limits<std::size_t>::max() - events_notified_ +
        events_triggered_;
    } else {
      return events_triggered_ - events_notified_;
    }
  }

  void on_event()
  {
    std::lock_guard<std::mutex> lock(wait_mutex_);
    events_triggered_ += 1;
    wait_cond_.notify_all();
  }

  std::mutex wait_mutex_;
  std::condition_variable wait_cond_;
  bool waiting_{false};
  bool active_{true};
  size_t events_triggered_{0};
  size_t events_notified_{0};
  std::vector<std::shared_ptr<Event>> events_;
};

class MultiThreadedEventNotifier : public EventNotifier
{
public:
  explicit MultiThreadedEventNotifier()
  : MultiThreadedEventNotifier(rti::core::cond::AsyncWaitSet::PROPERTY_DEFAULT)
  {}

  explicit MultiThreadedEventNotifier(const rti::core::cond::AsyncWaitSetProperty & awaitset_props)
  : EventNotifier(),
    awaitset_(std::make_unique<rti::core::cond::AsyncWaitSet>(awaitset_props))
  {
    awaitset_->start();
  }

  virtual ~MultiThreadedEventNotifier()
  {
    awaitset_->stop();
    for (size_t i = 0; i < events_.size(); i++) {
      auto e = events_[i];
      detach(*e);
    }
  }

protected:
  virtual void attach(Event & e)
  {
    *awaitset_ += e.condition();
  }

  virtual void detach(Event & e)
  {
    *awaitset_ -= e.condition();
  }

  std::unique_ptr<rti::core::cond::AsyncWaitSet> awaitset_;
};

class SingleThreadedEventNotifier : public EventNotifier
{
public:
  SingleThreadedEventNotifier()
  : SingleThreadedEventNotifier(rti::core::cond::WaitSetProperty())
  {}

  explicit SingleThreadedEventNotifier(const rti::core::cond::WaitSetProperty & waitset_props)
  : waitset_(std::make_unique<dds::core::cond::WaitSet>(waitset_props))
  {}

  virtual ~SingleThreadedEventNotifier()
  {}

  virtual void
  spin(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0))
  {
    auto start = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(wait_mutex_);
      if (waiting_) {
        throw std::runtime_error("notifier already spinning");
      }
    }
    waiting_ = true;
    auto wait_exit = rclcpp::make_scope_exit(
      [this]() {
        std::lock_guard<std::mutex> lock(wait_mutex_);
        waiting_ = false;
      });
    const bool infinite_duration = std::chrono::nanoseconds(0) == max_duration;
    bool timed_out = false;
    do {
      auto wait_duration = std::chrono::steady_clock::now() - start;
      const bool duration_elapsed = (!infinite_duration && wait_duration >= max_duration);
      timed_out = !active_ || duration_elapsed;
      if (!timed_out) {
        if (infinite_duration) {
          waitset_->dispatch(dds::core::Duration::infinite());
        } else {
          // const auto wait_timeout_ns = max_duration - wait_duration;
          const auto wait_timeout =
            std::chrono::duration_cast<std::chrono::microseconds>(
              max_duration - wait_duration);
          waitset_->dispatch(dds::core::Duration::from_microsecs(wait_timeout.count()));
        }
      }
    } while (!timed_out);
  }

protected:
  virtual void attach(Event & e)
  {
    *waitset_ += e.condition();
  }

  virtual void detach(Event & e)
  {
    *waitset_ -= e.condition();
  }

  std::unique_ptr<dds::core::cond::WaitSet> waitset_;
};

}  // namespace node
}  // namespace ros2
}  // namespace rti

#endif  // RTI__ROS2__NODE__EVENT_HPP_
