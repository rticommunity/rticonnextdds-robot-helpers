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
#ifndef ROS2DDS__WAITSET_HPP_
#define ROS2DDS__WAITSET_HPP_

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

namespace ros2dds
{
template<typename WaitSetT, typename WaitSetPropsT>
class WaitSetExecutor
{
public:
  explicit WaitSetExecutor(const WaitSetPropsT & properties)
  : waitset_(properties)
  {}

  explicit WaitSetExecutor()
  {}

  virtual void attach(const dds::core::cond::Condition & condition)
  {
    waitset_ += condition;
  }

  virtual void detach(const dds::core::cond::Condition & condition)
  {
    waitset_ -= condition;
  }

  virtual void
  spin(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) = 0;

  virtual void
  shutdown() = 0;

  virtual void
  wake_up() = 0;

  WaitSetT & waitset()
  {
    return waitset_;
  }

private:
  WaitSetT waitset_;
};

class AsyncWaitSetExecutor :
  public WaitSetExecutor<rti::core::cond::AsyncWaitSet, rti::core::cond::AsyncWaitSetProperty>
{
public:
  explicit AsyncWaitSetExecutor(const rti::core::cond::AsyncWaitSetProperty & properties)
  : WaitSetExecutor<rti::core::cond::AsyncWaitSet, rti::core::cond::AsyncWaitSetProperty>(properties)
  {
    waitset().start();
  }

  explicit AsyncWaitSetExecutor()
  : WaitSetExecutor<rti::core::cond::AsyncWaitSet, rti::core::cond::AsyncWaitSetProperty>()
  {
    waitset().start();
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

  virtual void
  shutdown()
  {
    active_ = false;
    wake_up();
  }

  virtual void
  wake_up()
  {
    wait_cond_.notify_all();
  }

private:
  std::mutex wait_mutex_;
  std::condition_variable wait_cond_;
  bool waiting_{false};
  bool active_{true};
};

class SyncWaitSetExecutor :
  public WaitSetExecutor<dds::core::cond::WaitSet, rti::core::cond::WaitSetProperty>
{
public:
  explicit SyncWaitSetExecutor(const rti::core::cond::WaitSetProperty & properties)
  : WaitSetExecutor<dds::core::cond::WaitSet, rti::core::cond::WaitSetProperty>(properties)
  {
    init_conditions();
  }

  explicit SyncWaitSetExecutor()
  : WaitSetExecutor<dds::core::cond::WaitSet, rti::core::cond::WaitSetProperty>()
  {
    init_conditions();
  }

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
          waitset()->dispatch(dds::core::Duration::infinite());
        } else {
          // const auto wait_timeout_ns = max_duration - wait_duration;
          const auto wait_timeout =
            std::chrono::duration_cast<std::chrono::microseconds>(
              max_duration - wait_duration);
          waitset()->dispatch(dds::core::Duration::from_microsecs(wait_timeout.count()));
        }
      }
    } while (!timed_out);
  }

  virtual void
  shutdown()
  {
    active_ = false;
    exit_condition_.trigger_value(true);
  }

  virtual void
  wake_up()
  {
    wakeup_condition_.trigger_value(true);
  }

private:
  void init_conditions()
  {
    wakeup_condition_->handler(
      [this]()
      {
        wakeup_condition_.trigger_value(false);
      });
    waitset() += wakeup_condition_;
    exit_condition_->handler(
      [this]()
      {
        exit_condition_.trigger_value(false);
        active_ = false;
      });
    waitset() += exit_condition_;
  }

  std::mutex wait_mutex_;
  bool waiting_{false};
  bool active_{true};
  dds::core::cond::GuardCondition exit_condition_;
  dds::core::cond::GuardCondition wakeup_condition_;
};

}  // namespace ros2dds

#endif  // ROS2DDS__WAITSET_HPP_
