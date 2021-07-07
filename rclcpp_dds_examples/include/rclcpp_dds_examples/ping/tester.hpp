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

#ifndef RCLCPP_DDS_EXAMPLES__PING__TESTER_HPP_
#define RCLCPP_DDS_EXAMPLES__PING__TESTER_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "rclcpp_dds/rclcpp_dds.hpp"

#ifndef UNUSED_ARG
#define UNUSED_ARG(a_)  (void)a_
#endif  // UNUSED_ARG

namespace rclcpp_dds_examples
{
struct PingPongTesterOptions
{
  int32_t domain_id = 0;
  int64_t max_samples = 0;
  int64_t max_execution_time = 0;
  int64_t ignored_initial_samples = 0;
  int64_t print_interval = 0;
  std::string type_name;
  std::string topic_name_ping;
  std::string topic_name_pong;
  std::string qos_profile_ping;
  std::string qos_profile_pong;
  bool display_received{false};
  bool dedicated_participant{false};

  template<typename N>
  static void declare(N * const node)
  {
    node->declare_parameter("domain_id", 0);
    node->declare_parameter("max_samples", 0);
    node->declare_parameter("max_execution_time", 30000000 /* 30s */);
    node->declare_parameter("ignored_initial_samples", 3);
    node->declare_parameter("print_interval", 1000000 /* 1s */);
    node->declare_parameter("type_name", "PingMessage");
    node->declare_parameter("topic_name_ping", "rt/ping");
    node->declare_parameter("topic_name_pong", "rt/pong");
    node->declare_parameter(
      "qos_profile_ping",
      "BuiltinQosLibExp::Generic.StrictReliable.LargeData");
    node->declare_parameter(
      "qos_profile_pong",
      "BuiltinQosLibExp::Generic.StrictReliable.LargeData");
    node->declare_parameter("display_received", false);
    node->declare_parameter("dedicated_participant", false);
  }

  template<typename N>
  static PingPongTesterOptions load(N * const node, const bool log_result = true)
  {
    PingPongTesterOptions opts;
    node->get_parameter("domain_id", opts.domain_id);
    node->get_parameter("max_samples", opts.max_samples);
    node->get_parameter("max_execution_time", opts.max_execution_time);
    node->get_parameter("ignored_initial_samples", opts.ignored_initial_samples);
    node->get_parameter("print_interval", opts.print_interval);
    node->get_parameter("type_name", opts.type_name);
    node->get_parameter("topic_name_ping", opts.topic_name_ping);
    node->get_parameter("topic_name_pong", opts.topic_name_pong);
    node->get_parameter("qos_profile_ping", opts.qos_profile_ping);
    node->get_parameter("qos_profile_pong", opts.qos_profile_pong);
    node->get_parameter("display_received", opts.display_received);
    node->get_parameter("dedicated_participant", opts.dedicated_participant);

    if (log_result) {
      log(node, opts);
    }

    return opts;
  }

  // Helper function to log test options to stdout.
  template<typename N>
  static void log(N * const node, const PingPongTesterOptions & opts)
  {
    RCLCPP_INFO(
      node->get_logger(), "test options:\n"
      "\tdomain_id = %d\n"
      "\tmax_samples = %lu\n"
      "\tmax_execution_time = %lfs\n"
      "\tignored_initial_samples = %lu\n"
      "\tprint_interval = %lfs\n"
      "\ttype_name = '%s'\n"
      "\ttopic_name_ping = '%s'\n"
      "\ttopic_name_pong = '%s'\n"
      "\tqos_profile_ping = '%s'\n"
      "\tqos_profile_pong = '%s'\n"
      "\tdisplay_received = %d\n"
      "\tdedicated_participant = %d",
      opts.domain_id,
      opts.max_samples,
      static_cast<double>(opts.max_execution_time) / 1000000.0,
      opts.ignored_initial_samples,
      static_cast<double>(opts.print_interval) / 1000000.0,
      opts.type_name.c_str(),
      opts.topic_name_ping.c_str(),
      opts.topic_name_pong.c_str(),
      opts.qos_profile_ping.c_str(),
      opts.qos_profile_pong.c_str(),
      opts.display_received,
      opts.dedicated_participant);
  }
};

template<typename T>
class PingPongTester : public rclcpp_dds::DdsNode
{
protected:
  PingPongTester(
    const char * const name,
    const rclcpp_dds::DdsNodeOptions & options,
    const bool ping)
  : DdsNode(name, options),
    ping_(ping)
  {
    // We use ROS 2 parameters to allow customization of test parameters, e.g.
    // via a YAML parameter file with `--ros-args --params-file <file>`.
    PingPongTesterOptions::declare(this);
  }

  // If we are using the "plain" binding this function will return a
  // preallocated sample. If we are using Zero-Copy or Flat-Data, then this
  // function must loan a sample from the writer.
  virtual T * alloc_sample() = 0;

  // Callback invoked every time (valid) data is received from the reader.
  virtual void on_data(dds::sub::LoanedSamples<T> & samples) = 0;

  // Initialize DDS entities and other data structures used by the tester.
  // Each operation is delegated to a virtual function, so that subclasses may
  // override each step as needed.
  virtual void init_test()
  {
    // Load test configuration from ROS 2 parameters
    test_options_ = PingPongTesterOptions::load(this);

    const std::string * writer_topic,
      * writer_profile,
      * reader_topic,
      * reader_profile;

    if (ping_) {
      writer_topic = &test_options_.topic_name_ping;
      writer_profile = &test_options_.qos_profile_ping;
      reader_topic = &test_options_.topic_name_pong;
      reader_profile = &test_options_.qos_profile_pong;
    } else {
      writer_topic = &test_options_.topic_name_pong;
      writer_profile = &test_options_.qos_profile_pong;
      reader_topic = &test_options_.topic_name_ping;
      reader_profile = &test_options_.qos_profile_ping;
    }

    create_endpoints(
      *writer_topic, *writer_profile, *reader_topic, *reader_profile);

    initialize_events();

    publisher_->enable();
    subscriber_->enable();
  }

  virtual void create_endpoints(
    const std::string & writer_topic,
    const std::string & writer_profile,
    const std::string & reader_topic,
    const std::string & reader_profile)
  {
    // Create a custom Publisher and Subscriber and configure them so that
    // DataWriters and DataReaders are created disabled. This way we can
    // configure them, and enable them without losing any discovery event.
    auto participant = this->domain_participant();

    dds::core::policy::EntityFactory entity_policy;
    entity_policy.autoenable_created_entities(false);

    auto publisher_qos = participant.default_publisher_qos();
    publisher_qos << entity_policy;
    publisher_ = dds::pub::Publisher(participant, publisher_qos);

    auto subscriber_qos = participant.default_subscriber_qos();
    subscriber_qos << entity_policy;
    subscriber_ = dds::sub::Subscriber(participant, subscriber_qos);

    // Create a DataWriter configured to handle "large data" (i.e. data which
    // exceeds the transport's MTU). We also configure it to use "transient local"
    // durability so that the first message is not lost in case of asymmetric
    // discovery (i.e. the writer matches the reader before the reader has
    // matched the writer).
    auto writer_qos = this->get_datawriter_qos_profile(writer_profile);
    // Only keep the latest written sample
    writer_qos << dds::core::policy::History::KeepLast(1);
    // Use transient local Durability
    writer_qos << dds::core::policy::Durability::TransientLocal();
    auto writer_dds_topic =
      dds::topic::Topic<T>(participant, writer_topic, test_options_.type_name);
    writer_ = dds::pub::DataWriter<T>(publisher_, writer_dds_topic, writer_qos);

    auto reader_qos = this->get_datareader_qos_profile(reader_profile);
    reader_qos << dds::core::policy::History::KeepLast(1);
    // Optional optimization: prevent dynamic allocation of received fragments.
    // This might cause more memory allocation upfront, but may improve latency.
    rti::core::policy::DataReaderResourceLimits dr_limits;
    dr_limits.dynamically_allocate_fragmented_samples(false);
    reader_qos << dr_limits;
    // Use transient local Durability
    reader_qos << dds::core::policy::Durability::TransientLocal();
    auto reader_dds_topic =
      dds::topic::Topic<T>(participant, reader_topic, test_options_.type_name);
    reader_ = dds::sub::DataReader<T>(subscriber_, reader_dds_topic, reader_qos);
  }

  // Disable event notification and call `rclcpp::shutdown()` to stop process.
  virtual void shutdown()
  {
    test_active_ = false;
    rclcpp::shutdown();
  }

  // Create an AsyncWaitSet to process events on dedicated pool of user threads,
  // instead of calling them from the middleware's transport receive threads.
  // By default, only 1 thread is allocated to the pool.
  virtual void initialize_events()
  {
    using dds::core::status::StatusMask;

    this->set_status_callback<T>(
      reader_,
      [this](dds::core::cond::StatusCondition &, dds::sub::DataReader<T> &) {
        on_reader_active();
      },
      StatusMask::subscription_matched() | StatusMask::data_available());

    this->set_status_callback<T>(
      writer_,
      [this](dds::core::cond::StatusCondition &, dds::pub::DataWriter<T> &) {
        on_writer_active();
      },
      StatusMask::publication_matched());

    // Start a watchdog timer to make sure that the test starts, because there
    // is currently a race condition when using Zero-Copy where some "matched"
    // events are never notified. To keep things consistent, we attach a
    // "watchdog guard condition" to the waitset, and we will trigger it from
    // the timer if it detects that the test is ready, but hasn't been marked
    // as such. The only reason to do this (instead of calling `test_start()`
    // directly from the timer) is so that the start event is processed on one
    // of the AsyncWaitset's threads, as it would be if all match events were
    // notified.
    wd_condition_ = this->add_user_callback(
      [this]() {
        // Force test to start
        test_start();
      });
    wd_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        if (is_test_ready()) {
          // Cancel watchdog timer
          wd_->cancel();
          // Trigger watchdog condition to wake up waitset
          wd_condition_.trigger_value(true);
        }
      });
  }

  // A simple wrapper to
  virtual uint64_t ts_now()
  {
    return this->domain_participant().current_time().to_microsecs();
  }

  // Mark test as active
  virtual void test_start()
  {
    wd_->cancel();
    RCLCPP_INFO(this->get_logger(), "all endpoints matched, beginning test");
    test_active_ = true;
    test_complete_ = false;
    count_ignored_ = 0;
    count_ = 0;
    start_ts_ = this->ts_now();
  }

  // Mark test as inactive and call shutdown() to terminate process.
  virtual void test_stop()
  {
    auto stop_ts = this->ts_now();
    const uint64_t run_time = stop_ts - start_ts_;
    const double run_time_s = run_time / 1000000.0;
    RCLCPP_INFO(this->get_logger(), "test stopped after %lf s", run_time_s);

    test_active_ = false;

    if (test_complete_) {
      RCLCPP_INFO(this->get_logger(), "test completed");
    } else {
      RCLCPP_ERROR(this->get_logger(), "test interrupted before completion");
    }

    shutdown();
  }

  // Callback to perform custom operation when the test is considered complete.
  virtual void test_complete()
  {
    test_complete_ = true;
  }

  // Consider the test "ready to run" if both the writer and the reader have at
  // least one matched remote endpoint. This could lead to false positives, for
  // example if there are some monitoring applications open (e.g. RTI Admin
  // Console, or rtiddsspy). More sophisticated coordination could be implemented
  // by introducing an explicit synchronization topic, but we keep things simple
  // for the sake of example.
  virtual bool is_test_ready()
  {
    auto sub_matched_status = writer_.publication_matched_status();
    auto pub_matched_status = reader_.subscription_matched_status();

    return sub_matched_status.current_count() > 0 &&
           pub_matched_status.current_count() > 0;
  }

  // Check if the test is complete, because either:
  // - The maximum number of samples were published/received.
  // - The maximum allowed run time was reached.
  virtual bool is_test_complete()
  {
    auto ts = this->ts_now();

    const bool time_expired =
      start_ts_ > 0 && test_options_.max_execution_time > 0 &&
      ts - static_cast<uint64_t>(test_options_.max_execution_time) > start_ts_;

    const bool max_samples_reached =
      test_options_.max_samples > 0 &&
      count_ >= static_cast<uint64_t>(test_options_.max_samples);

    return time_expired || max_samples_reached;
  }

  // Check if the test is "ready to run", and activate/deactivate it based on
  // outcome of the check and the current test state.
  virtual void check_test_state()
  {
    if (is_test_ready()) {
      if (!test_active_) {
        test_start();
      }
    } else {
      if (test_active_) {
        test_stop();
      }
    }
  }

  // Default handler for "data available" events which calls take() to reset
  // the status flag on the reader. Subclasses will tipically overload the
  // on_data() callback.
  virtual void on_data_available()
  {
    // Read samples from reader cache and notify a callback.
    // Always peform a take so the listener is not called repeatedly.
    auto samples = reader_.take();
    const bool has_data = samples.length() > 0;
    if (has_data && samples[0].info().valid()) {
      on_data(samples);
    } else if (has_data && !samples[0].info().valid()) {
      // An "invalid" sample generally indicates a state transition from an
      // unmatched/not-alive remote writer. Check this is expected (e.g if the
      // test has already been completed), or terminate the test early otherwise.
      if (!is_test_complete()) {
        RCLCPP_ERROR(this->get_logger(), "lost peer before end of test");
        test_stop();
      }
    } else if (!has_data) {
      // This should never happen, but just in case, print an error and exit.
      RCLCPP_ERROR(this->get_logger(), "woke up without data");
      test_stop();
    }
  }

  // Callback triggered when one of the enabled statuses is triggered on the
  // reader's status condition.
  virtual void on_reader_active()
  {
    using dds::core::status::StatusMask;

    if ((reader_.status_changes() & StatusMask::subscription_matched()).any()) {
      check_test_state();
    }
    if ((reader_.status_changes() & StatusMask::data_available()).any()) {
      on_data_available();
    }
  }

  // Callback triggered when one of the enabled statuses is triggered on the
  // writer's status condition.
  virtual void on_writer_active()
  {
    using dds::core::status::StatusMask;

    // no need to differentiate, since we only enabled one status.
    if ((writer_.status_changes() & StatusMask::publication_matched()).any()) {
      check_test_state();
    }
  }

  const bool ping_;
  PingPongTesterOptions test_options_;
  bool test_active_{false};
  bool test_complete_{false};
  uint64_t count_ = 0;
  uint64_t count_ignored_ = 0;
  uint64_t start_ts_ = 0;
  dds::pub::Publisher publisher_{nullptr};
  dds::sub::Subscriber subscriber_{nullptr};
  dds::pub::DataWriter<T> writer_{nullptr};
  dds::sub::DataReader<T> reader_{nullptr};
  dds::core::cond::GuardCondition wd_condition_;
  rclcpp::TimerBase::SharedPtr wd_;
};
}  // namespace namespace rclcpp_dds_examples

#endif  // RCLCPP_DDS_EXAMPLES__PING__TESTER_HPP_
