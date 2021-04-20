// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CONNEXT_NODE_HELPERS__RTI_ROS2_PING_TESTER_HPP
#define CONNEXT_NODE_HELPERS__RTI_ROS2_PING_TESTER_HPP

#include <chrono>

#include <dds/dds.hpp>

#include <rti/core/cond/AsyncWaitSet.hpp>

#include "rclcpp/rclcpp.hpp"

#ifndef UNUSED_ARG
#define UNUSED_ARG(a_)  (void)a_
#endif  // UNUSED_ARG

namespace rti { namespace ros2 { namespace ping {

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
    node->declare_parameter("qos_profile_ping",
      "BuiltinQosLibExp::Generic.StrictReliable.LargeData");
    node->declare_parameter("qos_profile_pong",
      "BuiltinQosLibExp::Generic.StrictReliable.LargeData");
    node->declare_parameter("display_received", false);
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

    if (log_result)
    {
      log(node, opts);
    }

    return opts;
  }

  // Helper function to log test options to stdout.
  template<typename N>
  static void log(N * const node, const PingPongTesterOptions & opts) {
    RCLCPP_INFO(node->get_logger(), "test options:\n"
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
      "\tdisplay_received = %d",
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
      opts.display_received);
  }
};

template<typename T>
class PingPongTester : public rclcpp::Node
{
protected:
  PingPongTester(
    const char * const name,
    const rclcpp::NodeOptions & options,
    const bool ping)
  : Node(name, options),
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

    initialize_waitset();

    enable_endpoints();
  }

  virtual void create_endpoints(
    const std::string & writer_topic,
    const std::string & writer_profile,
    const std::string & reader_topic,
    const std::string & reader_profile)
  {
    participant_ = create_participant();

    publisher_ = create_publisher();

    subscriber_ = create_subscriber();

    writer_ = create_writer(
      test_options_.type_name.c_str(),
      writer_topic.c_str(),
      writer_profile.c_str());

    reader_ = create_reader(
      test_options_.type_name.c_str(),
      reader_topic.c_str(),
      reader_profile.c_str());
  }

  // The endpoints are not automatically "enabled" by DDS so that we may set up
  // all our listeners before they are, and not risk missing any notification.
  virtual void enable_endpoints()
  {
    publisher_->enable();
    subscriber_->enable();
    writer_->enable();
    reader_->enable();
  }

  // Disable event notification and call `rclcpp::shutdown()` to stop process.
  virtual void shutdown()
  {
    using namespace dds::core::status;

    test_active_ = false;

    // Disable all events on status conditions
    // reader_condition_.enabled_statuses(StatusMask::none());
    // writer_condition_.enabled_statuses(StatusMask::none());
    rclcpp::shutdown();
  }

  // Instead of creating a dedicated DomainParticipant, we try to reuse the one
  // created by `rmw_connextdds`. If that is not available (e.g. if the
  // application is running with another RMW), we create a new DomainParticipant.
  virtual dds::domain::DomainParticipant create_participant()
  {
    auto participant = dds::domain::find(test_options_.domain_id);
    if (dds::core::null == participant) {
      RCLCPP_ERROR(this->get_logger(),
        "failed to lookup DomainParticipant. Is the application running on rmw_connextdds?");
      return dds::domain::DomainParticipant(test_options_.domain_id);
    }
    return participant;
  }

  // Create a custom Publisher and Subscriber and configure them so that
  // DataWriters and DataReaders are created disabled. This way we can
  // configure them, and enable them without losing any discovery event.
  virtual dds::pub::Publisher create_publisher()
  {
    auto publisher_qos =  participant_->default_publisher_qos();
    dds::core::policy::EntityFactory entity_policy;
    entity_policy.autoenable_created_entities(false);
    publisher_qos << entity_policy;
    return dds::pub::Publisher(participant_, publisher_qos);
  }

  virtual dds::sub::Subscriber create_subscriber()
  {
    auto subscriber_qos =  participant_->default_subscriber_qos();
    dds::core::policy::EntityFactory entity_policy;
    entity_policy.autoenable_created_entities(false);
    subscriber_qos << entity_policy;
    return dds::sub::Subscriber(participant_, subscriber_qos);
  }

  // Create a DataWriter configured to handle "large data" (i.e. data which
  // exceeds the transport's MTU). We also configure it to use "transient local"
  // durability so that the first message is not lost in case of asymmetric
  // discovery (i.e. the writer matches the reader before the reader has
  // matched the writer).
  virtual dds::pub::DataWriter<T> create_writer(
    const char * const type_name,
    const char * const topic_name,
    const char * const qos_profile)
  {
    dds::topic::Topic<T> topic_ping(participant_, topic_name, type_name); 
    auto writer_qos = load_writer_qos(type_name, topic_name, qos_profile);
    return dds::pub::DataWriter<T>(publisher_, topic_ping, writer_qos);
  }

  // Loading of the writer's qos is delegated to a separate function so that it
  // may be overloaded by a subclass.
  virtual dds::pub::qos::DataWriterQos load_writer_qos(
    const char * const type_name,
    const char * const topic_name,
    const char * const qos_profile)
  {
    UNUSED_ARG(type_name);
    UNUSED_ARG(qos_profile);
    using namespace dds::core;
    auto qos_provider = QosProvider::Default();
    auto writer_qos = qos_provider.datawriter_qos(qos_profile);
    // Only keep the latest written sample
    writer_qos << policy::History(policy::HistoryKind::KEEP_LAST, 1);
    rti::core::policy::Property props;
    // Optional optimization: prevent dynamic allocation of serialization buffer.
    // This configuration is only possible because the data type is not "unbouded",
    // otherwise we must provide a finite limite to this pool or run out of memory.
    props.set({
      "dds.data_writer.history.memory_manager.fast_pool.pool_buffer_max_size",
      "LENGTH_UNLIMITED"
    }, false);
    writer_qos << props;
    // Use transient local Durability
    writer_qos << policy::Durability::TransientLocal();
    return writer_qos;
  }

  // Create a DataReader with "transient local" durability so we can receive the
  // initial message even if it was written before the remote writer is matched.
  virtual dds::sub::DataReader<T> create_reader(
    const char * const type_name,
    const char * const topic_name,
    const char * const qos_profile)
  {
    dds::topic::Topic<T> topic_pong(participant_, topic_name, type_name);
    auto reader_qos = load_reader_qos(type_name, topic_name, qos_profile);
    return dds::sub::DataReader<T>(subscriber_, topic_pong, reader_qos);
  }

  // Loading of the reader's qos is delegated to a separate function so that it
  // may be overloaded by a subclass.
  virtual dds::sub::qos::DataReaderQos load_reader_qos(
    const char * const type_name,
    const char * const topic_name,
    const char * const qos_profile)
  {
    UNUSED_ARG(type_name);
    UNUSED_ARG(qos_profile);
    using namespace dds::core;
    auto qos_provider = QosProvider::Default();
    auto reader_qos = qos_provider.datareader_qos(qos_profile);
    // Only keep the latest received sample
    reader_qos << policy::History(policy::HistoryKind::KEEP_LAST, 1);
    // Optional optimization: prevent dynamic allocation of received fragments.
    // This might cause more memory allocation upfront, but may improve latency.
    rti::core::policy::DataReaderResourceLimits dr_limits;
    dr_limits.dynamically_allocate_fragmented_samples(false);
    reader_qos << dr_limits;
    // Use transient local Durability
    reader_qos << policy::Durability::TransientLocal();
    return reader_qos;
  }

  // Create an AsyncWaitSet to process events on dedicated pool of user threads,
  // instead of calling them from the middleware's transport receive threads.
  // By default, only 1 thread is allocated to the pool.
  virtual void initialize_waitset()
  {
    using namespace dds::core::status;
    using namespace dds::core::cond;
    using namespace std::chrono_literals;

    awaitset_ = std::make_unique<rti::core::cond::AsyncWaitSet>(
        rti::core::cond::AsyncWaitSet::PROPERTY_DEFAULT);
    
    // Attach DataReader's StatusCondition to detect "subscription matched"
    // and "data available".
    reader_condition_ = StatusCondition(reader_);
    reader_condition_.enabled_statuses(
      StatusMask::subscription_matched() | StatusMask::data_available());
    reader_condition_->handler([this](){
      on_reader_active();
    });
    *awaitset_ += reader_condition_;

    // Attach DataWriter's StatusCondition to detect "publication matched"
    writer_condition_ = StatusCondition(writer_);
    writer_condition_.enabled_statuses(StatusMask::publication_matched());
    writer_condition_->handler([this](){
      on_writer_active();
    });
    *awaitset_ += writer_condition_;


    // Start a watchdog timer to make sure that the test starts, because it
    // seems like there is a race condition where some "matched" events are
    // never notified. To keep things consistent, we attach a "watchdog guard
    // condition" to the waitset, and we will trigger it from the timer if it
    // detects that the test is ready, but hasn't been marked as such.
    // The only reason to do this (instead of calling `test_start()` directly
    // from the timer) is so that the start event is processed on one of the
    // AsyncWaitset's threads, as it would be if all match events were notified.
    waitset_wd_cond_->handler([this](){
      on_watchdog_active();
    });
    *awaitset_ += waitset_wd_cond_;
    waitset_wd_ = this->create_wall_timer(100ms, [this](){
      on_watchdog_timer();
    });

    // Start waitset threads
    awaitset_->start();
  }

  // A simple wrapper to 
  virtual uint64_t ts_now()
  {
    return this->participant_->current_time().to_microsecs();
  }

  // Mark test as active
  virtual void test_start()
  {
    waitset_wd_->cancel();
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
  virtual bool is_test_ready() {
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
    if (is_test_ready())
    {
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
  virtual void on_data_available() {
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
  };

  // Callback triggered when one of the enabled statuses is triggered on the
  // reader's status condition.
  virtual void on_reader_active()
  {
    using namespace dds::core::status;

    if ((reader_.status_changes() & StatusMask::subscription_matched()).any())
    {
      check_test_state();
    }
    if ((reader_.status_changes() & StatusMask::data_available()).any())
    {
      on_data_available();
    }
  }

  // Callback triggered when one of the enabled statuses is triggered on the
  // writer's status condition.
  virtual void on_writer_active()
  {
    using namespace dds::core::status;

    // no need to differentiate, since we only enabled one status.
    if ((writer_.status_changes() & StatusMask::publication_matched()).any())
    {
      check_test_state();
    }
  }

  // Callback triggered by the watchdog timer when it detects the test to be
  // "ready to run".
  virtual void on_watchdog_active()
  {
    // Reset watchdog condition
    waitset_wd_cond_.trigger_value(false);
    // Force test to start
    test_start();
  }

  // Periodic watchdog function as a workaround to possible losses of events.
  virtual void on_watchdog_timer()
  {
    if (is_test_ready()) {
      // Cancel watchdog timer
      waitset_wd_->cancel();
      // Trigger watchdog condition to wake up waitset
      waitset_wd_cond_.trigger_value(true);
    }
  }

  const bool ping_;
  PingPongTesterOptions test_options_;
  bool test_active_{false};
  bool test_complete_{false};
  uint64_t count_ = 0;
  uint64_t count_ignored_ = 0;
  uint64_t start_ts_ = 0;
  dds::domain::DomainParticipant participant_{nullptr};
  dds::pub::Publisher publisher_{nullptr};
  dds::sub::Subscriber subscriber_{nullptr};
  dds::pub::DataWriter<T> writer_{nullptr};
  dds::sub::DataReader<T> reader_{nullptr};
  dds::core::cond::StatusCondition reader_condition_{nullptr};
  dds::core::cond::StatusCondition writer_condition_{nullptr};
  std::unique_ptr<rti::core::cond::AsyncWaitSet> awaitset_;
  dds::core::cond::GuardCondition waitset_wd_cond_;
  rclcpp::TimerBase::SharedPtr waitset_wd_;
};

}  // namespace ping
}  // namespace ros2
}  // namespace rti

#endif  // CONNEXT_NODE_HELPERS__RTI_ROS2_PING_TESTER_HPP
