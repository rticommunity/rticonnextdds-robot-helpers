// (c) 2021 Copyright, Real-Time Innovations, Inc. (RTI)
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

#include "test_node_event.hpp"

class TestSingleThreadEvent : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node_a = std::make_shared<rclcpp::Node>("node_a", "/ns");
    node_b = std::make_shared<rclcpp::Node>("node_b", "/ns");
  }

  void TearDown()
  {
    node_a.reset();
    node_b.reset();
  }

  rclcpp::Node::SharedPtr node_a;
  rclcpp::Node::SharedPtr node_b;
};

TEST_F(TestSingleThreadEvent, create) {
  // Create with default properties
  auto events = new rti::ros2::node::SingleThreadedEventNotifier();
  (void)events;
  // Create with custom properties
  rti::core::cond::WaitSetProperty props;
  events = new rti::ros2::node::SingleThreadedEventNotifier(props);
}

TEST_F(TestSingleThreadEvent, notify) {
  using std_msgs::msg::String;

  // Common history size (used to drive the publishing of test samples)
  const size_t history_depth = 10;

  // Create two readers on Node A
  dds::sub::qos::DataReaderQos reader_qos =
    rti::ros2::node::default_datareader_qos(*node_a, "bar");
  reader_qos << dds::core::policy::Durability::TransientLocal();
  reader_qos << dds::core::policy::Reliability::Reliable();
  reader_qos << dds::core::policy::History::KeepLast(history_depth);

  auto node_a_reader_1 =
    rti::ros2::node::create_datareader<String>(*node_a, "bar", reader_qos);
  // Create another reader to attach a query condition. Use different signature
  // of create_datareader<T>() which takes an existing topic.
  // NOTE: it would be nice to be able to "clone" the existing reader, and the
  // only missing bit is being able to pass the existing reader's qos to create
  // the new one. Unfortunately, if we do this, the creation fails, because the
  // qos reaturned by DataReader::qos() contains some information uniquely 
  // identifying the reader which causes the new reader to be considered
  // the same by Connext's persentation layer.
  dds::topic::Topic<String> reader_topic =
    rti::ros2::node::lookup_topic<String>(*node_a, "bar");
  auto node_a_reader_2 =
    rti::ros2::node::create_datareader<String>(*node_a, reader_topic, reader_qos);
  // Create another reader which will use a query with parameters
  auto node_a_reader_3 =
    rti::ros2::node::create_datareader<String>(*node_a, "bar", reader_qos);
  // Create another reader which will use a CFT
  auto cft_topic = rti::ros2::node::create_filtered_topic<String>(
    *node_a, "bar", "filtered_samples",
    "data MATCH %0 OR data MATCH %1",
    {"'Hello World 1'", "'Hello World 3'"});
  auto node_a_reader_4 =
    rti::ros2::node::create_datareader<String>(*node_a, cft_topic, reader_qos);

  // Quick test of is_filtered()
  ASSERT_FALSE(rti::ros2::node::is_filtered(node_a_reader_3.topic_description()));
  ASSERT_TRUE(rti::ros2::node::is_filtered(node_a_reader_4.topic_description()));

  // Register callbacks for readers on Node A
  // - Both readers will have a "status" callback.
  // - One reader will have a standard "data" callback.
  // - The other driver will have a "data query" callback.
  // The callbacks will collect the subscription matched status and count
  // the received data samples.
  ReaderState node_a_reader_state_1(0);
  ReaderState node_a_reader_state_2(1);
  ReaderState node_a_reader_state_3(2);
  ReaderState node_a_reader_state_4(3);
  
  WriterState node_b_writer_state;

  rti::ros2::node::SingleThreadedEventNotifier events;

  // Simple data callback
  auto event_a_on_data_1 = events.notify_data<String>(node_a_reader_1,
    [&node_a_reader_state_1](
        dds::sub::DataReader<String> & reader,
        dds::sub::cond::ReadCondition & condition) {
      node_a_reader_state_1.on_data(reader, condition);
    });

  // Test properties of returned event
  ASSERT_EQ(node_a_reader_1, event_a_on_data_1->entity());
  ASSERT_NE(nullptr, event_a_on_data_1->condition());
  ASSERT_NE(nullptr, event_a_on_data_1->read_condition());
  ASSERT_EQ(dds::sub::status::DataState::new_data(),
    event_a_on_data_1->read_condition().state_filter());
  ASSERT_EQ(dds::sub::AnyDataReader(node_a_reader_1),
    event_a_on_data_1->read_condition().data_reader());
  
  // Query data callback
  auto event_a_on_data_2 = events.notify_data<String>(node_a_reader_2,
    [&node_a_reader_state_2](
        dds::sub::DataReader<String> & reader,
        dds::sub::cond::QueryCondition & condition){
      node_a_reader_state_2.on_data(reader, condition);
    },
    "data MATCH 'Hello World [0-9]'");
  
  // Query data callback with parameters
  auto event_a_on_data_3 = events.notify_data<String>(node_a_reader_3,
    [&node_a_reader_state_3](
        dds::sub::DataReader<String> & reader,
        dds::sub::cond::QueryCondition & condition){
      node_a_reader_state_3.on_data(reader, condition, true /* take */);
    },
    "data MATCH %0 OR data MATCH %1",
    {"'Hello World 0'", "'Hello World 2'"},
    // this reader's callback will take() messages, so notify any()
    dds::sub::status::DataState::any());
  
  // CFT reader can just use a regular callback
  auto event_a_on_data_4 = events.notify_data<String>(node_a_reader_4,
    [&node_a_reader_state_4](
        dds::sub::DataReader<String> & reader,
        dds::sub::cond::ReadCondition & condition){
      node_a_reader_state_4.on_data(reader, condition);
    });

  auto on_reader_status =
    [&node_a_reader_1,
     &node_a_reader_2,
     &node_a_reader_3,
     &node_a_reader_4,
     &node_a_reader_state_1,
     &node_a_reader_state_2,
     &node_a_reader_state_3,
     &node_a_reader_state_4](
        dds::sub::DataReader<String> & reader){
      if (reader == node_a_reader_1) {
        node_a_reader_state_1.on_status(reader);
      } else if (reader == node_a_reader_2) {
        node_a_reader_state_2.on_status(reader);
      } else if (reader == node_a_reader_3) {
        node_a_reader_state_3.on_status(reader);
      } else if (reader == node_a_reader_4) {
        node_a_reader_state_4.on_status(reader);
      } else {
        FAIL();
      }
    };
  auto event_a_on_status_1 =
    events.notify_status_changed<String>(node_a_reader_1, on_reader_status,
      dds::core::status::StatusMask::subscription_matched());
  auto event_a_on_status_2 =
    events.notify_status_changed<String>(node_a_reader_2, on_reader_status,
      dds::core::status::StatusMask::subscription_matched());
  auto event_a_on_status_3 =
    events.notify_status_changed<String>(node_a_reader_3, on_reader_status,
      dds::core::status::StatusMask::subscription_matched());
  auto event_a_on_status_4 =
    events.notify_status_changed<String>(node_a_reader_4, on_reader_status,
      dds::core::status::StatusMask::subscription_matched());

  // Create a writer on Node B. We do this after configuring reader events so
  // that none will be missed.
  // The writer will have only a "status" callback which will collect
  // the publication matched status.
  dds::pub::qos::DataWriterQos writer_qos =
    rti::ros2::node::default_datawriter_qos(*node_b, "bar");
  writer_qos << dds::core::policy::Durability::TransientLocal();
  writer_qos << dds::core::policy::Reliability::Reliable();
  writer_qos << dds::core::policy::History::KeepLast(history_depth);
  auto node_b_writer =
    rti::ros2::node::create_datawriter<String>(*node_b, "bar", writer_qos);

  auto event_b_on_status = events.notify_status_changed<String>(node_b_writer,
    [&node_b_writer_state](dds::pub::DataWriter<String> & writer) {
      node_b_writer_state.on_status(writer);
    },
    dds::core::status::StatusMask::publication_matched());

  // Publish half samples which should not be notified to the filtered callback,
  // only to the standard data callback.
  String msg("Hello World");
  for (size_t i = 0; i < history_depth / 2; i++) {
    node_b_writer.write(msg);
  }

  // Publish some sample that will be received by both callbacks.
  for (size_t i = 0; i < history_depth / 2; i++) {
    std::ostringstream msg_data;
    msg_data << "Hello World " << i;
    String msg(msg_data.str());
    node_b_writer.write(msg);
  }

  // "Spin" the event notifier until a condition becomes true or a timeout expires.
  #define wait_with_timeout(t_, st_, cond_, expect_timeout_) \
    {\
      auto start_ts = std::chrono::steady_clock::now();\
      bool timed_out = false;\
      bool complete = false;\
      do {\
        timed_out = (std::chrono::steady_clock::now() - start_ts) >= t_;\
        complete = cond_;\
        if (!timed_out && !complete) {\
          events.spin(st_);\
        }\
      } while (!timed_out && !complete);\
      if (expect_timeout_) {\
        ASSERT_TRUE(timed_out);\
      } else {\
        ASSERT_FALSE(timed_out);\
      }\
    }

  using namespace std::literals::chrono_literals;
  wait_with_timeout(1000ms, 100ms,
    node_a_reader_state_1.data_count >= 1U &&
    node_a_reader_state_2.data_count >= 1U &&
    node_a_reader_state_3.data_count >= 1U &&
    node_a_reader_state_4.data_count >= 1U &&
    node_a_reader_state_1.received_count == history_depth &&
    node_a_reader_state_2.received_count == history_depth / 2 &&
    node_a_reader_state_3.received_count == 2U &&
    node_a_reader_state_4.received_count == 2U &&
    node_a_reader_state_1.received_invalid_count == 0 &&
    node_a_reader_state_2.received_invalid_count == 0 &&
    node_a_reader_state_3.received_invalid_count == 0 &&
    node_a_reader_state_4.received_invalid_count == 0 &&
    node_a_reader_state_1.status_count == 1U &&
    node_a_reader_state_2.status_count == 1U &&
    node_a_reader_state_3.status_count == 1U &&
    node_a_reader_state_4.status_count == 1U &&
    node_a_reader_state_1.matched_status.current_count() == 1 &&
    node_a_reader_state_2.matched_status.current_count() == 1 &&
    node_a_reader_state_3.matched_status.current_count() == 1 &&
    node_a_reader_state_4.matched_status.current_count() == 1 &&
    node_b_writer_state.status_count >= 1U &&
    node_b_writer_state.matched_status.current_count() == 4,
    false /* expect timeout */);

  // Delete a reader 1 and check that the writer callback is notified.
  node_b_writer_state.status_count = 0;

  // Cancel events associated with the reader and delete it.
  events.cancel(event_a_on_data_1);
  events.cancel(event_a_on_status_1);
  node_a_reader_1.close();

  wait_with_timeout(1000ms, 50ms,
    node_b_writer_state.status_count == 1U &&
    node_b_writer_state.matched_status.current_count() == 3,
    false /* expect timeout */);

  // If we cancel the reader 2 status callback and delete the writer then
  // we shouldn't receive a notification for reader 2, only for reader 1.
  events.cancel(event_a_on_status_2);
  node_a_reader_state_2.status_count = 0;
  node_a_reader_state_3.status_count = 0;
  node_a_reader_state_4.status_count = 0;

  // Cancel event associated with the writer and delete it.
  events.cancel(event_b_on_status);
  node_b_writer.close();

  wait_with_timeout(1000ms, 50ms,
    node_a_reader_state_3.status_count == 1 &&
    node_a_reader_state_3.matched_status.current_count() == 0 &&
    node_a_reader_state_4.status_count == 1 &&
    node_a_reader_state_4.matched_status.current_count() == 0,
    false /* expect timeout */);

  // Check that the writer was actually reported unmatched to reader 2
  // but the callback was not call
  ASSERT_EQ(node_a_reader_state_2.status_count, 0U);
  ASSERT_TRUE((node_a_reader_2.status_changes() &
    dds::core::status::StatusMask::subscription_matched()).any());
  ASSERT_EQ(node_a_reader_2.subscription_matched_status().current_count(), 0);
}
