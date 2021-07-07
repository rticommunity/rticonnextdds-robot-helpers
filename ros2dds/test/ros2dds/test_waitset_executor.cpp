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

#include <memory>

#include "test_waitset_executor.hpp"

#include "ros2dds/waitset_executor.hpp"
#include "ros2dds/condition.hpp"

class TestExecutor : public ::testing::Test
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

struct NotifierTestState
{
  ReaderState node_a_reader_1{0};
  ReaderState node_a_reader_2{1};
  ReaderState node_a_reader_3{2};
  ReaderState node_a_reader_4{3};

  WriterState node_b_writer;
};

template<typename ExecutorT>
void wait_with_timeout(
  ExecutorT & exec,
  const std::chrono::nanoseconds & max_wait,
  const std::chrono::nanoseconds & step_wait,
  std::function<bool()> functor,
  const bool expect_timeout)
{
  auto start_ts = std::chrono::steady_clock::now();
  bool timed_out = false;
  bool complete = false;
  do {
    timed_out = std::chrono::steady_clock::now() - start_ts >= max_wait;
    complete = functor();
    if (!timed_out && !complete) {
      exec.spin(step_wait);
    }
  } while (!timed_out && !complete);
  if (expect_timeout) {
    ASSERT_TRUE(timed_out);
  } else { \
    ASSERT_FALSE(timed_out);
  }
}

template<typename ExecutorT>
void test_executor(
  rclcpp::Node::SharedPtr node_a,
  rclcpp::Node::SharedPtr node_b)
{
  using std_msgs::msg::String;

  // Common history size (used to drive the publishing of test samples)
  const size_t history_depth = 10;

  // Create two readers on Node A
  dds::sub::qos::DataReaderQos reader_qos =
    ros2dds::default_datareader_qos(*node_a, "bar");
  reader_qos << dds::core::policy::Durability::TransientLocal();
  reader_qos << dds::core::policy::Reliability::Reliable();
  reader_qos << dds::core::policy::History::KeepLast(history_depth);

  auto node_a_reader_1 =
    ros2dds::template create_datareader<String>(*node_a, "bar", reader_qos);
  // Create another reader to attach a query condition. Use different signature
  // of create_datareader<T>() which takes an existing topic.
  // NOTE: it would be nice to be able to "clone" the existing reader, and the
  // only missing bit is being able to pass the existing reader's qos to create
  // the new one. Unfortunately, if we do this, the creation fails, because the
  // qos reaturned by DataReader::qos() contains some information uniquely
  // identifying the reader which causes the new reader to be considered
  // the same by Connext's persentation layer.
  dds::topic::Topic<String> reader_topic =
    ros2dds::template lookup_topic<String>(*node_a, "bar");
  auto node_a_reader_2 =
    ros2dds::template create_datareader<String>(*node_a, reader_topic, reader_qos);
  // Create another reader which will use a query with parameters
  auto node_a_reader_3 =
    ros2dds::template create_datareader<String>(*node_a, "bar", reader_qos);
  // Create another reader which will use a CFT
  auto cft_topic = ros2dds::create_content_filtered_topic<String>(
    *node_a,
    reader_topic,
    "filtered_samples",
    "data MATCH %0 OR data MATCH %1",
    {"'Hello World 1'", "'Hello World 3'"});
  auto node_a_reader_4 =
    ros2dds::template create_datareader<String>(*node_a, cft_topic, reader_qos);

  // Register callbacks for readers on Node A
  // - Both readers will have a "status" callback.
  // - One reader will have a standard "data" callback.
  // - The other driver will have a "data query" callback.
  // The callbacks will collect the subscription matched status and count
  // the received data samples.
  NotifierTestState state;

  ExecutorT exec;

  // Simple data callback
  auto condition_data_a_1 = ros2dds::condition::template read_data<String>(
    node_a_reader_1,
    [&state](const String & msg)
    {
      (void)msg;
      state.node_a_reader_1.on_data();
    });

  // Test properties of returned condition
  ASSERT_EQ(
    dds::sub::status::DataState::new_data(),
    condition_data_a_1.state_filter());
  ASSERT_EQ(
    dds::sub::AnyDataReader(node_a_reader_1),
    condition_data_a_1.data_reader());

  // Query data callback
  auto condition_data_a_2 = ros2dds::condition::template read_query<String>(
    node_a_reader_2,
    [&state](const String & msg)
    {
      (void)msg;
      state.node_a_reader_2.on_data();
    },
    "data MATCH 'Hello World [0-9]'");

  // Query data callback with parameters
  auto condition_data_a_3 = ros2dds::condition::template query<String>(
    node_a_reader_3,
    [&state](
      dds::sub::cond::QueryCondition & condition,
      dds::sub::DataReader<String> & reader)
    {
      state.node_a_reader_3.on_data(reader, condition, true /* take */);
    },
    "data MATCH %0 OR data MATCH %1",
    // this reader's callback will take() messages, so notify any()
    dds::sub::status::DataState::any(),
    {"'Hello World 0'", "'Hello World 2'"});

  // CFT reader can just use a regular callback
  auto condition_data_a_4 = ros2dds::condition::template data<String>(
    node_a_reader_4,
    [&state](
      dds::sub::cond::ReadCondition & condition,
      dds::sub::DataReader<String> & reader)
    {
      state.node_a_reader_4.on_data(reader, condition);
    });

  auto on_reader_status =
    [&node_a_reader_1,
      &node_a_reader_2,
      &node_a_reader_3,
      &node_a_reader_4,
      &state](
    dds::core::cond::StatusCondition & condition,
    dds::sub::DataReader<String> & reader)
    {
      (void)condition;
      if (reader == node_a_reader_1) {
        state.node_a_reader_1.on_status(reader);
      } else if (reader == node_a_reader_2) {
        state.node_a_reader_2.on_status(reader);
      } else if (reader == node_a_reader_3) {
        state.node_a_reader_3.on_status(reader);
      } else if (reader == node_a_reader_4) {
        state.node_a_reader_4.on_status(reader);
      } else {
        FAIL();
      }
    };
  auto condition_status_a_1 =
    ros2dds::condition::template status<String>(
    node_a_reader_1, on_reader_status,
    dds::core::status::StatusMask::subscription_matched());
  auto condition_status_a_2 =
    ros2dds::condition::template status<String>(
    node_a_reader_2, on_reader_status,
    dds::core::status::StatusMask::subscription_matched());
  auto condition_status_a_3 =
    ros2dds::condition::template status<String>(
    node_a_reader_3, on_reader_status,
    dds::core::status::StatusMask::subscription_matched());
  auto condition_status_a_4 =
    ros2dds::condition::template status<String>(
    node_a_reader_4, on_reader_status,
    dds::core::status::StatusMask::subscription_matched());

  // Create a writer on Node B. We do this after configuring reader events so
  // that none will be missed.
  // The writer will have only a "status" callback which will collect
  // the publication matched status.
  dds::pub::qos::DataWriterQos writer_qos =
    ros2dds::default_datawriter_qos(*node_b, "bar");
  writer_qos << dds::core::policy::Durability::TransientLocal();
  writer_qos << dds::core::policy::Reliability::Reliable();
  writer_qos << dds::core::policy::History::KeepLast(history_depth);
  auto node_b_writer =
    ros2dds::template create_datawriter<String>(*node_b, "bar", writer_qos);

  auto condition_status_b = ros2dds::condition::template status<String>(
    node_b_writer,
    [&state](
      dds::core::cond::StatusCondition & condition,
      dds::pub::DataWriter<String> & writer)
    {
      (void)condition;
      state.node_b_writer.on_status(writer);
    },
    dds::core::status::StatusMask::publication_matched());

  // Attach all conditions to the executor
  exec.attach(condition_data_a_1);
  exec.attach(condition_data_a_2);
  exec.attach(condition_data_a_3);
  exec.attach(condition_data_a_4);
  exec.attach(condition_status_a_1);
  exec.attach(condition_status_a_2);
  exec.attach(condition_status_a_3);
  exec.attach(condition_status_a_4);
  exec.attach(condition_status_b);

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

  using namespace std::literals::chrono_literals;
  wait_with_timeout(
    exec, 1000ms, 50ms,
    [&state]() {
      return
      state.node_a_reader_1.data_count >= 1U &&
      state.node_a_reader_2.data_count >= 1U &&
      state.node_a_reader_3.data_count >= 1U &&
      state.node_a_reader_4.data_count >= 1U &&
      state.node_a_reader_1.received_count == history_depth &&
      state.node_a_reader_2.received_count == history_depth / 2 &&
      state.node_a_reader_3.received_count == 2U &&
      state.node_a_reader_4.received_count == 2U &&
      state.node_a_reader_1.received_invalid_count == 0 &&
      state.node_a_reader_2.received_invalid_count == 0 &&
      state.node_a_reader_3.received_invalid_count == 0 &&
      state.node_a_reader_4.received_invalid_count == 0 &&
      state.node_a_reader_1.status_count == 1U &&
      state.node_a_reader_2.status_count == 1U &&
      state.node_a_reader_3.status_count == 1U &&
      state.node_a_reader_4.status_count == 1U &&
      state.node_a_reader_1.matched_status.current_count() == 1 &&
      state.node_a_reader_2.matched_status.current_count() == 1 &&
      state.node_a_reader_3.matched_status.current_count() == 1 &&
      state.node_a_reader_4.matched_status.current_count() == 1 &&
      state.node_b_writer.status_count >= 1U &&
      state.node_b_writer.matched_status.current_count() == 4;
    },
    false /* expect timeout */);

  // Delete a reader 1 and check that the writer callback is notified.
  state.node_b_writer.status_count = 0;

  // Cancel events associated with the reader and delete it.
  exec.detach(condition_data_a_1);
  exec.detach(condition_status_a_1);
  node_a_reader_1.close();

  wait_with_timeout(
    exec, 1000ms, 50ms,
    [&state]() {
      return
      state.node_b_writer.status_count == 1U &&
      state.node_b_writer.matched_status.current_count() == 3;
    },
    false /* expect timeout */);

  // If we cancel the reader 2 status callback and delete the writer then
  // we shouldn't receive a notification for reader 2, only for reader 1.
  exec.detach(condition_status_a_2);
  state.node_a_reader_2.status_count = 0;
  state.node_a_reader_3.status_count = 0;
  state.node_a_reader_4.status_count = 0;

  // Cancel event associated with the writer and delete it.
  exec.detach(condition_status_b);
  node_b_writer.close();

  wait_with_timeout(
    exec, 1000ms, 50ms,
    [&state]() {
      return
      state.node_a_reader_3.status_count == 1 &&
      state.node_a_reader_3.matched_status.current_count() == 0 &&
      state.node_a_reader_4.status_count == 1 &&
      state.node_a_reader_4.matched_status.current_count() == 0;
    },
    false /* expect timeout */);

  // Check that the writer was actually reported unmatched to reader 2
  // but the callback was not call
  ASSERT_EQ(state.node_a_reader_2.status_count, 0U);
  ASSERT_TRUE(
    (node_a_reader_2.status_changes() &
    dds::core::status::StatusMask::subscription_matched()).any());
  ASSERT_EQ(node_a_reader_2.subscription_matched_status().current_count(), 0);
}

class TestAsyncExecutor : public TestExecutor {};

TEST_F(TestAsyncExecutor, create) {
  // Create with default properties
  ros2dds::AsyncWaitSetExecutor exec;
  (void)exec;
}

TEST_F(TestAsyncExecutor, notify) {
  test_executor<ros2dds::AsyncWaitSetExecutor>(node_a, node_b);
}

class TestSyncExecutor : public TestExecutor {};

TEST_F(TestSyncExecutor, create) {
  // Create with default properties
  ros2dds::SyncWaitSetExecutor exec;
  (void)exec;
}

TEST_F(TestSyncExecutor, notify) {
  test_executor<ros2dds::SyncWaitSetExecutor>(node_a, node_b);
}
