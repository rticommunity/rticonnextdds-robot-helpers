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

#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>

#include "std_msgs/msg/String.hpp"
#include "rclcpp_dds/rclcpp_dds.hpp"

template<typename NodeT>
void wait_with_timeout(
  NodeT & node,
  const std::chrono::nanoseconds & max_wait,
  const std::chrono::nanoseconds & step_wait,
  std::function<bool()> functor,
  bool & timed_out)
{
  auto start_ts = std::chrono::steady_clock::now();
  bool complete = false;
  timed_out = false;
  do {
    timed_out = std::chrono::steady_clock::now() - start_ts >= max_wait;
    complete = functor();
    if (!timed_out && !complete) {
      node->dds_executor()->spin(step_wait);
    }
  } while (!timed_out && !complete);
}

class TestCallback : public ::testing::Test
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
    node = std::make_shared<rclcpp_dds::DDSNode>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  template<typename T>
  dds::pub::DataWriter<T>
  CreateWriter()
  {
    dds::pub::qos::DataWriterQos writer_qos = node->get_default_datawriter_qos();
    writer_qos << dds::core::policy::Durability::TransientLocal();
    writer_qos << dds::core::policy::Reliability::Reliable();
    writer_qos << dds::core::policy::History::KeepLast(10);

    return node->create_datawriter<T>("foo", writer_qos);
  }

  template<typename T>
  dds::sub::DataReader<T>
  CreateReader()
  {
    dds::sub::qos::DataReaderQos reader_qos = node->get_default_datareader_qos();
    reader_qos << dds::core::policy::Durability::TransientLocal();
    reader_qos << dds::core::policy::Reliability::Reliable();
    reader_qos << dds::core::policy::History::KeepLast(10);

    return node->create_datareader<T>("foo", reader_qos);
  }

  rclcpp_dds::DDSNode::SharedPtr node;
};

TEST_F(TestCallback, data_callback) {
  using std_msgs::msg::String;

  bool timed_out = false;

  auto writer = CreateWriter<String>();
  auto reader = CreateReader<String>();

  size_t received_count = 0;

  node->set_data_callback<String>(
    reader,
    [&received_count](
      dds::sub::cond::ReadCondition & condition,
      dds::sub::DataReader<String> & reader)
    {
      auto samples = reader.select().condition(condition).read();
      for (const auto & sample : samples) {
        (void)sample;
        received_count += 1;
      }
    });

  String msg("Hello World");
  writer.write(msg);

  using namespace std::chrono_literals;
  wait_with_timeout(
    node, 1000ms, 50ms,
    [&received_count]() {
      return received_count == 1;
    }, timed_out);
  ASSERT_FALSE(timed_out);

  // Replace callback and check previous one isn't notified;
  size_t received_count_2 = 0;
  received_count = 0;

  node->set_data_callback<String>(
    reader,
    [&received_count_2](const String & msg)
    {
      (void)msg;
      received_count_2 += 1;
    });

  // Callback should not be called for already read samples
  wait_with_timeout(
    node, 500ms, 50ms,
    [&received_count, &received_count_2]() {
      return received_count > 0 || received_count_2 > 0;
    }, timed_out);
  ASSERT_TRUE(timed_out);

  // New messages should only be notified to the new callback
  writer.write(msg);
  writer.write(msg);

  wait_with_timeout(
    node, 1000ms, 50ms,
    [&received_count, &received_count_2]() {
      return
      received_count == 0 &&
      received_count_2 == 2;
    }, timed_out);
  ASSERT_FALSE(timed_out);

  // Register another callback, this time to be notified of "any data".
  // Also, take messages out of the reader cache by using take_data_callback().
  size_t received_count_3 = 0;
  received_count_2 = 0;
  received_count = 0;

  node->set_data_callback<String, true>(
    reader,
    [&received_count_3](const String & msg)
    {
      (void)msg;
      received_count_3 += 1;
    },
    dds::sub::status::DataState::any_data());

  wait_with_timeout(
    node, 1000ms, 50ms,
    [&received_count, &received_count_2, &received_count_3]() {
      return
      received_count == 0 &&
      received_count_2 == 0 &&
      received_count_3 == 3;
    }, timed_out);
  ASSERT_FALSE(timed_out);
}


TEST_F(TestCallback, data_callback_w_query) {
  using std_msgs::msg::String;

  bool timed_out = false;

  auto writer = CreateWriter<String>();
  auto reader = CreateReader<String>();

  size_t received_count = 0;

  node->set_data_callback<String>(
    reader,
    [&received_count](
      dds::sub::cond::QueryCondition & condition,
      dds::sub::DataReader<String> & reader)
    {
      auto samples = reader.select().condition(condition).read();
      for (const auto & sample : samples) {
        (void)sample;
        received_count += 1;
      }
    },
    "data MATCH 'Hello World [0-9]'");

  writer.write(String("Hello World"));
  writer.write(String("Hello World 1"));
  writer.write(String("Hello World 2"));
  writer.write(String("Hello World 3"));

  using namespace std::chrono_literals;
  wait_with_timeout(
    node, 1000ms, 50ms,
    [&received_count]() {
      return received_count == 3;
    }, timed_out);
  ASSERT_FALSE(timed_out);

  // Replace callback and check previous one isn't notified
  // Use query parameters this time, and check for "any" data
  // (instead of just new).
  size_t received_count_2 = 0;
  received_count = 0;

  node->set_data_callback<String, true>(
    reader,
    [&received_count_2](const String & msg)
    {
      (void)msg;
      received_count_2 += 1;
    },
    "data MATCH %0 OR data MATCH %1",
    dds::sub::status::DataState::any_data(),
    {"'Hello World 1'", "'Hello World 3'"});

  wait_with_timeout(
    node, 1000ms, 50ms,
    [&received_count, &received_count_2]() {
      return
      received_count == 0 &&
      received_count_2 == 2;
    }, timed_out);
  ASSERT_FALSE(timed_out);

  // Replace callback to take() all samples out of the reader cache.
  size_t received_count_3 = 0;
  received_count_2 = 0;
  received_count = 0;

  node->set_data_callback<String, true>(
    reader,
    [&received_count_3](const String & msg)
    {
      (void)msg;
      received_count_3 += 1;
    },
    "data MATCH 'Hello World*'",
    dds::sub::status::DataState::any_data());

  wait_with_timeout(
    node, 1000ms, 50ms,
    [&received_count, &received_count_2, &received_count_3]() {
      return
      received_count == 0 &&
      received_count_2 == 0 &&
      received_count_3 == 2;
    }, timed_out);
  ASSERT_FALSE(timed_out);
}
