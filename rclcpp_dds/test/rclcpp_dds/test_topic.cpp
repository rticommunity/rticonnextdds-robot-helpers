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
#include <string>
#include <vector>

#include "std_msgs/msg/String.hpp"
#include "example_interfaces/srv/AddTwoInts.hpp"

#include "rclcpp_dds/rclcpp_dds.hpp"

class TestTopic : public ::testing::Test
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
    node = make_node();
  }

  void TearDown()
  {
    node.reset();
  }

  virtual rclcpp_dds::DdsNode::SharedPtr
  make_node()
  {
    return std::make_shared<rclcpp_dds::DdsNode>("my_node", "/ns");
  }

  rclcpp_dds::DdsNode::SharedPtr node;
};

TEST_F(TestTopic, topic) {
  using std_msgs::msg::String;

  // Create a topic with default options
  auto topic = node->create_topic<String>("foo");
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "rt/ns/foo");
  ASSERT_STREQ(topic.type_name().c_str(), "std_msgs::msg::dds_::String_");

  // Check that we can't create the topic again if it already exists
  ASSERT_THROW(
  {
    node->create_topic<String>("foo");
  }, dds::core::Error);
  ASSERT_THROW(
  {
    node->create_topic<String>("foo", "custom_type");
  }, dds::core::Error);

  // Check that we can lookup topic by name
  auto topic_lookup = node->lookup_topic<String>("foo");
  ASSERT_EQ(topic, topic_lookup);
}

TEST_F(TestTopic, topic_w_custom_type) {
  using std_msgs::msg::String;

  // Create a "standard" topic with a custom type name
  auto topic = node->create_topic<String>("foo", "custom_type");
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "rt/ns/foo");
  ASSERT_STREQ(topic.type_name().c_str(), "dds_::custom_type_");

  // Check that we can't create the topic again if it already exists
  ASSERT_THROW(
  {
    node->create_topic<String>("foo");
  }, dds::core::Error);
  ASSERT_THROW(
  {
    node->create_topic<String>("foo", "custom_type");
  }, dds::core::Error);

  // Check that we can lookup() the topic again if it already exists
  auto topic_lookup = node->lookup_topic<String>("foo");
  ASSERT_EQ(topic, topic_lookup);
}

TEST_F(TestTopic, request_topic) {
  using example_interfaces::srv::AddTwoInts_Request;

  // Create a request topic with default options
  auto topic = node->create_service_topic<AddTwoInts_Request>("foo");
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "rq/ns/fooRequest");
  ASSERT_STREQ(
    topic.type_name().c_str(),
    "example_interfaces::srv::dds_::AddTwoInts_Request_");

  // Check that we can't create the topic again
  ASSERT_THROW(
  {
    node->create_service_topic<AddTwoInts_Request>("foo");
  }, dds::core::Error);

  // Check that we can lookup() the topic again if it already exists
  auto topic_lookup = node->lookup_service_topic<AddTwoInts_Request>("foo");
  ASSERT_EQ(topic, topic_lookup);
}

TEST_F(TestTopic, reply_topic) {
  using example_interfaces::srv::AddTwoInts_Response;

  // Create a reply topic with default options
  auto topic = node->create_service_topic<AddTwoInts_Response>("foo", false);
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "rr/ns/fooReply");
  ASSERT_STREQ(
    topic.type_name().c_str(),
    "example_interfaces::srv::dds_::AddTwoInts_Response_");

  // Check that we can't create the topic again
  ASSERT_THROW(
  {
    node->create_service_topic<AddTwoInts_Response>("foo", false);
  }, dds::core::Error);

  // Check that we can lookup() the topic again if it already exists
  auto topic_assert = node->lookup_service_topic<AddTwoInts_Response>("foo", false);
  ASSERT_EQ(topic, topic_assert);
}

/******************************************************************************
 * Test native DDS topics
 ******************************************************************************/
class TestTopicDds : public TestTopic
{
protected:
  virtual rclcpp_dds::DdsNode::SharedPtr
  make_node()
  {
    rclcpp_dds::DdsNodeOptions opts;
    opts.use_ros_naming_conventions(false);
    return std::make_shared<rclcpp_dds::DdsNode>("my_node", "/ns", opts);
  }
};

TEST_F(TestTopicDds, topic) {
  using std_msgs::msg::String;

  // Create a topic with default options
  auto topic = node->create_topic<String>("foo");
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "foo");
  ASSERT_STREQ(topic.type_name().c_str(), "std_msgs::msg::String");

  // Check that we can't create the topic again
  ASSERT_THROW(
  {
    node->create_topic<String>("foo");
  }, dds::core::Error);
  ASSERT_THROW(
  {
    node->create_topic<String>("foo", "custom_type");
  }, dds::core::Error);

  // Check that we can lookup() the topic
  auto topic_lookup = node->lookup_topic<String>("foo");
  ASSERT_EQ(topic, topic_lookup);
}

TEST_F(TestTopicDds, topic_w_custom_type) {
  using std_msgs::msg::String;

  // Create a "standard" topic with a custom type name
  auto topic = node->create_topic<String>("foo", "custom_type");
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "foo");
  ASSERT_STREQ(topic.type_name().c_str(), "custom_type");

  // Check that we can't create the topic again
  ASSERT_THROW(
  {
    node->create_topic<String>("foo");
  }, dds::core::Error);
  ASSERT_THROW(
  {
    node->create_topic<String>("foo", "custom_type");
  }, dds::core::Error);

  // Check that we can lookup() the topic
  auto topic_lookup = node->lookup_topic<String>("foo");
  ASSERT_EQ(topic, topic_lookup);
}

TEST_F(TestTopicDds, request_topic) {
  using example_interfaces::srv::AddTwoInts_Request;

  // Create a request topic with default options
  auto topic = node->create_service_topic<AddTwoInts_Request>("foo");
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "fooRequest");
  ASSERT_STREQ(
    topic.type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Request");

  // Check that we can't create the topic again
  ASSERT_THROW(
  {
    node->create_service_topic<AddTwoInts_Request>("foo");
  }, dds::core::Error);

  // Check that we can lookup() the topic
  auto topic_lookup = node->lookup_service_topic<AddTwoInts_Request>("foo");
  ASSERT_EQ(topic, topic_lookup);
}

TEST_F(TestTopicDds, reply_topic) {
  using example_interfaces::srv::AddTwoInts_Response;

  // Create a reply topic with default options
  auto topic = node->create_service_topic<AddTwoInts_Response>("foo", false);
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "fooReply");
  ASSERT_STREQ(
    topic.type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Response");

  // Check that we can't create the topic again
  ASSERT_THROW(
  {
    node->create_service_topic<AddTwoInts_Response>("foo", false);
  }, dds::core::Error);

  // Check that we can lookup() the topic
  auto topic_lookup = node->lookup_service_topic<AddTwoInts_Response>("foo", false);
  ASSERT_EQ(topic, topic_lookup);
}

TEST_F(TestTopicDds, content_filtered_topic) {
  using std_msgs::msg::String;

  // Create the base topic with default options
  auto topic = node->create_topic<String>("foo");
  ASSERT_NE(topic, dds::core::null);

  // Create a CFT
  const char * const cft_expr = "data LIKE 'Hello World!'";
  auto cft = node->create_content_filtered_topic<String>(topic, "my_filter", cft_expr);
  ASSERT_NE(cft, dds::core::null);
  ASSERT_STREQ(cft.filter_expression().c_str(), cft_expr);
  ASSERT_EQ(cft.filter_parameters().size(), 0U);
  ASSERT_EQ(cft.topic(), topic);

  // Look up CFT by name
  auto cft_lookup = node->lookup_content_filtered_topic<String>("my_filter");
  ASSERT_EQ(cft, cft_lookup);

  // Try to look up an unknown CFT by name
  cft_lookup = node->lookup_content_filtered_topic<String>("my_unknown_filter");
  ASSERT_EQ(nullptr, cft_lookup);

  // Check that we can't create another CFT with the same name
  ASSERT_THROW(
  {
    node->create_content_filtered_topic<String>(topic, "my_filter", cft_expr);
  }, dds::core::Error);

  // Create a CFT with parameters
  const char * const cft_expr_w_params = "data LIKE %0";
  const std::vector<std::string> cft_params = {"'HelloWorld'"};
  auto cft_w_params = node->create_content_filtered_topic<String>(
    topic,
    "my_filter_w_params", cft_expr_w_params, cft_params);
  ASSERT_NE(cft_w_params, dds::core::null);
  ASSERT_STREQ(cft_w_params.filter_expression().c_str(), cft_expr_w_params);
  // ASSERT_EQ(cft.filter_parameters().size(), 1);
  ASSERT_EQ(cft_params, cft_w_params.filter_parameters());
  ASSERT_EQ(cft.topic(), topic);
}
