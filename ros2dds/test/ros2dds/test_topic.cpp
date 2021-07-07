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
#include "example_interfaces/srv/AddTwoInts.hpp"

#include "ros2dds/topic.hpp"

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
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

TEST_F(TestTopic, create_topic_ros) {
  using ros2dds::create_topic;
  using ros2dds::TopicKind;

  // Create a standard ROS topic
  auto topic = create_topic<std_msgs::msg::String>(*node, "foo");
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "rt/ns/foo");
  ASSERT_STREQ(topic.type_name().c_str(), "std_msgs::msg::dds_::String_");

  // Create a standard ROS topic w/custom type name
  auto topic_alt = create_topic<std_msgs::msg::String>(
    *node, "foo_alt", TopicKind::Topic, true, "custom_type");
  ASSERT_NE(topic_alt, dds::core::null);
  ASSERT_STREQ(topic_alt.name().c_str(), "rt/ns/foo_alt");
  ASSERT_STREQ(topic_alt.type_name().c_str(), "dds_::custom_type_");

  // Create a Request ROS topic
  auto topic_req = create_topic<example_interfaces::srv::AddTwoInts_Request>(
    *node, "foo", TopicKind::Request);
  ASSERT_NE(topic_req, dds::core::null);
  ASSERT_STREQ(topic_req.name().c_str(), "rq/ns/fooRequest");
  ASSERT_STREQ(topic_req.type_name().c_str(), "example_interfaces::srv::dds_::AddTwoInts_Request_");

  // Create a Reply ROS topic
  auto topic_rep = create_topic<example_interfaces::srv::AddTwoInts_Response>(
    *node, "foo", TopicKind::Reply);
  ASSERT_NE(topic_rep, dds::core::null);
  ASSERT_STREQ(topic_rep.name().c_str(), "rr/ns/fooReply");
  ASSERT_STREQ(
    topic_rep.type_name().c_str(),
    "example_interfaces::srv::dds_::AddTwoInts_Response_");
}

TEST_F(TestTopic, create_topic_dds) {
  using ros2dds::create_topic;
  using ros2dds::TopicKind;
  using std_msgs::msg::String;
  using example_interfaces::srv::AddTwoInts_Request;
  using example_interfaces::srv::AddTwoInts_Response;

  // Create a standard topic
  auto topic = create_topic<String>(*node, "foo", TopicKind::Topic, false);
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "foo");
  ASSERT_STREQ(topic.type_name().c_str(), "std_msgs::msg::String");

  // Create a standard topic with a custom type name
  auto topic_custom =
    create_topic<String>(*node, "foo_custom", TopicKind::Topic, false, "custom_type");
  ASSERT_NE(topic_custom, dds::core::null);
  ASSERT_STREQ(topic_custom.name().c_str(), "foo_custom");
  ASSERT_STREQ(topic_custom.type_name().c_str(), "custom_type");

  // Create a Request ROS topic
  auto topic_req = create_topic<AddTwoInts_Request>(*node, "foo", TopicKind::Request, false);
  ASSERT_NE(topic_req, dds::core::null);
  ASSERT_STREQ(topic_req.name().c_str(), "fooRequest");
  ASSERT_STREQ(topic_req.type_name().c_str(), "example_interfaces::srv::AddTwoInts_Request");

  // Create a Request ROS topic with a custom type name
  auto topic_req_custom = create_topic<AddTwoInts_Request>(
    *node, "foo_custom", TopicKind::Request, false, "custom_req_type");
  ASSERT_NE(topic_req_custom, dds::core::null);
  ASSERT_STREQ(topic_req_custom.name().c_str(), "foo_customRequest");
  ASSERT_STREQ(topic_req_custom.type_name().c_str(), "custom_req_type");

  // Create a Reply ROS topic
  auto topic_rep = create_topic<AddTwoInts_Response>(*node, "foo", TopicKind::Reply, false);
  ASSERT_NE(topic_rep, dds::core::null);
  ASSERT_STREQ(topic_rep.name().c_str(), "fooReply");
  ASSERT_STREQ(topic_rep.type_name().c_str(), "example_interfaces::srv::AddTwoInts_Response");

  // Create a Reply ROS topic
  auto topic_rep_custom = create_topic<AddTwoInts_Response>(
    *node, "foo_custom", TopicKind::Reply, false, "custom_rep_type");
  ASSERT_NE(topic_rep_custom, dds::core::null);
  ASSERT_STREQ(topic_rep_custom.name().c_str(), "foo_customReply");
  ASSERT_STREQ(topic_rep_custom.type_name().c_str(), "custom_rep_type");
}

TEST_F(TestTopic, lookup_topic) {
  using ros2dds::create_topic;
  using ros2dds::lookup_topic;
  using std_msgs::msg::String;

  auto topic_lookup = lookup_topic<String>(*node, "foo");
  ASSERT_EQ(topic_lookup, dds::core::null);

  auto topic = create_topic<String>(*node, "foo");
  ASSERT_NE(topic, dds::core::null);

  topic_lookup = lookup_topic<String>(*node, "foo");
  ASSERT_NE(topic_lookup, dds::core::null);
}

TEST_F(TestTopic, assert_topic) {
  using ros2dds::lookup_topic;
  using ros2dds::assert_topic;
  using ros2dds::create_topic;
  using ros2dds::TopicKind;
  using std_msgs::msg::String;

  auto topic = lookup_topic<String>(*node, "foo");
  ASSERT_EQ(topic, dds::core::null);

  topic = assert_topic<String>(*node, "foo");
  ASSERT_NE(topic, dds::core::null);

  ASSERT_THROW(
  {
    create_topic<String>(*node, "foo");
  }, dds::core::Error);

  auto topic_2 = assert_topic<String>(*node, "foo");
  ASSERT_EQ(topic, topic_2);

  auto topic_3 = lookup_topic<String>(*node, "foo");
  ASSERT_EQ(topic_2, topic_3);

  // Can't assert with a different type name
  ASSERT_THROW(
  {
    assert_topic<String>(*node, "foo", TopicKind::Topic, true, "different_type_name");
  }, rclcpp::exceptions::RCLInvalidArgument);
}
