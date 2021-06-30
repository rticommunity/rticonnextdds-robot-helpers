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

#include <gtest/gtest.h>

#include <rti/ros2/node/topic.hpp>

#include <cstdlib>
#include <memory>

#include "std_msgs/msg/String.hpp"
#include "example_interfaces/srv/AddTwoInts.hpp"

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
  // Create a standard ROS topic
  auto topic = rti::ros2::node::create_topic<std_msgs::msg::String>(*node, "foo");
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "rt/ns/foo");
  ASSERT_STREQ(topic.type_name().c_str(), "std_msgs::msg::String");

  // Create a standard ROS topic w/custom type name
  auto topic_alt = rti::ros2::node::create_topic<std_msgs::msg::String>(
    *node, "foo_alt", rti::ros2::node::NodeTopicKind::Topic, true, "custom_type_name");
  ASSERT_NE(topic_alt, dds::core::null);
  ASSERT_STREQ(topic_alt.name().c_str(), "rt/ns/foo_alt");
  ASSERT_STREQ(topic_alt.type_name().c_str(), "custom_type_name");

  // Create a Request ROS topic
  auto topic_req = rti::ros2::node::create_topic<example_interfaces::srv::AddTwoInts_Request>(
    *node, "foo", rti::ros2::node::NodeTopicKind::Request);
  ASSERT_NE(topic_req, dds::core::null);
  ASSERT_STREQ(topic_req.name().c_str(), "rq/ns/fooRequest");
  ASSERT_STREQ(topic_req.type_name().c_str(), "example_interfaces::srv::AddTwoInts_Request");

  // Create a Reply ROS topic
  auto topic_rep = rti::ros2::node::create_topic<example_interfaces::srv::AddTwoInts_Response>(
    *node, "foo", rti::ros2::node::NodeTopicKind::Reply);
  ASSERT_NE(topic_rep, dds::core::null);
  ASSERT_STREQ(topic_rep.name().c_str(), "rr/ns/fooReply");
  ASSERT_STREQ(topic_rep.type_name().c_str(), "example_interfaces::srv::AddTwoInts_Response");
}

TEST_F(TestTopic, create_topic_dds) {
  // Create a standard topic
  auto topic = rti::ros2::node::create_topic<std_msgs::msg::String>(
    *node, "foo", rti::ros2::node::NodeTopicKind::Topic, false);
  ASSERT_NE(topic, dds::core::null);
  ASSERT_STREQ(topic.name().c_str(), "foo");
  ASSERT_STREQ(topic.type_name().c_str(), "std_msgs::msg::String");

  // Create a Request ROS topic
  auto topic_req = rti::ros2::node::create_topic<example_interfaces::srv::AddTwoInts_Request>(
    *node, "foo", rti::ros2::node::NodeTopicKind::Request, false);
  ASSERT_NE(topic_req, dds::core::null);
  ASSERT_STREQ(topic_req.name().c_str(), "fooRequest");
  ASSERT_STREQ(topic_req.type_name().c_str(), "example_interfaces::srv::AddTwoInts_Request");

  // Create a Reply ROS topic
  auto topic_rep = rti::ros2::node::create_topic<example_interfaces::srv::AddTwoInts_Response>(
    *node, "foo", rti::ros2::node::NodeTopicKind::Reply, false);
  ASSERT_NE(topic_rep, dds::core::null);
  ASSERT_STREQ(topic_rep.name().c_str(), "fooReply");
  ASSERT_STREQ(topic_rep.type_name().c_str(), "example_interfaces::srv::AddTwoInts_Response");
}

TEST_F(TestTopic, lookup_topic) {
  auto topic_lookup = rti::ros2::node::lookup_topic<std_msgs::msg::String>(*node, "foo");
  ASSERT_EQ(topic_lookup, dds::core::null);

  auto topic = rti::ros2::node::create_topic<std_msgs::msg::String>(*node, "foo");
  ASSERT_NE(topic, dds::core::null);

  topic_lookup = rti::ros2::node::lookup_topic<std_msgs::msg::String>(*node, "foo");
  ASSERT_NE(topic_lookup, dds::core::null);
}

TEST_F(TestTopic, assert_topic) {
  auto topic = rti::ros2::node::lookup_topic<std_msgs::msg::String>(*node, "foo");
  ASSERT_EQ(topic, dds::core::null);

  topic = rti::ros2::node::assert_topic<std_msgs::msg::String>(*node, "foo");
  ASSERT_NE(topic, dds::core::null);

  ASSERT_THROW(
  {
    rti::ros2::node::create_topic<std_msgs::msg::String>(*node, "foo");
  }, dds::core::Error);

  auto topic_2 = rti::ros2::node::assert_topic<std_msgs::msg::String>(*node, "foo");
  ASSERT_EQ(topic, topic_2);

  auto topic_3 = rti::ros2::node::lookup_topic<std_msgs::msg::String>(*node, "foo");
  ASSERT_EQ(topic_2, topic_3);

  // Can't assert with a different type name
  ASSERT_THROW(
  {
    rti::ros2::node::assert_topic<std_msgs::msg::String>(
      *node, "foo",
      rti::ros2::node::NodeTopicKind::Topic, true, "different_type_name");
  }, std::runtime_error);
}
