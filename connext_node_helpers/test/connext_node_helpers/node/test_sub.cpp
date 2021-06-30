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

#include <rti/ros2/node/sub.hpp>

#include <cstdlib>
#include <memory>

#include "std_msgs/msg/String.hpp"
#include "example_interfaces/srv/AddTwoInts.hpp"

class TestSub : public ::testing::Test
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

TEST_F(TestSub, create_datareader_ros) {
  // Create a reader for a standard ROS topic
  auto reader = rti::ros2::node::create_datareader<std_msgs::msg::String>(*node, "foo");
  ASSERT_NE(reader, dds::core::null);
  ASSERT_STREQ(reader.topic_description().name().c_str(), "rt/ns/foo");
  ASSERT_STREQ(reader.topic_description().type_name().c_str(), "std_msgs::msg::String");

  // Create a reader for a standard ROS topic w/custom type name
  auto reader_alt = rti::ros2::node::create_datareader<std_msgs::msg::String>(
    *node, "foo_alt", rti::ros2::node::NodeTopicKind::Topic, true, "custom_type_name");
  ASSERT_NE(reader_alt, dds::core::null);
  ASSERT_STREQ(reader_alt.topic_description().name().c_str(), "rt/ns/foo_alt");
  ASSERT_STREQ(reader_alt.topic_description().type_name().c_str(), "custom_type_name");

  // Create a reader for a Request ROS topic
  auto reader_req =
    rti::ros2::node::create_datareader<example_interfaces::srv::AddTwoInts_Request>(
    *node, "foo", rti::ros2::node::NodeTopicKind::Request);
  ASSERT_NE(reader_req, dds::core::null);
  ASSERT_STREQ(reader_req.topic_description().name().c_str(), "rq/ns/fooRequest");
  ASSERT_STREQ(
    reader_req.topic_description().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Request");

  // Create a reader for a Reply ROS topic
  auto reader_rep =
    rti::ros2::node::create_datareader<example_interfaces::srv::AddTwoInts_Response>(
    *node, "foo", rti::ros2::node::NodeTopicKind::Reply);
  ASSERT_NE(reader_rep, dds::core::null);
  ASSERT_STREQ(reader_rep.topic_description().name().c_str(), "rr/ns/fooReply");
  ASSERT_STREQ(
    reader_rep.topic_description().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Response");
}

TEST_F(TestSub, create_reader_dds) {
  // Create a reader for a standard topic
  auto reader = rti::ros2::node::create_datareader<std_msgs::msg::String>(
    *node, "foo", rti::ros2::node::NodeTopicKind::Topic, false);
  ASSERT_NE(reader, dds::core::null);
  ASSERT_STREQ(reader.topic_description().name().c_str(), "foo");
  ASSERT_STREQ(reader.topic_description().type_name().c_str(), "std_msgs::msg::String");

  // Create a reader for a Request ROS topic
  auto reader_req =
    rti::ros2::node::create_datareader<example_interfaces::srv::AddTwoInts_Request>(
    *node, "foo", rti::ros2::node::NodeTopicKind::Request, false);
  ASSERT_NE(reader_req, dds::core::null);
  ASSERT_STREQ(reader_req.topic_description().name().c_str(), "fooRequest");
  ASSERT_STREQ(
    reader_req.topic_description().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Request");

  // Create a reader for a Reply ROS topic
  auto reader_rep =
    rti::ros2::node::create_datareader<example_interfaces::srv::AddTwoInts_Response>(
    *node, "foo", rti::ros2::node::NodeTopicKind::Reply, false);
  ASSERT_NE(reader_rep, dds::core::null);
  ASSERT_STREQ(reader_rep.topic_description().name().c_str(), "fooReply");
  ASSERT_STREQ(
    reader_rep.topic_description().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Response");
}
