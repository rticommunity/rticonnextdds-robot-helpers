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

#include <cstdlib>
#include <memory>

#include "std_msgs/msg/String.hpp"
#include "example_interfaces/srv/AddTwoInts.hpp"

#include "ros2dds/pub.hpp"

class TestPub : public ::testing::Test
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

TEST_F(TestPub, create_datawriter_ros) {
  using ros2dds::create_datawriter;
  using ros2dds::TopicKind;
  using std_msgs::msg::String;
  using example_interfaces::srv::AddTwoInts_Request;
  using example_interfaces::srv::AddTwoInts_Response;

  // Create a writer for a standard ROS topic
  auto writer = create_datawriter<String>(*node, "foo");
  ASSERT_NE(writer, dds::core::null);
  ASSERT_STREQ(writer.topic().name().c_str(), "rt/ns/foo");
  ASSERT_STREQ(writer.topic().type_name().c_str(), "std_msgs::msg::dds_::String_");

  // Create a writer for a standard ROS topic w/custom type name
  auto writer_alt = create_datawriter<String>(
    *node, "foo_alt", TopicKind::Topic, true, "custom_type_name");
  ASSERT_NE(writer_alt, dds::core::null);
  ASSERT_STREQ(writer_alt.topic().name().c_str(), "rt/ns/foo_alt");
  ASSERT_STREQ(writer_alt.topic().type_name().c_str(), "dds_::custom_type_name_");

  // Create a writer for a Request ROS topic
  auto writer_req =
    create_datawriter<AddTwoInts_Request>(*node, "foo", TopicKind::Request);
  ASSERT_NE(writer_req, dds::core::null);
  ASSERT_STREQ(writer_req.topic().name().c_str(), "rq/ns/fooRequest");
  ASSERT_STREQ(
    writer_req.topic().type_name().c_str(),
    "example_interfaces::srv::dds_::AddTwoInts_Request_");

  // Create a writer for a Reply ROS topic
  auto writer_rep =
    create_datawriter<AddTwoInts_Response>(*node, "foo", TopicKind::Reply);
  ASSERT_NE(writer_rep, dds::core::null);
  ASSERT_STREQ(writer_rep.topic().name().c_str(), "rr/ns/fooReply");
  ASSERT_STREQ(
    writer_rep.topic().type_name().c_str(),
    "example_interfaces::srv::dds_::AddTwoInts_Response_");
}

TEST_F(TestPub, create_writer_dds) {
  using ros2dds::create_datawriter;
  using ros2dds::TopicKind;
  using std_msgs::msg::String;
  using example_interfaces::srv::AddTwoInts_Request;
  using example_interfaces::srv::AddTwoInts_Response;

  // Create a writer for a standard topic
  auto writer = create_datawriter<String>(*node, "foo", TopicKind::Topic, false);
  ASSERT_NE(writer, dds::core::null);
  ASSERT_STREQ(writer.topic().name().c_str(), "foo");
  ASSERT_STREQ(writer.topic().type_name().c_str(), "std_msgs::msg::String");

  // Create a writer for a standard topic w/custom type name
  auto writer_alt = create_datawriter<String>(
    *node, "foo_alt", TopicKind::Topic, false, "custom_type_name");
  ASSERT_NE(writer_alt, dds::core::null);
  ASSERT_STREQ(writer_alt.topic().name().c_str(), "foo_alt");
  ASSERT_STREQ(writer_alt.topic().type_name().c_str(), "custom_type_name");

  // Create a writer for a Request ROS topic
  auto writer_req =
    create_datawriter<AddTwoInts_Request>(*node, "foo", TopicKind::Request, false);
  ASSERT_NE(writer_req, dds::core::null);
  ASSERT_STREQ(writer_req.topic().name().c_str(), "fooRequest");
  ASSERT_STREQ(
    writer_req.topic().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Request");

  // Create a writer for a Reply ROS topic
  auto writer_rep =
    create_datawriter<AddTwoInts_Response>(*node, "foo", TopicKind::Reply, false);
  ASSERT_NE(writer_rep, dds::core::null);
  ASSERT_STREQ(writer_rep.topic().name().c_str(), "fooReply");
  ASSERT_STREQ(
    writer_rep.topic().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Response");
}
