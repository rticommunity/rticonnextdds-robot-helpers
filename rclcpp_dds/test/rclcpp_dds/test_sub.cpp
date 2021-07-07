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

#include "rclcpp_dds/rclcpp_dds.hpp"

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
    node = std::make_shared<rclcpp_dds::DdsNode>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp_dds::DdsNode::SharedPtr node;
};

TEST_F(TestSub, create_datareader) {
  using std_msgs::msg::String;
  using example_interfaces::srv::AddTwoInts_Request;
  using example_interfaces::srv::AddTwoInts_Response;

  // Create a reader for a standard ROS topic
  auto reader = node->create_datareader<String>("foo");
  ASSERT_NE(reader, dds::core::null);
  ASSERT_STREQ(reader.topic_description().name().c_str(), "rt/ns/foo");
  ASSERT_STREQ(reader.topic_description().type_name().c_str(), "std_msgs::msg::dds_::String_");

  // Create a reader for a standard ROS topic w/custom type name
  auto reader_alt = node->create_datareader<String>("foo_alt", "custom_type_name");
  ASSERT_NE(reader_alt, dds::core::null);
  ASSERT_STREQ(reader_alt.topic_description().name().c_str(), "rt/ns/foo_alt");
  ASSERT_STREQ(reader_alt.topic_description().type_name().c_str(), "dds_::custom_type_name_");

  // Create a reader for a Request ROS topic
  auto reader_req = node->create_service_datareader<AddTwoInts_Request>("foo");
  ASSERT_NE(reader_req, dds::core::null);
  ASSERT_STREQ(reader_req.topic_description().name().c_str(), "rq/ns/fooRequest");
  ASSERT_STREQ(
    reader_req.topic_description().type_name().c_str(),
    "example_interfaces::srv::dds_::AddTwoInts_Request_");

  // Create a reader for a Reply ROS topic
  auto reader_rep = node->create_service_datareader<AddTwoInts_Response>("foo", false);
  ASSERT_NE(reader_rep, dds::core::null);
  ASSERT_STREQ(reader_rep.topic_description().name().c_str(), "rr/ns/fooReply");
  ASSERT_STREQ(
    reader_rep.topic_description().type_name().c_str(),
    "example_interfaces::srv::dds_::AddTwoInts_Response_");
}


/******************************************************************************
 * Test readers of DDS topics
 ******************************************************************************/
class TestSubDds : public ::testing::Test
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
    rclcpp_dds::DdsNodeOptions opts;
    opts.use_ros_naming_conventions(false);
    node = std::make_shared<rclcpp_dds::DdsNode>("my_node", "/ns", opts);
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp_dds::DdsNode::SharedPtr node;
};


TEST_F(TestSubDds, create_datareader) {
  using std_msgs::msg::String;
  using example_interfaces::srv::AddTwoInts_Request;
  using example_interfaces::srv::AddTwoInts_Response;

  // Create a reader for a standard topic
  auto reader = node->create_datareader<String>("foo");
  ASSERT_NE(reader, dds::core::null);
  ASSERT_STREQ(reader.topic_description().name().c_str(), "foo");
  ASSERT_STREQ(reader.topic_description().type_name().c_str(), "std_msgs::msg::String");

  // Create a reader for a standard topic w/custom type name
  auto reader_alt = node->create_datareader<String>("foo_alt", "custom_type_name");
  ASSERT_NE(reader_alt, dds::core::null);
  ASSERT_STREQ(reader_alt.topic_description().name().c_str(), "foo_alt");
  ASSERT_STREQ(reader_alt.topic_description().type_name().c_str(), "custom_type_name");

  // Create a reader for a Request topic
  auto reader_req = node->create_service_datareader<AddTwoInts_Request>("foo");
  ASSERT_NE(reader_req, dds::core::null);
  ASSERT_STREQ(reader_req.topic_description().name().c_str(), "fooRequest");
  ASSERT_STREQ(
    reader_req.topic_description().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Request");

  // Create a reader for a Reply topic
  auto reader_rep = node->create_service_datareader<AddTwoInts_Response>("foo", false);
  ASSERT_NE(reader_rep, dds::core::null);
  ASSERT_STREQ(reader_rep.topic_description().name().c_str(), "fooReply");
  ASSERT_STREQ(
    reader_rep.topic_description().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Response");
}
