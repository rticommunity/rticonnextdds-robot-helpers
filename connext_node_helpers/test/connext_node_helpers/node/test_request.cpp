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

#include <rti/ros2/node/request.hpp>

#include <cstdlib>
#include <memory>
#include <sstream>

#include "example_interfaces/srv/AddTwoInts.hpp"

class TestRequest : public ::testing::Test
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

TEST_F(TestRequest, create_requester_ros) {
  // Create a reader for a standard ROS topic
  auto requester = 
    rti::ros2::node::create_requester<
      example_interfaces::srv::AddTwoInts_Request,
      example_interfaces::srv::AddTwoInts_Response>(*node, "foo");
  ASSERT_NE(requester, dds::core::null);
  auto req_writer = requester.request_datawriter();
  ASSERT_STREQ(req_writer.topic().name().c_str(), "rq/ns/fooRequest");
  ASSERT_STREQ(req_writer.topic().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Request");
  auto req_reader = requester.reply_datareader();
  std::stringstream reply_topic;
  reply_topic << "rr/ns/fooReply_" << req_writer.instance_handle();
  ASSERT_STREQ(req_reader.topic_description().name().c_str(), reply_topic.str().c_str());
  ASSERT_STREQ(req_reader.topic_description().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Response");
}

TEST_F(TestRequest, create_requester_dds) {
  // Create a reader for a standard topic
  auto requester = 
    rti::ros2::node::create_requester<
      example_interfaces::srv::AddTwoInts_Request,
      example_interfaces::srv::AddTwoInts_Response>(*node, "foo", false);
  ASSERT_NE(requester, dds::core::null);
  auto req_writer = requester.request_datawriter();
  ASSERT_STREQ(req_writer.topic().name().c_str(), "fooRequest");
  ASSERT_STREQ(req_writer.topic().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Request");
  auto req_reader = requester.reply_datareader();
  std::stringstream reply_topic;
  reply_topic << "fooReply_" << req_writer.instance_handle();
  ASSERT_STREQ(req_reader.topic_description().name().c_str(), reply_topic.str().c_str());
  ASSERT_STREQ(req_reader.topic_description().type_name().c_str(),
    "example_interfaces::srv::AddTwoInts_Response");
}
