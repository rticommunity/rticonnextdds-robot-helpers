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


#include <cstdlib>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "std_msgs/msg/String.hpp"

#include "ros2dds/ros2dds.hpp"

struct CustomNode
{
  std::string name;
  std::string namespace_;
  dds::domain::DomainParticipant participant{0};
};
namespace ros2dds
{
template<>
size_t
domain_id(CustomNode & node)
{
  return node.participant.domain_id();
}
template<>
dds::domain::DomainParticipant
domain_participant(CustomNode & node)
{
  return node.participant;
}
template<>
std::string
node_name(CustomNode & node)
{
  return node.name;
}
template<>
std::string
node_namespace(CustomNode & node)
{
  return node.namespace_;
}
}  // namespace ros2dds

class TestCustomNode : public ::testing::Test
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
    node = CustomNode();
    node.name = "custom_node";
    node.namespace_ = "/custom_ns";
  }

  void TearDown()
  {
    node.participant = nullptr;
  }

  CustomNode node;
};

TEST_F(TestCustomNode, custom_node) {
  ASSERT_EQ(ros2dds::domain_id(node), 0U);
  dds::domain::DomainParticipant participant(dds::core::null);
  ASSERT_NO_THROW(
  {
    participant = ros2dds::domain_participant(node);
  });
  ASSERT_NE(participant, dds::core::null);
  ASSERT_EQ(participant.domain_id(), 0);

  auto writer = ros2dds::create_datawriter<std_msgs::msg::String>(node, "foo");
}
