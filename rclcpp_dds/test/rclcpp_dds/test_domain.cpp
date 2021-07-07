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

#include "rcutils/env.h"

#include "rclcpp/init_options.hpp"
#include "rclcpp/node_options.hpp"

#include "rclcpp_dds/rclcpp_dds.hpp"

class TestDomain : public ::testing::Test
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

  rclcpp_dds::DDSNode::SharedPtr node;
};

class TestDomainCustom : public TestDomain
{
protected:
  static void SetUpTestCase()
  {
#ifdef ROS_GALACTIC_OR_LATER
    // In Galactic+, we can use InitOptions::set_domain_id().
    // In earlier versions we must configure the domain id through env variables.
    rclcpp::InitOptions opts;
    opts.set_domain_id(46);
    rclcpp::init(0, nullptr, opts);
#else
    const char * const env_rc = rcutils_get_env("ROS_DOMAIN_ID", &env_domain_id);
    if (nullptr != env_rc) {
      throw std::runtime_error("failed to get ROS_DOMAIN_ID");
    }
    std::string custom_domain = std::to_string(46);
    if (!rcutils_set_env("ROS_DOMAIN_ID", custom_domain.c_str())) {
      throw std::runtime_error("failed to set ROS_DOMAIN_ID");
    }
    rclcpp::init(0, nullptr);
#endif
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
#ifndef ROS_GALACTIC_OR_LATER
    if (!rcutils_set_env("ROS_DOMAIN_ID", env_domain_id)) {
      throw std::runtime_error("failed to restore ROS_DOMAIN_ID");
    }
    env_domain_id = nullptr;
#endif
  }

  static const char * env_domain_id;
};

const char * TestDomainCustom::env_domain_id = nullptr;

TEST_F(TestDomain, domain_id) {
  ASSERT_EQ(node->domain_id(), 0U);
}

TEST_F(TestDomainCustom, domain_id) {
  ASSERT_EQ(node->domain_id(), 46U);
}

TEST_F(TestDomain, domain_participant) {
  ASSERT_NO_THROW(
  {
    node->domain_participant();
  });
  auto participant = node->domain_participant();
  ASSERT_NE(participant, dds::core::null);
  ASSERT_EQ(participant.domain_id(), 0);
}

TEST_F(TestDomainCustom, domain_participant) {
  ASSERT_NO_THROW(
  {
    node->domain_participant();
  });
  auto participant = node->domain_participant();
  ASSERT_NE(participant, dds::core::null);
  ASSERT_EQ(participant.domain_id(), 46);
}
