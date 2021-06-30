# Copyright 2021 Real-Time Innovations, Inc. (RTI)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

find_package(ament_cmake_gtest REQUIRED)

find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

connext_generate_typesupport_library(${PROJECT_NAME}_test_types
  MESSAGES
    std_msgs/String
  SERVICES
    example_interfaces/AddTwoInts
  ZEROCOPY)

ament_add_gtest(
  test_node_domain
  node/test_domain.cpp)
if(TARGET test_node_domain)
  target_link_libraries(test_node_domain
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(
  test_node_topic_name
  node/test_topic_name.cpp)
if(TARGET test_node_topic_name)
  target_link_libraries(test_node_topic_name
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(
  test_node_topic
  node/test_topic.cpp)
if(TARGET test_node_topic)
  target_link_libraries(test_node_topic
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(
  test_node_pub
  node/test_pub.cpp)
if(TARGET test_node_pub)
  target_link_libraries(test_node_pub
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(
  test_node_sub
  node/test_sub.cpp)
if(TARGET test_node_sub)
  target_link_libraries(test_node_sub
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(
  test_node_request
  node/test_request.cpp)
if(TARGET test_node_request)
  target_link_libraries(test_node_request
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(
  test_node_event_multi
  node/test_node_event_multi.cpp
  node/test_node_event.hpp)
if(TARGET test_node_event_multi)
  target_link_libraries(test_node_event_multi
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
  target_include_directories(test_node_event_multi
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/node)
endif()

ament_add_gtest(
  test_node_event_single
  node/test_node_event_single.cpp
  node/test_node_event.hpp)
if(TARGET test_node_event_single)
  target_link_libraries(test_node_event_single
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
  target_include_directories(test_node_event_single
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/node)
endif()
