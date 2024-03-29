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

ament_add_gtest(test_ros2dds_domain
  test_domain.cpp)
if(TARGET test_ros2dds_domain)
  target_link_libraries(test_ros2dds_domain
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(test_ros2dds_resolve
  test_resolve.cpp)
if(TARGET test_ros2dds_resolve)
  target_link_libraries(test_ros2dds_resolve
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(test_ros2dds_topic
  test_topic.cpp)
if(TARGET test_ros2dds_topic)
  target_link_libraries(test_ros2dds_topic
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(test_ros2dds_pub
  test_pub.cpp)
if(TARGET test_ros2dds_pub)
  target_link_libraries(test_ros2dds_pub
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(test_ros2dds_sub
  test_sub.cpp)
if(TARGET test_ros2dds_sub)
  target_link_libraries(test_ros2dds_sub
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(test_ros2dds_request
  test_request.cpp)
if(TARGET test_ros2dds_request)
  target_link_libraries(test_ros2dds_request
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(test_ros2dds_waitset_executor
  test_waitset_executor.cpp
  test_waitset_executor.hpp)
if(TARGET test_ros2dds_waitset_executor)
  target_link_libraries(test_ros2dds_waitset_executor
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
  target_include_directories(test_ros2dds_waitset_executor
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/ros2dds)
endif()

ament_add_gtest(test_ros2dds_custom_node
  test_custom_node.cpp)
if(TARGET test_ros2dds_custom_node)
  target_link_libraries(test_ros2dds_custom_node
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
  target_include_directories(test_ros2dds_custom_node
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/ros2dds)
endif()
