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

ament_add_gtest(test_domain
  test_domain.cpp)
if(TARGET test_domain)
  target_link_libraries(test_domain
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(test_topic
  test_topic.cpp)
if(TARGET test_topic)
  target_link_libraries(test_topic
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(test_pub
  test_pub.cpp)
if(TARGET test_pub)
  target_link_libraries(test_pub
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(test_sub
  test_sub.cpp)
if(TARGET test_sub)
  target_link_libraries(test_sub
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()

ament_add_gtest(test_callback
  test_callback.cpp)
if(TARGET test_callback)
  target_link_libraries(test_callback
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_types)
endif()
