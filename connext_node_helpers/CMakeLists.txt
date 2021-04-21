# (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.
cmake_minimum_required(VERSION 3.8)
project(connext_node_helpers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rti_connext_dds_cmake_module REQUIRED)
find_package(rmw_connextdds REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(RTIConnextDDS REQUIRED)
if(NOT RTIConnextDDS_FOUND)
  message(FATAL_ERROR "RTI Connext DDS (${CONNEXTDDS_ARCH}) not found in "
    "'${CONNEXTDDS_DIR}'")
elseif("${RTICONNEXTDDS_VERSION}" VERSION_LESS_EQUAL "6.0.0")
  message(FATAL_ERROR "RTI Connext DDS 6.x required. "
    "Found: ${RTICONNEXTDDS_VERSION}")
  set(RTIConnextDDS_FOUND false)
endif()

add_library(${PROJECT_NAME} SHARED
  src/lib.cpp
  include/rti/ros2/data/access.hpp
  include/rti/ros2/data/memory.hpp
  include/rti/ros2/ping/tester.hpp
  include/rti/ros2/ping/publisher.hpp
  include/rti/ros2/ping/subscriber.hpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp)
ament_export_dependencies(
  rclcpp
  rti_connext_dds_cmake_module
  rmw_connextdds)
ament_export_libraries(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} RTIConnextDDS::cpp2_api)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
    DIRECTORY include/
    DESTINATION include
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install CMake helpers
install(
    FILES
      cmake/FindRTIConnextDDS.cmake
      cmake/connext_add_executable.cmake
      cmake/connext_components_register_node.cmake
      cmake/connext_generate_message_typesupport_cpp.cmake
      cmake/connext_generate_typesupport_library.cmake
      cmake/node_main.cpp.in
    DESTINATION "share/${PROJECT_NAME}/cmake")

################################################################################
# Build included examples
################################################################################
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/connext_node_helpers.cmake)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

connext_generate_message_typesupport_cpp(String PACKAGE std_msgs)

add_library(connext_node_helpers_examples SHARED
  src/examples/ping_string.cpp
  ${std_msgs_String_FILES}
  include/connext_node_helpers/visibility_control.h)
ament_target_dependencies(connext_node_helpers_examples
  rclcpp
  rclcpp_components)
target_compile_definitions(connext_node_helpers_examples
  PRIVATE "CONNEXT_NODE_HELPERS_BUILDING_DLL")
target_include_directories(connext_node_helpers_examples
  PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(connext_node_helpers_examples
  RTIConnextDDS::cpp2_api
  ${PROJECT_NAME})
set_target_properties(
  connext_node_helpers_examples PROPERTIES ENABLE_EXPORTS true)
install(TARGETS connext_node_helpers_examples
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

connext_components_register_node(connext_node_helpers_examples
  PLUGIN "StringPingPublisher"
  EXECUTABLE ping_string_pub)

connext_components_register_node(connext_node_helpers_examples
  PLUGIN "StringPingSubscriber"
  EXECUTABLE ping_string_sub)

ament_package(
  CONFIG_EXTRAS_POST "${PROJECT_NAME}-extras.cmake.in"
)
