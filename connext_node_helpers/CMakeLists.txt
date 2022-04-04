# Copyright 2021 Real-Time Innovations, Inc.  All rights reserved.
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
find_package(rti_connext_dds_cmake_module REQUIRED)
find_package(rmw_connextdds REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(RTIConnextDDS REQUIRED COMPONENTS messaging_api)
if(NOT RTIConnextDDS_FOUND)
  message(FATAL_ERROR "RTI Connext DDS (${CONNEXTDDS_ARCH}) not found in "
    "'${CONNEXTDDS_DIR}'")
elseif("${RTICONNEXTDDS_VERSION}" VERSION_LESS_EQUAL "6.0.0")
  message(FATAL_ERROR "RTI Connext DDS 6.x required. "
    "Found: ${RTICONNEXTDDS_VERSION}")
  set(RTIConnextDDS_FOUND false)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/connext_node_helpers.cmake)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
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
  DESTINATION "share/${PROJECT_NAME}/cmake"
)

ament_export_dependencies(
  rti_connext_dds_cmake_module
  rclcpp
  rclcpp_components
)

ament_package(
  CONFIG_EXTRAS_POST "${PROJECT_NAME}-extras.cmake.in"
)
