cmake_minimum_required(VERSION 3.5)
project(rclcpp_dds)

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
find_package(ros2dds REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dds_node.cpp
  include/rclcpp_dds/rclcpp_dds.hpp
  include/rclcpp_dds/dds_node.hpp
  include/rclcpp_dds/dds_node_mixin.hpp
  include/rclcpp_dds/dds_node_options.hpp
  include/rclcpp_dds/visibility_control.hpp
)
ament_target_dependencies(${PROJECT_NAME}
  ros2dds
)
ament_export_libraries(${PROJECT_NAME})
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
  ament_lint_auto_find_test_dependencies()
  # add_subdirectory(test/${PROJECT_NAME})
endif()

ament_export_dependencies(
  ros2dds
)

ament_package()
