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
find_package(connext_node_helpers REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dds_node.cpp
  include/rclcpp_dds/rclcpp_dds.hpp
  include/rclcpp_dds/dds_node.hpp
  include/rclcpp_dds/dds_node_mixin.hpp
  include/rclcpp_dds/dds_node_options.hpp
  include/rclcpp_dds/visibility_control.hpp

  src/ros2dds/resolve.cpp
  include/ros2dds/ros2dds.hpp
  include/ros2dds/domain.hpp
  include/ros2dds/resolve.hpp
  include/ros2dds/topic.hpp
  include/ros2dds/pub.hpp
  include/ros2dds/sub.hpp
  include/ros2dds/callback.hpp
  include/ros2dds/notifier.hpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcpputils
  rcutils
)
ament_export_libraries(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
  # Modern C++ API
  RTIConnextDDS::cpp2_api
  # RTI Modern C++ Messaging API
  RTIConnextDDS::messaging_cpp2_api
)
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
  add_subdirectory(test/${PROJECT_NAME})
endif()

ament_export_dependencies(
  rclcpp
  rcpputils
  rcutils
)

ament_package()
