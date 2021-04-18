# ROS 2 utilities for RTI Connext DDS

This repository contains a collection of helper resources to simplify the
implementation of ROS 2 applications which use the RTI Connext DDS APIs.

- [Use `connext_node_helpers` in a ROS 2 package](#use-connext_node_helpers-in-a-ros-2-package)
- [CMake Helpers](#cmake-helpers)
  - [connext_generate_typesupport_library](#connext_generate_typesupport_library)
  - [connext_generate_message_typesupport_cpp](#connext_generate_message_typesupport_cpp)
  - [connext_components_register_node](#connext_components_register_node)
  - [connext_add_executable](#connext_add_executable)
- [Other useful resources](#other-useful-resources)

## Use `connext_node_helpers` in a ROS 2 package

Package `connext_node_helpers` provides CMake and C++ helpers to facilitate the
implementation of ROS 2 packages based on RTI Connext DDS.

Add this package to your `package.xml`'s dependencies and then load it in your
`CMakeLists.txt`:

- `package.xml`:

  ```xml
  <package format="3">
    <name>my_package</name>
    
    <!-- ... -->

    <depend>connext_node_helpers</depend>
  
    <!-- ... -->
  </package>
  ```

- `CMakeLists.txt`

  ```cmake
  cmake_minimum_required(VERSION 3.5)
  project(my_package)

  # ...

  find_package(connext_node_helpers REQUIRED)

  # ...
  ```

## CMake Helpers

Once loaded in a `CMakeList.txt`, the module offers several CMake functions that
can be used to facilitate some common build task for ROS 2 applications that
want to use RTI Connext DDS.

### connext_generate_typesupport_library

Generates a shared library containing DDS type support code from a list of ROS 2
messages and (soon) services, but also regular Connext IDL files.

This function takes a list of ROS 2 types and will generate a shared library
after defining appropriate code generation targets for each type using
[`connext_generate_message_typesupport_cpp`](#connext_generate_message_typesupport_cpp).

*Example usage:*

```cmake
find_package(connext_node_helpers REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

connext_generate_typesupport_library(my_connext_types_lib
  MESSAGES
    std_msgs/String
    std_msgs/Header
    builtin_interfaces/Time
    sensor_msgs/PointField
    sensor_msgs/PointCloud2
  IDLS
    idl/my/custom/ns/MyType.idl@my/custom/ns
    idl/SomeTypesWithoutNamespace.idl
  ZEROCOPY)
```

### connext_generate_message_typesupport_cpp

Generates type support code with `rtiddsgen` from a ROS 2 message definition.

This is a "lower-level" helper which only defines a code generation target, and
it will not actually build the generated files.

The list of generated files is returned in an output variable `<pkg>_<type>_FILES`.
Tt is up to the caller to consume it appropriately as part of an `add_executable()`
or `add_library()` command.

The compilation target will also need to be configured with the appropriate
include directories. By default, all files will be generated in
`${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen/<pkg>`, and it is sufficient to add
`${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen` to the include path (since all files
must always be included as `#include "<pkg>/<type>.idl"`).

*Example usage:*

```cmake
find_package(connext_node_helpers REQUIRED)

# Generate type support for type `std_msgs::msg::String`.
# When using this syntax, the first argument is the "base name" of the type
# and the PACKAGE argument must always be specified to qualify the type.
# The list of generated files will be stored as `${std_msgs_String_FILES}`.
# The generated code must be included with `#include "std_msgs/msg/String.hpp"`.
find_package(std_msgs REQUIRED)
connext_generate_message_typesupport_cpp(String PACKAGE std_msgs)

# Generate type support from a typical IDL file for Connext.
# In this case, the input is the path to the input file, and the
# PACKAGE argument can be used to specify an optional "include prefix".

# Generated files will be available as `${my_custom_ns_MyType_FILES}`
# The generated code must be included with `#include "my/custom/ns/MyType.hpp"`.
connext_generate_message_typesupport_cpp(idl/my/custom/ns/MyType.idl
  PACKAGE my/custom/ns)

# Generated files will be available as `${SomeTypesWithoutNamespace_FILES}`
# The generated code must be included with `#include "SomeTypesWithoutNamespace.hpp"`.
connext_generate_message_typesupport_cpp(idl/SomeTypesWithoutNamespace.idl)

add_executable(my_app
  main.c
  ${std_msgs_String_FILES}
  ${my_custom_ns_MyType_FILES}
  ${SomeTypesWithoutNamespace_FILES})

target_include_directories(my_app
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen)
```

### connext_components_register_node

Similar to `rclcpp_components_register_node()`, registers a node from a component
library, and generates an executable to spin it in a `main()` function.

This function differs from `rclcpp_components_register_node()` in that the
generate executable will link RTI Connext DDS directly, and it will also be
properly linked in order to allow sharing of the DomainParticipantFactory
between the node component and the RMW.

*Example usage:*

```cmake
find_package(connext_node_helpers REQUIRED)

# First create a library with node components
add_library(my_node_components SHARED
  my_node.cpp)
ament_target_dependencies(my_node_components
  rclcpp
  rclcpp_components)
target_link_libraries(my_node_components
  RTIConnextDDS::cpp2_api)

# Then register each component into its own executable
connext_components_register_node(my_node_components
  PLUGIN "my_node::MyNode"
  EXECUTABLE my_node)
```

### connext_add_executable

Build a ROS 2/DDS C++ application.

This function can be used to simplify the definition of a build target for a
ROS 2 C++ application that requires direct access to RTI Connext DDS. The
function will take care of most boiler plate commands required to configure the
target with the required dependencies and the appropriate linkage.

*Example usage:*

```cmake
find_package(connext_node_helpers REQUIRED)
find_package(std_msgs REQUIRED)

# Generate type support files
connext_generate_message_typesupport_cpp(String
  PACKAGE std_msgs)

# Build application and generated files
connext_add_executable(
  NAME my_app
  SOURCES
    my_app.cpp
    ${std_msgs_String_FILES}
  INCLUDES
    ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen)
```

## Other useful resources

- [`rticonnextdds-ros2-demos`](https://github.com/asorbini/rticonnextdds-ros2-demos)
  - Collection of example hybrid ROS 2/Connext applications.
- [`rticonnextdds-ros2-msgs`](https://github.com/asorbini/rticonnextdds-ros2-msgs)
  - Helper library containing C++11 message type supports generated with
   `rtiddsgen` for almost every type include in ROS 2.
