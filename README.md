# ROS 2 utilities for RTI Connext DDS

This repository contains a collection of helper resources to simplify the
implementation of ROS 2 applications which use the APIs provided by
[RTI Connext DDS Connectivity Framework](https://www.rti.com/products).

- [How to use `connext_node_helpers` in a ROS 2 package](#how-to-use-connext_node_helpers-in-a-ros-2-package)
- [DDS Node API](#dds-node-api)
  - [DataWriter Helpers](#datawriter-helpers)
    - [DDSNode::create_datawriter()](#ddsnodecreate_datawriter)
  - [DataReader Helpers](#datareader-helpers)
    - [DDSNode::create_datareader()](#ddsnodecreate_datareader)
  - [DomainParticipant Helpers](#domainparticipant-helpers)
    - [DDSNode::domain_id()](#ddsnodedomain_id)
    - [DDSNode::domain_participant()](#ddsnodedomain_participant)
  - [Topic Helpers](#topic-helpers)
    - [DDSNode::create_topic()](#ddsnodecreate_topic)
    - [DDSNode::lookup_topic()](#ddsnodelookup_topic)
    - [DDSNode::create_content_filtered_topic()](#ddsnodecreate_content_filtered_topic)
  - [Callback Notification](#callback-notification)
    - [DDSNode::set_data_callback()](#ddsnodeset_data_callback)
    - [DDSNode::cancel_data_callback()](#ddsnodecancel_data_callback)
    - [DDSNode::set_status_callback()](#ddsnodeset_status_callback)
    - [DDSNode::cancel_status_callback()](#ddsnodecancel_status_callback)
    - [DDSNode::add_user_callback()](#ddsnodeadd_user_callback)
    - [DDSNode::cancel_user_callback()](#ddsnodecancel_user_callback)
- [CMake Helpers](#cmake-helpers)
  - [connext_generate_typesupport_library](#connext_generate_typesupport_library)
  - [connext_generate_message_typesupport_cpp](#connext_generate_message_typesupport_cpp)
  - [connext_components_register_node](#connext_components_register_node)
  - [connext_add_executable](#connext_add_executable)
- [Other useful resources](#other-useful-resources)

## How to use this repository in your ROS 2 package

Add the packages you wish to yous to your package's `package.xml`, and then load
them in your `CMakeLists.txt`. For example, to use `rclcpp_dds`:

- `package.xml`:

  ```xml
  <package format="3">
    <name>my_package</name>
    
    <!-- ... -->

    <depend>rclcpp_dds</depend>
  
    <!-- ... -->
  </package>
  ```

- `CMakeLists.txt`

  - Load `rclcpp_dds` as a dependency:

    ```cmake
    find_package(rclcpp_dds REQUIRED)
    ```

  - Have build targets link `rclcpp_dds`' library:

    ```cmake
    ament_target_dependencies(my_build_target
      rclcpp_dds)
    ```
  
  - Export `rclcpp_dds` as a dependency:

    ```cmake
    ament_export_dependencies(rclcpp_dds)
    ```

## DDS Node API

Package `ros2dds` contains a helper API which ROS 2 applications
may use to simplify the creation of "native" DDS endpoints.

The API simplifies the integration of these endpoints in a ROS 2 system by
automatically applying some of the ROS 2 naming conventions.

These APIs can also be accessed by extending `rclcpp_dds::DDSNode`, which is
a subclass of `rclcpp::Node` provided by package `rclcpp_dds`.

In order to use the DDS Node API, a ROS 2 application must be running with
[`rmw_connextdds`](https://github.com/ros2/rmw_connextdds) as its underlying
RMW layer.

Typically, you will extend `rclcpp_dds::DDSNode`, and use the provided methods
similarly to how you normally use the `rclcpp::Node` API:

```cpp
#include "rclcpp_dds/rclcpp_dds.hpp"
#include "std_msgs/msg/String.hpp"

class MyNode : public rclcpp_dds::DDSNode
{
public:
  using std_msgs::msg::String;

  explicit MyNode(const rclcpp::NodeOptions & options)
  : DDSNode("my_node", options)
  {
    reader_ = this->create_datareader<String>("chatter");
    
    this->set_data_callback<String>(
      reader_,
      [this](const String & msg)
      {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data().c_str());
      });

    writer_ = this->create_datawriter<String>("chatter");

    using namespace std::chrono_literals;
    publish_timer_ = this->create_wall_timer(1s,
      [this](){
        String msg("Hello Connext!");
        writer_.write(msg);
      });
  }

private:
  dds::pub::DataWriter<String> writer_;
  dds::sub::DataReader<String> reader_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

```

### DataWriter Helpers

#### DDSNode::create_datawriter()

*Example usage:*

```cpp
#include "rclcpp_dds/rclcpp_dds.hpp"
#include "std_msgs/msg/String.hpp"

class MyNode : public rclcpp_dds::DDSNode
{
public:
  explicit MyNode(const rclcpp::NodeOptions & options)
  : DDSNode("my_node", options)
  {
    using std_msgs::msg::String;

    // Create a DataWriter for ROS topic "chatter" (i.e. DDS topic "rt/chatter").
    auto writer = this->create_datawriter<String>("chatter");

    // Create a DataWriter for ROS topic "chatter" with custom QoS.
    dds::pub::qos::DataWriterQos writer_qos;
    auto writer_custom_qos = this->create_datawriter<String>("chatter", writer_qos);

    // Create a DataWriter for a custom DDS topic "my_custom_topic".
    dds::topic::Topic topic(this->domain_participant(), "my_custom_topic");
    auto writer_custom_topic = this->create_datawriter<String>(topic);
  }
};
```

### DataReader Helpers

#### DDSNode::create_datareader()

*Example usage:*

```cpp
class MyNode : public rclcpp_dds::DDSNode
{
public:
  explicit MyNode(const rclcpp::NodeOptions & options)
  : DDSNode("my_node", options)
  {
    using std_msgs::msg::String;
    
    // Create a DataReader for ROS topic "chatter" (i.e. DDS topic "rt/chatter").
    auto reader = this->create_datareader<String>("chatter");

    // Create a DataReader for ROS topic "chatter" with custom QoS.
    dds::sub::qos::DataReaderQos reader_qos;
    auto reader_custom_qos = this->create_datareader<String>("chatter", reader_qos);

    // Create a DataReader for a custom DDS topic "my_custom_topic".
    dds::topic::Topic topic(this->domain_participant(), "my_custom_topic");
    auto writer_custom_topic = this->create_datawriter<String>(topic);
  }
};
```

### DomainParticipant Helpers

The Connext Node API uses the ROS 2 and Connext DDS APIs to provide easy
access to the DDS DomainParticipant used by a ROS 2 node.

This DomainParticipant is created by the ROS 2 middleware layer (RMW) whenever
a ROS 2 context is initialized by a ROS 2 application (e.g. by means of
`rclcpp::init()`).

The DomainParticipant is shared by all nodes associated with a context.

Applications built using the Connext Node API will rarely need to access the
underlying DomainParticipant directly, since all public functions take a node
reference and automatically retrieve it's associated DomainParticipant.

#### DDSNode::domain_id()

*Return the id of the DDS domain joined by a node.*

*Example usage:*

```cpp
#include "rclcpp_dds/rclcpp_dds.hpp"

void print_domain(rclcpp_dds::DDSNode & node) {
  std::cout << "Node " << node.get_name() << " is part of DDS domain " <<
    node.domain_id() << "\n";
}
```

#### DDSNode::domain_participant()

*Access the DDS DomainParticipant associated with a node.*

This function will return the first DomainParticipant created within the
application's DomainParticipantFactory on the DDS domain returned by
`DDSNode::domain_id()`.

**WARNING** It is possible for (advanced) applications to create multiple ROS 2
contexts, and thus multiple DDS DomainParticipants.

If multiple DomainParticipants on the same DDS domain are created by a single
process, Connext Node API will likely fail to return the correct DomainParticipant
for two nodes on the same domain but on different context.

*Example usage:*

```cpp
#include "rclcpp_dds/rclcpp_dds.hpp"

void print_domain(rclcpp_dds::DDSNode & node) {
  std::cout << "Node " << node.get_name() << " is part of DDS domain " <<
    node.domain_participant().domain_id() << "\n";
}
```

### Topic Helpers

#### DDSNode::create_topic()

*Example usage:*

```cpp
#include "rclcpp_dds/rclcpp_dds.hpp"
#include "std_msgs/msg/String.hpp"

void create_topic(rclcpp_dds::DDSNode & node)
{
  // Create a topic and retain it for later use
  auto topic = node.create_topic<std_msgs::msg::String>("my_topic");
  topic.retain();
}
```

#### DDSNode::lookup_topic()

*Example usage:*

```cpp
#include "ros2dds/ros2dds.hpp"
#include "std_msgs/msg/String.hpp"

bool has_topic(rclcpp_dds::DDSNode & node)
{
  // The function will return nullptr (or dds::core::nullptr)
  // if the topic doesn't exist.
  auto topic = node.lookup_topic<std_msgs::msg::String>("my_topic");
  return nullptr != topic;
}
```

#### DDSNode::create_content_filtered_topic()

*Example usage:*

```cpp
#include "rclcpp_dds/rclcpp_dds.hpp"
#include "std_msgs/msg/String.hpp"

void create_content_filtered_topic(rclcpp_dds::DDSNode & node)
{
  using std_msgs::msg::String;
  auto topic = node.create_content_filtered_topic<std_msgs::msg::String>(
    "my_topic", "my_custom_filter", "data LIKE 'Hello Connext!'");
  topic.retain();
}
```

### Callback Notification

#### DDSNode::set_data_callback()

#### DDSNode::cancel_data_callback()

#### DDSNode::set_status_callback()

#### DDSNode::cancel_status_callback()

#### DDSNode::add_user_callback()

#### DDSNode::cancel_user_callback()

## CMake Helpers

Once loaded in a `CMakeList.txt`, `connext_node_helpers` offers several CMake
functions that can be used to facilitate some common build task for
ROS 2/Connext applications.

### connext_generate_typesupport_library

*Generate type support code for multiple data types and build it into a single shared library.*

Function `connext_generate_typesupport_library` simplifies the process of
defining custom data types for a ROS 2/Connext application using standard
[OMG IDL](https://www.omg.org/spec/IDL) syntax.

`connext_generate_typesupport_library` takes a list of IDL files, and it
will generate type support code to use the data types they define with 
Connext DDS' [Modern C++ API](https://community.rti.com/static/documentation/connext-dds/6.1.0/doc/api/connext_dds/api_cpp2/index.html)
using [`rtiddsgen`](https://community.rti.com/static/documentation/connext-dds/6.1.0/doc/manuals/connext_dds_professional/code_generator/users_manual/index.htm).

The input IDL files can be specified in multiple ways:

- As ROS message identifiers of the form `<message-namespace>/<message-name>`, e.g. `std_msgs/String`. Use argument `MESSAGES`
  to specify IDL files with this syntax.

- As ROS service identifiers of the form `<service-namespace>/<service-name>`, e.g. `std_srvs/Trigger`. Use argument `SERVICES`
  to specify IDL files with this syntax.

- As IDL file paths with an optional, slash-separated, namespace, using syntax
  `<idl-file-path>[@<idl-namespace>]`. Use argument `IDLS` to specify IDL files
  with this syntax.

When IDL files are specified as ROS messages and/or services, the function will
automatically try to load the containing package using `find_package(REQUIRED)`.

Once the package has been loaded, the function will try to load the IDL files
that are automatically generated by ROS 2 for every custom interface. These
files must be installed under `<package-install-prefix>/share/<package>/msg` and
`<package-install-prefix>/share/<package>/srv`.

**WARNING** Not all of the IDL files that are automatically generated by ROS 2
can be correctly parsed by `rtiddsgen`. If you plan on using the "standard" ROS
types with native Connext DDS endpoints, consider using package [`connext_msgs`](https://github.com/asorbini/rticonnextdds-ros2-msgs)
which contains slightly modified (yet still interoperable) versions of these
types that can be successfully fed to `rtiddsgen`.

The function offers a few more options. Some of these are similar to the ones
accepted by [`connext_generate_message_typesupport_cpp`](#connext_generate_message_typesupport_cpp)
which this function uses to actually define a code generation target for each
input IDL.

- `DEPENDS` (List)
  - List of targets which will be added as a dependency to each code generation target.
- `EXPORT_TYPES_LIST` (String)
  - Name of a variable which will be set with the list of IDL files included
    in the library.
- `INCLUDES` (List)
  - List of directories which will be added to the include path of the generated library.
- `INSTALL_PREFIX` (String)
  - Installation prefix where the generated header files will be installed.
    Default: `include/`.
- `NO_DEFAULT_INCLUDES` (Boolean)
  - By default, the function will automatically add the directory of each input
    IDL to `rtiddsgen`'s include path, to enable `#include` directives that may
    be contained in the files. When a large number of files is provided, this
    may cause the resulting command line to be too large to be parsed by
    `rtiddsgen`, particularly if running in server mode.
  - If this variable is true, no additional directory will be automatically
    added to the include path.
- `SERVER` (Boolean)
  - Run `rtiddsgen` in server mode. This can be speed up code generation considerably,
    and it should always be used when processing a large number of input files.
- `WORKING_DIRECTORY` (String)
  - Directory from where to invoke `rtiddsgen`. Defaults to `${CMAKE_CURRENT_SOURCE_DIR}`.
- `ZEROCOPY` (Boolean)
  - Link the generated library with the Connext DDS libraries required to support
    Zero Copy Transfer Over Shared Memory.


*Example usage:*

```cmake
find_package(connext_node_helpers REQUIRED)

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
  ZEROCOPY
)
```

### connext_generate_message_typesupport_cpp

*Generates type support code with `rtiddsgen` from an input IDL file.*

`connext_generate_message_typesupport_cpp` is similar to
`connext_generate_typesupport_library`, but it provides lower-level functionality,
only to defines a code generation target, and not to actually build the
generated files.

The function returns a list of generated files  in an output variable named `<type_pkg>_<type>_FILES`.

It is up to the caller to consume this list appropriately, typically by
pasing it `add_executable()` or `add_library()`.

The compilation target will also need to be configured with the appropriate
include directories. By default, all files will be generated in
`${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen/<pkg>`, and it is sufficient to add
`${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen` to the include path (since all files
should always be included as `#include "<pkg>/<type>.idl"`).

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
  PACKAGE my/custom/ns
)

# Generated files will be available as `${SomeTypesWithoutNamespace_FILES}`
# The generated code must be included with `#include "SomeTypesWithoutNamespace.hpp"`.
connext_generate_message_typesupport_cpp(idl/SomeTypesWithoutNamespace.idl)

add_executable(my_app
  main.c
  ${std_msgs_String_FILES}
  ${my_custom_ns_MyType_FILES}
  ${SomeTypesWithoutNamespace_FILES}
)

target_include_directories(my_app
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen
)
```

### connext_components_register_node

*Generate an executable to "spin up" a ROS 2/Connext node.*

Function `connext_components_register_node` is similar to
`rclcpp_components_register_node`. The function takes a node from a component
library, and generates an executable to spin it in a `main()` function.

This function differs from `rclcpp_components_register_node` in that the
generate executable will link Connext DDS directly, and it will also be
properly linked in order to allow sharing of the DomainParticipantFactory
between the node component and the RMW layer.

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

*Build a ROS 2/DDS C++ application.*

This function can be used to simplify the definition of a build target for a
ROS 2 C++ application that requires direct access to RTI Connext DDS. The
function will take care of most "boiler plate" commands required to configure the
target with the required Connext DDS dependencies and the appropriate linkage
to share DDS entities with the RMW layer.

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

- [`rticonnextdds-ros2-msgs`](https://github.com/asorbini/rticonnextdds-ros2-msgs)
  - Helper library containing C++11 message type supports generated with
   `rtiddsgen` for almost every type include in ROS 2.
