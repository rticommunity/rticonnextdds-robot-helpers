# Copyright 2021 Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.

# This file can be used to load the helpers provided by this package from a
# non-ROS CMake file.


if(NOT DEFINED CONNEXT_NODE_HELPERS_DIR)
  get_filename_component(CONNEXT_NODE_HELPERS_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../" REALPATH)
endif()

include("${CONNEXT_NODE_HELPERS_DIR}/cmake/connext_components_register_node.cmake")
include("${CONNEXT_NODE_HELPERS_DIR}/cmake/connext_add_executable.cmake")
include("${CONNEXT_NODE_HELPERS_DIR}/cmake/connext_generate_message_typesupport_cpp.cmake")
include("${CONNEXT_NODE_HELPERS_DIR}/cmake/connext_generate_typesupport_library.cmake")

if(NOT DEFINED CONNEXTDDS_DIR)
  if(ENV{CONNEXTDDS_DIR})
    file(TO_CMAKE_PATH "$ENV{CONNEXTDDS_DIR}" CONNEXTDDS_DIR)
  elseif(NOT NDDSHOME)
    file(TO_CMAKE_PATH "$ENV{NDDSHOME}" CONNEXTDDS_DIR)
  endif()
endif()

if(NOT CONNEXTDDS_DIR)
  message(FATAL_ERROR "RTI Connext DDS must be available to use this package. "
    "Please set NDDSHOME or CONNEXTDDS_DIR.")
endif()

if(NOT CONNEXTDDS_ARCH AND ENV{CONNEXTDDS_ARCH})
  set(CONNEXTDDS_ARCH "$ENV{CONNEXTDDS_ARCH}")
endif()

# Instead of using the default `FindRTIConnextDDS.cmake` shipped with Connext
# use the one included in `connext_node_helpers` which is up to date with the
# latest version from `rticonnextdds-examples`.
list(INSERT CMAKE_MODULE_PATH 0 "${CONNEXT_NODE_HELPERS_DIR}/cmake")

# Determine list of optional Connext components to load.
# Auto-loading of components may be overridden by specifying variable
# CONNEXTDDS_COMPONENTS before loading `connext_node_helpers`.
if("${CONNEXTDDS_COMPONENTS}" STREQUAL "")
  set(CONNEXTDDS_COMPONENTS
    metp
    messaging_api
  )
endif()

set(CONNEXTDDS_COMPONENTS ${CONNEXTDDS_COMPONENTS}
  CACHE INTERNAL "List of optional RTI Connext DDS components to load" FORCE)

message(STATUS
  "loading RTI Connext DDS with components: ${CONNEXTDDS_COMPONENTS}")

find_package(RTIConnextDDS REQUIRED COMPONENTS ${CONNEXTDDS_COMPONENTS})
if(NOT RTIConnextDDS_FOUND)
  message(FATAL_ERROR "RTI Connext DDS (${CONNEXTDDS_ARCH}) not found in "
    "'${CONNEXTDDS_DIR}'")
elseif("${RTICONNEXTDDS_VERSION}" VERSION_LESS_EQUAL "6.0.0")
  message(FATAL_ERROR "RTI Connext DDS 6.x required. "
    "Found: ${RTICONNEXTDDS_VERSION}")
  set(RTIConnextDDS_FOUND false)
endif()

set(connext_node_helpers_NODE_TEMPLATE
  "${CONNEXT_NODE_HELPERS_DIR}/cmake/node_main.cpp.in")
