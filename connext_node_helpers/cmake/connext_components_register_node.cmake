# (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.

################################################################################
# Similar to `rclcpp_components_register_node()` but generates a main() which
# links Connext DDS directly, and which is linked with `-rdynamic` to enable
# sharing of the DomainParticipantFactory and other entities with the RMW.
################################################################################
function(connext_components_register_node target)
  cmake_parse_arguments(_component
    "" # boolean arguments
    "PLUGIN;EXECUTABLE;RESOURCE_INDEX" # single value arguments
    "" # multi-value arguments
    ${ARGN} # current function arguments
    )

  # use a custom template to generate each node's main, because we must make
  # sure that the DomainParticipantFactory's global static variable is linked
  # into the process. We do this by calling some DDS API that accesses it.
  set(rclcpp_components_NODE_TEMPLATE ${connext_node_helpers_NODE_TEMPLATE})

  rclcpp_components_register_node(${target}
    PLUGIN ${_component_PLUGIN}
    EXECUTABLE ${_component_EXECUTABLE}
    RESOURCE_INDEX ${_component_RESOURCE_INDEX})

  set_target_properties(${_component_EXECUTABLE} PROPERTIES ENABLE_EXPORTS true)
  target_link_libraries(${_component_EXECUTABLE} RTIConnextDDS::cpp2_api)
endfunction()
