# Copyright 2021 Real-Time Innovations, Inc.  All rights reserved.
#
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software.  Licensee has the right to distribute object form
# only for use with RTI products.  The Software is provided "as is", with no
# warranty of any type, including any warranty for fitness for any purpose.
# RTI is under no obligation to maintain or support the Software.  RTI shall
# not be liable for any incidental or consequential damages arising out of the
# use or inability to use the software.

################################################################################
# Helper function to build simple C++ DDS/ROS 2 applications
################################################################################
function(connext_add_executable)
  cmake_parse_arguments(_exec
    "STANDALONE;ZEROCOPY" # boolean arguments
    "NAME" # single value arguments
    "SOURCES;LIBRARIES;DEFINES;PKG_DEPS;INCLUDES" # multi-value arguments
    ${ARGN} # current function arguments
  )

  add_executable(${_exec_NAME} ${_exec_SOURCES})
  # Link RTI Connext DDS' "modern C++" API
  target_link_libraries(${_exec_NAME}
    RTIConnextDDS::cpp2_api ${_exec_LIBRARIES})
  if(_exec_ZEROCOPY)
    target_link_libraries(${_exec_NAME} RTIConnextDDS::metp)
  endif()
  # Set property ENABLE_EXPORTS to link the library with `-rdynamic` and
  # enable sharing of static variables with dynamic libraries.
  set_target_properties(${_exec_NAME} PROPERTIES ENABLE_EXPORTS true)
  target_include_directories(${_exec_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${_exec_INCLUDES})
  if(_exec_DEFINES)
    target_compile_definitions(${_exec_NAME} PRIVATE ${_exec_DEFINES})
  endif()
  install(
    TARGETS ${_exec_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  if(NOT STANDALONE)
    ament_target_dependencies(${_exec_NAME}
      rclcpp connext_node_helpers ${_exec_PKG_DEPS})
  endif()
endfunction()
