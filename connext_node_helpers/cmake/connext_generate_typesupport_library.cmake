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
# Helper function to generate a shared library containing DDS type support code
# from a list of ROS 2 messages and (soon) services.
################################################################################
function(connext_generate_typesupport_library lib)
  cmake_parse_arguments(_tslib
    "ZEROCOPY;SERVER" # boolean arguments
    "INSTALL_PREFIX;WORKING_DIRECTORY;EXPORT_TYPES_LIST" # single value arguments
    "MESSAGES;SERVICES;DEPENDS;IDLS;INCLUDES" # multi-value arguments
    ${ARGN} # current function arguments
    )
  
  if("${_tslib_INSTALL_PREFIX}" STREQUAL "")
    set(_tslib_INSTALL_PREFIX include)
  endif()
  
  # Collect list of all included packages
  set(_tslib_PACKAGES)
  foreach(_msg ${_tslib_MESSAGES} ${_tslib_SERVICES})
    string(REGEX REPLACE "/[^/]*$" "" _msg_pkg "${_msg}")
    if(NOT _msg_pkg)
      message(FATAL_ERROR "Invalid package detected for message "
        "'${__msg}': '$(_msg_pkg)'")
    endif()
    list(APPEND _tslib_PACKAGES "${_msg_pkg}")
    find_package(${_msg_pkg} REQUIRED)
  endforeach()

  set(_tslib_OUTPUT_DIR  "${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen/${lib}")

  if(_tslib_SERVER)
    set(_tslib_SERVER_OPT SERVER)
  endif()

  set(_tslib_TYPES)

  # Generate type supports for all types
  set(_tslib_GENERATED_FILES)
  foreach(_msg ${_tslib_MESSAGES})
    string(REGEX REPLACE "/[^/]*$" "" _msg_pkg "${_msg}")
    string(REGEX REPLACE "^${_msg_pkg}/" "" _msg_name "${_msg}")
    set(_msg_includes ${_tslib_MESSAGES})
    list(REMOVE_ITEM _msg_includes "${_msg_pkg}")
    string(REGEX REPLACE "/" "_" _msg_tgt "${_msg}")
    
    connext_generate_message_typesupport_cpp(${_msg_name}
      PACKAGE ${_msg_pkg}
      INCLUDES ${_msg_includes} ${_tslib_INCLUDES}
      OUTPUT_DIR ${_tslib_OUTPUT_DIR}
      INSTALL_PREFIX ${_tslib_INSTALL_PREFIX}
      DEPENDS ${_tslib_DEPENDS}
      TARGET rtiddsgen_${_msg_tgt}
      WORKING_DIRECTORY ${_tslib_WORKING_DIRECTORY}
      ${_tslib_SERVER_OPT})
    list(APPEND _tslib_GENERATED_FILES ${${_msg_pkg}_${_msg_name}_FILES})

    list(APPEND _tslib_TYPES "${_msg}")
  endforeach()


  # Generate a list of include paths based on the input IDL files
  set(_idl_includes)
  foreach(_idl_ENTRY ${_tslib_IDLS})
    string(REGEX REPLACE "@.*$" "" _idl_PATH "${_idl_ENTRY}")
    string(REGEX REPLACE "^${_idl_PATH}" "" _idl_PREFIX "${_idl_ENTRY}")
    if(NOT "${_idl_PREFIX}" STREQUAL "")
      string(REGEX REPLACE "^@" "" _idl_PREFIX "${_idl_PREFIX}")
    endif()

    if(NOT "${_id_PREFIX}" STREQUAL "")
      get_filename_component(_idl_file "${_idl_PATH}" NAME)
      string(REGEX REPLACE
        "${_idl_PREFIX}/${_idl_file}$" "" _idl_inc_dir "${idl_PATH}")
    else()
      get_filename_component(_idl_inc_dir "${_idl_PATH}" DIRECTORY)
    endif()

    get_filename_component(_idl_inc_dir "${_idl_inc_dir}" REALPATH)
    list(APPEND _idl_includes "${_idl_inc_dir}")
  endforeach()
  list(REMOVE_DUPLICATES _idl_includes)
  
  # Generate type support for all IDL Files
  # These are passed in the form <idl-file>[@<optional-include-prefix>]
  foreach(_idl_ENTRY ${_tslib_IDLS})
    string(REGEX REPLACE "@.*$" "" _idl_PATH "${_idl_ENTRY}")
    string(REGEX REPLACE "^${_idl_PATH}" "" _idl_PREFIX "${_idl_ENTRY}")
    get_filename_component(_idl_file "${_idl_PATH}" NAME)
    if(_idl_PREFIX)
      string(REGEX REPLACE "^@" "" _idl_PREFIX "${_idl_PREFIX}")
      string(REGEX REPLACE "[/.]" "_" _idl_tgt "rtiddsgen_${_idl_PREFIX}/${_idl_file}")
    else()
      string(REGEX REPLACE "[/.]" "_" _idl_tgt "rtiddsgen_${_idl_file}")
    endif()
    string(REGEX REPLACE "[.]idl$" "" _idl_type "${_idl_file}")

    connext_generate_message_typesupport_cpp(${_idl_PATH}
      PACKAGE ${_idl_PREFIX}
      INCLUDES ${_idl_includes} ${_tslib_INCLUDES}
      OUTPUT_DIR ${_tslib_OUTPUT_DIR}
      INSTALL_PREFIX ${_tslib_INSTALL_PREFIX}
      DEPENDS ${_tslib_DEPENDS}
      TARGET ${_idl_tgt}
      WORKING_DIRECTORY ${_tslib_WORKING_DIRECTORY}
      ${_tslib_SERVER_OPT})
    
    if(NOT "${_idl_PREFIX}" STREQUAL "")
      string(REPLACE "/" "_" _idl_prefix "${_idl_PREFIX}")
      set(_outvar "${_idl_prefix}_${_idl_type}_FILES")
    else()
      set(_outvar "${_idl_type}_FILES")
    endif()

    list(APPEND _tslib_GENERATED_FILES ${${_outvar}})

    if(_idl_PREFIX)
      list(APPEND _tslib_TYPES "${_idl_PREFIX}/${_idl_type}")
    else()
      list(APPEND _tslib_TYPES "${_idl_type}")
    endif()
  endforeach()

  # Define library target to build all generated files into shared library
  add_library(${lib} SHARED
    ${_tslib_GENERATED_FILES})
  target_link_libraries(${lib} RTIConnextDDS::cpp2_api)
  add_dependencies(${lib} rtiddsgen)
  if(_tslib_ZEROCOPY)
    target_link_libraries(${lib} RTIConnextDDS::metp)
  endif()
  target_include_directories(${lib}
    PUBLIC
      "$<BUILD_INTERFACE:${_tslib_OUTPUT_DIR}>"
      "$<INSTALL_INTERFACE:include>")
  install(
    TARGETS ${lib}
    EXPORT idl_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  
  if(_tslib_EXPORT_TYPES_LIST)
    set(${_tslib_EXPORT_TYPES_LIST} ${_tslib_TYPES} PARENT_SCOPE)
  endif()
endfunction()
