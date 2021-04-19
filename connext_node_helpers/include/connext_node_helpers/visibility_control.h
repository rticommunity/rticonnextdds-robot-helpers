// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#ifndef CONNEXT_NODE_HELPERS__VISIBILITY_CONTROL_H_
#define CONNEXT_NODE_HELPERS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONNEXT_NODE_HELPERS_EXPORT __attribute__ ((dllexport))
    #define CONNEXT_NODE_HELPERS_IMPORT __attribute__ ((dllimport))
  #else
    #define CONNEXT_NODE_HELPERS_EXPORT __declspec(dllexport)
    #define CONNEXT_NODE_HELPERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONNEXT_NODE_HELPERS_BUILDING_DLL
    #define CONNEXT_NODE_HELPERS_PUBLIC CONNEXT_NODE_HELPERS_EXPORT
  #else
    #define CONNEXT_NODE_HELPERS_PUBLIC CONNEXT_NODE_HELPERS_IMPORT
  #endif
  #define CONNEXT_NODE_HELPERS_PUBLIC_TYPE CONNEXT_NODE_HELPERS_PUBLIC
  #define CONNEXT_NODE_HELPERS_LOCAL
#else
  #define CONNEXT_NODE_HELPERS_EXPORT __attribute__ ((visibility("default")))
  #define CONNEXT_NODE_HELPERS_IMPORT
  #if __GNUC__ >= 4
    #define CONNEXT_NODE_HELPERS_PUBLIC __attribute__ ((visibility("default")))
    #define CONNEXT_NODE_HELPERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONNEXT_NODE_HELPERS_PUBLIC
    #define CONNEXT_NODE_HELPERS_LOCAL
  #endif
  #define CONNEXT_NODE_HELPERS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CONNEXT_NODE_HELPERS__VISIBILITY_CONTROL_H_
