# Locate the Google C++ Mocking Framework.
# (This file is almost an identical copy of the original FindGTest.cmake file,
#  feel free to use it as it is or modify it for your own needs.)
#
#
# Defines the following variables:
#
#   GMOCK_FOUND - Found the Google Testing framework
#   GMOCK_INCLUDE_DIRS - Include directories
#
# Also defines the library variables below as normal
# variables. These contain debug/optimized keywords when
# a debugging library is found.
#
#   GMOCK_BOTH_LIBRARIES - Both libgmock & libgmock-main
#   GMOCK_LIBRARIES - libgmock
#   GMOCK_MAIN_LIBRARIES - libgmock-main
#
# Accepts the following variables as input:
#
#   GMOCK_ROOT - (as a CMake or environment variable)
#                The root directory of the gmock install prefix
#
#   GMOCK_MSVC_SEARCH - If compiling with MSVC, this variable can be set to
#                       "MD" or "MT" to enable searching a gmock build tree
#                       (defaults: "MD")
#
#-----------------------
# Example Usage:
#
#    find_package(GMock REQUIRED)
#    include_directories(${GMOCK_INCLUDE_DIRS})
#
#    add_executable(foo foo.cc)
#    target_link_libraries(foo ${GMOCK_BOTH_LIBRARIES})
#
#=============================================================================
# This file is released under the MIT licence:
#
# Copyright (c) 2011 Matej Svec
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
#=============================================================================


function(_gmock_append_debugs _endvar _library)
  if(${_library} AND ${_library}_DEBUG)
    set(_output optimized ${${_library}} debug ${${_library}_DEBUG})
  else()
    set(_output ${${_library}})
  endif()
  set(${_endvar} ${_output} PARENT_SCOPE)
endfunction()

function(_gmock_find_library _name)
  find_library(${_name}
    NAMES ${ARGN}
    HINTS
      $ENV{GMOCK_ROOT}
      ${GMOCK_ROOT}
    PATH_SUFFIXES ${_gmock_libpath_suffixes}
  )
  mark_as_advanced(${_name})
endfunction()


if(NOT DEFINED GMOCK_MSVC_SEARCH)
  set(GMOCK_MSVC_SEARCH MD)
endif()

set(_gmock_libpath_suffixes lib)
if(MSVC)
  if(GMOCK_MSVC_SEARCH STREQUAL "MD")
    list(APPEND _gmock_libpath_suffixes
      msvc/gmock-md/Debug
      msvc/gmock-md/Release)
  elseif(GMOCK_MSVC_SEARCH STREQUAL "MT")
    list(APPEND _gmock_libpath_suffixes
      msvc/gmock/Debug
      msvc/gmock/Release)
  endif()
endif()

find_path(GMOCK_INCLUDE_DIR gmock/gmock.h
  HINTS
    $ENV{GMOCK_ROOT}/include
    ${GMOCK_ROOT}/include
)
mark_as_advanced(GMOCK_INCLUDE_DIR)

if(MSVC AND GMOCK_MSVC_SEARCH STREQUAL "MD")
  # The provided /MD project files for Google Mock add -md suffixes to the
  # library names.
  _gmock_find_library(GMOCK_LIBRARY            gmock-md  gmock)
  _gmock_find_library(GMOCK_LIBRARY_DEBUG      gmock-mdd gmockd)
  _gmock_find_library(GMOCK_MAIN_LIBRARY       gmock_main-md  gmock_main)
  _gmock_find_library(GMOCK_MAIN_LIBRARY_DEBUG gmock_main-mdd gmock_maind)
else()
  _gmock_find_library(GMOCK_LIBRARY            gmock)
  _gmock_find_library(GMOCK_LIBRARY_DEBUG      gmockd)
  _gmock_find_library(GMOCK_MAIN_LIBRARY       gmock_main)
  _gmock_find_library(GMOCK_MAIN_LIBRARY_DEBUG gmock_maind)
endif()

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GMock DEFAULT_MSG GMOCK_LIBRARY GMOCK_INCLUDE_DIR GMOCK_MAIN_LIBRARY)

if(GMOCK_FOUND)
  set(GMOCK_INCLUDE_DIRS ${GMOCK_INCLUDE_DIR})
  _gmock_append_debugs(GMOCK_LIBRARIES      GMOCK_LIBRARY)
  _gmock_append_debugs(GMOCK_MAIN_LIBRARIES GMOCK_MAIN_LIBRARY)
  set(GMOCK_BOTH_LIBRARIES ${GMOCK_LIBRARIES} ${GMOCK_MAIN_LIBRARIES})
endif()

