# This is a copy of <PICO_SDK_PATH>/external/pico_sdk_import.cmake

# This can be dropped into an external project to help locate this SDK
# It should be include()ed prior to project()

# Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
# disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    message("Using PICO_SDK_PATH from environment ('${PICO_SDK_PATH}')")
endif ()

if (DEFINED ENV{PICO_SDK_FETCH_FROM_GIT} AND (NOT PICO_SDK_FETCH_FROM_GIT))
    set(PICO_SDK_FETCH_FROM_GIT $ENV{PICO_SDK_FETCH_FROM_GIT})
    message("Using PICO_SDK_FETCH_FROM_GIT from environment ('${PICO_SDK_FETCH_FROM_GIT}')")
endif ()

if (DEFINED ENV{PICO_SDK_FETCH_FROM_GIT_PATH} AND (NOT PICO_SDK_FETCH_FROM_GIT_PATH))
    set(PICO_SDK_FETCH_FROM_GIT_PATH $ENV{PICO_SDK_FETCH_FROM_GIT_PATH})
    message("Using PICO_SDK_FETCH_FROM_GIT_PATH from environment ('${PICO_SDK_FETCH_FROM_GIT_PATH}')")
endif ()

if (DEFINED ENV{PICO_SDK_FETCH_FROM_GIT_TAG} AND (NOT PICO_SDK_FETCH_FROM_GIT_TAG))
    set(PICO_SDK_FETCH_FROM_GIT_TAG $ENV{PICO_SDK_FETCH_FROM_GIT_TAG})
    message("Using PICO_SDK_FETCH_FROM_GIT_TAG from environment ('${PICO_SDK_FETCH_FROM_GIT_TAG}')")
endif ()

if (PICO_SDK_FETCH_FROM_GIT AND NOT PICO_SDK_FETCH_FROM_GIT_TAG)
  set(PICO_SDK_FETCH_FROM_GIT_TAG "master")
  message("Using master as default value for PICO_SDK_FETCH_FROM_GIT_TAG")
endif()

set(PICO_SDK_PATH "${PICO_SDK_PATH}" CACHE PATH "Path to the Raspberry Pi Pico SDK")
set(PICO_SDK_FETCH_FROM_GIT "${PICO_SDK_FETCH_FROM_GIT}" CACHE BOOL "Set to ON to fetch copy of SDK from git if not otherwise locatable")
set(PICO_SDK_FETCH_FROM_GIT_PATH "${PICO_SDK_FETCH_FROM_GIT_PATH}" CACHE FILEPATH "location to download SDK")
set(PICO_SDK_FETCH_FROM_GIT_TAG "${PICO_SDK_FETCH_FROM_GIT_TAG}" CACHE FILEPATH "release tag for SDK")

if (NOT PICO_SDK_PATH)
    if (PICO_SDK_FETCH_FROM_GIT)
        include(FetchContent)
        set(FETCHCONTENT_BASE_DIR_SAVE ${FETCHCONTENT_BASE_DIR})
        if (PICO_SDK_FETCH_FROM_GIT_PATH)
            get_filename_component(FETCHCONTENT_BASE_DIR "${PICO_SDK_FETCH_FROM_GIT_PATH}" REALPATH BASE_DIR "${CMAKE_SOURCE_DIR}")
        endif ()
        FetchContent_Declare(
                pico_sdk
                GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk
                GIT_TAG ${PICO_SDK_FETCH_FROM_GIT_TAG}
        )

        if (NOT pico_sdk)
            message("Downloading Raspberry Pi Pico SDK")
            # GIT_SUBMODULES_RECURSE was added in 3.17
            if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.17.0")
                FetchContent_Populate(
                        pico_sdk
                        QUIET
                        GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk
                        GIT_TAG ${PICO_SDK_FETCH_FROM_GIT_TAG}
                        GIT_SUBMODULES_RECURSE FALSE

                        SOURCE_DIR ${FETCHCONTENT_BASE_DIR}/pico_sdk-src
                        BINARY_DIR ${FETCHCONTENT_BASE_DIR}/pico_sdk-build
                        SUBBUILD_DIR ${FETCHCONTENT_BASE_DIR}/pico_sdk-subbuild
                )
            else ()
                FetchContent_Populate(
                        pico_sdk
                        QUIET
                        GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk
                        GIT_TAG ${PICO_SDK_FETCH_FROM_GIT_TAG}

                        SOURCE_DIR ${FETCHCONTENT_BASE_DIR}/pico_sdk-src
                        BINARY_DIR ${FETCHCONTENT_BASE_DIR}/pico_sdk-build
                        SUBBUILD_DIR ${FETCHCONTENT_BASE_DIR}/pico_sdk-subbuild
                )
            endif ()

            set(PICO_SDK_PATH ${pico_sdk_SOURCE_DIR})
        endif ()
        set(FETCHCONTENT_BASE_DIR ${FETCHCONTENT_BASE_DIR_SAVE})
    else ()
        message(FATAL_ERROR
                "SDK location was not specified. Please set PICO_SDK_PATH or set PICO_SDK_FETCH_FROM_GIT to on to fetch from git."
                )
    endif ()
endif ()

get_filename_component(PICO_SDK_PATH "${PICO_SDK_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
if (NOT EXISTS ${PICO_SDK_PATH})
    message(FATAL_ERROR "Directory '${PICO_SDK_PATH}' not found")
endif ()

set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake)
if (NOT EXISTS ${PICO_SDK_INIT_CMAKE_FILE})
    message(FATAL_ERROR "Directory '${PICO_SDK_PATH}' does not appear to contain the Raspberry Pi Pico SDK")
endif ()

set(PICO_SDK_PATH ${PICO_SDK_PATH} CACHE PATH "Path to the Raspberry Pi Pico SDK" FORCE)

include(${PICO_SDK_INIT_CMAKE_FILE})
