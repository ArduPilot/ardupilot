# Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

include(ExternalProject)

unset(_deps)

enable_language(C)
enable_language(CXX)

if(UCDR_BUILD_TESTS)
    unset(googletest_DIR CACHE)
    enable_language(CXX)
    find_package(GTest QUIET)
    find_package(GMock QUIET)
    if(NOT GTest_FOUND OR NOT GMock_FOUND)
        unset(GTEST_ROOT CACHE)
        unset(GMOCK_ROOT CACHE)
        ExternalProject_Add(googletest
            GIT_REPOSITORY
                https://github.com/google/googletest.git
            GIT_TAG
                release-1.11.0
            PREFIX
                ${PROJECT_BINARY_DIR}/googletest
            INSTALL_DIR
                ${PROJECT_BINARY_DIR}/temp_install/googletest
            CMAKE_ARGS
                -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
                $<$<PLATFORM_ID:Windows>:-Dgtest_force_shared_crt:BOOL=ON>
            BUILD_COMMAND
                COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release --target install
                COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Debug --target install
            INSTALL_COMMAND
                ""
            )
        set(GTEST_ROOT ${PROJECT_BINARY_DIR}/temp_install/googletest CACHE PATH "" FORCE)
        set(GMOCK_ROOT ${PROJECT_BINARY_DIR}/temp_install/googletest CACHE PATH "" FORCE)
        list(APPEND _deps googletest)
    endif()

    unset(fastcdr_DIR CACHE)
    enable_language(CXX)
    unset(FASTCDR_ROOT CACHE)
    ExternalProject_Add(fastcdr
        GIT_REPOSITORY
            https://github.com/eProsima/Fast-CDR
        GIT_TAG
        v1.0.13
        PREFIX
            ${PROJECT_BINARY_DIR}/fastcdr
        INSTALL_DIR
            ${PROJECT_BINARY_DIR}/temp_install/fastcdr
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DCMAKE_BUILD_TYPE=${DCMAKE_BUILD_TYPE}
        BUILD_COMMAND
            COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release --target install
            COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Debug --target install
        INSTALL_COMMAND
            ""
        )
    set(FASTCDR_ROOT ${PROJECT_BINARY_DIR}/temp_install/fastcdr CACHE PATH "" FORCE)
    set(FASTCDR_INCLUDE_DIRS ${PROJECT_BINARY_DIR}/temp_install/fastcdr/include CACHE PATH "" FORCE)

    list(APPEND _deps fastcdr)
endif()

# Client project.
ExternalProject_Add(ucdr
    SOURCE_DIR
        ${PROJECT_SOURCE_DIR}
    BINARY_DIR
        ${CMAKE_CURRENT_BINARY_DIR}
    CMAKE_CACHE_ARGS
        -DUCDR_SUPERBUILD:BOOL=OFF
    INSTALL_COMMAND
        ""
    DEPENDS
        ${_deps}
    )
