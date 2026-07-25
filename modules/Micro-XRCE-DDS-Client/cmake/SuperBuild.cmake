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

# Micro CDR.
unset(microcdr_DIR CACHE)

if(NOT UCLIENT_BUILD_MICROCDR)
    find_package(microcdr ${_microcdr_version} EXACT QUIET)
endif()

if(NOT microcdr_FOUND)
    ExternalProject_Add(microcdr
        GIT_REPOSITORY
            https://github.com/eProsima/Micro-CDR.git
        GIT_TAG
            ${_microcdr_tag}
        PREFIX
            ${PROJECT_BINARY_DIR}/microcdr
        INSTALL_DIR
            ${PROJECT_BINARY_DIR}/temp_install
        CMAKE_CACHE_ARGS
            -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
            -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
        CMAKE_ARGS
            -DBUILD_SHARED_LIBS=${BUILD_SHARED_LIBS}
            -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
            -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
            -DCMAKE_SYSROOT:PATH=${CMAKE_SYSROOT}
            -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
            -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
            -DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
            -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
            -DCONFIG_BIG_ENDIANNESS=${UCLIENT_BIG_ENDIANNESS}
            -DUCDR_PIC=${UCLIENT_PIC}
        )
    list(APPEND _deps microcdr)
endif()

if(UCLIENT_BUILD_TESTS)
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
endif()

# Client project.
ExternalProject_Add(uclient
    SOURCE_DIR
        ${PROJECT_SOURCE_DIR}
    BINARY_DIR
        ${CMAKE_CURRENT_BINARY_DIR}
    CMAKE_CACHE_ARGS
        -DUCLIENT_SUPERBUILD:BOOL=OFF
    INSTALL_COMMAND
        ""
    DEPENDS
        ${_deps}
    )
