###############################################################################
#
# Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
#
###############################################################################

cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

# New project because of the CXX GTest dependency.
if(CMAKE_VERSION VERSION_LESS 3.0)
    project(serial_comm_transport_test C CXX)
else()
    cmake_policy(SET CMP0048 NEW)
    project(serial_comm_transport_test LANGUAGES C CXX)
endif()

if(NOT UCLIENT_PROFILE_SERIAL)
    message(WARNING "Can not compile test: The UCLIENT_PROFILE_SERIAL must be enabled.")
else()

    set(SRCS SerialComm.cpp)

    add_executable(${PROJECT_NAME} ${SRCS})

    set_common_compile_options(${PROJECT_NAME})

    if(MSVC OR MSVC_IDE)
        target_compile_options(${PROJECT_NAME} PRIVATE /wd4996)
    endif()

    add_gtest(${PROJECT_NAME}
        SOURCES
            ${SRCS}
        DEPENDENCIES
            microxrcedds_client
        )

    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            microxrcedds_client
            ${GTEST_BOTH_LIBRARIES}
            ${CMAKE_THREAD_LIBS_INIT}
        )

    target_include_directories(${PROJECT_NAME}
        PUBLIC
            ${PROJECT_SOURCE_DIR}
        PRIVATE
            ${GTEST_INCLUDE_DIRS}
        )

    set_target_properties(${PROJECT_NAME} PROPERTIES
        CXX_STANDARD
            11
        CXX_STANDARD_REQUIRED
            YES
        )
endif()
