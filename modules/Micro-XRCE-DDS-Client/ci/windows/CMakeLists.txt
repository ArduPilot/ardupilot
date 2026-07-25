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

cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)

project(microxrcedds_client_ci LANGUAGES C CXX)

include(ExternalProject)

ExternalProject_Add(microxrcedds_client
    SOURCE_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/../../
    BINARY_DIR
        ${PROJECT_BINARY_DIR}/microxrcedds_client-build
    INSTALL_DIR
        ${PROJECT_BINARY_DIR}/temp_install
    TEST_AFTER_INSTALL
        TRUE
    TEST_COMMAND
        COMMAND ${CMAKE_CTEST_COMMAND} -VV -C ${CMAKE_BUILD_TYPE} -T Test
    CMAKE_CACHE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_GENERATOR_TOOLSET:STRING=${CMAKE_GENERATOR_TOOLSET}
        -DCMAKE_GENERATOR_PLATFORM:STRING=${CMAKE_GENERATOR_PLATFORM}
        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
        -DUCLIENT_BUILD_CI_TESTS:BOOL=ON
        -DGTEST_INDIVIDUAL:BOOL=ON
    )