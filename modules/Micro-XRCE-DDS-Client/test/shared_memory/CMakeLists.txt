# Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

project(uxr_sharedmem_unitary_tests LANGUAGES C CXX)

include(ExternalProject)

ExternalProject_Add(uxr_sharedmem_project
    SOURCE_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/../../
    BINARY_DIR
        ${PROJECT_BINARY_DIR}/microxrcedds_client_sharedmem
    INSTALL_DIR
        ${PROJECT_BINARY_DIR}/temp_install/sharedmem
    CMAKE_CACHE_ARGS
        -DUCLIENT_PROFILE_SHARED_MEMORY:BOOL=ON
        -DUCLIENT_PROFILE_MULTITHREAD:BOOL=${UCLIENT_PROFILE_MULTITHREAD}
        -DUCLIENT_PROFILE_MATCHING:BOOL=ON
        -DUCLIENT_SHARED_MEMORY_MAX_ENTITIES:STRING=30
        -DUCLIENT_SHARED_MEMORY_STATIC_MEM_SIZE:STRING=30
        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
        -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    BUILD_BYPRODUCTS ${PROJECT_BINARY_DIR}/temp_install/lib/sharedmem/libmicroxrcedds_client.a
    BUILD_BYPRODUCTS ${PROJECT_BINARY_DIR}/temp_install/lib/sharedmem/libmicrocdr.a
    )

ExternalProject_Get_Property(uxr_sharedmem_project INSTALL_DIR)

add_library(ucdr_sharedmem STATIC IMPORTED GLOBAL)
add_dependencies(ucdr_sharedmem uxr_sharedmem_project)
set_property(TARGET ucdr_sharedmem PROPERTY IMPORTED_LOCATION  ${PROJECT_BINARY_DIR}/temp_install/sharedmem/lib/libmicrocdr.a)

add_library(uxr_sharedmem STATIC IMPORTED GLOBAL)
add_dependencies(uxr_sharedmem uxr_sharedmem_project)
target_link_libraries(uxr_sharedmem INTERFACE ucdr_sharedmem)
set_property(TARGET uxr_sharedmem PROPERTY IMPORTED_LOCATION  ${PROJECT_BINARY_DIR}/temp_install/sharedmem/lib/libmicroxrcedds_client.a)

add_executable(sharedmem_test SharedMemory.cpp)

set_common_compile_options(sharedmem_test)

if(MSVC OR MSVC_IDE)
    target_compile_options(sharedmem_test PRIVATE /wd4996)
endif()

add_gtest(sharedmem_test SOURCES SharedMemory.cpp)

target_link_libraries(sharedmem_test
    PUBLIC 
        ucdr_sharedmem
        uxr_sharedmem
    PRIVATE
        ${GTEST_BOTH_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
    )

target_include_directories(sharedmem_test
    PUBLIC
        ${PROJECT_BINARY_DIR}/temp_install/sharedmem/include
    PRIVATE
        ${GTEST_INCLUDE_DIRS}
    )

set_target_properties(sharedmem_test PROPERTIES
    CXX_STANDARD
        11
    CXX_STANDARD_REQUIRED
        YES
    )
