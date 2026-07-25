# Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

cmake_minimum_required(VERSION 2.8.12)
if (${CMAKE_VERSION} VERSION_GREATER 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(ShapeDemoClient C)

if(NOT UCLIENT_BUILD_EXAMPLES)
    find_package(microxrcedds_client REQUIRED)
endif()

set(BUILD_EXAMPLE TRUE)

if(WIN32 AND (NOT UCLIENT_PROFILE_UDP OR
 NOT UCLIENT_PROFILE_TCP))
    message(WARNING "Can not compile example: The UCLIENT_PROFILE_UDP and UCLIENT_PROFILE_TCP must be enabled.")
    set(BUILD_EXAMPLE FALSE)
elseif(UNIX AND (NOT UCLIENT_PROFILE_UDP OR
 NOT UCLIENT_PROFILE_TCP OR NOT UCLIENT_PROFILE_SERIAL))
    message(WARNING "Can not compile example: The UCLIENT_PROFILE_UDP, UCLIENT_PROFILE_TCP and UCLIENT_PROFILE_SERIAL must be enabled.")
    set(BUILD_EXAMPLE FALSE)
endif()

if(BUILD_EXAMPLE)
    add_executable(${PROJECT_NAME} main.c ShapeType.c)
    if(MSVC OR MSVC_IDE)
        target_compile_options(${PROJECT_NAME} PRIVATE /wd4996)
    endif()

    set_target_properties(${PROJECT_NAME} PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
        )

    target_link_libraries(${PROJECT_NAME}
        PRIVATE
        microxrcedds_client
        $<$<BOOL:${WIN32}>:ws2_32>
        $<$<C_COMPILER_ID:GNU>:-Wl,--gc-section,--no-export-dynamic>
        )

    if(UCLIENT_INSTALL_EXAMPLES)
        install(
            TARGETS
                ${PROJECT_NAME}
            RUNTIME DESTINATION
                ${BIN_INSTALL_DIR}
            )
    endif()
endif()
