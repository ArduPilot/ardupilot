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

cmake_minimum_required(VERSION 2.8.12)
if (${CMAKE_VERSION} VERSION_GREATER 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(SubscribeHelloWorldClientP2P C)

if(NOT UCLIENT_BUILD_EXAMPLES)
    find_package(microxrcedds_client REQUIRED)
endif()

if(NOT UCLIENT_PROFILE_UDP)
    message(WARNING "Can not compile example: The UCLIENT_PROFILE_UDP must be enabled.")
else()
    add_executable(${PROJECT_NAME} main.c HelloWorld.c)
    if(MSVC OR MSVC_IDE)
        target_compile_options(${PROJECT_NAME} PRIVATE /wd4996)
    endif()

    set_target_properties(${PROJECT_NAME} PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
        )


    target_link_libraries(${PROJECT_NAME} microxrcedds_client $<$<C_COMPILER_ID:GNU>:-Wl,--gc-section,--no-export-dynamic>)

    if(UCLIENT_INSTALL_EXAMPLES)
        install(
            TARGETS
                ${PROJECT_NAME}
            RUNTIME DESTINATION
                ${BIN_INSTALL_DIR}
            )
    endif()

endif()
