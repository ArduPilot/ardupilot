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


if(NOT UCLIENT_PROFILE_UDP)
    message(WARNING "Can not compile example: The UCLIENT_PROFILE_UDP must be enabled.")
else()

    #############################################################
    ###                     CONFIGURATOR
    #############################################################
    project(ConfiguratorClient C)

    if(NOT UCLIENT_BUILD_EXAMPLES)
        find_package(microxrcedds_client REQUIRED)
    endif()

    add_executable(ConfiguratorClient configurator_client.c)
    if(MSVC OR MSVC_IDE)
        target_compile_options(${PROJECT_NAME} PRIVATE /wd4996)
    endif()

    target_link_libraries(ConfiguratorClient microxrcedds_client $<$<C_COMPILER_ID:GNU>:-Wl,--gc-section,--no-export-dynamic>)

    if(UCLIENT_INSTALL_EXAMPLES)
        install(
            TARGETS
                ${PROJECT_NAME}
            RUNTIME DESTINATION
                ${BIN_INSTALL_DIR}
            )
    endif()

    #############################################################
    ###                      PUBLISHER
    #############################################################
    project(PublisherClient C)
    add_executable(PublisherClient publisher.c HelloWorld.c)
    if(MSVC OR MSVC_IDE)
        target_compile_options(${PROJECT_NAME} PRIVATE /wd4996)
    endif()

    target_link_libraries(PublisherClient microxrcedds_client $<$<C_COMPILER_ID:GNU>:-Wl,--gc-section,--no-export-dynamic>)

    if(UCLIENT_INSTALL_EXAMPLES)
        install(
            TARGETS
                ${PROJECT_NAME}
            RUNTIME DESTINATION
                ${BIN_INSTALL_DIR}
            )
    endif()

    #############################################################
    ###                      SUBSCRIBER
    #############################################################
    project(SubscriberClient C)
    add_executable(SubscriberClient subscriber.c HelloWorld.c)
    if(MSVC OR MSVC_IDE)
        target_compile_options(${PROJECT_NAME} PRIVATE /wd4996)
    endif()

    target_link_libraries(SubscriberClient microxrcedds_client $<$<C_COMPILER_ID:GNU>:-Wl,--gc-section,--no-export-dynamic>)

    if(UCLIENT_INSTALL_EXAMPLES)
        install(
            TARGETS
                ${PROJECT_NAME}
            RUNTIME DESTINATION
                ${BIN_INSTALL_DIR}
            )
    endif()
endif()
