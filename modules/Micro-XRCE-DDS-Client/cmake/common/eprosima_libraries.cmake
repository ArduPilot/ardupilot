# Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

macro(eprosima_find_package package)

    if(NOT ${package}_FOUND)

        # Parse arguments.
        set(options REQUIRED)
        set(multiValueArgs OPTIONS)
        cmake_parse_arguments(FIND "${options}" "" "${multiValueArgs}" ${ARGN})

        option(THIRDPARTY "Activate the use of internal thirdparties" OFF)
        option(THIRDPARTY_UPDATE "Activate the auto update of internal thirdparties" ON)

        if(EPROSIMA_BUILD)
            set(THIRDPARTY ON)
        endif()

        option(THIRDPARTY_${package} "Activate the use of internal thirdparty ${package}" OFF)

        if(NOT EPROSIMA_INSTALLER)
            find_package(${package} QUIET)
        endif()

        if(NOT ${package}_FOUND AND (THIRDPARTY OR THIRDPARTY_${package}))
            set(SUBDIRECTORY_EXIST TRUE)
            if(THIRDPARTY_UPDATE OR NOT EXISTS "${PROJECT_SOURCE_DIR}/thirdparty/${package}/CMakeLists.txt")
                message(STATUS "${package} thirdparty is being updated...")
                execute_process(
                    COMMAND git submodule update --recursive --init "thirdparty/${package}"
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                    RESULT_VARIABLE EXECUTE_RESULT
                    )
                if(NOT EXECUTE_RESULT EQUAL 0)
                    message(WARNING "Cannot configure Git submodule ${package}")
                    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/thirdparty/${package}/CMakeLists.txt")
                        set(SUBDIRECTORY_EXIST FALSE)
                    endif()
                endif()
            endif()

            if(SUBDIRECTORY_EXIST)
                foreach(opt_ ${FIND_OPTIONS})
                    set(${opt_} ON)
                endforeach()
                # Keep temp value in order to avoid call packages' installer.
                set(EPROSIMA_INSTALLER_TEMP ${EPROSIMA_INSTALLER})
                unset(EPROSIMA_INSTALLER CACHE)
                add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/${package})
                set(EPROSIMA_INSTALLER ${EPROSIMA_INSTALLER_TEMP})
                set(${package}_FOUND TRUE)
                if(NOT IS_TOP_LEVEL)
                    set(${package}_FOUND TRUE PARENT_SCOPE)
                endif()
            endif()
        endif()

        if(${package}_FOUND)
            message(STATUS "${package} library found...")
        elseif(${FIND_REQUIRED})
            message(FATAL_ERROR "${package} library not found...")
        else()
            message(STATUS "${package} library not found...")
        endif()
    endif()
endmacro()

macro(eprosima_find_thirdparty package thirdparty_name)
    if(NOT (EPROSIMA_INSTALLER AND (MSVC OR MSVC_IDE)))

        option(THIRDPARTY_${package} "Activate the use of internal thirdparty ${package}" OFF)

        find_package(${package} QUIET)

        if(NOT ${package}_FOUND AND (THIRDPARTY OR THIRDPARTY_${package}))
            execute_process(
                COMMAND git submodule update --recursive --init "thirdparty/${thirdparty_name}"
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                RESULT_VARIABLE EXECUTE_RESULT
                )

            if(EXECUTE_RESULT EQUAL 0)
                set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/thirdparty/${thirdparty_name})
                set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/thirdparty/${thirdparty_name}/${thirdparty_name})
                find_package(${package} REQUIRED)
            else()
                message(FATAL_ERROR "Cannot configure Git submodule ${package}")
            endif()
        endif()

    endif()
endmacro()
