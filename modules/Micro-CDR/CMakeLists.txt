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

###############################################################################
# CMake build rules for Micro CDR
###############################################################################
cmake_minimum_required(VERSION 3.15)

###############################################################################
# Build options
###############################################################################
option(UCDR_SUPERBUILD "Enable superbuild compilation." ON)
option(UCDR_BUILD_TESTS "Build tests." OFF)
option(UCDR_BUILD_EXAMPLES "Build examples." OFF)
option(UCDR_PIC "Control Position Independent Code." ON)
option(UCDR_ISOLATED_INSTALL "Install the project into a separated folder with version control." ON)
option(BUILD_SHARED_LIBS "Control shared/static library building." OFF)

option(UCDR_BUILD_CI_TESTS "Build CI test cases." OFF)
if(UCDR_BUILD_CI_TESTS)
    set(UCDR_BUILD_TESTS ON)
endif()

set(CONFIG_BIG_ENDIANNESS OFF CACHE BOOL "Set the machine endianness to big endianness (default: little endianness).")

# Set CMAKE_BUILD_TYPE to Release by default.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

###############################################################################
# Product information
###############################################################################
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
if(NOT UCDR_SUPERBUILD)
    project(microcdr VERSION "2.0.1" LANGUAGES C)
else()
    project(ucdr_superbuild NONE)
    include(${PROJECT_SOURCE_DIR}/cmake/SuperBuild.cmake)
    return()
endif()

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_BINARY_DIR}/temp_install)

###############################################################################
# Config
###############################################################################
if(CONFIG_BIG_ENDIANNESS)
    set(CONFIG_MACHINE_ENDIANNESS UCDR_BIG_ENDIANNESS) #big
else()
    set(CONFIG_MACHINE_ENDIANNESS UCDR_LITTLE_ENDIANNESS) #little
endif()

###############################################################################
# Check MSVC architecture
###############################################################################
include(${PROJECT_SOURCE_DIR}/cmake/common/check_configuration.cmake)
if(MSVC OR MSVC_IDE)
    check_msvc_arch()
endif()

###############################################################################
# Set install directories
###############################################################################
if(UCDR_ISOLATED_INSTALL)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}-${PROJECT_VERSION}")
endif()

include(GNUInstallDirs)
set(BIN_INSTALL_DIR     ${CMAKE_INSTALL_BINDIR}     CACHE PATH "Installation directory for binaries")
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for C headers")
set(LIB_INSTALL_DIR     ${CMAKE_INSTALL_LIBDIR}     CACHE PATH "Installation directory for libraries")
set(DATA_INSTALL_DIR    ${CMAKE_INSTALL_DATADIR}    CACHE PATH "Installation directory for data")
if(WIN32)
    set(LICENSE_INSTALL_DIR . CACHE PATH "Installation directory for licenses")
else()
    set(LICENSE_INSTALL_DIR ${DATA_INSTALL_DIR}/${PROJECT_NAME} CACHE PATH "Installation directory for licenses")
endif()

###############################################################################
# Targets
###############################################################################
# Library
add_library(${PROJECT_NAME}
    src/c/common.c
    src/c/types/basic.c
    src/c/types/string.c
    src/c/types/array.c
    src/c/types/sequence.c
    )

# Set common properties
set_common_compile_options(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION
        ${PROJECT_VERSION}
    SOVERSION
        ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    C_STANDARD
        99
    C_STANDARD_REQUIRED
        YES
    POSITION_INDEPENDENT_CODE
        ${UCDR_PIC}
    )

target_compile_options(${PROJECT_NAME}
    PRIVATE
        $<$<C_COMPILER_ID:GNU>:-fdata-sections -ffunction-sections>
    )

# Set Windows shared libraries name
get_target_property(UCDR_TARGET_TYPE ${PROJECT_NAME} TYPE)
if((CMAKE_SYSTEM_NAME STREQUAL "Windows") AND (UCDR_TARGET_TYPE STREQUAL "SHARED_LIBRARY"))
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RELEASE_POSTFIX
            -${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        RELWITHDEBINFO_POSTFIX
            -${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        DEBUG_POSTFIX
            d-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )

    target_compile_definitions(${PROJECT_NAME}
        PUBLIC
            ${PROJECT_NAME}_SHARED
        )
endif()

# Define public headers
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

###############################################################################
# Examples
###############################################################################
if(UCDR_BUILD_EXAMPLES)
    add_subdirectory(examples/basic)
    add_subdirectory(examples/fragmentation)
endif()

###############################################################################
# Testing
###############################################################################
if(UCDR_BUILD_CI_TESTS)
    include(CTest)
    add_subdirectory(test/case)
endif()

if(UCDR_BUILD_TESTS)
    include(${PROJECT_SOURCE_DIR}/cmake/common/gtest.cmake)
    enable_testing()
    add_subdirectory(test)
endif()

###############################################################################
# Packaging
###############################################################################
# Install library
install(
    TARGETS
        ${PROJECT_NAME}
    EXPORT
        ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION
        ${LIB_INSTALL_DIR}
    LIBRARY DESTINATION
        ${LIB_INSTALL_DIR}
    RUNTIME DESTINATION
        ${BIN_INSTALL_DIR}
    COMPONENT
        libraries
    )

# Install includes
install(
    DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/ucdr/
    DESTINATION
        ${INCLUDE_INSTALL_DIR}/ucdr
    FILES_MATCHING
        PATTERN "*.h"
    )

# Generate config.h
configure_file(${PROJECT_SOURCE_DIR}/include/ucdr/config.h.in
    ${PROJECT_BINARY_DIR}/include/ucdr/config.h
    )

# Install config.h
install(
    FILES
        ${PROJECT_BINARY_DIR}/include/ucdr/config.h
    DESTINATION
        ${INCLUDE_INSTALL_DIR}/ucdr
    )

# Export library
install(
    EXPORT
        ${PROJECT_NAME}Targets
    DESTINATION
        ${DATA_INSTALL_DIR}/${PROJECT_NAME}/cmake
    )

# Package configuration
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/packaging/Config.cmake.in
        ${PROJECT_BINARY_DIR}/cmake/config/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION
        ${DATA_INSTALL_DIR}/${PROJECT_NAME}/cmake
    PATH_VARS
        BIN_INSTALL_DIR
        INCLUDE_INSTALL_DIR
        LIB_INSTALL_DIR
        DATA_INSTALL_DIR
    )

write_basic_package_version_file(
    ${PROJECT_BINARY_DIR}/cmake/config/${PROJECT_NAME}ConfigVersion.cmake
    VERSION
        ${PROJECT_VERSION}
    COMPATIBILITY
        SameMajorVersion
    )

install(
    FILES
        ${PROJECT_BINARY_DIR}/cmake/config/${PROJECT_NAME}Config.cmake
        ${PROJECT_BINARY_DIR}/cmake/config/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION
        ${DATA_INSTALL_DIR}/${PROJECT_NAME}/cmake
    )
