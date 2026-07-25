project (mavlink)

if (NOT DEFINED MAVLINK_SOURCE_DIR)
    set(MAVLINK_SOURCE_DIR ${PROJECT_SOURCE_DIR})
endif ()

# settings
cmake_minimum_required (VERSION 2.8.2)
set(PROJECT_VERSION_MAJOR "1")
set(PROJECT_VERSION_MINOR "0")
set(PROJECT_VERSION_PATCH "9")
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")
set(PROJECT_CONTACT_EMAIL http://groups.google.com/group/mavlink)
set(PROJECT_CONTACT_VENDOR mavlink)
set(LIBRARY_VERSION ${PROJECT_VERSION})
set(LIBRARY_SOVERSION "0.0.0")

# third party 
# none required

# options
option(USE_SYSTEM_PYMAVLINK "Generate mavlink files with system-wide installed mavgen" OFF)
option(WITH_TESTS "Build test programs." OFF)
option(WITH_BUILD_DEPS "Build dependencies." OFF) # no deps currently to build
option(WITH_BUILD_STATIC "Build preferring static linking." ON)

# variables
set(ROOT_THREAD TRUE CACHE INTERNAL "Is this the top level of the recursion?")

# modules
list(APPEND CMAKE_MODULE_PATH ${MAVLINK_SOURCE_DIR}/cmake ${MAVLINK_SOURCE_DIR}/cmake/arkcmake)
include(DefineCMakeDefaults)
include(CheckIncludeFiles)
include(CheckFunctionExists)
include(CheckSymbolExists)
include(CheckLibraryExists)
#include(CheckTypeSize)
#include(CheckPrototypeExists)
#include(CheckCXXSourceCompiles)
#include(CheckCSourceCompiles)
include(ExternalProjectWithFilename)

if (UNIX)
    include(GNUInstallDirs)
    set(LIB_INSTALL_DIR "lib${LIB_SUFFIX}" CACHE PATH "Installation path for libraries")
endif ()


# spawn new cmake to build deps
if (WITH_BUILD_DEPS AND ROOT_THREAD)
    execute_process(COMMAND ${CMAKE_COMMAND} "${MAVLINK_SOURCE_DIR}"
        "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
        "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
        "-DWITH_BUILD_DEPS=${WITH_BUILD_DEPS}"
        "-DWITH_BUILD_STATIC=${WITH_BUILD_STATIC}"
        "-DWITH_TESTS=${WITH_TESTS}"
        "-DROOT_THREAD=FALSE"
        RESULT_VARIABLE ERROR)
    if (ERROR)
        message(FATAL_ERROR "error, recursing loop returned error code: ${ERROR}")
    endif()
    message("** Making dependencies")
    execute_process(COMMAND ${CMAKE_MAKE_PROGRAM} "-j4" "-f${CMAKE_BINARY_DIR}/Makefile")
    message("** Configuring ${PROJECT_NAME}")
endif()

# external projects find path
if(NOT EP_BASE_DIR)
    set(EP_BASE_DIR "${CMAKE_BINARY_DIR}/CMakeExternals")
endif()
set_property(DIRECTORY PROPERTY EP_BASE ${EP_BASE_DIR})
set(EP_INSTALL_DIR "${EP_BASE_DIR}/Install")
list(APPEND CMAKE_FIND_ROOT_PATH ${EP_BASE_DIR})

# prefer static packages if building static library
message("** Finding libraries")
if (WITH_BUILD_STATIC)
    # prefer static libs
    if(WIN32)
        set(CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
    else()
        set(CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
    endif()
endif()

# find libraries with cmake modules
if(${CMAKE_VERSION} VERSION_LESS 3.12)
  option(USE_PYTHON3 "Use python3 to build MAVLink" OFF)
else()
  option(USE_PYTHON3 "Use python3 to build MAVLink" ON)
endif()

if(USE_PYTHON3)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
  set(PYTHON_SITELIB ${Python3_SITELIB})  
else()
  find_package(PythonInterp)
  set(PYTHON_SITELIB ${CMAKE_INSTALL_LIBDIR}/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)
endif()
  
# enable languages
if (WITH_TESTS)
    enable_language(C)
    enable_language(CXX)
    include(DefineCompilerFlags)
endif()

# build dependencies
if (WITH_BUILD_DEPS AND (NOT ROOT_THREAD) )
    message("** Configuring dependencies")

    # add external projects

    # none required currently

    set(CMAKE_DEFAULT_ARGS
        -DEP_BASE_DIR=${EP_BASE_DIR}
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
        )

    # terminate non root cmake thread
    return()
endif()

# configure
#check_include_files(string.h HAVE_STRING_H)
#check_function_exists(memcopy HAVE_MEMCOPY)
#check_symbol_exists(LC_MESSAGES "locale.h" HAVE_LC_MESSAGES) 
#check_library_exists(pthread  attachNode "" HAVE_PTHREAD)

# config files
configure_file(config.h.in config.h)
install(FILES ${CMAKE_BINARY_DIR}/config.h DESTINATION include/${PROJECT_NAME} COMPONENT Dev)

# mavgen location for mavlink generation
set(mavgen_EXECUTABLE_WITH_PYTHON_INVOCATION ${CMAKE_COMMAND} -E env "PYTHONPATH=$ENV{PYTHONPATH}:${CMAKE_CURRENT_SOURCE_DIR}" ${PYTHON_EXECUTABLE} -m pymavlink.tools.mavgen)
if(USE_SYSTEM_PYMAVLINK)
    find_program (
        mavgen_EXECUTABLE
        NAMES mavgen mavgen.py
    )

    # Fallback to Python invocation, if not found.
    if (mavgen_EXECUTABLE-NOTFOUND)
        set(mavgen_EXECUTABLE ${mavgen_EXECUTABLE_WITH_PYTHON_INVOCATION})
    endif()
else()
    set(mavgen_EXECUTABLE ${mavgen_EXECUTABLE_WITH_PYTHON_INVOCATION})
endif()
message(STATUS "mavgen_EXECUTABLE: ${mavgen_EXECUTABLE}")

macro(generateMavlink version definitions)
    foreach(definition ${definitions})
        set(targetName ${definition}-v${version})
        set(definitionAbsPath ${MAVLINK_SOURCE_DIR}/message_definitions/v1.0/${definition})
        message(STATUS "processing: ${definitionAbsPath}")
        add_custom_command( 
            OUTPUT ${targetName}-stamp
            COMMAND ${mavgen_EXECUTABLE} --lang=C
                                         --wire-protocol=${version}
                                         --output=include/v${version}
                                         ${definitionAbsPath}
            COMMAND touch ${targetName}-stamp
        )
        add_custom_target(${targetName} ALL DEPENDS ${targetName}-stamp)
    endforeach()
endmacro()

# build
set(v1.0Definitions
    ASLUAV.xml
    ardupilotmega.xml
    common.xml
    matrixpilot.xml
    minimal.xml
    paparazzi.xml
    python_array_test.xml
    test.xml
    ualberta.xml
    )
generateMavlink("1.0" "${v1.0Definitions}")
generateMavlink("2.0" "${v1.0Definitions}")

# testing
if (BUILD_TEST)
    if (UNIX) 
        include_directories(${CMAKE_BINARY_DIR}/include/v1.0/common)
        # TODO fix udp example
        #add_executable(mavlink_udp examples/linux/mavlink_udp.c)
    endif()
endif()

# install files
install(DIRECTORY ${CMAKE_BINARY_DIR}/include/ DESTINATION include/${PROJECT_NAME} COMPONENT Dev FILES_MATCHING PATTERN "*.h*")
install(DIRECTORY ${CMAKE_BINARY_DIR}/src/ DESTINATION share/${PROJECT_NAME} COMPONENT Dev FILES_MATCHING PATTERN "*.c*")
install(DIRECTORY ${MAVLINK_SOURCE_DIR}/share/${PROJECT_NAME} DESTINATION share COMPONENT Dev FILES_MATCHING PATTERN "*.c*")
if (UNIX)
    install(DIRECTORY ${MAVLINK_SOURCE_DIR}/pymavlink DESTINATION ${PYTHON_SITELIB} COMPONENT Dev)
else ()
    install(DIRECTORY ${MAVLINK_SOURCE_DIR}/pymavlink DESTINATION "share/pyshared" COMPONENT Dev)
endif ()

configure_file(pc.in ${PROJECT_NAME}.pc)
install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc
    DESTINATION lib${LIB_SUFFIX}/pkgconfig COMPONENT Dev
    )

### packaging

# apple bundle icon
if (APPLE)
    # set how it shows up in Info.plist
    set(MACOSX_BUNDLE_ICON_FILE mavlink.icns)
    # set where in the bundle to put the icns file
    set_source_files_properties(cmake/mavlink.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    # include the icns file in the target
    #list(APPEND MAVLINKGUI_SRCS cmake/mavlink.icns)
endif()

# set NSIS image
if (WIN32)
    set(CPACK_PACKAGE_ICON "${MAVLINK_SOURCE_DIR}/cmake/mavlink.bmp")
endif()

# add file extensions and set resource files
configure_file("COPYING" "COPYING.txt" COPYONLY)
configure_file("README.md" "README.md" COPYONLY)
set(CPACK_RESOURCE_FILE_LICENSE "${MAVLINK_SOURCE_DIR}/COPYING")
set(CPACK_RESOURCE_FILE_README "${MAVLINK_SOURCE_DIR}/README.md")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CPACK_RESOURCE_FILE_README}")
set(CPACK_RESOURCE_FILE_WELCOME "${MAVLINK_SOURCE_DIR}/cmake/WELCOME.txt")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "mavlink message marshalling library")
set(CPACK_PACKAGE_VENDOR ${PROJECT_CONTACT_VENDOR})
set(CPACK_PACKAGE_CONTACT "${PROJECT_CONTACT_EMAIL}")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_SET_DESTDIR TRUE)
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PROJECT_VERSION}")
set(CPACK_COMPONENTS_GROUPING "ALL_COMPONENTS_IN_ONE")
include(CPack)


# vim:sw=4:ts=4:expandtab
