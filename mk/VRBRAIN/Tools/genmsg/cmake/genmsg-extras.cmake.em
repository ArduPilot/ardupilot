# generated from genmsg/cmake/genmsg-extras.cmake.in

if(_GENMSG_EXTRAS_INCLUDED_)
  return()
endif()
set(_GENMSG_EXTRAS_INCLUDED_ TRUE)

# set destination for langs
set(GENMSG_LANGS_DESTINATION "etc/ros/genmsg")

@[if DEVELSPACE]@
# bin dir variables in develspace
set(GENMSG_CHECK_DEPS_SCRIPT "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/genmsg_check_deps.py")
@[else]@
# bin dir variables in installspace
set(GENMSG_CHECK_DEPS_SCRIPT "${genmsg_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/genmsg_check_deps.py")
@[end if]@

include(CMakeParseArguments)

# find message generators in all workspaces
set(message_generators "")
foreach(workspace ${CATKIN_WORKSPACES})
  file(GLOB workspace_message_generators
    RELATIVE ${workspace}/${GENMSG_LANGS_DESTINATION}
    ${workspace}/${GENMSG_LANGS_DESTINATION}/gen*)
  list(APPEND message_generators ${workspace_message_generators})
endforeach()
if(message_generators)
  list(SORT message_generators)
endif()

foreach(message_generator ${message_generators})
  find_package(${message_generator} REQUIRED)
  list(FIND CATKIN_MESSAGE_GENERATORS ${message_generator} _index)
  if(_index EQUAL -1)
    list(APPEND CATKIN_MESSAGE_GENERATORS ${message_generator})
  endif()
endforeach()
if(CATKIN_MESSAGE_GENERATORS)
  list(SORT CATKIN_MESSAGE_GENERATORS)
endif()

# disable specific message generators
string(REPLACE ":" ";" _disabled_message_generators "$ENV{ROS_LANG_DISABLE}")
# remove unknown generators from disabled list
foreach(message_generator ${_disabled_message_generators})
  list(FIND CATKIN_MESSAGE_GENERATORS ${message_generator} _index)
  if(_index EQUAL -1)
    list(REMOVE_ITEM _disabled_message_generators ${message_generator})
    message(WARNING "Unknown message generator specified in ROS_LANG_DISABLE: ${message_generator}")
  endif()
endforeach()
if(_disabled_message_generators)
  message(STATUS "Disabling the following message generators: ${_disabled_message_generators}")
  list(REMOVE_ITEM CATKIN_MESSAGE_GENERATORS ${_disabled_message_generators})
endif()
message(STATUS "Using these message generators: ${CATKIN_MESSAGE_GENERATORS}")

macro(_prepend_path ARG_PATH ARG_FILES ARG_OUTPUT_VAR)
  cmake_parse_arguments(ARG "UNIQUE" "" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "_prepend_path() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  # todo, check for proper path, slasheds, etc
  set(${ARG_OUTPUT_VAR} "")
  foreach(_file ${ARG_FILES})
    set(_value ${ARG_PATH}/${_file})
    list(FIND ${ARG_OUTPUT_VAR} ${_value} _index)
    if(NOT ARG_UNIQUE OR _index EQUAL -1)
      list(APPEND ${ARG_OUTPUT_VAR} ${_value})
    endif()
  endforeach()
endmacro()

macro(add_message_files)
  cmake_parse_arguments(ARG "NOINSTALL" "DIRECTORY;BASE_DIR" "FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "add_message_files() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_DIRECTORY)
    set(ARG_DIRECTORY "msg")
  endif()

  set(MESSAGE_DIR "${ARG_DIRECTORY}")
  if(NOT IS_ABSOLUTE "${MESSAGE_DIR}")
    set(MESSAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${MESSAGE_DIR}")
  endif()
  # override message directory (used by add_action_files())
  if(ARG_BASE_DIR)
    set(MESSAGE_DIR ${ARG_BASE_DIR})
  endif()

  if(NOT IS_DIRECTORY ${MESSAGE_DIR})
    message(FATAL_ERROR "add_message_files() directory not found: ${MESSAGE_DIR}")
  endif()

  if(${PROJECT_NAME}_GENERATE_MESSAGES)
    message(FATAL_ERROR "generate_messages() must be called after add_message_files()")
  endif()

  # if FILES are not passed search message files in the given directory
  # note: ARGV is not variable, so it can not be passed to list(FIND) directly
  set(_argv ${ARGV})
  list(FIND _argv "FILES" _index)
  if(_index EQUAL -1)
    file(GLOB ARG_FILES RELATIVE "${MESSAGE_DIR}" "${MESSAGE_DIR}/*.msg")
    list(SORT ARG_FILES)
  endif()
  _prepend_path(${MESSAGE_DIR} "${ARG_FILES}" FILES_W_PATH)

  list(APPEND ${PROJECT_NAME}_MESSAGE_FILES ${FILES_W_PATH})
  foreach(file ${FILES_W_PATH})
    assert_file_exists(${file} "message file not found")
  endforeach()

  # remember path to messages to resolve them as dependencies
  list(FIND ${PROJECT_NAME}_MSG_INCLUDE_DIRS_DEVELSPACE ${MESSAGE_DIR} _index)
  if(_index EQUAL -1)
    list(APPEND ${PROJECT_NAME}_MSG_INCLUDE_DIRS_DEVELSPACE ${MESSAGE_DIR})
  endif()

  if(NOT ARG_NOINSTALL)
    # ensure that destination variables are initialized
    catkin_destinations()

    list(APPEND ${PROJECT_NAME}_MSG_INCLUDE_DIRS_INSTALLSPACE ${ARG_DIRECTORY})
    install(FILES ${FILES_W_PATH}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${ARG_DIRECTORY})

    _prepend_path("${ARG_DIRECTORY}" "${ARG_FILES}" FILES_W_PATH)
    list(APPEND ${PROJECT_NAME}_INSTALLED_MESSAGE_FILES ${FILES_W_PATH})
  endif()
endmacro()

macro(add_service_files)
  cmake_parse_arguments(ARG "NOINSTALL" "DIRECTORY" "FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "add_service_files() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_DIRECTORY)
    set(ARG_DIRECTORY "srv")
  endif()

  set(SERVICE_DIR "${ARG_DIRECTORY}")
  if(NOT IS_ABSOLUTE "${SERVICE_DIR}")
    set(SERVICE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${SERVICE_DIR}")
  endif()

  if(NOT IS_DIRECTORY ${SERVICE_DIR})
    message(FATAL_ERROR "add_service_files() directory not found: ${SERVICE_DIR}")
  endif()

  if(${PROJECT_NAME}_GENERATE_MESSAGES)
    message(FATAL_ERROR "generate_messages() must be called after add_service_files()")
  endif()

  # if FILES are not passed search service files in the given directory
  # note: ARGV is not variable, so it can not be passed to list(FIND) directly
  set(_argv ${ARGV})
  list(FIND _argv "FILES" _index)
  if(_index EQUAL -1)
    file(GLOB ARG_FILES RELATIVE "${SERVICE_DIR}" "${SERVICE_DIR}/*.srv")
    list(SORT ARG_FILES)
  endif()
  _prepend_path(${SERVICE_DIR} "${ARG_FILES}" FILES_W_PATH)

  list(APPEND ${PROJECT_NAME}_SERVICE_FILES ${FILES_W_PATH})
  foreach(file ${FILES_W_PATH})
    assert_file_exists(${file} "service file not found")
  endforeach()

  if(NOT ARG_NOINSTALL)
    # ensure that destination variables are initialized
    catkin_destinations()

    install(FILES ${FILES_W_PATH}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${ARG_DIRECTORY})

    _prepend_path("${ARG_DIRECTORY}" "${ARG_FILES}" FILES_W_PATH)
    list(APPEND ${PROJECT_NAME}_INSTALLED_SERVICE_FILES ${FILES_W_PATH})
  endif()
endmacro()

macro(generate_messages)
  cmake_parse_arguments(ARG "" "" "DEPENDENCIES;LANGS" ${ARGN})

  if(${PROJECT_NAME}_GENERATE_MESSAGES)
    message(FATAL_ERROR "generate_messages() must only be called once per project'")
  endif()

  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "generate_messages() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(${PROJECT_NAME}_CATKIN_PACKAGE)
    message(FATAL_ERROR "generate_messages() must be called before catkin_package() in project '${PROJECT_NAME}'")
  endif()

  set(ARG_MESSAGES ${${PROJECT_NAME}_MESSAGE_FILES})
  set(ARG_SERVICES ${${PROJECT_NAME}_SERVICE_FILES})
  set(ARG_DEPENDENCIES ${ARG_DEPENDENCIES})

  if(ARG_LANGS)
    set(GEN_LANGS ${ARG_LANGS})
  else()
    set(GEN_LANGS ${CATKIN_MESSAGE_GENERATORS})
  endif()

@[if DEVELSPACE]@
  # cmake dir in develspace
  set(genmsg_CMAKE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/cmake")
@[else]@
  # cmake dir in installspace
  set(genmsg_CMAKE_DIR "@(PKG_CMAKE_DIR)")
@[end if]@

  # ensure that destination variables are initialized
  catkin_destinations()

  # generate devel space config of message include dirs for project
  set(PKG_MSG_INCLUDE_DIRS "${${PROJECT_NAME}_MSG_INCLUDE_DIRS_DEVELSPACE}")
  configure_file(
    ${genmsg_CMAKE_DIR}/pkg-msg-paths.cmake.develspace.in
    ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/${PROJECT_NAME}-msg-paths.cmake
    @@ONLY)
  # generate and install config of message include dirs for project
  set(PKG_MSG_INCLUDE_DIRS "${${PROJECT_NAME}_MSG_INCLUDE_DIRS_INSTALLSPACE}")
  configure_file(
    ${genmsg_CMAKE_DIR}/pkg-msg-paths.cmake.installspace.in
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}-msg-paths.cmake
    @@ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/installspace/${PROJECT_NAME}-msg-paths.cmake
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

  # generate devel space pkg config extra defining variables with all processed message and service files
  set(PKG_MSG_FILES "${${PROJECT_NAME}_MESSAGE_FILES}")
  set(PKG_SRV_FILES "${${PROJECT_NAME}_SERVICE_FILES}")
  configure_file(
    ${genmsg_CMAKE_DIR}/pkg-msg-extras.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/${PROJECT_NAME}-msg-extras.cmake.develspace.in
    @@ONLY)
  # generate install space pkg config extra defining variables with all processed and installed message and service files
  set(PKG_MSG_FILES "${${PROJECT_NAME}_INSTALLED_MESSAGE_FILES}")
  set(PKG_SRV_FILES "${${PROJECT_NAME}_INSTALLED_SERVICE_FILES}")
  configure_file(
    ${genmsg_CMAKE_DIR}/pkg-msg-extras.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/${PROJECT_NAME}-msg-extras.cmake.installspace.in
    @@ONLY)
  # register pkg config files as cmake extra file for the project
  list(APPEND ${PROJECT_NAME}_CFG_EXTRAS ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/${PROJECT_NAME}-msg-extras.cmake)

  # find configuration containing include dirs for projects in all devel- and installspaces
  set(workspaces ${CATKIN_WORKSPACES})
  list(FIND workspaces ${CATKIN_DEVEL_PREFIX} _index)
  if(_index EQUAL -1)
    list(INSERT workspaces 0 ${CATKIN_DEVEL_PREFIX})
  endif()

  set(pending_deps ${PROJECT_NAME} ${ARG_DEPENDENCIES})
  set(handled_deps "")
  while(pending_deps)
    list(GET pending_deps 0 dep)
    list(REMOVE_AT pending_deps 0)
    list(APPEND handled_deps ${dep})

    if(NOT ${dep}_FOUND AND NOT ${dep}_SOURCE_DIR)
      message(FATAL_ERROR "Messages depends on unknown pkg: ${dep} (Missing find_package(${dep}?))")
    endif()

    unset(_dep_msg_paths_file CACHE)
    set(filename "share/${dep}/cmake/${dep}-msg-paths.cmake")
    find_file(_dep_msg_paths_file ${filename} PATHS ${workspaces}
      NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
    if("${_dep_msg_paths_file}" STREQUAL "_dep_msg_paths_file-NOTFOUND")
      message(FATAL_ERROR "Could not find '${filename}' (searched in '${workspaces}').")
    endif()
    include(${_dep_msg_paths_file})
    unset(_dep_msg_paths_file CACHE)

    # explicitly set message include dirs for current project since information from pkg-msg-paths.cmake is not yet available
    if(${dep} STREQUAL ${PROJECT_NAME})
      set(${dep}_MSG_INCLUDE_DIRS ${${PROJECT_NAME}_MSG_INCLUDE_DIRS_DEVELSPACE})
    endif()
    foreach(path ${${dep}_MSG_INCLUDE_DIRS})
      list(APPEND MSG_INCLUDE_DIRS "${dep}")
      list(APPEND MSG_INCLUDE_DIRS "${path}")
    endforeach()

    # add transitive msg dependencies
    if(NOT ${dep} STREQUAL ${PROJECT_NAME})
      foreach(recdep ${${dep}_MSG_DEPENDENCIES})
        set(all_deps ${handled_deps} ${pending_deps})
        list(FIND all_deps ${recdep} _index)
        if(_index EQUAL -1)
          list(APPEND pending_deps ${recdep})
        endif()
      endforeach()
    endif()
  endwhile()

  # mark that generate_messages() was called in order to detect wrong order of calling with catkin_python_setup()
  set(${PROJECT_NAME}_GENERATE_MESSAGES TRUE)
  # check if catkin_python_setup() installs an __init__.py file for a package with the current project name
  # in order to skip the installation of a generated __init__.py file
  set(package_has_static_sources ${${PROJECT_NAME}_CATKIN_PYTHON_SETUP_HAS_PACKAGE_INIT})

  em_expand(${genmsg_CMAKE_DIR}/pkg-genmsg.context.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}-genmsg-context.py
    ${genmsg_CMAKE_DIR}/pkg-genmsg.cmake.em
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}-genmsg.cmake)
  include(${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}-genmsg.cmake)
endmacro()
