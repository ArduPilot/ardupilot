@[if DEVELSPACE]@
# bin and template dir variables in develspace
set(GENCPP_BIN "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/gen_cpp.py")
set(GENCPP_TEMPLATE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[else]@
# bin and template dir variables in installspace
set(GENCPP_BIN "${gencpp_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/gen_cpp.py")
set(GENCPP_TEMPLATE_DIR "${gencpp_DIR}/..")
@[end if]@

# Generate .msg->.h for cpp
# The generated .h files should be added ALL_GEN_OUTPUT_FILES_cpp
macro(_generate_msg_cpp ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  file(MAKE_DIRECTORY ${ARG_GEN_OUTPUT_DIR})

  #Create input and output filenames
  get_filename_component(MSG_NAME ${ARG_MSG} NAME)
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  set(MSG_GENERATED_NAME ${MSG_SHORT_NAME}.h)
  set(GEN_OUTPUT_FILE ${ARG_GEN_OUTPUT_DIR}/${MSG_GENERATED_NAME})

  assert(CATKIN_ENV)
  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    DEPENDS ${GENCPP_BIN} ${ARG_MSG} ${ARG_MSG_DEPS} "${GENCPP_TEMPLATE_DIR}/msg.h.template" ${ARGN}
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENCPP_BIN} ${ARG_MSG}
    ${ARG_IFLAGS}
    -p ${ARG_PKG}
    -o ${ARG_GEN_OUTPUT_DIR}
    -e ${GENCPP_TEMPLATE_DIR}
    COMMENT "Generating C++ code from ${ARG_PKG}/${MSG_NAME}"
    )
  list(APPEND ALL_GEN_OUTPUT_FILES_cpp ${GEN_OUTPUT_FILE})

  gencpp_append_include_dirs()
endmacro()

#gencpp uses the same program to generate srv and msg files, so call the same macro
macro(_generate_srv_cpp ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  _generate_msg_cpp(${ARG_PKG} ${ARG_SRV} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" ${ARG_GEN_OUTPUT_DIR} "${GENCPP_TEMPLATE_DIR}/srv.h.template")
endmacro()

macro(_generate_module_cpp)
  # the macros, they do nothing
endmacro()

set(gencpp_INSTALL_DIR include)

macro(gencpp_append_include_dirs)
  if(NOT gencpp_APPENDED_INCLUDE_DIRS)
    # make sure we can find generated messages and that they overlay all other includes
    include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR})
    # pass the include directory to catkin_package()
    list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR})
    set(gencpp_APPENDED_INCLUDE_DIRS TRUE)
  endif()
endmacro()
