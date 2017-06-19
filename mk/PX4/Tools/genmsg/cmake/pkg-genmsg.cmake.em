# generated from genmsg/cmake/pkg-genmsg.cmake.em

@{
import os
import sys

import genmsg
import genmsg.base
genmsg.base.log_verbose('GENMSG_VERBOSE' in os.environ)
import genmsg.deps
import genmsg.gentools

# split incoming variables
messages = messages_str.split(';') if messages_str != '' else []
services = services_str.split(';') if services_str != '' else []
dependencies = dependencies_str.split(';') if dependencies_str != '' else []
dep_search_paths = dep_include_paths_str.split(';') if dep_include_paths_str != '' else []

dep_search_paths_dict = {}
dep_search_paths_tuple_list = []
is_even = True
for val in dep_search_paths:
    if is_even:
        dep_search_paths_dict.setdefault(val, [])
        val_prev = val
        is_even = False
    else:
        dep_search_paths_dict[val_prev].append(val)
        dep_search_paths_tuple_list.append((val_prev, val))
        is_even = True
dep_search_paths = dep_search_paths_dict

if not messages and not services:
    print('message(WARNING "Invoking generate_messages() without having added any message or service file before.\nYou should either add add_message_files() and/or add_service_files() calls or remove the invocation of generate_messages().")')

msg_deps = {}
msg_dep_types = {}
for m in messages:
  try:
    _deps = genmsg.deps.find_msg_dependencies_with_type(pkg_name, m, dep_search_paths)
    msg_deps[m] = [d[1] for d in _deps]
    msg_dep_types[m] = [d[0] for d in _deps]
  except genmsg.MsgNotFound as e:
    print('message(FATAL_ERROR "Could not find messages which \'%s\' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?\n%s")' % (m, str(e).replace('"', '\\"')))

srv_deps = {}
srv_dep_types = {}
for s in services:
  try:
    _deps = genmsg.deps.find_srv_dependencies_with_type(pkg_name, s, dep_search_paths)
    srv_deps[s] = [d[1] for d in _deps]
    srv_dep_types[s] = [d[0] for d in _deps]
  except genmsg.MsgNotFound as e:
    print('message(FATAL_ERROR "Could not find messages which \'%s\' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?\n%s")' % (s, str(e).replace('"', '\\"')))

}@
message(STATUS "@(pkg_name): @(len(messages)) messages, @(len(services)) services")

set(MSG_I_FLAGS "@(';'.join(["-I%s:%s" % (dep, dir) for dep, dir in dep_search_paths_tuple_list]))")

# Find all generators
@[if langs]@
@[for l in langs.split(';')]@
find_package(@l REQUIRED)
@[end for]@
@[end if]@

add_custom_target(@(pkg_name)_generate_messages ALL)

# verify that message/service dependencies have not changed since configure
@{all_deps = dict(list(msg_deps.items()) + list(srv_deps.items()))}
@{all_dep_types = dict(list(msg_dep_types.items()) + list(srv_dep_types.items()))}
@[for f in all_deps.keys()]@
@{dep_types = ':'.join(all_dep_types[f]).replace('\\','/')}
get_filename_component(_filename "@(f)" NAME_WE)
add_custom_target(_@(pkg_name)_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "@(pkg_name)" "@(f)" "@(dep_types)"
)
@[end for]@# messages and services

#
#  langs = @langs
#

@[if langs]@
@[for l in langs.split(';')]@
### Section generating for lang: @l
### Generating Messages
@[for m in msg_deps.keys()]@
_generate_msg_@(l[3:])(@pkg_name
  "@m"
  "${MSG_I_FLAGS}"
  "@(';'.join(msg_deps[m]).replace("\\","/"))"
  ${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name
)
@[end for]@# messages

### Generating Services
@[for s in srv_deps.keys()]@
_generate_srv_@(l[3:])(@pkg_name
  "@s"
  "${MSG_I_FLAGS}"
  "@(';'.join(srv_deps[s]).replace("\\","/"))"
  ${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name
)
@[end for]@# services

### Generating Module File
_generate_module_@(l[3:])(@pkg_name
  ${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name
  "${ALL_GEN_OUTPUT_FILES_@(l[3:])}"
)

add_custom_target(@(pkg_name)_generate_messages_@(l[3:])
  DEPENDS ${ALL_GEN_OUTPUT_FILES_@(l[3:])}
)
add_dependencies(@(pkg_name)_generate_messages @(pkg_name)_generate_messages_@(l[3:]))

# add dependencies to all check dependencies targets
@[for f in all_deps.keys()]@
get_filename_component(_filename "@(f)" NAME_WE)
add_dependencies(@(pkg_name)_generate_messages_@(l[3:]) _@(pkg_name)_generate_messages_check_deps_${_filename})
@[end for]@# messages and services

# target for backward compatibility
add_custom_target(@(pkg_name)_@(l))
add_dependencies(@(pkg_name)_@(l) @(pkg_name)_generate_messages_@(l[3:]))

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS @(pkg_name)_generate_messages_@(l[3:]))

@[end for]@# langs
@[end if]@

@[if langs]@
@[for l in langs.split(';')]@

if(@(l)_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name)
@[if l == 'genpy']@
  install(CODE "execute_process(COMMAND \"@(PYTHON_EXECUTABLE)\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name\")")
@[end if]@
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name
    DESTINATION ${@(l)_INSTALL_DIR}
@[if l == 'genpy' and package_has_static_sources]@
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name
    DESTINATION ${@(l)_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@(pkg_name)/.+/__init__.pyc?$"
@[end if]@
  )
endif()
@[for d in dependencies]@
add_dependencies(@(pkg_name)_generate_messages_@(l[3:]) @(d)_generate_messages_@(l[3:]))
@[end for]@# dependencies
@[end for]@# langs
@[end if]@
