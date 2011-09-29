#  - MACRO_ADD_PLUGIN(name [WITH_PREFIX] file1 .. fileN)
#
#  Create a plugin from the given source files.
#  If WITH_PREFIX is given, the resulting plugin will have the
#  prefix "lib", otherwise it won't.
#
# Copyright (c) 2006, Alexander Neundorf, <neundorf@kde.org>
# Copyright (c) 2006, Laurent Montel, <montel@kde.org>
# Copyright (c) 2006, Andreas Schneider, <mail@cynapses.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.


macro (MACRO_ADD_PLUGIN _target_NAME _with_PREFIX)

  if (${_with_PREFIX} STREQUAL "WITH_PREFIX")
    set(_first_SRC)
  else (${_with_PREFIX} STREQUAL "WITH_PREFIX")
    set(_first_SRC ${_with_PREFIX})
  endif (${_with_PREFIX} STREQUAL "WITH_PREFIX")

  add_library(${_target_NAME} MODULE ${_first_SRC} ${ARGN})

  if (_first_SRC)
    set_target_properties(${_target_NAME} PROPERTIES PREFIX "")
  endif (_first_SRC)

endmacro (MACRO_ADD_PLUGIN _name _sources)

