# - MACRO_ENSURE_OUT_OF_SOURCE_BUILD(<errorMessage>)
# MACRO_ENSURE_OUT_OF_SOURCE_BUILD(<errorMessage>)

# Copyright (c) 2006, Alexander Neundorf, <neundorf@kde.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

macro (MACRO_ENSURE_OUT_OF_SOURCE_BUILD _errorMessage)

    string(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}" _insource)
    if (_insource)
        file(REMOVE [CMakeCache.txt CMakeFiles]) 
        message(FATAL_ERROR "${_errorMessage}")
    endif (_insource)

endmacro (MACRO_ENSURE_OUT_OF_SOURCE_BUILD)

# vim:ts=4:sw=4:expandtab
