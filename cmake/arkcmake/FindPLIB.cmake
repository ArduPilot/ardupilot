# Locate plib
# This module defines
# PLIB_LIBRARY
# PLIB_FOUND, if false, do not try to link to plib
# PLIB_INCLUDE_DIR, where to find the headers
#
# $PLIB_DIR is an environment variable that would
# correspond to the ./configure --prefix=$PLIB_DIR
#
# Created David Guthrie with code by Robert Osfield. 

FIND_PATH(PLIB_INCLUDE_DIR plib/js.h
    $ENV{PLIB_DIR}/include
    $ENV{PLIB_DIR}
    $ENV{PLIB_ROOT}/include
    ${DELTA3D_EXT_DIR}/inc
    $ENV{DELTA_ROOT}/ext/inc
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/include
    /usr/include
    /sw/include # Fink
    /opt/local/include # DarwinPorts
    /opt/csw/include # Blastwave
    /opt/include
    [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;PLIB_ROOT]/include
    /usr/freeware/include
)

MACRO(FIND_PLIB_LIBRARY MYLIBRARY MYLIBRARYNAMES)

    FIND_LIBRARY(${MYLIBRARY}
        NAMES ${MYLIBRARYNAMES}
        PATHS
        $ENV{PLIB_DIR}/lib
        $ENV{PLIB_DIR}
        $ENV{OSGDIR}/lib
        $ENV{OSGDIR}
        $ENV{PLIB_ROOT}/lib
        ${DELTA3D_EXT_DIR}/lib
        $ENV{DELTA_ROOT}/ext/lib
        ~/Library/Frameworks
        /Library/Frameworks
        /usr/local/lib
        /usr/lib
        /sw/lib
        /opt/local/lib
        /opt/csw/lib
        /opt/lib
        [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;PLIB_ROOT]/lib
        /usr/freeware/lib64
    )

ENDMACRO(FIND_PLIB_LIBRARY MYLIBRARY MYLIBRARYNAMES)

SET(PLIB_RELEASE_JS_LIB_NAMES js plibjs)
SET(PLIB_RELEASE_UL_LIB_NAMES ul plibul)
SET(PLIB_DEBUG_JS_LIB_NAMES js_d plibjs_d)
SET(PLIB_DEBUG_UL_LIB_NAMES ul_d plibul_d)


FIND_PLIB_LIBRARY(PLIB_JS_LIBRARY "${PLIB_RELEASE_JS_LIB_NAMES}")
FIND_PLIB_LIBRARY(PLIB_JS_LIBRARY_DEBUG "${PLIB_DEBUG_JS_LIB_NAMES}")
FIND_PLIB_LIBRARY(PLIB_UL_LIBRARY "${PLIB_RELEASE_UL_LIB_NAMES}")
FIND_PLIB_LIBRARY(PLIB_UL_LIBRARY_DEBUG "${PLIB_DEBUG_UL_LIB_NAMES}")

SET(PLIB_FOUND "NO")
IF(PLIB_JS_LIBRARY AND PLIB_INCLUDE_DIR)
    SET(PLIB_FOUND "YES")
ENDIF(PLIB_JS_LIBRARY AND PLIB_INCLUDE_DIR)
