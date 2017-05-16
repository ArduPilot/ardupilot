# Build script for ArduPilot.
# .. Let's check, whether we have I2C tools installed
#
# Author: Daniel Frenzel
#

include ( CheckIncludeFileCXX )

set (i2c_path
  /usr/include/linux
  # whatever more
)

set (i2c_tools 
  i2c-dev.h
  # whatever more
)

macro(CHECK_INCLUDE_FILE_CXX_ERROR INCLUDE_FILE HAVE_FILE FOUND)
  CHECK_INCLUDE_FILE_CXX( ${INCLUDE_FILE} ${HAVE_FILE} )
  if ( NOT ${HAVE_FILE} )
    set ( ${FOUND} 0 ) 
    unset (HAVE_UNITTESTXX CACHE)
  else()
    set ( ${FOUND} 1 ) 
  endif ()
endmacro()

set( FILE_FOUND 0 )
foreach( PATH_VAR ${i2c_path} )
  set( H_FILE "${PATH_VAR}/${i2c_tools}" ) 
  CHECK_INCLUDE_FILE_CXX_ERROR( ${H_FILE} HAVE_UNITTESTXX FILE_FOUND )
  
  if( ${FILE_FOUND} )
    break()
  endif()
  
endforeach()

if( NOT ${FILE_FOUND} )
  message( FATAL_ERROR "${i2c_tools} is maybe not installed on this system :( " )
endif()