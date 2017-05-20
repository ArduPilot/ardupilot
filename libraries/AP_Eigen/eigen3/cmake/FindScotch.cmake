# Pastix requires SCOTCH or METIS (partitioning and reordering tools)

if (SCOTCH_INCLUDES AND SCOTCH_LIBRARIES)
  set(SCOTCH_FIND_QUIETLY TRUE)
endif (SCOTCH_INCLUDES AND SCOTCH_LIBRARIES)

find_path(SCOTCH_INCLUDES 
  NAMES 
  scotch.h 
  PATHS 
  $ENV{SCOTCHDIR} 
  ${INCLUDE_INSTALL_DIR} 
  PATH_SUFFIXES 
  scotch
)


find_library(SCOTCH_LIBRARIES scotch PATHS $ENV{SCOTCHDIR} ${LIB_INSTALL_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SCOTCH DEFAULT_MSG
                                  SCOTCH_INCLUDES SCOTCH_LIBRARIES)

mark_as_advanced(SCOTCH_INCLUDES SCOTCH_LIBRARIES)
