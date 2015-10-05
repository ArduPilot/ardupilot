# Pastix requires METIS or METIS (partitioning and reordering tools)

if (METIS_INCLUDES AND METIS_LIBRARIES)
  set(METIS_FIND_QUIETLY TRUE)
endif (METIS_INCLUDES AND METIS_LIBRARIES)

find_path(METIS_INCLUDES 
  NAMES 
  metis.h 
  PATHS 
  $ENV{METISDIR} 
  ${INCLUDE_INSTALL_DIR} 
  PATH_SUFFIXES
  .
  metis
  include
)

macro(_metis_check_version)
  file(READ "${METIS_INCLUDES}/metis.h" _metis_version_header)

  string(REGEX MATCH "define[ \t]+METIS_VER_MAJOR[ \t]+([0-9]+)" _metis_major_version_match "${_metis_version_header}")
  set(METIS_MAJOR_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+METIS_VER_MINOR[ \t]+([0-9]+)" _metis_minor_version_match "${_metis_version_header}")
  set(METIS_MINOR_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+METIS_VER_SUBMINOR[ \t]+([0-9]+)" _metis_subminor_version_match "${_metis_version_header}")
  set(METIS_SUBMINOR_VERSION "${CMAKE_MATCH_1}")
  if(NOT METIS_MAJOR_VERSION)
    message(STATUS "Could not determine Metis version. Assuming version 4.0.0")
    set(METIS_VERSION 4.0.0)
  else()
    set(METIS_VERSION ${METIS_MAJOR_VERSION}.${METIS_MINOR_VERSION}.${METIS_SUBMINOR_VERSION})
  endif()
  if(${METIS_VERSION} VERSION_LESS ${Metis_FIND_VERSION})
    set(METIS_VERSION_OK FALSE)
  else()
    set(METIS_VERSION_OK TRUE)
  endif()

  if(NOT METIS_VERSION_OK)
    message(STATUS "Metis version ${METIS_VERSION} found in ${METIS_INCLUDES}, "
                   "but at least version ${Metis_FIND_VERSION} is required")
  endif(NOT METIS_VERSION_OK)
endmacro(_metis_check_version)

  if(METIS_INCLUDES AND Metis_FIND_VERSION)
    _metis_check_version()
  else()
    set(METIS_VERSION_OK TRUE)
  endif()


find_library(METIS_LIBRARIES metis PATHS $ENV{METISDIR} ${LIB_INSTALL_DIR} PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(METIS DEFAULT_MSG
                                  METIS_INCLUDES METIS_LIBRARIES METIS_VERSION_OK)

mark_as_advanced(METIS_INCLUDES METIS_LIBRARIES)
