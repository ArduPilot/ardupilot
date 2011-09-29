# - Try to find  ARKSCICOS
# Once done, this will define
#
#  ARKSCICOS_FOUND - system has scicoslab 
#  ARKSCICOS_INCLUDE_DIRS - the scicoslab include directories
#  ARKSCICOS_LIBRARIES - libraries to link to

include(LibFindMacroos)
include(MacroCommonPaths)

MacroCommonPaths(ARKSCICOS)

# Include dir
find_path(ARKSCICOS_INCLUDE_DIR
	NAMES definiotions.hpp
	PATHS ${COMMON_INCLUDE_PATHS_ARKSCICOS}
)

# the library itself
find_library(ARKSCICOS_LIBRARY
	NAMES arkscicos
	PATHS ${COMMON_LIBRARY_PATHS_ARKSCICOS}
)

# the import file
find_path(ARKSCICOS_LIBRARY_DIR
	NAMES arkscicos/arkscicos-targets.cmake
	PATHS ${COMMON_LIBRARY_PATHS_ARKSCICOS}
)
set(ARKSCICOS_LIB_IMPORT ${ARKSCICOS_LIBRARY_DIR}/arkscicos/arkscicos-targets.cmake)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(ARKSCICOS_PROCESS_INCLUDES ARKSCICOS_INCLUDE_DIR)
set(ARKSCICOS_PROCESS_LIBS ARKSCICOS_LIBRARY ARKSCICOS_LIBRARIES)
libfind_process(ARKSCICOS)
