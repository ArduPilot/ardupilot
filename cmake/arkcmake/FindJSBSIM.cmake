# - Try to find  JSBSIM
# Once done, this will define
#
#  JSBSIM_FOUND - system has scicoslab 
#  JSBSIM_INCLUDE_DIRS - the scicoslab include directories
#  JSBSIM_LIBRARIES - libraries to link to

include(LibFindMacros)
include(MacroCommonPaths)

MacroCommonPaths(JSBSIM)

# Include dir
find_path(JSBSIM_INCLUDE_DIR
	NAMES JSBSim/initialization/FGTrimmer.h
	PATHS ${COMMON_INCLUDE_PATHS_JSBSIM}
)

# data dir
find_path(JSBSIM_DATA_DIR_SEARCH
	NAMES jsbsim/aircraft/aircraft_template.xml
	PATHS ${COMMON_DATA_PATHS_JSBSIM}
)
set(JSBSIM_DATA_DIR ${JSBSIM_DATA_DIR_SEARCH}/jsbsim)

# Finally the library itself
find_library(JSBSIM_LIBRARY
	NAMES JSBSim
	PATHS ${COMMON_LIBRARY_PATHS_JSBSIM}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(JSBSIM_PROCESS_INCLUDES JSBSIM_INCLUDE_DIR)
set(JSBSIM_PROCESS_LIBS JSBSIM_LIBRARY JSBSIM_LIBRARIES)
set(JSBSIM_INCLUDE_DIR ${JSBSIM_INCLUDE_DIR} ${JSBSIM_INCLUDE_DIR}/JSBSim)
set(JSBSIM_INCLUDES ${JSBSIM_INCLUDES} ${JSBSIM_INCLUDE_DIR}/JSBSim)

libfind_process(JSBSIM)
