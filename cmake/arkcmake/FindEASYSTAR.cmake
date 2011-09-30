# - Try to find  EASYSTAR
# Once done, this will define
#
#  EASYSTAR_FOUND - system has easystar 
#  EASYSTAR_INCLUDE_DIRS - the easystar include directories

include(LibFindMacros)
include(MacroCommonPaths)

MacroCommonPaths(EASYSTAR)

# Include dir
find_path(EASYSTAR_INCLUDE_DIR
	NAMES easystar/easystar.xml
	PATHS ${COMMON_INCLUDE_PATHS_EASYSTAR}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(EASYSTAR_PROCESS_INCLUDES EASYSTAR_INCLUDE_DIR)
libfind_process(EASYSTAR)
