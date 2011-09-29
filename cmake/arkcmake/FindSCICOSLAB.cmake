# - Try to find  SCICOSLAB
# Once done, this will define
#
#  SCICOSLAB_FOUND - system has scicoslab 
#  SCICOSLAB_INCLUDE_DIRS - the scicoslab include directories
#  SCICOSLAB_CONTRIB_DIR - the scicoslab contrib directory

include(LibFindMacros)

# find scicos
if (APPLE)
    execute_process(COMMAND mdfind "kMDItemKind == Application && kMDItemDisplayName == ScicosLabGtk"
        COMMAND head -1
        RESULT_VARIABLE RESULT
        OUTPUT_VARIABLE SCICOS_APP_BUNDLE
        ERROR_VARIABLE ERROR_MESSAGE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if (RESULT) 
        MESSAGE(FATAL_ERROR "Could not locate 'ScicosLabGtk.app' - ${ERROR_MESSAGE}")
    endif (RESULT)
    execute_process(COMMAND find ${SCICOS_APP_BUNDLE} -name routines
        COMMAND head -1
        RESULT_VARIABLE RESULT
        OUTPUT_VARIABLE SCICOSLAB_GUESS_INCLUDE_DIRS
        ERROR_VARIABLE ERROR_MESSAGE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if (RESULT) 
        MESSAGE(FATAL_ERROR "Could not locate 'scicos_block4.h' in ScicosLabGtk.app - ${ERROR_MESSAGE}")
    endif (RESULT)  
    execute_process(COMMAND find ${SCICOS_APP_BUNDLE} -name contrib 
        COMMAND head -1
        RESULT_VARIABLE RESULT
        OUTPUT_VARIABLE SCICOSLAB_GUESS_CONTRIB_DIRS
        ERROR_VARIABLE ERROR_MESSAGE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if (RESULT) 
        MESSAGE(FATAL_ERROR "Could not locate 'loader.sce' in ScicosLabGtk.app - ${ERROR_MESSAGE}")
    endif (RESULT)  
elseif(UNIX)
    set(SCICOSLAB_GUESS_INCLUDE_DIRS
        /usr/lib/scicoslab-gtk-4.4b7/routines
        /usr/lib/scicoslab-gtk-4.4/routines
        /usr/lib/scicoslab-gtk-4.4.1/routines
        )
	set(SCICOSLAB_GUESS_CONTRIB_DIRS
		/usr/lib/scicoslab-gtk-4.4b7/contrib
		/usr/lib/scicoslab-gtk-4.4/contrib
		/usr/lib/scicoslab-gtk-4.4.1/contrib
	)
elseif(WIN32)
    message(FATAL_ERROR "scicoslab cmake find module doesn't work for windows")
endif()


# Include dir
find_path(SCICOSLAB_INCLUDE_DIR
  NAMES scicos/scicos_block4.h
  PATHS ${SCICOSLAB_GUESS_INCLUDE_DIRS}
)

# Contrib dir
find_path(SCICOSLAB_CONTRIB_DIR
  NAMES loader.sce
  PATHS ${SCICOSLAB_GUESS_CONTRIB_DIRS}
)
message(STATUS "contrib dir ${SCICOSLAB_CONTRIB_DIR}")

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(SCICOSLAB_PROCESS_INCLUDES SCICOSLAB_INCLUDE_DIR)
libfind_process(SCICOSLAB)
# vim:ts=4:sw=4:expandtab
