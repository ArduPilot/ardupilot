# apm_option: options parsing for ardupilotmega
#
# OPTIONS:
#   ADVANCED - indicates this option is advaned (hidden in gui unless advanced selected)
#   DEFINE_ONLY - this flag is either indicates that the variable is defined if true, and not defined if false
#   BUILD_FLAG - this flag forces the variable to be added to build flags, for files that
#       don't include the config header
#
# SINGLE INPUT ARGUMENTS: (can only pass one arguments)
#   TYPE - the type of the argument (BOOL for on/off variables/ STRING for all others)
#   DESCRIPTION - description of option, shown as tool-tip and written in config file
#   DEFAULT - the default value of the option
#
# MULTIPLE VARIABLE ARGUMENTS: (can pass a lit of items)
#   OPTIONS - values that this option may take, if only a finite set is possible, creates combo-box in gui 
#   DEPENDS - a list of booleans that this argument depends on, can be used to disable options when the
#       are not appropriate
#
# Author: James Goppert
#
function(apm_option NAME)
    cmake_parse_arguments(ARG
        "ADVANCED;DEFINE_ONLY;BUILD_FLAG"
        "TYPE;DESCRIPTION;DEFAULT" "OPTIONS;DEPENDS" ${ARGN})

    #message(STATUS "parsing argument: ${NAME}")

    # if option dependencies not met, hide the option
    foreach(DEPEND ${ARG_DEPENDS})
        if (NOT ${${DEPEND}})
            #message(STATUS "\tfailed dep: ${DEPEND}")
            set(ARG_TYPE "INTERNAL")
            set("${NAME}" "${ARG_DEFAULT}" CACHE INTERNAL "${ARG_DESCRIPTION}" FORCE) 
            return()
        endif()
    endforeach()

    # set variable
    set("${NAME}" "${ARG_DEFAULT}" CACHE "${ARG_TYPE}" "${ARG_DESCRIPTION}") 

    # force variable reinit if it was internal (hidden)
    get_property(VAR_TYPE CACHE ${NAME} PROPERTY TYPE) 
    if ("${VAR_TYPE}" STREQUAL "INTERNAL")
        #message(STATUS "\tVAR_TYPE: ${VAR_TYPE}")
        set("${NAME}" "${ARG_DEFAULT}" CACHE "${ARG_TYPE}" "${ARG_DESCRIPTION}" FORCE) 

    # if a build flag, initialize it and add it to the global options list
    elseif(${ARG_BUILD_FLAG}) 
        #message(STATUS "build flag found for item ${NAME}")
        set(APM_BUILD_FLAGS_LIST ${APM_BUILD_FLAGS_LIST} ${NAME} CACHE INTERNAL "list of all build flags")
        list(REMOVE_DUPLICATES APM_BUILD_FLAGS_LIST)

    # if not hidden, and not a build flag, add it to the global options list
    else()
        set(APM_OPTIONS_LIST ${APM_OPTIONS_LIST} ${NAME} CACHE INTERNAL "list of all options")
        list(REMOVE_DUPLICATES APM_OPTIONS_LIST)

    endif()

    # set list of options
    if (NOT "${ARG_OPTIONS}" STREQUAL "")
        set_property(CACHE "${NAME}" PROPERTY STRINGS ${ARG_OPTIONS})
        list(FIND ARG_OPTIONS "${ARG_DEFAULT}" ARG_POSITION)
        if (ARG_POSITION EQUAL -1)
            message(FATAL_ERROR "default value: ${ARG_DEFAULT} not in given set of options: ${ARG_OPTIONS}")
        endif()
    endif()

    # mark as advanced if advanced option given
    if(ARG_ADVANCED)
        mark_as_advanced(FORCE "${NAME}")
    endif()

    if(ARG_DEFINE_ONLY)
        set("${NAME}_DEFINE_ONLY" TRUE CACHE INTERNAL "Define only?" FORCE) 
    else()
        set("${NAME}_DEFINE_ONLY" FALSE CACHE INTERNAL "Define only?" FORCE) 
    endif()

endfunction()

# apm_option_generate_config: generates a config file using the list of options.
#
# SINGLE INPUT ARGUMENTS: (can only pass one arguments)
#   FILE - the file to write the config to
#   BUILD_FLAGS - variable to store build flags in
#
# Author: James Goppert
#
function(apm_option_generate_config)
    cmake_parse_arguments(ARG "" "FILE;BUILD_FLAGS" "" ${ARGN})

    # options
    if (NOT "${APM_OPTIONS_LIST}" STREQUAL "")
        list(REMOVE_DUPLICATES APM_OPTIONS_LIST)
    endif()
    file (WRITE "${CMAKE_BINARY_DIR}/${ARG_FILE}" "//automatically generated, do not edit\n")
    file (APPEND "${CMAKE_BINARY_DIR}/${ARG_FILE}" "#define OFF 0\n#define ON 1\n")
    foreach(ITEM ${APM_OPTIONS_LIST})
        #message(STATUS "option: ${ITEM}")
        get_property(ITEM_VALUE CACHE ${ITEM} PROPERTY VALUE) 
        get_property(ITEM_HELP CACHE ${ITEM} PROPERTY HELPSTRING) 
        if (${${ITEM}_DEFINE_ONLY})
            if (${ITEM_VALUE})
                file(APPEND "${CMAKE_BINARY_DIR}/${ARG_FILE}" "\n#define ${ITEM} // ${ITEM_HELP}")
            else()
                file(APPEND "${CMAKE_BINARY_DIR}/${ARG_FILE}" "\n//#define ${ITEM} // ${ITEM_HELP}")
            endif()
        else()
            file(APPEND "${CMAKE_BINARY_DIR}/${ARG_FILE}" "\n#define ${ITEM} ${ITEM_VALUE} // ${ITEM_HELP}")
        endif()
    endforeach()

    # build flags
    if (NOT "${APM_BUILD_FLAGS_LIST}" STREQUAL "")
        list(REMOVE_DUPLICATES APM_BUILD_FLAGS_LIST)
    endif()
    foreach(ITEM ${APM_BUILD_FLAGS_LIST})
        #message(STATUS "build flags: ${ITEM}")
        set(${ARG_BUILD_FLAGS} "" CACHE INTERNAL "build flags" FORCE)
        get_property(ITEM_VALUE CACHE ${ITEM} PROPERTY VALUE) 
        if (${${ITEM}_DEFINE_ONLY})
            if (${ITEM_VALUE})
                set(${ARG_BUILD_FLAGS} ${${ARG_BUILD_FLAGS}} "-D${ITEM}" CACHE INTERNAL "build flags")
            endif()
        else()
            set(${ARG_BUILD_FLAGS} ${${ARG_BUILD_FLAGS}} "-D${ITEM}=${ITEM_VALUE}" CACHE INTERNAL "build flags")
        endif()
    endforeach()

endfunction()
