function(apm_option NAME)
    cmake_parse_arguments(ARG
        "ADVANCED"
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
        message(STATUS "\tVAR_TYPE: ${VAR_TYPE}")
        set("${NAME}" "${ARG_DEFAULT}" CACHE "${ARG_TYPE}" "${ARG_DESCRIPTION}" FORCE) 

    # if not hidden, add it to the global options list
    else() 
        set(APM_OPTIONS ${APM_OPTIONS} ${NAME} CACHE INTERNAL "list of all options")
        list(REMOVE_DUPLICATES APM_OPTIONS)
        #message(STATUS "APM_OPTIONS: ${APM_OPTIONS}")
    endif()

    # set options for combo box
    set_property(CACHE "${NAME}" PROPERTY STRINGS ${ARG_OPTIONS})

    # mark as advanced if advanced option given
    if(ARG_ADVANCED)
        mark_as_advanced(FORCE "${NAME}")
    endif()

endfunction()

function(apm_option_generate_config)
    cmake_parse_arguments(ARG "" "FILE" "" ${ARGN})
    list(REMOVE_DUPLICATES APM_OPTIONS)
    file (WRITE "${CMAKE_BINARY_DIR}/${ARG_FILE}" "//automatically generated, do not edit\n")
    file (APPEND "${CMAKE_BINARY_DIR}/${ARG_FILE}" "#define OFF 0\n#define ON 1\n")
    foreach(ITEM ${APM_OPTIONS})
        #message(STATUS "item: ${ITEM}")
        get_property(ITEM_VALUE CACHE ${ITEM} PROPERTY VALUE) 
        get_property(ITEM_HELP CACHE ${ITEM} PROPERTY HELPSTRING) 
        file(APPEND "${CMAKE_BINARY_DIR}/${ARG_FILE}" "\n#define ${ITEM} ${ITEM_VALUE} // ${ITEM_HELP}")
    endforeach()
endfunction()
