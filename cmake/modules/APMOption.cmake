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
    endif()

    # set options for combo box
    set_property(CACHE "${NAME}" PROPERTY STRINGS ${ARG_OPTIONS})

    # mark as advanced if advanced option given
    if(ARG_ADVANCED)
        mark_as_advanced(FORCE "${NAME}")
    endif()

endfunction()
