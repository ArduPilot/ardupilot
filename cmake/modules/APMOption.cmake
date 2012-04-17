macro(apm_option NAME)
    cmake_parse_arguments(ARG
        "ADVANCED"
        "TYPE;DESCRIPTION;DEFAULT" "OPTIONS" ${ARGN})

    if ("${ARG_TYPE}" STREQUAL "BOOL")
        set("${NAME}" "${ARG_DEFAULT}" CACHE BOOL "${ARG_DESCRIPTION}") 
    elseif ( ("${ARG_TYPE}" STREQUAL "STRING") OR ("${ARG_TYPE}" STREQUAL "COMBO"))
        set("${NAME}" "${ARG_DEFAULT}" CACHE STRING "${ARG_DESCRIPTION}") 
    else()
        message(FATAL_ERROR "unknown type: \""${ARG_TYPE}"\" for add_option(${NAME}...")
    endif()

    if ("${ARG_TYPE}" STREQUAL "COMBO")
        if ("${ARG_OPTIONS}" STREQUAL "")
            message(FATAL_ERROR "must set OPTIONS for add_option(${NAME}...")
        else()
            set_property(CACHE "${NAME}" PROPERTY STRINGS "${ARG_OPTIONS}")
        endif()
    endif()

    if (ARG_ADVANCED)
        mark_as_advanced(FORCE "${NAME}")
    endif()

endmacro()

