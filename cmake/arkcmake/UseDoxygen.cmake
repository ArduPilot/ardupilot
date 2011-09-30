# - Run Doxygen
#
# Adds a doxygen target that runs doxygen to generate the html
# and optionally the LaTeX API documentation.
# The doxygen target is added to the doc target as dependency.
# i.e.: the API documentation is built with:
#  make doc
#
# USAGE: INCLUDE IN PROJECT
#
#  set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
#  include(UseDoxygen)
# Add the Doxyfile.in and UseDoxygen.cmake files to the projects source directory.
#
#
# Variables you may define are:
#  DOXYFILE_OUTPUT_DIR - Path where the Doxygen output is stored. Defaults to "doc".
#
#  DOXYFILE_LATEX_DIR - Directory where the Doxygen LaTeX output is stored. Defaults to "latex".
#
#  DOXYFILE_HTML_DIR - Directory where the Doxygen html output is stored. Defaults to "html".
#

#
#  Copyright (c) 2009-2010 Tobias Rautenkranz <tobias@rautenkranz.ch>
#  Copyright (c) 2010      Andreas Schneider <mail@cynapses.org>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

macro(usedoxygen_set_default name value)
    if(NOT DEFINED "${name}")
        set("${name}" "${value}")
    endif()
endmacro()

find_package(Doxygen)

if(DOXYGEN_FOUND)
    find_file(DOXYFILE_IN
        NAMES
            doxy.config.in
        PATHS
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_ROOT}/Modules/
        NO_DEFAULT_PATH)
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(DOXYFILE_IN DEFAULT_MSG "DOXYFILE_IN")
endif()

if(DOXYGEN_FOUND AND DOXYFILE_IN_FOUND)
    add_custom_target(doxygen ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxy.config)

    usedoxygen_set_default(DOXYFILE_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    usedoxygen_set_default(DOXYFILE_HTML_DIR "html")

    set_property(DIRECTORY APPEND PROPERTY
            ADDITIONAL_MAKE_CLEAN_FILES "${DOXYFILE_OUTPUT_DIR}/${DOXYFILE_HTML_DIR}")

    set(DOXYFILE_LATEX FALSE)
    set(DOXYFILE_PDFLATEX FALSE)
    set(DOXYFILE_DOT FALSE)

    find_package(LATEX)
    if(LATEX_COMPILER AND MAKEINDEX_COMPILER)
        set(DOXYFILE_LATEX TRUE)
        usedoxygen_set_default(DOXYFILE_LATEX_DIR "latex")

        set_property(DIRECTORY APPEND PROPERTY
                ADDITIONAL_MAKE_CLEAN_FILES
                "${DOXYFILE_OUTPUT_DIR}/${DOXYFILE_LATEX_DIR}")

        if(PDFLATEX_COMPILER)
            set(DOXYFILE_PDFLATEX TRUE)
        endif()
        if(DOXYGEN_DOT_EXECUTABLE)
            set(DOXYFILE_DOT TRUE)
        endif()

        add_custom_command(TARGET doxygen
            POST_BUILD
            COMMAND ${CMAKE_MAKE_PROGRAM}
            WORKING_DIRECTORY "${DOXYFILE_OUTPUT_DIR}/${DOXYFILE_LATEX_DIR}")
    endif()

    configure_file(${DOXYFILE_IN} ${CMAKE_CURRENT_BINARY_DIR}/doxy.config ESCAPE_QUOTES IMMEDIATE @ONLY)
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/doxy.trac.in)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxy.trac.in ${CMAKE_CURRENT_BINARY_DIR}/doxy.trac ESCAPE_QUOTES IMMEDIATE @ONLY)
        add_custom_target(doxygen-trac ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxy.trac)
    endif()

    get_target_property(DOC_TARGET doc TYPE)
    if(NOT DOC_TARGET)
        add_custom_target(doc)
    endif()

    add_dependencies(doc doxygen)
endif()
