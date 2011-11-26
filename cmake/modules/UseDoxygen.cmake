# - Run Doxygen
#
# Adds a doxygen target that runs doxygen to generate the html
# and optionally the LaTeX API documentation.
# The doxygen target is added to the doc target as a dependency.
# i.e.: the API documentation is built with:
#  make doc
#
# USAGE: GLOBAL INSTALL
#
# Install it with:
#  cmake ./ && sudo make install
# Add the following to the CMakeLists.txt of your project:
#  include(UseDoxygen OPTIONAL)
# Optionally copy Doxyfile.in in the directory of CMakeLists.txt and edit it.
#
# USAGE: INCLUDE IN PROJECT
#
#  set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
#  include(UseDoxygen)
# Add the Doxyfile.in and UseDoxygen.cmake files to the projects source directory.
#
#
# CONFIGURATION
#
# To configure Doxygen you can edit Doxyfile.in and set some variables in cmake.
# Variables you may define are:
#  DOXYFILE_SOURCE_DIR - Path where the Doxygen input files are.
#  	Defaults to the current source directory.
#  DOXYFILE_EXTRA_SOURCES - Additional source diretories/files for Doxygen to scan.
#  	The Paths should be in double quotes and separated by space. e.g.:
#  	 "${CMAKE_CURRENT_BINARY_DIR}/foo.c" "${CMAKE_CURRENT_BINARY_DIR}/bar/"
#  
#  DOXYFILE_OUTPUT_DIR - Path where the Doxygen output is stored.
#  	Defaults to "${CMAKE_CURRENT_BINARY_DIR}/doc".
#  
#  DOXYFILE_LATEX - ON/OFF; Set to "ON" if you want the LaTeX documentation
#  	to be built.
#  DOXYFILE_LATEX_DIR - Directory relative to DOXYFILE_OUTPUT_DIR where
#  	the Doxygen LaTeX output is stored. Defaults to "latex".
#  
#  DOXYFILE_HTML_DIR - Directory relative to DOXYFILE_OUTPUT_DIR where
#  	the Doxygen html output is stored. Defaults to "html".
#

#
#  Copyright (c) 2009, 2010, 2011 Tobias Rautenkranz <tobias@rautenkranz.ch>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

macro(usedoxygen_set_default name value type docstring)
	if(NOT DEFINED "${name}")
		set("${name}" "${value}" CACHE "${type}" "${docstring}")
	endif()
endmacro()

find_package(Doxygen)

if(DOXYGEN_FOUND)
	find_file(DOXYFILE_IN "Doxyfile.in"
			PATHS "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_ROOT}/Modules/"
			NO_DEFAULT_PATH
			DOC "Path to the doxygen configuration template file")
	set(DOXYFILE "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")
	include(FindPackageHandleStandardArgs)
	find_package_handle_standard_args(DOXYFILE_IN DEFAULT_MSG "DOXYFILE_IN")
endif()

if(DOXYGEN_FOUND AND DOXYFILE_IN_FOUND)
	usedoxygen_set_default(DOXYFILE_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/doc"
		PATH "Doxygen output directory")
	usedoxygen_set_default(DOXYFILE_HTML_DIR "html"
		STRING "Doxygen HTML output directory")
	usedoxygen_set_default(DOXYFILE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
		PATH "Input files source directory")
	usedoxygen_set_default(DOXYFILE_EXTRA_SOURCE_DIRS ""
		STRING "Additional source files/directories separated by space")
	set(DOXYFILE_SOURCE_DIRS "\"${DOXYFILE_SOURCE_DIR}\" ${DOXYFILE_EXTRA_SOURCES}")

	usedoxygen_set_default(DOXYFILE_LATEX YES BOOL "Generate LaTeX API documentation" OFF)
	usedoxygen_set_default(DOXYFILE_LATEX_DIR "latex" STRING "LaTex output directory")

	mark_as_advanced(DOXYFILE_OUTPUT_DIR DOXYFILE_HTML_DIR DOXYFILE_LATEX_DIR
		DOXYFILE_SOURCE_DIR DOXYFILE_EXTRA_SOURCE_DIRS DOXYFILE_IN)


	set_property(DIRECTORY 
		APPEND PROPERTY
		ADDITIONAL_MAKE_CLEAN_FILES
		"${DOXYFILE_OUTPUT_DIR}/${DOXYFILE_HTML_DIR}")

	add_custom_target(doxygen
		COMMAND "${DOXYGEN_EXECUTABLE}"
			"${DOXYFILE}" 
		COMMENT "Writing documentation to ${DOXYFILE_OUTPUT_DIR}..."
		WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

	set(DOXYFILE_DOT "NO")
	if(DOXYGEN_DOT_EXECUTABLE)
		set(DOXYFILE_DOT "YES")
	endif()

	## LaTeX
	set(DOXYFILE_PDFLATEX "NO")

	set_property(DIRECTORY APPEND PROPERTY
		ADDITIONAL_MAKE_CLEAN_FILES
		"${DOXYFILE_OUTPUT_DIR}/${DOXYFILE_LATEX_DIR}")

	if(DOXYFILE_LATEX STREQUAL "ON")
		set(DOXYFILE_GENERATE_LATEX "YES")
		find_package(LATEX)
		find_program(DOXYFILE_MAKE make)
		mark_as_advanced(DOXYFILE_MAKE)
		if(LATEX_COMPILER AND MAKEINDEX_COMPILER AND DOXYFILE_MAKE)
			if(PDFLATEX_COMPILER)
				set(DOXYFILE_PDFLATEX "YES")
			endif()

			add_custom_command(TARGET doxygen
				POST_BUILD
				COMMAND "${DOXYFILE_MAKE}"
				COMMENT	"Running LaTeX for Doxygen documentation in ${DOXYFILE_OUTPUT_DIR}/${DOXYFILE_LATEX_DIR}..."
				WORKING_DIRECTORY "${DOXYFILE_OUTPUT_DIR}/${DOXYFILE_LATEX_DIR}")
		else()
			set(DOXYGEN_LATEX "NO")
		endif()
	else()
		set(DOXYFILE_GENERATE_LATEX "NO")
	endif()


	configure_file("${DOXYFILE_IN}" "${DOXYFILE}" @ONLY)

	get_target_property(DOC_TARGET doc TYPE)
	if(NOT DOC_TARGET)
		add_custom_target(doc)
	endif()

	add_dependencies(doc doxygen)
endif()
