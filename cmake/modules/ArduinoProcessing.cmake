# 1. Concatenate all PDE files
# 2. Write #include "WProgram.h"
# 3. Write prototypes
# 4. Write original sources
# 
# 
# Prefix Writer
# 1. Scrub comments
# 2. Optionally subsitute Unicode
# 3. Find imports
# 4. Find prototypes
# 
# Find prototypes
# 1. Strip comments, quotes, preprocessor directives
# 2. Collapse braches
# 3. Regex


set(SINGLE_QUOTES_REGEX  "('.')")
set(DOUBLE_QUOTES_REGEX  "(\"([^\"\\\\]|\\\\.)*\")")
set(SINGLE_COMMENT_REGEX "([ ]*//[^\n]*)")
set(MULTI_COMMENT_REGEX  "(/[*][^/]*[*]/)")
set(PREPROC_REGEX        "([ ]*#(\\\\[\n]|[^\n])*)")

#"[\w\[\]\*]+\s+[&\[\]\*\w\s]+\([&,\[\]\*\w\s]*\)(?=\s*\{)"
set(PROTOTPYE_REGEX      "([a-zA-Z0-9]+[ ]*)*[a-zA-Z0-9]+[ ]*\([^{]*\)[ ]*{")

function(READ_SKETCHES VAR_NAME )
    set(SKETCH_SOURCE)
    foreach(SKETCH ${ARGN})
        if(EXISTS ${SKETCH})
            message(STATUS "${SKETCH}")
            file(READ ${SKETCH} SKETCH_CONTENTS)
            set(SKETCH_SOURCE "${SKETCH_SOURCE}\n${SKETCH_CONTENTS}")
        else()
            message(FATAL_ERROR "Sketch file does not exist: ${SKETCH}")
        endif()
    endforeach()
    set(${VAR_NAME} "${SKETCH_SOURCE}" PARENT_SCOPE)
endfunction()

function(STRIP_SOURCES VAR_NAME SOURCES)
    string(REGEX REPLACE "${SINGLE_QUOTES_REGEX}|${DOUBLE_QUOTES_REGEX}|${SINGLE_COMMENT_REGEX}|${MULTI_COMMENT_REGEX}|${PREPROC_REGEX}"
                         ""
                         SOURCES
                         "${SOURCES}")
    set(${VAR_NAME} "${SOURCES}" PARENT_SCOPE)
endfunction()

function(COLLAPSE_BRACES VAR_NAME SOURCES)
    set(PARSED_SOURCES)
    string(LENGTH "${SOURCES}" SOURCES_LENGTH)
    math(EXPR SOURCES_LENGTH "${SOURCES_LENGTH}-1")

    set(NESTING 0)
    set(START   0)
    foreach(INDEX RANGE ${SOURCES_LENGTH})
        string(SUBSTRING "${SOURCES}" ${INDEX} 1 CURRENT_CHAR)
        #message("${CURRENT_CHAR}")
        if(CURRENT_CHAR STREQUAL "{")
            if(NESTING EQUAL 0)
                math(EXPR SUBLENGTH "${INDEX}-${START} +1")
                string(SUBSTRING "${SOURCES}" ${START} ${SUBLENGTH} CURRENT_CHUNK)
                set(PARSED_SOURCES "${PARSED_SOURCES}${CURRENT_CHUNK}")
                #message("INDEX: ${INDEX} START: ${START} LENGTH: ${SUBLENGTH}")
            endif()
            math(EXPR NESTING "${NESTING}+1")
        elseif(CURRENT_CHAR STREQUAL "}")
            math(EXPR NESTING "${NESTING}-1")
            if(NESTING EQUAL 0)
                set(START ${INDEX})
            endif()
        endif()
    endforeach()

    math(EXPR SUBLENGTH "${SOURCES_LENGTH}-${START} +1")
    string(SUBSTRING "${SOURCES}" ${START} ${SUBLENGTH} CURRENT_CHUNK)
    set(PARSED_SOURCES "${PARSED_SOURCES}${CURRENT_CHUNK}")

    set(${VAR_NAME} "${PARSED_SOURCES}" PARENT_SCOPE)
endfunction()

function(extract_prototypes VAR_NAME SOURCES)
    string(REGEX MATCHALL "${PROTOTPYE_REGEX}"
                         SOURCES
                         "${SOURCES}")
    set(${VAR_NAME} "${SOURCES}" PARENT_SCOPE)
endfunction()

read_sketches(SKETCH_SOURCE ${FILES})
strip_sources(SKETCH_SOURCE "${SKETCH_SOURCE}")
collapse_braces(SKETCH_SOURCE "${SKETCH_SOURCE}")
extract_prototypes(SKETCH_SOURCE "${SKETCH_SOURCE}")




message("===============")
foreach(ENTRY ${SKETCH_SOURCE})
    message("START]]]${ENTRY}[[[END")
endforeach()
message("===============")
#message("${SKETCH_SOURCE}")
