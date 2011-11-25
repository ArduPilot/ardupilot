# - Generate firmware and libraries for Arduino Devices
# generate_arduino_firmware(TARGET_NAME)
#        TARGET_NAME - Name of target
# Creates a Arduino firmware target.
#
# The target options can be configured by setting options of
# the following format:
#      ${TARGET_NAME}${SUFFIX}
# The following suffixes are availabe:
#      _SRCS           # Sources
#      _HDRS           # Headers
#      _SKETCHES       # Arduino sketch files
#      _LIBS           # Libraries to linked in
#      _BOARD          # Board name (such as uno, mega2560, ...)
#      _PORT           # Serial port, for upload and serial targets [OPTIONAL]
#      _AFLAGS         # Override global Avrdude flags for target
#      _SERIAL         # Serial command for serial target           [OPTIONAL]
#      _NO_AUTOLIBS    # Disables Arduino library detection
# Here is a short example for a target named test:
#       set(test_SRCS  test.cpp)
#       set(test_HDRS  test.h)
#       set(test_BOARD uno)
#    
#       generate_arduino_firmware(test)
#
#
# generate_arduino_library(TARGET_NAME)
#        TARGET_NAME - Name of target
# Creates a Arduino firmware target.
#
# The target options can be configured by setting options of
# the following format:
#      ${TARGET_NAME}${SUFFIX}
# The following suffixes are availabe:
#
#      _SRCS           # Sources
#      _HDRS           # Headers
#      _LIBS           # Libraries to linked in
#      _BOARD          # Board name (such as uno, mega2560, ...)
#      _NO_AUTOLIBS    # Disables Arduino library detection
# 
# Here is a short example for a target named test:
#       set(test_SRCS  test.cpp)
#       set(test_HDRS  test.h)
#       set(test_BOARD uno)
#    
#       generate_arduino_library(test)


find_path(ARDUINO_SDK_PATH
          NAMES lib/version.txt hardware libraries
          PATH_SUFFIXES  share/arduino
          DOC "Arduino Development Kit path.")


# load_board_settings()
#
# Load the Arduino SDK board settings from the boards.txt file.
#
function(LOAD_BOARD_SETTINGS)
    if(NOT ARDUINO_BOARDS AND ARDUINO_BOARDS_PATH)
    file(STRINGS ${ARDUINO_BOARDS_PATH} BOARD_SETTINGS)
    foreach(BOARD_SETTING ${BOARD_SETTINGS})
        if("${BOARD_SETTING}" MATCHES "^.*=.*")
            string(REGEX MATCH "^[^=]+" SETTING_NAME ${BOARD_SETTING})
            string(REGEX MATCH "[^=]+$" SETTING_VALUE ${BOARD_SETTING})
            string(REPLACE "." ";" SETTING_NAME_TOKENS ${SETTING_NAME})

            list(LENGTH SETTING_NAME_TOKENS SETTING_NAME_TOKENS_LEN)


            # Add Arduino to main list of arduino boards
            list(GET SETTING_NAME_TOKENS 0 BOARD_ID)
            list(FIND ARDUINO_BOARDS ${BOARD_ID} ARDUINO_BOARD_INDEX)
            if(ARDUINO_BOARD_INDEX LESS 0)
                list(APPEND ARDUINO_BOARDS ${BOARD_ID})
            endif()

            # Add setting to board settings list
            list(GET SETTING_NAME_TOKENS 1 BOARD_SETTING)
            list(FIND ${BOARD_ID}.SETTINGS ${BOARD_SETTING} BOARD_SETTINGS_LEN)
            if(BOARD_SETTINGS_LEN LESS 0)
                list(APPEND ${BOARD_ID}.SETTINGS ${BOARD_SETTING})
                set(${BOARD_ID}.SETTINGS ${${BOARD_ID}.SETTINGS}
                    CACHE INTERNAL "Arduino ${BOARD_ID} Board settings list")
            endif()

            set(ARDUINO_SETTING_NAME ${BOARD_ID}.${BOARD_SETTING})

            # Add sub-setting to board sub-settings list
            if(SETTING_NAME_TOKENS_LEN GREATER 2)
                list(GET SETTING_NAME_TOKENS 2 BOARD_SUBSETTING)
                list(FIND ${BOARD_ID}.${BOARD_SETTING}.SUBSETTINGS ${BOARD_SUBSETTING} BOARD_SUBSETTINGS_LEN)
                if(BOARD_SUBSETTINGS_LEN LESS 0)
                    list(APPEND ${BOARD_ID}.${BOARD_SETTING}.SUBSETTINGS ${BOARD_SUBSETTING})
                    set(${BOARD_ID}.${BOARD_SETTING}.SUBSETTINGS ${${BOARD_ID}.${BOARD_SETTING}.SUBSETTINGS}
                        CACHE INTERNAL "Arduino ${BOARD_ID} Board sub-settings list")
                endif()
                set(ARDUINO_SETTING_NAME ${ARDUINO_SETTING_NAME}.${BOARD_SUBSETTING})
            endif()

            # Save setting value
            set(${ARDUINO_SETTING_NAME} ${SETTING_VALUE} CACHE INTERNAL "Arduino ${BOARD_ID} Board setting")
            

        endif()
    endforeach()
    set(ARDUINO_BOARDS ${ARDUINO_BOARDS} CACHE STRING "List of detected Arduino Board configurations")
    mark_as_advanced(ARDUINO_BOARDS)
    endif()
endfunction()

# print_board_settings(ARDUINO_BOARD)
#
#        ARDUINO_BOARD - Board id
#
# Print the detected Arduino board settings.
#
function(PRINT_BOARD_SETTINGS ARDUINO_BOARD)
    if(${ARDUINO_BOARD}.SETTINGS)

        message(STATUS "Arduino ${ARDUINO_BOARD} Board:")
        foreach(BOARD_SETTING ${${ARDUINO_BOARD}.SETTINGS})
            if(${ARDUINO_BOARD}.${BOARD_SETTING})
                message(STATUS "   ${ARDUINO_BOARD}.${BOARD_SETTING}=${${ARDUINO_BOARD}.${BOARD_SETTING}}")
            endif()
            if(${ARDUINO_BOARD}.${BOARD_SETTING}.SUBSETTINGS)
                foreach(BOARD_SUBSETTING ${${ARDUINO_BOARD}.${BOARD_SETTING}.SUBSETTINGS})
                    if(${ARDUINO_BOARD}.${BOARD_SETTING}.${BOARD_SUBSETTING})
                        message(STATUS "   ${ARDUINO_BOARD}.${BOARD_SETTING}.${BOARD_SUBSETTING}=${${ARDUINO_BOARD}.${BOARD_SETTING}.${BOARD_SUBSETTING}}")
                    endif()
                endforeach()
            endif()
            message(STATUS "")
        endforeach()
    endif()
endfunction()



# generate_arduino_library(TARGET_NAME)
#
# see documentation at top
function(GENERATE_ARDUINO_LIBRARY TARGET_NAME)
    load_generator_settings(${TARGET_NAME} INPUT _SRCS       # Sources
                                                 _HDRS       # Headers
                                                 _LIBS       # Libraries to linked in
                                                 _BOARD)     # Board name (such as uno, mega2560, ...)
    set(INPUT_AUTOLIBS True)
    if(DEFINED ${TARGET_NAME}_NO_AUTOLIBS AND ${TARGET_NAME}_NO_AUTOLIBS)
        set(INPUT_AUTOLIBS False)
    endif()

    message(STATUS "Generating ${TARGET_NAME}")
    
    set(ALL_LIBS)
    set(ALL_SRCS ${INPUT_SRCS} ${INPUT_HDRS})

    setup_arduino_compiler(${INPUT_BOARD})
    setup_arduino_core(CORE_LIB ${INPUT_BOARD})

    if(INPUT_AUTOLIBS)
        setup_arduino_libraries(ALL_LIBS  ${INPUT_BOARD} "${ALL_SRCS}")
    endif()

    list(APPEND ALL_LIBS ${CORE_LIB} ${INPUT_LIBS})
        
    add_library(${TARGET_NAME} ${ALL_SRCS})
    target_link_libraries(${TARGET_NAME} ${ALL_LIBS})
endfunction()

# generate_arduino_firmware(TARGET_NAME)
#
# see documentation at top
function(GENERATE_ARDUINO_FIRMWARE TARGET_NAME)
    load_generator_settings(${TARGET_NAME} INPUT _SRCS       # Sources
                                                 _HDRS       # Headers
                                                 _LIBS       # Libraries to linked in
                                                 _BOARD      # Board name (such as uno, mega2560, ...)
                                                 _PORT       # Serial port, for upload and serial targets
                                                 _AFLAGS     # Override global Avrdude flags for target
                                                 _SKETCHES   # Arduino sketch files
                                                 _SERIAL)    # Serial command for serial target
                            
    set(INPUT_AUTOLIBS True)
    if(DEFINED ${TARGET_NAME}_NO_AUTOLIBS AND ${TARGET_NAME}_NO_AUTOLIBS)
        set(INPUT_AUTOLIBS False)
    endif()

    message(STATUS "Generating ${TARGET_NAME}")

    set(ALL_LIBS)
    set(ALL_SRCS ${INPUT_SRCS} ${INPUT_HDRS})

    setup_arduino_compiler(${INPUT_BOARD})
    setup_arduino_core(CORE_LIB ${INPUT_BOARD})

    #setup_arduino_sketch(SKETCH_SRCS ${INPUT_SKETCHES})

    if(INPUT_AUTOLIBS)
        setup_arduino_libraries(ALL_LIBS ${INPUT_BOARD} "${ALL_SRCS}")
    endif()

    
    list(APPEND ALL_LIBS ${CORE_LIB} ${INPUT_LIBS})
    
    setup_arduino_target(${TARGET_NAME} "${ALL_SRCS}" "${ALL_LIBS}")
    
    if(INPUT_PORT)
        setup_arduino_upload(${INPUT_BOARD} ${TARGET_NAME} ${INPUT_PORT})
    endif()
    
    if(INPUT_SERIAL)
        setup_serial_target(${TARGET_NAME} "${INPUT_SERIAL}")
    endif()
endfunction()


# load_generator_settings(TARGET_NAME PREFIX [SUFFIX_1 SUFFIX_2 .. SUFFIX_N])
#
#         TARGET_NAME - The base name of the user settings
#         PREFIX      - The prefix name used for generator settings
#         SUFFIX_XX   - List of suffixes to load
#
#  Loads a list of user settings into the generators scope. User settings have
#  the following syntax:
#
#      ${BASE_NAME}${SUFFIX}
#
#  The BASE_NAME is the target name and the suffix is a specific generator settings.
#
#  For every user setting found a generator setting is created of the follwoing fromat:
#
#      ${PREFIX}${SUFFIX}
#
#  The purpose of loading the settings into the generator is to not modify user settings
#  and to have a generic naming of the settings within the generator.
#
function(LOAD_GENERATOR_SETTINGS TARGET_NAME PREFIX)
    foreach(GEN_SUFFIX ${ARGN})
        if(${TARGET_NAME}${GEN_SUFFIX})
            set(${PREFIX}${GEN_SUFFIX} ${${TARGET_NAME}${GEN_SUFFIX}} PARENT_SCOPE)
        endif()
    endforeach()
endfunction()

# setup_arduino_compiler(BOARD_ID)
#
#       BOARD_ID - The board id name
#
# Configures the the build settings for the specified Arduino Board.
#
macro(setup_arduino_compiler BOARD_ID)
    set(BOARD_CORE ${${BOARD_ID}.build.core})
    if(BOARD_CORE)
        set(BOARD_CORE_PATH ${ARDUINO_CORES_PATH}/${BOARD_CORE})
        include_directories(${BOARD_CORE_PATH})
        include_directories(${ARDUINO_LIBRARIES_PATH})
        add_definitions(-DF_CPU=${${BOARD_ID}.build.f_cpu}
                        -DARDUINO=${ARDUINO_SDK_VERSION}
                        -mmcu=${${BOARD_ID}.build.mcu}
                        )
        set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS}    -mmcu=${${BOARD_ID}.build.mcu}" PARENT_SCOPE)
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -mmcu=${${BOARD_ID}.build.mcu}" PARENT_SCOPE)
        set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -mmcu=${${BOARD_ID}.build.mcu}" PARENT_SCOPE)
    endif()
endmacro()

# setup_arduino_core(VAR_NAME BOARD_ID)
#
#        VAR_NAME    - Variable name that will hold the generated library name
#        BOARD_ID    - Arduino board id
#
# Creates the Arduino Core library for the specified board,
# each board gets it's own version of the library.
#
function(setup_arduino_core VAR_NAME BOARD_ID)
    set(CORE_LIB_NAME ${BOARD_ID}_CORE)
    set(BOARD_CORE ${${BOARD_ID}.build.core})
    if(BOARD_CORE AND NOT TARGET ${CORE_LIB_NAME})
        set(BOARD_CORE_PATH ${ARDUINO_CORES_PATH}/${BOARD_CORE})
        find_sources(CORE_SRCS ${BOARD_CORE_PATH} True)

        # Debian/Ubuntu fix
        list(REMOVE_ITEM CORE_SRCS "${BOARD_CORE_PATH}/main.cxx")

        add_library(${CORE_LIB_NAME} ${CORE_SRCS})
        set(${VAR_NAME} ${CORE_LIB_NAME} PARENT_SCOPE)
    endif()
endfunction()

# find_arduino_libraries(VAR_NAME SRCS)
#
#      VAR_NAME - Variable name which will hold the results
#      SRCS     - Sources that will be analized
#
#     returns a list of paths to libraries found.
#
#  Finds all Arduino type libraries included in sources. Available libraries
#  are ${ARDUINO_SDK_PATH}/libraries and ${CMAKE_CURRENT_SOURCE_DIR}.
#
#  A Arduino library is a folder that has the same name as the include header.
#  For example, if we have a include "#include <LibraryName.h>" then the following
#  directory structure is considered a Arduino library:
#
#     LibraryName/
#          |- LibraryName.h
#          `- LibraryName.c
#
#  If such a directory is found then all sources within that directory are considred
#  to be part of that Arduino library.
#
function(find_arduino_libraries VAR_NAME SRCS)
    set(ARDUINO_LIBS )
    foreach(SRC ${SRCS})
        file(STRINGS ${SRC} SRC_CONTENTS)
        foreach(SRC_LINE ${SRC_CONTENTS})
            if("${SRC_LINE}" MATCHES "^ *#include *[<\"](.*)[>\"]")
                get_filename_component(INCLUDE_NAME ${CMAKE_MATCH_1} NAME_WE)
                get_property(LIBRARY_SEARCH_PATH
                             DIRECTORY     # Property Scope
                             PROPERTY LINK_DIRECTORIES)
                foreach(LIB_SEARCH_PATH ${LIBRARY_SEARCH_PATH} ${ARDUINO_LIBRARIES_PATH} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/libraries)
                    if(EXISTS ${LIB_SEARCH_PATH}/${INCLUDE_NAME}/${CMAKE_MATCH_1})
                        list(APPEND ARDUINO_LIBS ${LIB_SEARCH_PATH}/${INCLUDE_NAME})
                        break()
                    endif()
                endforeach()
            endif()
        endforeach()
    endforeach()
    if(ARDUINO_LIBS)
        list(REMOVE_DUPLICATES ARDUINO_LIBS)
    endif()
    set(${VAR_NAME} ${ARDUINO_LIBS} PARENT_SCOPE)
endfunction()

# setup_arduino_library(VAR_NAME BOARD_ID LIB_PATH)
#
#        VAR_NAME    - Vairable wich will hold the generated library names
#        BOARD_ID    - Board name
#        LIB_PATH    - path of the library
#
# Creates an Arduino library, with all it's library dependencies.
#
#      ${LIB_NAME}_RECURSE controls if the library will recurse
#      when looking for source files.
#

# For known libraries can list recurse here
set(Wire_RECURSE True)
set(Ethernet_RECURSE True)
function(setup_arduino_library VAR_NAME BOARD_ID LIB_PATH)
    set(LIB_TARGETS)

    get_filename_component(LIB_NAME ${LIB_PATH} NAME)
    set(TARGET_LIB_NAME ${BOARD_ID}_${LIB_NAME})
    if(NOT TARGET ${TARGET_LIB_NAME})
        string(REGEX REPLACE ".*/" "" LIB_SHORT_NAME ${LIB_NAME})

        # Detect if recursion is needed
        if (NOT DEFINED ${LIB_SHORT_NAME}_RECURSE)
            set(${LIB_SHORT_NAME}_RECURSE False)
        endif()

        find_sources(LIB_SRCS ${LIB_PATH} ${${LIB_SHORT_NAME}_RECURSE})
        if(LIB_SRCS)

            message(STATUS "Generating Arduino ${LIB_NAME} library")
            include_directories(${LIB_PATH} ${LIB_PATH}/utility)
            add_library(${TARGET_LIB_NAME} STATIC ${LIB_SRCS})

            find_arduino_libraries(LIB_DEPS "${LIB_SRCS}")
            foreach(LIB_DEP ${LIB_DEPS})
                setup_arduino_library(DEP_LIB_SRCS ${BOARD_ID} ${LIB_DEP})
                list(APPEND LIB_TARGETS ${DEP_LIB_SRCS})
            endforeach()

            target_link_libraries(${TARGET_LIB_NAME} ${BOARD_ID}_CORE ${LIB_TARGETS})
            list(APPEND LIB_TARGETS ${TARGET_LIB_NAME})
        endif()
    else()
        # Target already exists, skiping creating
        include_directories(${LIB_PATH} ${LIB_PATH}/utility)
        list(APPEND LIB_TARGETS ${TARGET_LIB_NAME})
    endif()
    if(LIB_TARGETS)
        list(REMOVE_DUPLICATES LIB_TARGETS)
    endif()
    set(${VAR_NAME} ${LIB_TARGETS} PARENT_SCOPE)
endfunction()

# setup_arduino_libraries(VAR_NAME BOARD_ID SRCS)
#
#        VAR_NAME    - Vairable wich will hold the generated library names
#        BOARD_ID    - Board ID
#        SRCS        - source files
#
# Finds and creates all dependency libraries based on sources.
#
function(setup_arduino_libraries VAR_NAME BOARD_ID SRCS)
    set(LIB_TARGETS)
    find_arduino_libraries(TARGET_LIBS "${SRCS}")
    foreach(TARGET_LIB ${TARGET_LIBS})
        setup_arduino_library(LIB_DEPS ${BOARD_ID} ${TARGET_LIB}) # Create static library instead of returning sources
        list(APPEND LIB_TARGETS ${LIB_DEPS})
    endforeach()
    set(${VAR_NAME} ${LIB_TARGETS} PARENT_SCOPE)
endfunction()


# setup_arduino_target(TARGET_NAME ALL_SRCS ALL_LIBS)
#
#        TARGET_NAME - Target name
#        ALL_SRCS    - All sources
#        ALL_LIBS    - All libraries
#
# Creates an Arduino firmware target.
#
function(setup_arduino_target TARGET_NAME ALL_SRCS ALL_LIBS)
    add_executable(${TARGET_NAME} ${ALL_SRCS})
    target_link_libraries(${TARGET_NAME} ${ALL_LIBS})
    set_target_properties(${TARGET_NAME} PROPERTIES SUFFIX ".elf")

    set(TARGET_PATH ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME})
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                        COMMAND ${CMAKE_OBJCOPY}
                        ARGS     ${ARDUINO_OBJCOPY_EEP_FLAGS}
                                 ${TARGET_PATH}.elf
                                 ${TARGET_PATH}.eep
                        VERBATIM)
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                        COMMAND ${CMAKE_OBJCOPY}
                        ARGS    ${ARDUINO_OBJCOPY_HEX_FLAGS}
                                ${TARGET_PATH}.elf
                                ${TARGET_PATH}.hex
                        VERBATIM)
endfunction()

# setup_arduino_upload(BOARD_ID TARGET_NAME PORT)
#
#        BOARD_ID    - Arduino board id
#        TARGET_NAME - Target name
#        PORT        - Serial port for upload
#
# Create an upload target (${TARGET_NAME}-upload) for the specified Arduino target.
#
function(setup_arduino_upload BOARD_ID TARGET_NAME PORT)
    set(AVRDUDE_FLAGS ${ARDUINO_AVRDUDE_FLAGS})
    if(DEFINED ${TARGET_NAME}_AFLAGS)
        set(AVRDUDE_FLAGS ${${TARGET_NAME}_AFLAGS})
    endif()
    if (${${BOARD_ID}.upload.protocol} STREQUAL "stk500") 
        set(${BOARD_ID}.upload.protocol "stk500v1")
    endif()
    add_custom_target(${TARGET_NAME}-upload
                     ${ARDUINO_AVRDUDE_PROGRAM}
                        ${AVRDUDE_FLAGS}
                         -C${ARDUINO_AVRDUDE_CONFIG_PATH}
                         -p${${BOARD_ID}.build.mcu} 
                         -c${${BOARD_ID}.upload.protocol} 
                         -P${PORT} -b${${BOARD_ID}.upload.speed}
                         #-D
                         -Uflash:w:${CMAKE_BINARY_DIR}/${TARGET_NAME}.hex:i
                         DEPENDS ${TARGET_NAME})
    if(NOT TARGET upload)
        add_custom_target(upload)
    endif()
    add_dependencies(upload ${TARGET_NAME}-upload)
endfunction()

# find_sources(VAR_NAME LIB_PATH RECURSE)
#
#        VAR_NAME - Variable name that will hold the detected sources
#        LIB_PATH - The base path
#        RECURSE  - Whether or not to recurse
#
# Finds all C/C++ sources located at the specified path.
#
function(find_sources VAR_NAME LIB_PATH RECURSE)
    set(FILE_SEARCH_LIST
        ${LIB_PATH}/*.cpp
        ${LIB_PATH}/*.c
        ${LIB_PATH}/*.cc
        ${LIB_PATH}/*.cxx
        ${LIB_PATH}/*.h
        ${LIB_PATH}/*.hh
        ${LIB_PATH}/*.hxx)

    if(RECURSE)
        file(GLOB_RECURSE LIB_FILES ${FILE_SEARCH_LIST})
    else()
        file(GLOB LIB_FILES ${FILE_SEARCH_LIST})
    endif()

    if(LIB_FILES)
        set(${VAR_NAME} ${LIB_FILES} PARENT_SCOPE)
    endif()
endfunction()

# setup_serial_target(TARGET_NAME CMD)
#
#         TARGET_NAME - Target name
#         CMD         - Serial terminal command
#
# Creates a target (${TARGET_NAME}-serial) for launching the serial termnial.
#
function(setup_serial_target TARGET_NAME CMD)
    string(CONFIGURE "${CMD}" FULL_CMD @ONLY)
    add_custom_target(${TARGET_NAME}-serial
                      ${FULL_CMD})
endfunction()


# detect_arduino_version(VAR_NAME)
#
#       VAR_NAME - Variable name where the detected version will be saved
#
# Detects the Arduino SDK Version based on the revisions.txt file.
#
function(detect_arduino_version VAR_NAME)
    if(ARDUINO_VERSION_PATH)
        file(READ ${ARDUINO_VERSION_PATH} ARD_VERSION)
        if("${ARD_VERSION}" MATCHES " *[0]+([0-9]+)")
            set(${VAR_NAME} ${CMAKE_MATCH_1} PARENT_SCOPE)
        endif()
    endif()
endfunction()


function(convert_arduino_sketch VAR_NAME SRCS)
endfunction()


# Setting up Arduino enviroment settings
find_file(ARDUINO_CORES_PATH
    NAMES cores
    PATHS ${ARDUINO_SDK_PATH}
    PATH_SUFFIXES hardware/arduino
    NO_DEFAULT_PATH) 

find_file(ARDUINO_LIBRARIES_PATH
    NAMES libraries
    PATHS ${ARDUINO_SDK_PATH}
    NO_DEFAULT_PATH) 

find_file(ARDUINO_BOARDS_PATH
    NAMES boards.txt
    PATHS ${ARDUINO_SDK_PATH}
    PATH_SUFFIXES hardware/arduino
    NO_DEFAULT_PATH) 

find_file(ARDUINO_PROGRAMMERS_PATH
    NAMES programmers.txt
    PATHS ${ARDUINO_SDK_PATH}
    PATH_SUFFIXES hardware/arduino
    NO_DEFAULT_PATH) 

find_file(ARDUINO_REVISIONS_PATH
    NAMES revisions.txt
    PATHS ${ARDUINO_SDK_PATH}
    NO_DEFAULT_PATH) 

find_file(ARDUINO_VERSION_PATH
    NAMES lib/version.txt
    PATHS ${ARDUINO_SDK_PATH}
    NO_DEFAULT_PATH) 

find_program(ARDUINO_AVRDUDE_PROGRAM
    NAMES avrdude
    PATHS ${ARDUINO_SDK_PATH}
    PATH_SUFFIXES hardware/tools
    NO_DEFAULT_PATH) 

find_file(ARDUINO_AVRDUDE_CONFIG_PATH
    NAMES avrdude.conf
    PATHS ${ARDUINO_SDK_PATH} /etc/avrdude
    PATH_SUFFIXES hardware/tools
                  hardware/tools/avr/etc
    NO_DEFAULT_PATH)

set(ARDUINO_OBJCOPY_EEP_FLAGS -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0
    CACHE STRING "")
set(ARDUINO_OBJCOPY_HEX_FLAGS -O ihex -R .eeprom
    CACHE STRING "")
set(ARDUINO_AVRDUDE_FLAGS -V -F
    CACHE STRING "Arvdude global flag list.")

if(ARDUINO_SDK_PATH)
    detect_arduino_version(ARDUINO_SDK_VERSION)
    set(ARDUINO_SDK_VERSION ${ARDUINO_SDK_VERSION} CACHE STRING "Arduino SDK Version")
endif(ARDUINO_SDK_PATH)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Arduino
                                  REQUIRED_VARS ARDUINO_SDK_PATH
                                                ARDUINO_SDK_VERSION
                                  VERSION_VAR ARDUINO_SDK_VERSION)


mark_as_advanced(ARDUINO_CORES_PATH
    ARDUINO_SDK_VERSION
    ARDUINO_LIBRARIES_PATH
    ARDUINO_BOARDS_PATH
    ARDUINO_PROGRAMMERS_PATH
    ARDUINO_REVISIONS_PATH
    ARDUINO_AVRDUDE_PROGRAM
    ARDUINO_AVRDUDE_CONFIG_PATH
    ARDUINO_OBJCOPY_EEP_FLAGS
    ARDUINO_OBJCOPY_HEX_FLAGS)
load_board_settings()
