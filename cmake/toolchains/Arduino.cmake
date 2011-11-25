set(CMAKE_SYSTEM_NAME Arduino)

set(CMAKE_C_COMPILER   avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)

#=============================================================================#
#                              C Flags                                        #
#=============================================================================#
if (NOT DEFINED ARDUINO_C_FLAGS)
    set(ARDUINO_C_FLAGS "-ffunction-sections -fdata-sections")
endif()
set(CMAKE_C_FLAGS                "-g -Os       ${ARDUINO_C_FLAGS}" CACHE STRING "")
set(CMAKE_C_FLAGS_DEBUG          "-g           ${ARDUINO_C_FLAGS}" CACHE STRING "")
set(CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG ${ARDUINO_C_FLAGS}" CACHE STRING "")
set(CMAKE_C_FLAGS_RELEASE        "-0s -DNDEBUG -w ${ARDUINO_C_FLAGS}" CACHE STRING "")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-0s -g       -w ${ARDUINO_C_FLAGS}" CACHE STRING "")

#=============================================================================#
#                             C++ Flags                                       #
#=============================================================================#
if (NOT DEFINED ARDUINO_CXX_FLAGS)
    set(ARDUINO_CXX_FLAGS "${ARDUINO_C_FLAGS} -fno-exceptions")
endif()
set(CMAKE_CXX_FLAGS                "-g -Os       ${ARDUINO_CXX_FLAGS}" CACHE STRING "")
set(CMAKE_CXX_FLAGS_DEBUG          "-g           ${ARDUINO_CXX_FLAGS}" CACHE STRING "")
set(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG ${ARDUINO_CXX_FLAGS}" CACHE STRING "")
set(CMAKE_CXX_FLAGS_RELEASE        "-0s -DNDEBUG ${ARDUINO_CXX_FLAGS}" CACHE STRING "")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-0s -g       ${ARDUINO_CXX_FLAGS}" CACHE STRING "")

#=============================================================================#
#                       Executable Linker Flags                               #
#=============================================================================#
if (NOT DEFINED ARDUINO_LINKER_FLAGS)
    set(ARDUINO_LINKER_FLAGS "-Wl,--gc-sections")
endif()
set(CMAKE_EXE_LINKER_FLAGS                "${ARDUINO_LINKER_FLAGS}" CACHE STRING "")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG          "${ARDUINO_LINKER_FLAGS}" CACHE STRING "")
set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL     "${ARDUINO_LINKER_FLAGS}" CACHE STRING "")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE        "${ARDUINO_LINKER_FLAGS}" CACHE STRING "")
set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${ARDUINO_LINKER_FLAGS}" CACHE STRING "")

#=============================================================================#
#                       Shared Lbrary Linker Flags                            #
#=============================================================================#
set(CMAKE_SHARED_LINKER_FLAGS                ""                        CACHE STRING "")
set(CMAKE_SHARED_LINKER_FLAGS_DEBUG          ""                        CACHE STRING "")
set(CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL     ""                        CACHE STRING "")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE        ""                        CACHE STRING "")
set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO ""                        CACHE STRING "")

set(CMAKE_MODULE_LINKER_FLAGS                ""                        CACHE STRING "")
set(CMAKE_MODULE_LINKER_FLAGS_DEBUG          ""                        CACHE STRING "")
set(CMAKE_MODULE_LINKER_FLAGS_MINSIZEREL     ""                        CACHE STRING "")
set(CMAKE_MODULE_LINKER_FLAGS_RELEASE        ""                        CACHE STRING "")
set(CMAKE_MODULE_LINKER_FLAGS_RELWITHDEBINFO ""                        CACHE STRING "")




set(ARDUINO_PATHS)
foreach(VERSION RANGE 22 1)
    list(APPEND ARDUINO_PATHS arduino-00${VERSION})
endforeach()

find_path(ARDUINO_SDK_PATH
          NAMES lib/version.txt
          PATH_SUFFIXES share/arduino
                        Arduino.app/Contents/Resources/Java/
                        ${ARDUINO_PATHS}
          DOC "Arduino Development Kit path.")

include(Platform/ArduinoPaths)
