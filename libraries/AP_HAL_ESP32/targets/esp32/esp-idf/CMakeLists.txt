cmake_minimum_required(VERSION 3.5)

set(CMAKE_TOOLCHAIN_FILE $ENV{IDF_PATH}/tools/cmake/toolchain-esp32.cmake CACHE STRING "")

project(ardupilot)

# Include for ESP-IDF build system functions
include($ENV{IDF_PATH}/tools/cmake/idf.cmake)

# Create idf::esp32 and idf::freertos static libraries
idf_build_process(esp32
    # try and trim the build; additional components
    # will be included as needed based on dependency tree
    #
    # although esptool_py does not generate static library,
    # processing the component is needed for flashing related
    # targets and file generation
    COMPONENTS  freertos
                fatfs
                nvs_flash
                esptool_py
                app_update
                esp_adc
                esp_driver_mcpwm
                driver
                lwip
                esp_wifi
                esp_system
                esp_rom
                esp_timer
                
    SDKCONFIG ${CMAKE_CURRENT_LIST_DIR}/sdkconfig
    SDKCONFIG_DEFAULTS ${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults
    BUILD_DIR ${CMAKE_BINARY_DIR})

set(elf_file ardupilot.elf)

add_executable(${elf_file} main.c)

if(NOT DEFINED ARDUPILOT_CMD)
    set(ARDUPILOT_CMD "none")
endif()

message("ARDUPILOT_CMD=${ARDUPILOT_CMD}")
message("WAF_BUILD_TARGET=${WAF_BUILD_TARGET}")

string(REGEX MATCH "^(examples|tool)/" IS_EXAMPLE "${WAF_BUILD_TARGET}")

if (IS_EXAMPLE)
    string(REPLACE "/" ";" A ${WAF_BUILD_TARGET})
    list(GET A 0 EXAMPLE_BASE)
    list(GET A 1 EXAMPLE_NAME)
    message("Building ${EXAMPLE_BASE} ${EXAMPLE_NAME}")
    target_link_libraries(${elf_file} "${ARDUPILOT_LIB}/${EXAMPLE_BASE}/lib${EXAMPLE_NAME}.a")
    target_link_libraries(${elf_file} "${ARDUPILOT_LIB}/libap.a")
ELSEIF(${ARDUPILOT_CMD} STREQUAL "plane")
    message("Building for plane")
    target_link_libraries(${elf_file} "${ARDUPILOT_BIN}/libarduplane.a")
    target_link_libraries(${elf_file} "${ARDUPILOT_LIB}/libArduPlane_libs.a")
ELSEIF(${ARDUPILOT_CMD} STREQUAL "copter")
    message("Building for copter")
    target_link_libraries(${elf_file} "${ARDUPILOT_BIN}/libarducopter.a")
    target_link_libraries(${elf_file} "${ARDUPILOT_LIB}/libArduCopter_libs.a")
ELSEIF(${ARDUPILOT_CMD} STREQUAL "rover")
    message("Building for rover")
    target_link_libraries(${elf_file} "${ARDUPILOT_BIN}/libardurover.a")
    target_link_libraries(${elf_file} "${ARDUPILOT_LIB}/libRover_libs.a")
ELSEIF(${ARDUPILOT_CMD} STREQUAL "sub")
    message("Building for submarine")
    target_link_libraries(${elf_file} "${ARDUPILOT_BIN}/libardusub.a")
    target_link_libraries(${elf_file} "${ARDUPILOT_LIB}/libArduSub_libs.a")
ENDIF()

add_custom_target(showinc ALL
        COMMAND echo -e
        "$<TARGET_PROPERTY:${elf_file},INCLUDE_DIRECTORIES>"
        > includes.list
    VERBATIM
    BYPRODUCTS includes.list
    COMMAND_EXPAND_LISTS
)

#Find files name lib to link
#function(SUBLINK target curdir)
#  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
#  FOREACH(child ${children})
#    IF(NOT IS_DIRECTORY ${curdir}/${child})
#      SET(PATH "${curdir}/${child}")
#      message("Linking ${PATH}")
#      target_link_libraries(${target} "${PATH}")
#    ENDIF()
#  ENDFOREACH()
#endfunction()

#IF (DEFINED ARDUPILOT_BIN)
#    SUBLINK(${elf_file} ${ARDUPILOT_BIN})
#ENDIF()
#IF (DEFINED ARDUPILOT_LIB)
#    SUBLINK(${elf_file} ${ARDUPILOT_LIB})
#ENDIF()

# Link the static libraries to the executable
target_link_libraries(${elf_file}
                idf::freertos
                idf::fatfs
                idf::nvs_flash
                idf::spi_flash
                idf::app_update
                idf::esp_adc
                idf::esp_driver_mcpwm
                idf::driver
                idf::lwip
                idf::esp_wifi
                idf::esp_system
                idf::esp_rom
                idf::esp_timer
                )

# Attach additional targets to the executable file for flashing,
# linker script generation, partition_table generation, etc.
idf_build_executable(${elf_file})

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

