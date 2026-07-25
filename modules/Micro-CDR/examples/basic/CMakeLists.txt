project(basic_example)

add_executable(${PROJECT_NAME} basic.c)
if(MSVC OR MSVC_IDE)
    target_compile_options(${PROJECT_NAME} PRIVATE /wd4996)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED YES
    )

target_link_libraries(${PROJECT_NAME} microcdr)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION examples/ucdr/${PROJECT_NAME}/${BIN_INSTALL_DIR}
    )

install(DIRECTORY ${PROJECT_SOURCE_DIR}/
    DESTINATION  examples/ucdr/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h"
                   PATTERN "*.c"
    )
