FetchContent_Declare(
  DSDL
  GIT_REPOSITORY https://github.com/DroneCAN/DSDL.git
  GIT_TAG        master
)

FetchContent_Declare(
  dronecan_dsdlc
  GIT_REPOSITORY https://github.com/DroneCAN/dronecan_dsdlc.git
  GIT_TAG        master
)

FetchContent_MakeAvailable(googletest DSDL dronecan_dsdlc)

# execute process to generate libcanard headers using dronecan_dsdlc/dronecan_dsdlc.py
execute_process(COMMAND python3 ${dronecan_dsdlc_SOURCE_DIR}/dronecan_dsdlc.py
    -O ${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated
    ${dsdl_SOURCE_DIR}/uavcan
    ${dsdl_SOURCE_DIR}/dronecan
    ${dsdl_SOURCE_DIR}/ardupilot
    ${dsdl_SOURCE_DIR}/com
    )

if (CANARD_ENABLE_DEADLINE)
    add_definitions(-DCANARD_ENABLE_DEADLINE=1)
endif()

# check arguments for 32-bit build
if (CMAKE_32BIT)
     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
endif()

if (CANARD_ENABLE_CANFD)
    add_definitions(-DCANARD_ENABLE_CANFD=1)
endif()

# generate dsdl messages
include_directories(${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated/include)

# glob all generated dsdlc files
file(GLOB DSDL_GENERATED_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated/src/*.c*)

# append DSDL_GENERATED_SOURCES_C to SRC_FILES
list(APPEND SRC_FILES ${DSDL_GENERATED_SOURCES})

set(SRC_FILES_TEST cxx_test_interface.cpp
                   test_cxx_wrappers.cpp)
set(SRC_FILES_CANARD canard_interface.cpp
                     test_canard_interface.cpp)

# create test target
add_executable(${PROJECT_NAME}_test_cf ${SRC_FILES} ${SRC_FILES_TEST})
add_executable(${PROJECT_NAME}_test_canard ${SRC_FILES} ${SRC_FILES_CANARD})

set(DSDL_SOURCE_CFLAGS "-Wno-gnu-empty-struct -Wno-implicit-int-conversion -Wno-unused-parameter -Wno-unused-const-variable -Wno-sign-conversion -Wno-missing-declarations -Wno-conversion -Wno-pedantic")

set_source_files_properties(${SRC_FILES} PROPERTIES COMPILE_FLAGS "${CANARD_C_FLAGS} ${DSDL_SOURCE_CFLAGS}")
set_source_files_properties(${SRC_FILES_TEST} PROPERTIES COMPILE_FLAGS "${CANARD_CXX_FLAGS}")
set_source_files_properties(${SRC_FILES_CANARD} PROPERTIES COMPILE_FLAGS "${CANARD_CXX_FLAGS}")

if (CANARD_LINK_FLAGS)
    set_target_properties(${PROJECT_NAME}_test_cf PROPERTIES LINK_FLAGS "${CANARD_LINK_FLAGS}")
    set_target_properties(${PROJECT_NAME}_test_canard PROPERTIES LINK_FLAGS "${CANARD_LINK_FLAGS}")
endif()

target_link_libraries(${PROJECT_NAME}_test_cf GTest::gtest_main canard_tgt canard_private_tgt pthread)
include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_test_cf)

target_link_libraries(${PROJECT_NAME}_test_canard GTest::gtest_main canard_tgt canard_private_tgt pthread)
gtest_discover_tests(${PROJECT_NAME}_test_canard)
