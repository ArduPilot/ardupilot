#
# Copyright (c) 2016-2017 UAVCAN Team
#

add_executable(${PROJECT_NAME}_tests
    common_test.h
    test_crc.cpp
    test_float16.cpp
    test_init.cpp
    test_memory_allocator.cpp
    test_rxerr.cpp
    test_scalar_encoding.cpp
)

# add source properties
set_target_properties(${PROJECT_NAME}_tests PROPERTIES COMPILE_FLAGS "${CANARD_CXX_FLAGS}")
set_source_files_properties(canard_tgt PROPERTIES COMPILE_FLAGS "${CANARD_C_FLAGS}")

target_link_libraries(${PROJECT_NAME}_tests PRIVATE GTest::gtest_main canard_tgt canard_private_tgt pthread)
if (CANARD_LINK_FLAGS)
    set_target_properties(${PROJECT_NAME}_tests PROPERTIES LINK_FLAGS "${CANARD_LINK_FLAGS}")
endif()

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_tests)
