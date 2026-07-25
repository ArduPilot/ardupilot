cmake_policy(SET CMP0048 NEW)
project(unitary_tests LANGUAGES C CXX)

if(UCLIENT_PROFILE_MULTITHREAD)
    include(ExternalProject)
    find_package(Threads REQUIRED)

    ExternalProject_Add(unitary_tests_project
        SOURCE_DIR
            ${CMAKE_CURRENT_SOURCE_DIR}/../../
        BINARY_DIR
            ${PROJECT_BINARY_DIR}/microxrcedds_client_unitary
        INSTALL_DIR
            ${PROJECT_BINARY_DIR}/temp_install/unitary
        CMAKE_CACHE_ARGS
            -DUCLIENT_PROFILE_MULTITHREAD:BOOL=ON
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
        BUILD_BYPRODUCTS ${PROJECT_BINARY_DIR}/temp_install/lib/unitary/libmicroxrcedds_client.a
        BUILD_BYPRODUCTS ${PROJECT_BINARY_DIR}/temp_install/lib/unitary/libmicrocdr.a
        )

    ExternalProject_Get_Property(unitary_tests_project INSTALL_DIR)

    add_library(ucdr_unitary STATIC IMPORTED GLOBAL)
    add_dependencies(ucdr_unitary unitary_tests_project)
    set_property(TARGET ucdr_unitary PROPERTY IMPORTED_LOCATION  ${PROJECT_BINARY_DIR}/temp_install/unitary/lib/libmicrocdr.a)

    add_library(uxr_unitary STATIC IMPORTED GLOBAL)
    add_dependencies(uxr_unitary unitary_tests_project)
    target_link_libraries(uxr_unitary INTERFACE ucdr_unitary)
    set_property(TARGET uxr_unitary PROPERTY IMPORTED_LOCATION  ${PROJECT_BINARY_DIR}/temp_install/unitary/lib/libmicroxrcedds_client.a)
endif()

macro(unitary_test TEST_NAME TEST_SOURCE)
    set(SRCS ${TEST_SOURCE})

    add_executable(${TEST_NAME} ${TEST_SOURCE})

    set_common_compile_options(${TEST_NAME})

    if(MSVC OR MSVC_IDE)
        target_compile_options(${TEST_NAME} PRIVATE /wd4996)
    endif()

    add_gtest(${TEST_NAME} SOURCES ${TEST_SOURCE})

    get_target_property(CLIENT_INCLUDES microxrcedds_client INCLUDE_DIRECTORIES)
    include_directories(SYSTEM ${PROJECT_SOURCE_DIR}/../../src)

    target_link_libraries(${TEST_NAME}
        PRIVATE
            microcdr
            $<$<BOOL:${UCLIENT_PROFILE_MULTITHREAD}>:ucdr_unitary>
            $<$<BOOL:${UCLIENT_PROFILE_MULTITHREAD}>:uxr_unitary>
            ${GTEST_BOTH_LIBRARIES}
            ${CMAKE_THREAD_LIBS_INIT}
        )

    target_include_directories(${TEST_NAME}
        PUBLIC
            ${PROJECT_SOURCE_DIR}
        PRIVATE
            ${GTEST_INCLUDE_DIRS}
            ${CLIENT_INCLUDES}
        )

    set_target_properties(${TEST_NAME} PROPERTIES
        CXX_STANDARD
            11
        CXX_STANDARD_REQUIRED
            YES
        )

endmacro()

unitary_test(StreamId               session/streams/StreamId.cpp)
unitary_test(SeqNum                 session/streams/SeqNum.cpp)
unitary_test(InputBestEffortStream  session/streams/InputBestEffortStream.cpp)
unitary_test(OutputBestEffortStream session/streams/OutputBestEffortStream.cpp)
unitary_test(CommonReliableStream   session/streams/CommonReliableStream.cpp)
unitary_test(InputReliableStream    session/streams/InputReliableStream.cpp)
unitary_test(OutputReliableStream   session/streams/OutputReliableStream.cpp)
unitary_test(StreamStorage          session/streams/StreamStorage.cpp)

unitary_test(ObjectId           session/ObjectId.cpp)
unitary_test(Submessage         session/Submessage.cpp)
unitary_test(SessionInfo        session/SessionInfo.cpp)
unitary_test(Session            session/Session.cpp)
unitary_test(WriteReadAccess    session/WriteReadAccess.cpp)
unitary_test(Utils              session/Utils.cpp)

