cmake_minimum_required(VERSION 3.5)

project(minimal)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
#set(CMAKE_MFC_FLAG 1)

#file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
#file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

#source_group("Header Files" FILES ${HEADERS})
#source_group("Source Files" FILES ${SOURCES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(libAP_JSON STATIC libAP_JSON.cpp)
add_library(SocketExample STATIC SocketExample.cpp)

add_executable(minimal
  minimal.cpp
)

target_compile_options(minimal PUBLIC /MT)
target_link_options(minimal PUBLIC /INCREMENTAL:NO /NODEFAULTLIB:MSVCRT)
target_link_libraries(minimal
                      libAP_JSON
                      SocketExample
                      wsock32 ws2_32)
#target_include_directories(minimal PUBLIC
#                           "${PROJECT_BINARY_DIR}"
#                           ${PROJECT_SOURCEDIR})

add_executable(simpleRover
  simpleRover.cpp
)
