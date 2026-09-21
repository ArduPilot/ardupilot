cmake_minimum_required(VERSION 3.5)

project(minimal)

set(CMAKE_CXX_STANDARD 11)

add_executable(minimal
  minimal.cpp SocketExample.cpp libAP_JSON.cpp
)

add_executable(simpleRover
  simpleRover.cpp SocketExample.cpp libAP_JSON.cpp
)
