cmake_minimum_required(VERSION 3.5)

project(nuttx_toolchain NONE)

# Makefile flags
set(CROSSDEV "" CACHE STRING "GCC compiler use in NuttX.")
set(ARCH_CPU_FLAGS "" CACHE STRING "Makefile arquitecture flags.")
set(ARCH_OPT_FLAGS "" CACHE STRING "Makefile optimization flags.")
#separate_arguments(ARCH_CPU_FLAGS)
#separate_arguments(ARCH_OPT_FLAGS)

# Generate toolchain
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/nuttx_toolchain.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/nuttx_toolchain.cmake
    @ONLY
    )
