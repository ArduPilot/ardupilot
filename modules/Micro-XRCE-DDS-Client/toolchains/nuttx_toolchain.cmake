include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(PLATFORM_NAME "nuttx")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Makefile flags
set(CROSSDEV "" CACHE STRING "GCC compiler use in NuttX.")
set(ARCH_CPU_FLAGS "" CACHE STRING "Makefile arquitecture flags.")
set(ARCH_OPT_FLAGS "" CACHE STRING "Makefile optimization flags.")
separate_arguments(ARCH_CPU_FLAGS)
separate_arguments(ARCH_OPT_FLAGS)

# Compiler tools
foreach(tool gcc ld ar)
	string(TOUPPER ${tool} TOOL)
    find_program(${TOOL} ${CROSSDEV}${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${tool}")
	endif()
endforeach()

CMAKE_FORCE_C_COMPILER(${CROSSDEV}gcc GNU)
CMAKE_FORCE_CXX_COMPILER(${CROSSDEV}g++ GNU)

include_directories(SYSTEM ${CMAKE_SYSROOT}/include)
add_compile_options(
    -std=c99 
    ${ARCH_CPU_FLAGS}
    ${ARCH_OPT_FLAGS}
    )
