# Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(CTEST_SOURCE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
set(CTEST_BINARY_DIRECTORY "${CTEST_SOURCE_DIRECTORY}/build")
set(CTEST_TEST_TIMEOUT 300)

site_name(CTEST_SITE)
set(CTEST_BUILD_NAME "${JENKINS_BUILD_NAME}")
set(CTEST_BUILD_OPTIONS "${JENKINS_BUILD_OPTIONS}")
set(CTEST_BUILD_CONFIGURATION "${JENKINS_BUILD_CONFIGURATION}")

set(CTEST_MEMORYCHECK_COMMAND_OPTIONS "${CTEST_MEMORYCHECK_COMMAND_OPTIONS} --quiet --tool=memcheck --leak-check=yes --show-reachable=yes --num-callers=50 --xml=yes --xml-file=test_%p_memcheck.xml \"--suppressions=${CTEST_SOURCE_DIRECTORY}/valgrind.supp\"")

set(CTEST_COVERAGE_C_FLAGS "-DCMAKE_C_FLAGS:STRING=-fwrapv -fprofile-arcs -ftest-coverage")
set(CTEST_COVERAGE_CXX_FLAGS "-DCMAKE_CXX_FLAGS:STRING=-fwrapv -fprofile-arcs -ftest-coverage")
set(CTEST_COVERAGE_EXE_LD_FLAGS "-DCMAKE_EXE_LINKER_FLAGS:STRING=-fprofile-arcs -ftest-coverage")
set(CTEST_COVERAGE_SHARED_LD_FLAGS "-DCMAKE_SHARED_LINKER_FLAGS:STRING=-fprofile-arcs -ftest-coverage")

include(ProcessorCount)
ProcessorCount(NUMBER_PROCESSORS)
message("Number of processors: " ${NUMBER_PROCESSORS})
if(NOT NUMBER_PROCESSORS EQUAL 0)
    if(${JENKINS_GENERATOR} MATCHES "Unix Makefiles")
        set(CTEST_BUILD_FLAGS "-j${NUMBER_PROCESSORS} -l${NUMBER_PROCESSORS}")
    # Error using visual studio with multicore
    #elseif(${JENKINS_GENERATOR} MATCHES "Visual Studio")
        #set(CTEST_WIN_CXX_FLAGS "-DEPROSIMA_EXTRA_CMAKE_CXX_FLAGS:STRING=/MP")
    endif()
    set(CTEST_TEST_ARGS ${CTEST_TEST_ARGS} PARALLEL_LEVEL ${NUMBER_PROCESSORS})
endif()

# Check CMake version for QUIET parameter
if(${CMAKE_MAJOR_VERSION} GREATER 3 OR (${CMAKE_MAJOR_VERSION} EQUAL 3 AND ${CMAKE_MINOR_VERSION} GREATER 2))
    set(QUIET_ QUIET)
endif()

ctest_empty_binary_directory(${CTEST_BINARY_DIRECTORY})

if(UNIX)
    find_program(CTEST_COVERAGE_COMMAND NAMES gcov)
    find_program(CTEST_MEMORYCHECK_COMMAND NAMES valgrind)
endif()

set(CTEST_CONFIGURE_COMMAND "${CMAKE_COMMAND}")
set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} -DCMAKE_BUILD_TYPE=${CTEST_BUILD_CONFIGURATION}")
set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} ${CTEST_BUILD_OPTIONS}")
set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} ${CTEST_WIN_CXX_FLAGS}")
if(CTEST_COVERAGE_COMMAND AND NOT DISABLE_CTEST_COVERAGE)
    set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} \"${CTEST_COVERAGE_C_FLAGS}\"")
    set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} \"${CTEST_COVERAGE_CXX_FLAGS}\"")
    set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} \"${CTEST_COVERAGE_EXE_LD_FLAGS}\"")
    set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} \"${CTEST_COVERAGE_SHARED_LD_FLAGS}\"")
endif()
set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} \"${CTEST_SOURCE_DIRECTORY}\"")
set(CTEST_BUILD_COMMAND "${CMAKE_COMMAND} --build .")
if(WIN32)
    set(CTEST_BUILD_COMMAND "${CTEST_BUILD_COMMAND} --config ${CTEST_BUILD_CONFIGURATION}")
endif()
set(CTEST_CONFIGURATION_TYPE ${CTEST_BUILD_CONFIGURATION})

ctest_start("${JENKINS_DASHBOARD}" ${QUIET_})
ctest_configure(RETURN_VALUE CONFIGURING_RET_VALUE ${QUIET_})
ctest_build(RETURN_VALUE BUILDING_RET_VALUE ${QUIET_})
ctest_test(${QUIET_})
if(CTEST_COVERAGE_COMMAND AND NOT DISABLE_CTEST_COVERAGE)
    ctest_coverage(${QUIET_})
endif()
if(CTEST_MEMORYCHECK_COMMAND AND NOT DISABLE_CTEST_MEMORYCHECK)
    ctest_memcheck(EXCLUDE_LABEL NoMemoryCheck ${QUIET_})
endif()

if(NOT CONFIGURING_RET_VALUE AND NOT BUILDING_RET_VALUE)
    message(0)
else()
    message(255)
endif()
