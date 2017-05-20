include(EigenTesting)
include(CheckCXXSourceCompiles)

# configure the "site" and "buildname" 
ei_set_sitename()

# retrieve and store the build string
ei_set_build_string()

add_custom_target(buildtests)
add_custom_target(check COMMAND "ctest")
add_dependencies(check buildtests)

# check whether /bin/bash exists
find_file(EIGEN_BIN_BASH_EXISTS "/bin/bash" PATHS "/" NO_DEFAULT_PATH)

# This call activates testing and generates the DartConfiguration.tcl
include(CTest)

set(EIGEN_TEST_BUILD_FLAGS "" CACHE STRING "Options passed to the build command of unit tests")

# Overwrite default DartConfiguration.tcl such that ctest can build our unit tests.
# Recall that our unit tests are not in the "all" target, so we have to explicitely ask ctest to build our custom 'buildtests' target.
# At this stage, we can also add custom flags to the build tool through the user defined EIGEN_TEST_BUILD_FLAGS variable.
file(READ  "${CMAKE_CURRENT_BINARY_DIR}/DartConfiguration.tcl" EIGEN_DART_CONFIG_FILE)
# try to grab the default flags
string(REGEX MATCH "MakeCommand:.*-- (.*)\nDefaultCTestConfigurationType" EIGEN_DUMMY ${EIGEN_DART_CONFIG_FILE})
if(NOT CMAKE_MATCH_1)
string(REGEX MATCH "MakeCommand:.*[^c]make (.*)\nDefaultCTestConfigurationType" EIGEN_DUMMY ${EIGEN_DART_CONFIG_FILE})
endif()
string(REGEX REPLACE "MakeCommand:.*DefaultCTestConfigurationType" "MakeCommand: ${CMAKE_COMMAND} --build . --target buildtests --config \"\${CTEST_CONFIGURATION_TYPE}\" -- ${CMAKE_MATCH_1} ${EIGEN_TEST_BUILD_FLAGS}\nDefaultCTestConfigurationType"
       EIGEN_DART_CONFIG_FILE2 ${EIGEN_DART_CONFIG_FILE})
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/DartConfiguration.tcl" ${EIGEN_DART_CONFIG_FILE2})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CTestCustom.cmake.in ${CMAKE_BINARY_DIR}/CTestCustom.cmake)

# some documentation of this function would be nice
ei_init_testing()

# configure Eigen related testing options
option(EIGEN_NO_ASSERTION_CHECKING "Disable checking of assertions using exceptions" OFF)
option(EIGEN_DEBUG_ASSERTS "Enable advanced debuging of assertions" OFF)

if(CMAKE_COMPILER_IS_GNUCXX)
  option(EIGEN_COVERAGE_TESTING "Enable/disable gcov" OFF)
  if(EIGEN_COVERAGE_TESTING)
    set(COVERAGE_FLAGS "-fprofile-arcs -ftest-coverage")
    set(CTEST_CUSTOM_COVERAGE_EXCLUDE "/test/")
  else(EIGEN_COVERAGE_TESTING)
    set(COVERAGE_FLAGS "")
  endif(EIGEN_COVERAGE_TESTING)
  
  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_FLAGS} -g2")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${COVERAGE_FLAGS} -O2 -g2")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${COVERAGE_FLAGS} -fno-inline-functions")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${COVERAGE_FLAGS} -O0 -g3")
  endif(CMAKE_SYSTEM_NAME MATCHES Linux)
elseif(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CRT_SECURE_NO_WARNINGS /D_SCL_SECURE_NO_WARNINGS")
endif(CMAKE_COMPILER_IS_GNUCXX)


check_cxx_compiler_flag("-std=c++11" EIGEN_COMPILER_SUPPORT_CXX11)

if(EIGEN_TEST_CXX11 AND EIGEN_COMPILER_SUPPORT_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()
