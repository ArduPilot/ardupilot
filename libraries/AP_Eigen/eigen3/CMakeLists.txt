project(Eigen)

cmake_minimum_required(VERSION 2.8.4)

# guard against in-source builds

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

# guard against bad build-type strings

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

string(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type_tolower)
if(    NOT cmake_build_type_tolower STREQUAL "debug"
   AND NOT cmake_build_type_tolower STREQUAL "release"
   AND NOT cmake_build_type_tolower STREQUAL "relwithdebinfo")
  message(FATAL_ERROR "Unknown build type \"${CMAKE_BUILD_TYPE}\". Allowed values are Debug, Release, RelWithDebInfo (case-insensitive).")
endif()


#############################################################################
# retrieve version infomation                                               #
#############################################################################

# automatically parse the version number
file(READ "${PROJECT_SOURCE_DIR}/Eigen/src/Core/util/Macros.h" _eigen_version_header)
string(REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _eigen_world_version_match "${_eigen_version_header}")
set(EIGEN_WORLD_VERSION "${CMAKE_MATCH_1}")
string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen_major_version_match "${_eigen_version_header}")
set(EIGEN_MAJOR_VERSION "${CMAKE_MATCH_1}")
string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen_minor_version_match "${_eigen_version_header}")
set(EIGEN_MINOR_VERSION "${CMAKE_MATCH_1}")
set(EIGEN_VERSION_NUMBER ${EIGEN_WORLD_VERSION}.${EIGEN_MAJOR_VERSION}.${EIGEN_MINOR_VERSION})

# if the mercurial program is absent, this will leave the EIGEN_HG_CHANGESET string empty,
# but won't stop CMake.
execute_process(COMMAND hg tip -R ${CMAKE_SOURCE_DIR} OUTPUT_VARIABLE EIGEN_HGTIP_OUTPUT)
execute_process(COMMAND hg branch -R ${CMAKE_SOURCE_DIR} OUTPUT_VARIABLE EIGEN_BRANCH_OUTPUT)

# if this is the default (aka development) branch, extract the mercurial changeset number from the hg tip output...
if(EIGEN_BRANCH_OUTPUT MATCHES "default")
string(REGEX MATCH "^changeset: *[0-9]*:([0-9;a-f]+).*" EIGEN_HG_CHANGESET_MATCH "${EIGEN_HGTIP_OUTPUT}")
set(EIGEN_HG_CHANGESET "${CMAKE_MATCH_1}")
endif(EIGEN_BRANCH_OUTPUT MATCHES "default")
#...and show it next to the version number
if(EIGEN_HG_CHANGESET)
  set(EIGEN_VERSION "${EIGEN_VERSION_NUMBER} (mercurial changeset ${EIGEN_HG_CHANGESET})")
else(EIGEN_HG_CHANGESET)
  set(EIGEN_VERSION "${EIGEN_VERSION_NUMBER}")
endif(EIGEN_HG_CHANGESET)


include(CheckCXXCompilerFlag)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#############################################################################
# find how to link to the standard libraries                                #
#############################################################################

find_package(StandardMathLibrary)


set(EIGEN_TEST_CUSTOM_LINKER_FLAGS  "" CACHE STRING "Additional linker flags when linking unit tests.")
set(EIGEN_TEST_CUSTOM_CXX_FLAGS     "" CACHE STRING "Additional compiler flags when compiling unit tests.")

set(EIGEN_STANDARD_LIBRARIES_TO_LINK_TO "")

if(NOT STANDARD_MATH_LIBRARY_FOUND)

  message(FATAL_ERROR
    "Can't link to the standard math library. Please report to the Eigen developers, telling them about your platform.")

else()

  if(EIGEN_STANDARD_LIBRARIES_TO_LINK_TO)
    set(EIGEN_STANDARD_LIBRARIES_TO_LINK_TO "${EIGEN_STANDARD_LIBRARIES_TO_LINK_TO} ${STANDARD_MATH_LIBRARY}")
  else()
    set(EIGEN_STANDARD_LIBRARIES_TO_LINK_TO "${STANDARD_MATH_LIBRARY}")
  endif()

endif()

if(EIGEN_STANDARD_LIBRARIES_TO_LINK_TO)
  message(STATUS "Standard libraries to link to explicitly: ${EIGEN_STANDARD_LIBRARIES_TO_LINK_TO}")
else()
  message(STATUS "Standard libraries to link to explicitly: none")
endif()

option(EIGEN_BUILD_BTL "Build benchmark suite" OFF)
if(NOT WIN32)
  option(EIGEN_BUILD_PKGCONFIG "Build pkg-config .pc file for Eigen" ON)
endif(NOT WIN32)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

option(EIGEN_SPLIT_LARGE_TESTS "Split large tests into smaller executables" ON)

option(EIGEN_DEFAULT_TO_ROW_MAJOR "Use row-major as default matrix storage order" OFF)
if(EIGEN_DEFAULT_TO_ROW_MAJOR)
  add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")
endif()

set(EIGEN_TEST_MAX_SIZE "320" CACHE STRING "Maximal matrix/vector size, default is 320")

macro(ei_add_cxx_compiler_flag FLAG)
  string(REGEX REPLACE "-" "" SFLAG1 ${FLAG})
  string(REGEX REPLACE "\\+" "p" SFLAG ${SFLAG1})
  check_cxx_compiler_flag(${FLAG} COMPILER_SUPPORT_${SFLAG})
  if(COMPILER_SUPPORT_${SFLAG})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}")
  endif()
endmacro(ei_add_cxx_compiler_flag)

if(NOT MSVC)
  # We assume that other compilers are partly compatible with GNUCC
  
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")
  set(CMAKE_CXX_FLAGS_DEBUG "-g3")
  set(CMAKE_CXX_FLAGS_RELEASE "-g0 -O2")
  
  # clang outputs some warnings for unknwon flags that are not caught by check_cxx_compiler_flag
  # adding -Werror turns such warnings into errors
  check_cxx_compiler_flag("-Werror" COMPILER_SUPPORT_WERROR)
  if(COMPILER_SUPPORT_WERROR)
    set(CMAKE_REQUIRED_FLAGS "-Werror")
  endif()
  
  ei_add_cxx_compiler_flag("-pedantic")
  ei_add_cxx_compiler_flag("-Wall")
  ei_add_cxx_compiler_flag("-Wextra")
  #ei_add_cxx_compiler_flag("-Weverything")              # clang
  
  ei_add_cxx_compiler_flag("-Wundef")
  ei_add_cxx_compiler_flag("-Wcast-align")
  ei_add_cxx_compiler_flag("-Wchar-subscripts")
  ei_add_cxx_compiler_flag("-Wnon-virtual-dtor")
  ei_add_cxx_compiler_flag("-Wunused-local-typedefs")
  ei_add_cxx_compiler_flag("-Wpointer-arith")
  ei_add_cxx_compiler_flag("-Wwrite-strings")
  ei_add_cxx_compiler_flag("-Wformat-security")
  ei_add_cxx_compiler_flag("-Wshorten-64-to-32")
  ei_add_cxx_compiler_flag("-Wenum-conversion")
  ei_add_cxx_compiler_flag("-Wc++11-extensions")
  
  # -Wshadow is insanely too strict with gcc, hopefully it will become usable with gcc 6
  # if(NOT CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "5.0.0"))
  if(NOT CMAKE_COMPILER_IS_GNUCXX)
    ei_add_cxx_compiler_flag("-Wshadow")
  endif()
  
  ei_add_cxx_compiler_flag("-Wno-psabi")
  ei_add_cxx_compiler_flag("-Wno-variadic-macros")
  ei_add_cxx_compiler_flag("-Wno-long-long")
  
  ei_add_cxx_compiler_flag("-fno-check-new")
  ei_add_cxx_compiler_flag("-fno-common")
  ei_add_cxx_compiler_flag("-fstrict-aliasing")
  ei_add_cxx_compiler_flag("-wd981")                    # disable ICC's "operands are evaluated in unspecified order" remark
  ei_add_cxx_compiler_flag("-wd2304")                   # disbale ICC's "warning #2304: non-explicit constructor with single argument may cause implicit type conversion" produced by -Wnon-virtual-dtor
  
  
  # The -ansi flag must be added last, otherwise it is also used as a linker flag by check_cxx_compiler_flag making it fails
  # Moreover we should not set both -strict-ansi and -ansi
  check_cxx_compiler_flag("-strict-ansi" COMPILER_SUPPORT_STRICTANSI)
  ei_add_cxx_compiler_flag("-Qunused-arguments")        # disable clang warning: argument unused during compilation: '-ansi'
  
  if(COMPILER_SUPPORT_STRICTANSI)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -strict-ansi")
  else()
    ei_add_cxx_compiler_flag("-ansi")
  endif()

  if(ANDROID_NDK)
    ei_add_cxx_compiler_flag("-pie")
    ei_add_cxx_compiler_flag("-fPIE")
  endif()
  
  set(CMAKE_REQUIRED_FLAGS "")

  option(EIGEN_TEST_SSE2 "Enable/Disable SSE2 in tests/examples" OFF)
  if(EIGEN_TEST_SSE2)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
    message(STATUS "Enabling SSE2 in tests/examples")
  endif()

  option(EIGEN_TEST_SSE3 "Enable/Disable SSE3 in tests/examples" OFF)
  if(EIGEN_TEST_SSE3)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3")
    message(STATUS "Enabling SSE3 in tests/examples")
  endif()

  option(EIGEN_TEST_SSSE3 "Enable/Disable SSSE3 in tests/examples" OFF)
  if(EIGEN_TEST_SSSE3)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
    message(STATUS "Enabling SSSE3 in tests/examples")
  endif()

  option(EIGEN_TEST_SSE4_1 "Enable/Disable SSE4.1 in tests/examples" OFF)
  if(EIGEN_TEST_SSE4_1)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
    message(STATUS "Enabling SSE4.1 in tests/examples")
  endif()

  option(EIGEN_TEST_SSE4_2 "Enable/Disable SSE4.2 in tests/examples" OFF)
  if(EIGEN_TEST_SSE4_2)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.2")
    message(STATUS "Enabling SSE4.2 in tests/examples")
  endif()

  option(EIGEN_TEST_AVX "Enable/Disable AVX in tests/examples" OFF)
  if(EIGEN_TEST_AVX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx")
    message(STATUS "Enabling AVX in tests/examples")
  endif()

  option(EIGEN_TEST_FMA "Enable/Disable FMA in tests/examples" OFF)
  if(EIGEN_TEST_FMA AND NOT EIGEN_TEST_NEON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfma")
    message(STATUS "Enabling FMA in tests/examples")
  endif()

  option(EIGEN_TEST_ALTIVEC "Enable/Disable AltiVec in tests/examples" OFF)
  if(EIGEN_TEST_ALTIVEC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -maltivec -mabi=altivec")
    message(STATUS "Enabling AltiVec in tests/examples")
  endif()

  option(EIGEN_TEST_VSX "Enable/Disable VSX in tests/examples" OFF)
  if(EIGEN_TEST_VSX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -mvsx")
    message(STATUS "Enabling VSX in tests/examples")
  endif()

  option(EIGEN_TEST_NEON "Enable/Disable Neon in tests/examples" OFF)
  if(EIGEN_TEST_NEON)
    if(EIGEN_TEST_FMA)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon-vfpv4")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfloat-abi=softfp")
    message(STATUS "Enabling NEON in tests/examples")
  endif()

  option(EIGEN_TEST_NEON64 "Enable/Disable Neon in tests/examples" OFF)
  if(EIGEN_TEST_NEON64)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    message(STATUS "Enabling NEON in tests/examples")
  endif()



  check_cxx_compiler_flag("-fopenmp" COMPILER_SUPPORT_OPENMP)
  if(COMPILER_SUPPORT_OPENMP)
    option(EIGEN_TEST_OPENMP "Enable/Disable OpenMP in tests/examples" OFF)
    if(EIGEN_TEST_OPENMP)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
      message(STATUS "Enabling OpenMP in tests/examples")
    endif()
  endif()

else(NOT MSVC)

  # C4127 - conditional expression is constant
  # C4714 - marked as __forceinline not inlined (I failed to deactivate it selectively)
  #         We can disable this warning in the unit tests since it is clear that it occurs
  #         because we are oftentimes returning objects that have a destructor or may
  #         throw exceptions - in particular in the unit tests we are throwing extra many
  #         exceptions to cover indexing errors.
  # C4505 - unreferenced local function has been removed (impossible to deactive selectively)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /wd4127 /wd4505 /wd4714")

  # replace all /Wx by /W4
  string(REGEX REPLACE "/W[0-9]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

  check_cxx_compiler_flag("/openmp" COMPILER_SUPPORT_OPENMP)
  if(COMPILER_SUPPORT_OPENMP)
    option(EIGEN_TEST_OPENMP "Enable/Disable OpenMP in tests/examples" OFF)
    if(EIGEN_TEST_OPENMP)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp")
      message(STATUS "Enabling OpenMP in tests/examples")
    endif()
  endif()

  option(EIGEN_TEST_SSE2 "Enable/Disable SSE2 in tests/examples" OFF)
  if(EIGEN_TEST_SSE2)
    if(NOT CMAKE_CL_64)
      # arch is not supported on 64 bit systems, SSE is enabled automatically.
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2")
    endif(NOT CMAKE_CL_64)
    message(STATUS "Enabling SSE2 in tests/examples")
  endif(EIGEN_TEST_SSE2)
endif(NOT MSVC)

option(EIGEN_TEST_NO_EXPLICIT_VECTORIZATION "Disable explicit vectorization in tests/examples" OFF)
option(EIGEN_TEST_X87 "Force using X87 instructions. Implies no vectorization." OFF)
option(EIGEN_TEST_32BIT "Force generating 32bit code." OFF)

if(EIGEN_TEST_X87)
  set(EIGEN_TEST_NO_EXPLICIT_VECTORIZATION ON)
  if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpmath=387")
    message(STATUS "Forcing use of x87 instructions in tests/examples")
  else()
    message(STATUS "EIGEN_TEST_X87 ignored on your compiler")
  endif()
endif()

if(EIGEN_TEST_32BIT)
  if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
    message(STATUS "Forcing generation of 32-bit code in tests/examples")
  else()
    message(STATUS "EIGEN_TEST_32BIT ignored on your compiler")
  endif()
endif()

if(EIGEN_TEST_NO_EXPLICIT_VECTORIZATION)
  add_definitions(-DEIGEN_DONT_VECTORIZE=1)
  message(STATUS "Disabling vectorization in tests/examples")
endif()

option(EIGEN_TEST_NO_EXPLICIT_ALIGNMENT "Disable explicit alignment (hence vectorization) in tests/examples" OFF)
if(EIGEN_TEST_NO_EXPLICIT_ALIGNMENT)
  add_definitions(-DEIGEN_DONT_ALIGN=1)
  message(STATUS "Disabling alignment in tests/examples")
endif()

option(EIGEN_TEST_NO_EXCEPTIONS "Disables C++ exceptions" OFF)
if(EIGEN_TEST_NO_EXCEPTIONS)
  ei_add_cxx_compiler_flag("-fno-exceptions")
  message(STATUS "Disabling exceptions in tests/examples")
endif()

option(EIGEN_TEST_CXX11 "Enable testing with C++11 and C++11 features (e.g. Tensor module)." OFF)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

# the user modifiable install path for header files
set(EIGEN_INCLUDE_INSTALL_DIR ${EIGEN_INCLUDE_INSTALL_DIR} CACHE PATH "The directory where we install the header files (optional)")

# set the internal install path for header files which depends on wether the user modifiable
# EIGEN_INCLUDE_INSTALL_DIR has been set by the user or not.
if(EIGEN_INCLUDE_INSTALL_DIR)
  set(INCLUDE_INSTALL_DIR
    ${EIGEN_INCLUDE_INSTALL_DIR}
    CACHE INTERNAL
    "The directory where we install the header files (internal)"
  )
else()
  set(INCLUDE_INSTALL_DIR
    "${CMAKE_INSTALL_PREFIX}/include/eigen3"
    CACHE INTERNAL
    "The directory where we install the header files (internal)"
  )
endif()

# similar to set_target_properties but append the property instead of overwriting it
macro(ei_add_target_property target prop value)

  get_target_property(previous ${target} ${prop})
  # if the property wasn't previously set, ${previous} is now "previous-NOTFOUND" which cmake allows catching with plain if()
  if(NOT previous)
    set(previous "")
  endif(NOT previous)
  set_target_properties(${target} PROPERTIES ${prop} "${previous} ${value}")
endmacro(ei_add_target_property)

install(FILES
  signature_of_eigen3_matrix_library
  DESTINATION ${INCLUDE_INSTALL_DIR} COMPONENT Devel
  )

if(EIGEN_BUILD_PKGCONFIG)
    SET(path_separator ":")
    STRING(REPLACE ${path_separator} ";" pkg_config_libdir_search "$ENV{PKG_CONFIG_LIBDIR}")
    message(STATUS "searching for 'pkgconfig' directory in PKG_CONFIG_LIBDIR ( $ENV{PKG_CONFIG_LIBDIR} ), ${CMAKE_INSTALL_PREFIX}/share, and ${CMAKE_INSTALL_PREFIX}/lib")
    FIND_PATH(pkg_config_libdir pkgconfig ${pkg_config_libdir_search} ${CMAKE_INSTALL_PREFIX}/share ${CMAKE_INSTALL_PREFIX}/lib ${pkg_config_libdir_search})
    if(pkg_config_libdir)
        SET(pkg_config_install_dir ${pkg_config_libdir})
        message(STATUS "found ${pkg_config_libdir}/pkgconfig" )
    else(pkg_config_libdir)
        SET(pkg_config_install_dir ${CMAKE_INSTALL_PREFIX}/share)
        message(STATUS "pkgconfig not found; installing in ${pkg_config_install_dir}" )
    endif(pkg_config_libdir)

    configure_file(eigen3.pc.in eigen3.pc)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/eigen3.pc
        DESTINATION ${pkg_config_install_dir}/pkgconfig
        )
endif(EIGEN_BUILD_PKGCONFIG)

add_subdirectory(Eigen)

add_subdirectory(doc EXCLUDE_FROM_ALL)

include(EigenConfigureTesting)

# fixme, not sure this line is still needed:
enable_testing() # must be called from the root CMakeLists, see man page


if(EIGEN_LEAVE_TEST_IN_ALL_TARGET)
  add_subdirectory(test) # can't do EXCLUDE_FROM_ALL here, breaks CTest
else()
  add_subdirectory(test EXCLUDE_FROM_ALL)
endif()

if(EIGEN_LEAVE_TEST_IN_ALL_TARGET)
  add_subdirectory(blas)
  add_subdirectory(lapack)
else()
  add_subdirectory(blas EXCLUDE_FROM_ALL)
  add_subdirectory(lapack EXCLUDE_FROM_ALL)
endif()

add_subdirectory(unsupported)

add_subdirectory(demos EXCLUDE_FROM_ALL)

# must be after test and unsupported, for configuring buildtests.in
add_subdirectory(scripts EXCLUDE_FROM_ALL)

# TODO: consider also replacing EIGEN_BUILD_BTL by a custom target "make btl"?
if(EIGEN_BUILD_BTL)
  add_subdirectory(bench/btl EXCLUDE_FROM_ALL)
endif(EIGEN_BUILD_BTL)

if(NOT WIN32)
  add_subdirectory(bench/spbench EXCLUDE_FROM_ALL)
endif(NOT WIN32)

configure_file(scripts/cdashtesting.cmake.in cdashtesting.cmake @ONLY)

ei_testing_print_summary()

message(STATUS "")
message(STATUS "Configured Eigen ${EIGEN_VERSION_NUMBER}")
message(STATUS "")

option(EIGEN_FAILTEST "Enable failtests." OFF)
if(EIGEN_FAILTEST)
  add_subdirectory(failtest)
endif()

string(TOLOWER "${CMAKE_GENERATOR}" cmake_generator_tolower)
if(cmake_generator_tolower MATCHES "makefile")
  message(STATUS "Some things you can do now:")
  message(STATUS "--------------+--------------------------------------------------------------")
  message(STATUS "Command       |   Description")
  message(STATUS "--------------+--------------------------------------------------------------")
  message(STATUS "make install  | Install to ${CMAKE_INSTALL_PREFIX}. To change that:")
  message(STATUS "              |     cmake . -DCMAKE_INSTALL_PREFIX=yourpath")
  message(STATUS "              |   Eigen headers will then be installed to:")
  message(STATUS "              |     ${INCLUDE_INSTALL_DIR}")
  message(STATUS "              |   To install Eigen headers to a separate location, do:")
  message(STATUS "              |     cmake . -DEIGEN_INCLUDE_INSTALL_DIR=yourpath")
  message(STATUS "make doc      | Generate the API documentation, requires Doxygen & LaTeX")
  message(STATUS "make check    | Build and run the unit-tests. Read this page:")
  message(STATUS "              |   http://eigen.tuxfamily.org/index.php?title=Tests")
  message(STATUS "make blas     | Build BLAS library (not the same thing as Eigen)")
  message(STATUS "make uninstall| Removes files installed by make install")
  message(STATUS "--------------+--------------------------------------------------------------")
else()
  message(STATUS "To build/run the unit tests, read this page:")
  message(STATUS "  http://eigen.tuxfamily.org/index.php?title=Tests")
endif()

message(STATUS "")

set ( EIGEN_CONFIG_CMAKE_PATH
      lib${LIB_SUFFIX}/cmake/eigen3
      CACHE PATH "The directory where the CMake files are installed"
    )
if ( NOT IS_ABSOLUTE EIGEN_CONFIG_CMAKE_PATH )
  set ( EIGEN_CONFIG_CMAKE_PATH ${CMAKE_INSTALL_PREFIX}/${EIGEN_CONFIG_CMAKE_PATH} )
endif ()

set ( EIGEN_USE_FILE ${EIGEN_CONFIG_CMAKE_PATH}/UseEigen3.cmake )
set ( EIGEN_VERSION_STRING ${EIGEN_VERSION_NUMBER} )
set ( EIGEN_VERSION_MAJOR  ${EIGEN_WORLD_VERSION} )
set ( EIGEN_VERSION_MINOR  ${EIGEN_MAJOR_VERSION} )
set ( EIGEN_VERSION_PATCH  ${EIGEN_MINOR_VERSION} )
set ( EIGEN_DEFINITIONS "")
set ( EIGEN_INCLUDE_DIR ${INCLUDE_INSTALL_DIR} )
set ( EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR} )
set ( EIGEN_ROOT_DIR ${CMAKE_INSTALL_PREFIX} )

configure_file ( ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Eigen3Config.cmake.in
                 ${CMAKE_CURRENT_BINARY_DIR}/Eigen3Config.cmake
                 @ONLY ESCAPE_QUOTES
               )

install ( FILES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/UseEigen3.cmake
                ${CMAKE_CURRENT_BINARY_DIR}/Eigen3Config.cmake
          DESTINATION ${EIGEN_CONFIG_CMAKE_PATH}
        )

# Add uninstall target
add_custom_target ( uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/EigenUninstall.cmake)
