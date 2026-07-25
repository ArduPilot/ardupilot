# define system dependent compiler flags

include(CheckCCompilerFlag)
include(MacroCheckCCompilerFlagSSP)

#
# Define GNUCC compiler flags
#
if (${CMAKE_C_COMPILER_ID} MATCHES GNU)

    # add -Wconversion ?
    #set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -pedantic -pedantic-errors")
    #set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wshadow -Wmissing-prototypes -Wdeclaration-after-statement")
    #set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wunused -Wfloat-equal -Wpointer-arith -Wwrite-strings -Wformat-security")
    #set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wmissing-format-attribute")

    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic -pedantic-errors")
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow")
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunused -Wfloat-equal -Wpointer-arith -Wwrite-strings -Wformat-security")
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wmissing-format-attribute")

    if (UNIX AND NOT WIN32)

        # with -fPIC
        check_c_compiler_flag("-fPIC" WITH_FPIC)
        if (WITH_FPIC)
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
        endif (WITH_FPIC)

    endif(UNIX AND NOT WIN32)

    check_c_compiler_flag_ssp("-fstack-protector" WITH_STACK_PROTECTOR)
    if (WITH_STACK_PROTECTOR)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fstack-protector")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
        set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fstack-protector")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKDER_FLAGS} -fstack-protector")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKDER_FLAGS} -fstack-protector")
    endif (WITH_STACK_PROTECTOR)

    check_c_compiler_flag("-D_FORTIFY_SOURCE=2" WITH_FORTIFY_SOURCE)
    if (WITH_FORTIFY_SOURCE)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_FORTIFY_SOURCE=2")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORTIFY_SOURCE=2")
    endif (WITH_FORTIFY_SOURCE)
endif (${CMAKE_C_COMPILER_ID} MATCHES GNU)

if (UNIX AND NOT WIN32)
    #
    # Check for large filesystem support
    #
    if (CMAKE_SIZEOF_VOID_P MATCHES "8")
        # with large file support
        execute_process(
            COMMAND
                getconf LFS64_CFLAGS
            OUTPUT_VARIABLE
                _lfs_CFLAGS
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
    else (CMAKE_SIZEOF_VOID_P MATCHES "8")
        # with large file support
        execute_process(
            COMMAND
                getconf LFS_CFLAGS
            OUTPUT_VARIABLE
                _lfs_CFLAGS
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
    endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
    if (_lfs_CFLAGS)
        string(REGEX REPLACE "[\r\n]" " " "${_lfs_CFLAGS}" "${${_lfs_CFLAGS}}")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${_lfs_CFLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${_lfs_CFLAGS}")
    endif (_lfs_CFLAGS)

endif (UNIX AND NOT WIN32)

if (MSVC)
    # Use secure functions by defaualt and suppress warnings about
    #"deprecated" functions
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES=1")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES_COUNT=1")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_NONSTDC_NO_WARNINGS=1 /D _CRT_SECURE_NO_WARNINGS=1")
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_C_FLAGS}")
endif (MSVC)
