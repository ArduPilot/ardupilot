/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_BUILD_CONFIG_HPP_INCLUDED
#define UAVCAN_BUILD_CONFIG_HPP_INCLUDED

/**
 * UAVCAN version definition
 */
#define UAVCAN_VERSION_MAJOR    1
#define UAVCAN_VERSION_MINOR    0

/**
 * UAVCAN_CPP_VERSION - version of the C++ standard used during compilation.
 * This definition contains the integer year number after which the standard was named:
 *  - 2003 for C++03
 *  - 2011 for C++11
 *
 * This config automatically sets according to the actual C++ standard used by the compiler.
 *
 * In C++03 mode the library has almost zero dependency on the C++ standard library, which allows
 * to use it on platforms with a very limited C++ support. On the other hand, C++11 mode requires
 * many parts of the standard library (e.g. <functional>), thus the user might want to force older
 * standard than used by the compiler, in which case this symbol can be overridden manually via
 * compiler flags.
 */
#define UAVCAN_CPP11    2011
#define UAVCAN_CPP03    2003

#ifndef UAVCAN_CPP_VERSION
# if __cplusplus > 201200
#  error Unsupported C++ standard. You can explicitly set UAVCAN_CPP_VERSION=UAVCAN_CPP11 to silence this error.
# elif (__cplusplus > 201100) || defined(__GXX_EXPERIMENTAL_CXX0X__)
#  define UAVCAN_CPP_VERSION    UAVCAN_CPP11
# else
#  define UAVCAN_CPP_VERSION    UAVCAN_CPP03
# endif
#endif

/**
 * The library uses UAVCAN_NULLPTR instead of UAVCAN_NULLPTR and nullptr in order to allow the use of
 * -Wzero-as-null-pointer-constant.
 */
#ifndef UAVCAN_NULLPTR
# if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
#  define UAVCAN_NULLPTR nullptr
# else
#  define UAVCAN_NULLPTR NULL
# endif
#endif

/**
 * By default, libuavcan enables all features if it detects that it is being built for a general-purpose
 * target like Linux. Value of this macro influences other configuration options located below in this file.
 * This macro can be overriden if needed.
 */
#ifndef UAVCAN_GENERAL_PURPOSE_PLATFORM
# if (defined(__linux__)    || defined(__linux)     || defined(__APPLE__)   ||\
      defined(_WIN64)       || defined(_WIN32)      || defined(__ANDROID__) ||\
      defined(_SYSTYPE_BSD) || defined(__FreeBSD__))
#  define UAVCAN_GENERAL_PURPOSE_PLATFORM 1
# else
#  define UAVCAN_GENERAL_PURPOSE_PLATFORM 0
# endif
#endif

/**
 * This macro enables built-in runtime checks and debug output via printf().
 * Should be used only for library development.
 */
#ifndef UAVCAN_DEBUG
# define UAVCAN_DEBUG 0
#endif

/**
 * This option allows to select whether libuavcan should throw exceptions on fatal errors, or try to handle
 * errors differently. By default, exceptions will be enabled only if the library is built for a general-purpose
 * OS like Linux. Set UAVCAN_EXCEPTIONS explicitly to override.
 */
#ifndef UAVCAN_EXCEPTIONS
# define UAVCAN_EXCEPTIONS UAVCAN_GENERAL_PURPOSE_PLATFORM
#endif

/**
 * This specification is used by some error reporting functions like in the Logger class.
 * The default can be overriden by defining the macro UAVCAN_NOEXCEPT explicitly, e.g. via compiler options.
 */
#ifndef UAVCAN_NOEXCEPT
# if UAVCAN_EXCEPTIONS
#  if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
#   define UAVCAN_NOEXCEPT noexcept
#  else
#   define UAVCAN_NOEXCEPT throw()
#  endif
# else
#  define UAVCAN_NOEXCEPT
# endif
#endif

/**
 * Declaration visibility
 * http://gcc.gnu.org/wiki/Visibility
 */
#ifndef UAVCAN_EXPORT
# define UAVCAN_EXPORT
#endif

/**
 * Trade-off between ROM/RAM usage and functionality/determinism.
 * Note that this feature is not well tested and should be avoided.
 * Use code search for UAVCAN_TINY to find what functionality will be disabled.
 * This is particularly useful for embedded systems with less than 40kB of ROM.
 */
#ifndef UAVCAN_TINY
# define UAVCAN_TINY 0
#endif

/**
 * Disable the global data type registry, which can save some space on embedded systems.
 */
#ifndef UAVCAN_NO_GLOBAL_DATA_TYPE_REGISTRY
# define UAVCAN_NO_GLOBAL_DATA_TYPE_REGISTRY 0
#endif

/**
 * toString() methods will be disabled by default, unless the library is built for a general-purpose target like Linux.
 * It is not recommended to enable toString() on embedded targets as code size will explode.
 */
#ifndef UAVCAN_TOSTRING
# if UAVCAN_EXCEPTIONS
#  define UAVCAN_TOSTRING UAVCAN_GENERAL_PURPOSE_PLATFORM
# else
#  define UAVCAN_TOSTRING 0
# endif
#endif

#if UAVCAN_TOSTRING
# if !UAVCAN_EXCEPTIONS
#  error UAVCAN_TOSTRING requires UAVCAN_EXCEPTIONS
# endif
# include <string>
#endif

/**
 * Some C++ implementations are half-broken because they don't implement the placement new operator.
 * If UAVCAN_IMPLEMENT_PLACEMENT_NEW is defined, libuavcan will implement its own operator new (std::size_t, void*)
 * and its delete() counterpart, instead of relying on the standard header <new>.
 */
#ifndef UAVCAN_IMPLEMENT_PLACEMENT_NEW
# define UAVCAN_IMPLEMENT_PLACEMENT_NEW 0
#endif

/**
 * Allows the user's application to provide custom implementation of uavcan::snprintf(),
 * which is often useful on deeply embedded systems.
 */
#ifndef UAVCAN_USE_EXTERNAL_SNPRINTF
# define UAVCAN_USE_EXTERNAL_SNPRINTF   0
#endif

/**
 * Allows the user's application to provide a custom implementation of IEEE754Converter::nativeIeeeToHalf and
 * IEEE754Converter::halfToNativeIeee.
 */
#ifndef UAVCAN_USE_EXTERNAL_FLOAT16_CONVERSION
# define UAVCAN_USE_EXTERNAL_FLOAT16_CONVERSION 0
#endif

/**
 * Run time checks.
 * Resolves to the standard assert() by default.
 * Disabled completely if UAVCAN_NO_ASSERTIONS is defined.
 */
#ifndef UAVCAN_ASSERT
# ifndef UAVCAN_NO_ASSERTIONS
#  define UAVCAN_NO_ASSERTIONS 0
# endif
# if UAVCAN_NO_ASSERTIONS
#  define UAVCAN_ASSERT(x)
# else
#  define UAVCAN_ASSERT(x) assert(x)
# endif
#endif

#ifndef UAVCAN_LIKELY
# if __GNUC__
#  define UAVCAN_LIKELY(x) __builtin_expect(!!(x), true)
# else
#  define UAVCAN_LIKELY(x) (x)
# endif
#endif

#ifndef UAVCAN_UNLIKELY
# if __GNUC__
#  define UAVCAN_UNLIKELY(x) __builtin_expect(!!(x), false)
# else
#  define UAVCAN_UNLIKELY(x) (x)
# endif
#endif

/**
 * This option allows to enable/disable CANFD frames support
 * enabling CANFD support increases frame and buffer size requirements 
 */
#ifndef UAVCAN_SUPPORT_CANFD
# define UAVCAN_SUPPORT_CANFD 0
#endif

namespace uavcan
{
/**
 * Memory pool block size.
 *
 * The default of 64 bytes should be OK for any target arch up to AMD64 and any compiler.
 *
 * The library leverages compile-time checks to ensure that all types that are subject to dynamic allocation
 * fit this size, otherwise compilation fails.
 *
 * For platforms featuring small pointer width (16..32 bits), UAVCAN_MEM_POOL_BLOCK_SIZE can often be safely
 * reduced to 56 or even 48 bytes, which leads to lower memory footprint.
 *
 * Note that the pool block size shall be aligned at biggest alignment of the target platform (detected and
 * checked automatically at compile time).
 */
#ifdef UAVCAN_MEM_POOL_BLOCK_SIZE
/// Explicitly specified by the user.
static const unsigned MemPoolBlockSize = UAVCAN_MEM_POOL_BLOCK_SIZE;
#elif defined(__BIGGEST_ALIGNMENT__) && (__BIGGEST_ALIGNMENT__ <= 8)
/// Convenient default for GCC-like compilers - if alignment allows, pool block size can be safely reduced.
#if UAVCAN_SUPPORT_CANFD
static const unsigned MemPoolBlockSize = 96;
#else
static const unsigned MemPoolBlockSize = 56;
#endif //UAVCAN_SUPPORT_CANFD
#else
/// Safe default that should be OK for any platform.
#if UAVCAN_SUPPORT_CANFD
static const unsigned MemPoolBlockSize = 128;
#else
static const unsigned MemPoolBlockSize = 64;
#endif // UAVCAN_SUPPORT_CANFD
#endif

#ifdef __BIGGEST_ALIGNMENT__
static const unsigned MemPoolAlignment = __BIGGEST_ALIGNMENT__;
#else
static const unsigned MemPoolAlignment = 16;
#endif

typedef char _alignment_check_for_MEM_POOL_BLOCK_SIZE[((MemPoolBlockSize & (MemPoolAlignment - 1)) == 0) ? 1 : -1];

/**
 * This class that allows to check at compile time whether type T can be allocated using the memory pool.
 * If the check fails, compilation fails.
 */
template <typename T>
struct UAVCAN_EXPORT IsDynamicallyAllocatable
{
    static void check()
    {
        char dummy[(sizeof(T) <= MemPoolBlockSize) ? 1 : -1] = { '0' };
        (void)dummy;
    }
};

/**
 * Float comparison precision.
 * For details refer to:
 *  http://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
 *  https://code.google.com/p/googletest/source/browse/trunk/include/gtest/internal/gtest-internal.h
 */
#ifdef UAVCAN_FLOAT_COMPARISON_EPSILON_MULT
static const unsigned FloatComparisonEpsilonMult = UAVCAN_FLOAT_COMPARISON_EPSILON_MULT;
#else
static const unsigned FloatComparisonEpsilonMult = 10;
#endif

/**
 * Maximum number of CAN acceptance filters available on the platform
 */
#ifdef UAVCAN_MAX_CAN_ACCEPTANCE_FILTERS
/// Explicitly specified by the user.
static const unsigned MaxCanAcceptanceFilters = UAVCAN_MAX_CAN_ACCEPTANCE_FILTERS;
#else
/// Default that should be OK for any platform.
static const unsigned MaxCanAcceptanceFilters = 32;
#endif

}

#endif // UAVCAN_BUILD_CONFIG_HPP_INCLUDED
