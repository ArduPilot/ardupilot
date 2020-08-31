// config_misc.h - written and placed in public domain by Jeffrey Walton
//                 the bits that make up this source file are from the
//                 library's monolithic config.h.

/// \file config_misc.h
/// \brief Library configuration file
/// \details <tt>config.h</tt> was split into components in May 2019 to better
///  integrate with Autoconf and its feature tests. The splitting occured so
///  users could continue to include <tt>config.h</tt> while allowing Autoconf
///  to write new <tt>config_asm.h</tt> and new <tt>config_cxx.h</tt> using
///  its feature tests.
/// \sa <A HREF="https://github.com/weidai11/cryptopp/issues/835">Issue 835</A>
/// \since Crypto++ 8.3

#ifndef CRYPTOPP_CONFIG_MISC_H
#define CRYPTOPP_CONFIG_MISC_H

#include "config_asm.h"
#include "config_cxx.h"
#include "config_os.h"
#include "config_ver.h"

// Define this if running on a big-endian CPU
// big endian will be assumed if CRYPTOPP_LITTLE_ENDIAN is not non-0
#if !defined(CRYPTOPP_LITTLE_ENDIAN) && !defined(CRYPTOPP_BIG_ENDIAN) && (defined(__BIG_ENDIAN__) || (defined(__s390__) || defined(__s390x__) || defined(__zarch__)) || (defined(__m68k__) || defined(__MC68K__)) || defined(__sparc) || defined(__sparc__) || defined(__hppa__) || defined(__MIPSEB__) || defined(__ARMEB__) || (defined(__MWERKS__) && !defined(__INTEL__)))
#	define CRYPTOPP_BIG_ENDIAN 1
#endif

// Define this if running on a little-endian CPU
// big endian will be assumed if CRYPTOPP_LITTLE_ENDIAN is not non-0
#if !defined(CRYPTOPP_BIG_ENDIAN) && !defined(CRYPTOPP_LITTLE_ENDIAN)
#	define CRYPTOPP_LITTLE_ENDIAN 1
#endif

// Define this if you want to set a prefix for TestData/ and TestVectors/
// Be sure to add the trailing slash since its simple concatenation.
// After https://github.com/weidai11/cryptopp/issues/760 the library
// should find the test vectors and data without much effort. It
// will search in "./" and "$ORIGIN/../share/cryptopp" automatically.
#ifndef CRYPTOPP_DATA_DIR
# define CRYPTOPP_DATA_DIR ""
#endif

// Define this to disable the test suite from searching for test
// vectors and data in "./" and "$ORIGIN/../share/cryptopp". The
// library will still search in CRYPTOPP_DATA_DIR, regardless.
// Some distros may want to disable this feature. Also see
// https://github.com/weidai11/cryptopp/issues/760
// #ifndef CRYPTOPP_DISABLE_DATA_DIR_SEARCH
// # define CRYPTOPP_DISABLE_DATA_DIR_SEARCH
// #endif

// Define this if you want or need the library's memcpy_s and memmove_s.
// See http://github.com/weidai11/cryptopp/issues/28.
// #if !defined(CRYPTOPP_WANT_SECURE_LIB)
// # define CRYPTOPP_WANT_SECURE_LIB
// #endif

// Define this if ARMv8 shifts are slow. ARM Cortex-A53 and Cortex-A57 shift
// operation perform poorly, so NEON and ASIMD code that relies on shifts
// or rotates often performs worse than C/C++ code. Also see
// http://github.com/weidai11/cryptopp/issues/367.
#define CRYPTOPP_SLOW_ARMV8_SHIFT 1

// CRYPTOPP_DEBUG enables the library's CRYPTOPP_ASSERT. CRYPTOPP_ASSERT
// raises a SIGTRAP (Unix) or calls DebugBreak() (Windows). CRYPTOPP_ASSERT
// is only in effect when CRYPTOPP_DEBUG, DEBUG or _DEBUG is defined. Unlike
// Posix assert, CRYPTOPP_ASSERT is not affected by NDEBUG (or failure to
// define it).
// Also see http://github.com/weidai11/cryptopp/issues/277, CVE-2016-7420
#if (defined(DEBUG) || defined(_DEBUG)) && !defined(CRYPTOPP_DEBUG)
# define CRYPTOPP_DEBUG 1
#endif

// File system code to use when creating GZIP archive.
// http://www.gzip.org/format.txt
#if !defined(GZIP_OS_CODE)
# if defined(__macintosh__)
#  define GZIP_OS_CODE 7
# elif defined(__unix__) || defined(__linux__)
#  define GZIP_OS_CODE 3
# else
#  define GZIP_OS_CODE 0
# endif
#endif

// Try this if your CPU has 256K internal cache or a slow multiply instruction
// and you want a (possibly) faster IDEA implementation using log tables
// #define IDEA_LARGECACHE

// Define this if, for the linear congruential RNG, you want to use
// the original constants as specified in S.K. Park and K.W. Miller's
// CACM paper.
// #define LCRNG_ORIGINAL_NUMBERS

// Define this if you want Integer's operator<< to honor std::showbase (and
// std::noshowbase). If defined, Integer will use a suffix of 'b', 'o', 'h'
// or '.' (the last for decimal) when std::showbase is in effect. If
// std::noshowbase is set, then the suffix is not added to the Integer. If
// not defined, existing behavior is preserved and Integer will use a suffix
// of 'b', 'o', 'h' or '.' (the last for decimal).
// #define CRYPTOPP_USE_STD_SHOWBASE

// Define this if you want to decouple AlgorithmParameters and Integer
// The decoupling should make it easier for the linker to remove Integer
// related code for those who do not need Integer, and avoid a potential
// race during AssignIntToInteger pointer initialization. Also
// see http://github.com/weidai11/cryptopp/issues/389.
// #define CRYPTOPP_NO_ASSIGN_TO_INTEGER

// Need GCC 4.6/Clang 1.7/Apple Clang 2.0 or above due to "GCC diagnostic {push|pop}"
#if (CRYPTOPP_GCC_VERSION >= 40600) || (CRYPTOPP_LLVM_CLANG_VERSION >= 10700) || \
    (CRYPTOPP_APPLE_CLANG_VERSION >= 20000)
	#define CRYPTOPP_GCC_DIAGNOSTIC_AVAILABLE 1
#endif

// Portable way to suppress warnings.
// Moved from misc.h due to circular depenedencies.
#ifndef CRYPTOPP_UNUSED
	#define CRYPTOPP_UNUSED(x) ((void)(x))
#endif

// how to disable inlining
#if defined(_MSC_VER)
#	define CRYPTOPP_NOINLINE_DOTDOTDOT
#	define CRYPTOPP_NOINLINE __declspec(noinline)
#elif defined(__xlc__) || defined(__xlC__) || defined(__ibmxl__)
#	define CRYPTOPP_NOINLINE_DOTDOTDOT ...
#	define CRYPTOPP_NOINLINE __attribute__((noinline))
#elif defined(__GNUC__)
#	define CRYPTOPP_NOINLINE_DOTDOTDOT
#	define CRYPTOPP_NOINLINE __attribute__((noinline))
#else
#	define CRYPTOPP_NOINLINE_DOTDOTDOT ...
#	define CRYPTOPP_NOINLINE
#endif

// http://stackoverflow.com/a/13867690/608639
#if defined(CRYPTOPP_CXX11_CONSTEXPR)
#  define CRYPTOPP_STATIC_CONSTEXPR static constexpr
#  define CRYPTOPP_CONSTEXPR constexpr
#else
#  define CRYPTOPP_STATIC_CONSTEXPR static
#  define CRYPTOPP_CONSTEXPR
#endif // CRYPTOPP_CXX11_CONSTEXPR

#if defined(CRYPTOPP_DOXYGEN_PROCESSING)
# define CRYPTOPP_CONSTANT(x) static const int x
#elif defined(CRYPTOPP_CXX11_STRONG_ENUM)
# define CRYPTOPP_CONSTANT(x) enum : int { x }
#elif defined(CRYPTOPP_CXX11_CONSTEXPR)
# define CRYPTOPP_CONSTANT(x) constexpr static int x
#else
# define CRYPTOPP_CONSTANT(x) static const int x
#endif

// Warnings
#ifdef _MSC_VER
	// 4127: conditional expression is constant
	// 4512: assignment operator not generated
	// 4661: no suitable definition provided for explicit template instantiation request
	// 4910: '__declspec(dllexport)' and 'extern' are incompatible on an explicit instantiation
#	pragma warning(disable: 4127 4512 4661 4910)
	// Security related, possible defects
	// http://blogs.msdn.com/b/vcblog/archive/2010/12/14/off-by-default-compiler-warnings-in-visual-c.aspx
#	pragma warning(once: 4191 4242 4263 4264 4266 4302 4826 4905 4906 4928)
#endif

#ifdef __BORLANDC__
// 8037: non-const function called for const object. needed to work around BCB2006 bug
#	pragma warn -8037
#endif

// [GCC Bug 53431] "C++ preprocessor ignores #pragma GCC diagnostic". Clang honors it.
#if CRYPTOPP_GCC_DIAGNOSTIC_AVAILABLE
# pragma GCC diagnostic ignored "-Wunknown-pragmas"
# pragma GCC diagnostic ignored "-Wunused-function"
#endif

#endif  // CRYPTOPP_CONFIG_MISC_H
