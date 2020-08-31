// config_os.h - written and placed in public domain by Jeffrey Walton
//               the bits that make up this source file are from the
//               library's monolithic config.h.

/// \file config_os.h
/// \brief Library configuration file
/// \details <tt>config.h</tt> was split into components in May 2019 to better
///  integrate with Autoconf and its feature tests. The splitting occured so
///  users could continue to include <tt>config.h</tt> while allowing Autoconf
///  to write new <tt>config_asm.h</tt> and new <tt>config_cxx.h</tt> using
///  its feature tests.
/// \sa <A HREF="https://github.com/weidai11/cryptopp/issues/835">Issue 835</A>
/// \since Crypto++ 8.3

#ifndef CRYPTOPP_CONFIG_OS_H
#define CRYPTOPP_CONFIG_OS_H

#include "config_ver.h"

// It is OK to remove the hard stop below, but you are on your own.
// After building the library be sure to run self tests described
// https://www.cryptopp.com/wiki/Release_Process#Self_Tests
// Some relevant bug reports can be found at:
// * Clang: http://github.com/weidai11/cryptopp/issues/147
#if (defined(_MSC_VER) && defined(__clang__) && !(defined( __clang_analyzer__)))
# error: "Unsupported configuration"
#endif

// Windows platform
#if defined(_WIN32) || defined(_WIN64) || defined(__CYGWIN__)
#define CRYPTOPP_WIN32_AVAILABLE
#endif

// Unix and Linux platforms
#if defined(__unix__) || defined(__MACH__) || defined(__NetBSD__) || defined(__sun)
#define CRYPTOPP_UNIX_AVAILABLE
#endif

// BSD platforms
#if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__) || defined(__DragonFly__)
#define CRYPTOPP_BSD_AVAILABLE
#endif

// Microsoft compilers
#if defined(_MSC_VER) || defined(__fastcall)
	#define CRYPTOPP_FASTCALL __fastcall
#else
	#define CRYPTOPP_FASTCALL
#endif

// Microsoft compilers
#if defined(_MSC_VER)
	#define CRYPTOPP_NO_VTABLE __declspec(novtable)
#else
	#define CRYPTOPP_NO_VTABLE
#endif

// Define this if you want to disable all OS-dependent features,
// such as sockets and OS-provided random number generators
// #define NO_OS_DEPENDENCE

// Define this to use features provided by Microsoft's CryptoAPI.
// Currently the only feature used is Windows random number generation.
// This macro will be ignored if NO_OS_DEPENDENCE is defined.
// #define USE_MS_CRYPTOAPI

// Define this to use features provided by Microsoft's CryptoNG API.
// CryptoNG API is available in Vista and above and its cross platform,
// including desktop apps and store apps. Currently the only feature
// used is Windows random number generation.
// This macro will be ignored if NO_OS_DEPENDENCE is defined.
// #define USE_MS_CNGAPI

// If the user did not make a choice, then select CryptoNG if
// targeting Windows 8 or above.
#if !defined(USE_MS_CRYPTOAPI) && !defined(USE_MS_CNGAPI)
# if !defined(_USING_V110_SDK71_) && ((WINVER >= 0x0602 /*_WIN32_WINNT_WIN8*/) || \
     (_WIN32_WINNT >= 0x0602 /*_WIN32_WINNT_WIN8*/))
#  define USE_MS_CNGAPI
# else
#  define USE_MS_CRYPTOAPI
# endif
#endif

// Begin OS features, like init priorities and random numbers
#ifndef NO_OS_DEPENDENCE

// CRYPTOPP_INIT_PRIORITY attempts to manage initialization of C++ static objects.
// Under GCC, the library uses init_priority attribute in the range
// [CRYPTOPP_INIT_PRIORITY, CRYPTOPP_INIT_PRIORITY+100]. Under Windows,
// CRYPTOPP_INIT_PRIORITY enlists "#pragma init_seg(lib)". The platforms
// with gaps are Apple and Sun because they require linker scripts. Apple and
// Sun will use the library's Singletons to initialize and acquire resources.
// Also see http://cryptopp.com/wiki/Static_Initialization_Order_Fiasco
#ifndef CRYPTOPP_INIT_PRIORITY
# define CRYPTOPP_INIT_PRIORITY 250
#endif

// CRYPTOPP_USER_PRIORITY is for other libraries and user code that is using Crypto++
// and managing C++ static object creation. It is guaranteed not to conflict with
// values used by (or would be used by) the Crypto++ library.
#ifndef CRYPTOPP_USER_PRIORITY
# define CRYPTOPP_USER_PRIORITY (CRYPTOPP_INIT_PRIORITY+101)
#endif

// Most platforms allow us to specify when to create C++ objects. Apple and Sun do not.
#if (CRYPTOPP_INIT_PRIORITY > 0) && !(defined(NO_OS_DEPENDENCE) || defined(__APPLE__) || defined(__sun__))
# if (CRYPTOPP_GCC_VERSION >= 30000) || (CRYPTOPP_LLVM_CLANG_VERSION >= 20900) || (_INTEL_COMPILER >= 800)
#  define HAVE_GCC_INIT_PRIORITY 1
# elif (CRYPTOPP_MSC_VERSION >= 1310)
#  define HAVE_MSC_INIT_PRIORITY 1
# elif defined(__xlc__) || defined(__xlC__) || defined(__ibmxl__)
#  define HAVE_XLC_INIT_PRIORITY 1
# endif
#endif  // CRYPTOPP_INIT_PRIORITY, NO_OS_DEPENDENCE, Apple, Sun

#if defined(CRYPTOPP_WIN32_AVAILABLE) || defined(CRYPTOPP_UNIX_AVAILABLE)
#	define HIGHRES_TIMER_AVAILABLE
#endif

#ifdef CRYPTOPP_WIN32_AVAILABLE
# if !defined(WINAPI_FAMILY)
#	define THREAD_TIMER_AVAILABLE
# elif defined(WINAPI_FAMILY)
#   if (WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP))
#	  define THREAD_TIMER_AVAILABLE
#  endif
# endif
#endif

#if defined(CRYPTOPP_UNIX_AVAILABLE) || defined(CRYPTOPP_DOXYGEN_PROCESSING)
#	define NONBLOCKING_RNG_AVAILABLE
#	define BLOCKING_RNG_AVAILABLE
#	define OS_RNG_AVAILABLE
#endif

// Cygwin/Newlib requires _XOPEN_SOURCE=600
#if defined(CRYPTOPP_UNIX_AVAILABLE)
# define UNIX_SIGNALS_AVAILABLE 1
#endif

#ifdef CRYPTOPP_WIN32_AVAILABLE
# if !defined(WINAPI_FAMILY)
#	define NONBLOCKING_RNG_AVAILABLE
#	define OS_RNG_AVAILABLE
# elif defined(WINAPI_FAMILY)
#   if (WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP))
#	  define NONBLOCKING_RNG_AVAILABLE
#	  define OS_RNG_AVAILABLE
#   elif !(WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP))
#     if ((WINVER >= 0x0A00 /*_WIN32_WINNT_WIN10*/) || (_WIN32_WINNT >= 0x0A00 /*_WIN32_WINNT_WIN10*/))
#	    define NONBLOCKING_RNG_AVAILABLE
#	    define OS_RNG_AVAILABLE
#     endif
#   endif
# endif
#endif

#endif	// NO_OS_DEPENDENCE

#endif  // CRYPTOPP_CONFIG_OS_H
