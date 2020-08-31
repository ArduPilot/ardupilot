// config_cxx.h - written and placed in public domain by Jeffrey Walton
//                the bits that make up this source file are from the
//                library's monolithic config.h.

/// \file config_cxx.h
/// \brief Library configuration file
/// \details <tt>config.h</tt> was split into components in May 2019 to better
///  integrate with Autoconf and its feature tests. The splitting occured so
///  users could continue to include <tt>config.h</tt> while allowing Autoconf
///  to write new <tt>config_asm.h</tt> and new <tt>config_cxx.h</tt> using
///  its feature tests.
/// \sa <A HREF="https://github.com/weidai11/cryptopp/issues/835">Issue 835</A>
/// \since Crypto++ 8.3

// Visual Studio began at VS2010, http://msdn.microsoft.com/en-us/library/hh567368%28v=vs.110%29.aspx
//   and https://docs.microsoft.com/en-us/cpp/visual-cpp-language-conformance
// Intel, http://software.intel.com/en-us/articles/c0x-features-supported-by-intel-c-compiler
// GCC, http://gcc.gnu.org/projects/cxx0x.html
// Clang, http://clang.llvm.org/cxx_status.html

#ifndef CRYPTOPP_CONFIG_CXX_H
#define CRYPTOPP_CONFIG_CXX_H

#include "config_os.h"
#include "config_cpu.h"
#include "config_ver.h"

// https://github.com/weidai11/cryptopp/issues/960
#include <string>
#include <exception>

// You may need to force include a C++ header on Android when using STLPort
// to ensure _STLPORT_VERSION is defined
#if (defined(_MSC_VER) && _MSC_VER <= 1300) || defined(__MWERKS__) || (defined(_STLPORT_VERSION) && ((_STLPORT_VERSION < 0x450) || defined(_STLP_NO_UNCAUGHT_EXCEPT_SUPPORT)))
#define CRYPTOPP_DISABLE_UNCAUGHT_EXCEPTION
#endif

// Ancient Crypto++ define, dating back to C++98.
#ifndef CRYPTOPP_DISABLE_UNCAUGHT_EXCEPTION
# define CRYPTOPP_UNCAUGHT_EXCEPTION_AVAILABLE 1
# define CRYPTOPP_CXX98_UNCAUGHT_EXCEPTION 1
#endif

// Compatibility with non-clang compilers.
#ifndef __has_feature
# define __has_feature(x) 0
#endif

// Define CRYPTOPP_NO_CXX11 to avoid C++11 related features shown at the
// end of this file. Some compilers and standard C++ headers advertise C++11
// but they are really just C++03 with some additional C++11 headers and
// non-conforming classes. Also see Issues 529.
// #define CRYPTOPP_NO_CXX11 1

// Define CRYPTOPP_NO_CXX17 to avoid C++17 related features shown at the end of
// this file. At the moment it should only affect std::uncaught_exceptions.
// #define CRYPTOPP_NO_CXX17 1

// C++11 macro version, https://stackoverflow.com/q/7223991/608639
#if !defined(CRYPTOPP_NO_CXX11)
#  if ((_MSC_VER >= 1600) || (__cplusplus >= 201103L)) && !defined(_STLPORT_VERSION)
#    define CRYPTOPP_CXX11 1
#  endif
#endif

// Hack ahead. Apple's standard library does not have C++'s unique_ptr in C++11.
// We can't test for unique_ptr directly because some of the non-Apple Clangs
// on OS X fail the same way. However, modern standard libraries have
// <forward_list>, so we test for it instead. Thanks to Jonathan Wakely for
// devising the clever test for modern/ancient versions. TODO: test under
// Xcode 3, where g++ is really g++.
#if defined(__APPLE__) && defined(__clang__)
#  if !(defined(__has_include) && __has_include(<forward_list>))
#    undef CRYPTOPP_CXX11
#  endif
#endif

// C++14 macro version, https://stackoverflow.com/q/26089319/608639
#if defined(CRYPTOPP_CXX11) && !defined(CRYPTOPP_NO_CXX14)
#  if ((_MSC_VER >= 1900) || (__cplusplus >= 201402L)) && !defined(_STLPORT_VERSION)
#    define CRYPTOPP_CXX14 1
#  endif
#endif

// C++17 macro version, https://stackoverflow.com/q/38456127/608639
#if defined(CRYPTOPP_CXX14) && !defined(CRYPTOPP_NO_CXX17)
#  if ((_MSC_VER >= 1900) || (__cplusplus >= 201703L)) && !defined(_STLPORT_VERSION)
#    define CRYPTOPP_CXX17 1
#  endif
#endif

// ***************** C++11 and above ********************

#if defined(CRYPTOPP_CXX11)

// atomics: MS at VS2012 (17.00); GCC at 4.4; Clang at 3.1/3.2; Intel 13.0; SunCC 5.14.
#if (CRYPTOPP_MSC_VERSION >= 1700) || __has_feature(cxx_atomic) || \
	(__INTEL_COMPILER >= 1300) || (CRYPTOPP_GCC_VERSION >= 40400) || (__SUNPRO_CC >= 0x5140)
# define CRYPTOPP_CXX11_ATOMIC 1
#endif // atomics

// synchronization: MS at VS2012 (17.00); GCC at 4.4; Clang at 3.3; Xcode 5.0; Intel 12.0; SunCC 5.13.
// TODO: verify Clang and Intel versions; find __has_feature(x) extension for Clang
#if (CRYPTOPP_MSC_VERSION >= 1700) || (CRYPTOPP_LLVM_CLANG_VERSION >= 30300) || \
	(CRYPTOPP_APPLE_CLANG_VERSION >= 50000) || (__INTEL_COMPILER >= 1200) || \
	(CRYPTOPP_GCC_VERSION >= 40400) || (__SUNPRO_CC >= 0x5130)
// Hack ahead. New GCC compilers like GCC 6 on AIX 7.0 or earlier as well as original MinGW
// don't have the synchronization gear. However, Wakely's test used for Apple does not work
// on the GCC/AIX combination. Another twist is we need other stuff from C++11,
// like no-except destructors. Dumping preprocessors shows the following may
// apply: http://stackoverflow.com/q/14191566/608639.
# include <cstddef>
# if !defined(__GLIBCXX__) || defined(_GLIBCXX_HAS_GTHREADS)
#  define CRYPTOPP_CXX11_SYNCHRONIZATION 1
# endif
#endif // synchronization

// Dynamic Initialization and Destruction with Concurrency ("Magic Statics")
// MS at VS2015 with Vista (19.00); GCC at 4.3; LLVM Clang at 2.9; Apple Clang at 4.0; Intel 11.1; SunCC 5.13.
// Microsoft's implementation only works for Vista and above, so its further
// limited. http://connect.microsoft.com/VisualStudio/feedback/details/1789709
// Clang may not support this as early as we indicate. Also see https://bugs.llvm.org/show_bug.cgi?id=47012.
#if (__cpp_threadsafe_static_init >= 200806) || \
	(CRYPTOPP_MSC_VERSION >= 1900) && ((WINVER >= 0x0600) || (_WIN32_WINNT >= 0x0600)) || \
	(CRYPTOPP_LLVM_CLANG_VERSION >= 20900) || (CRYPTOPP_APPLE_CLANG_VERSION >= 40000)  || \
	(__INTEL_COMPILER >= 1110) || (CRYPTOPP_GCC_VERSION >= 40300) || (__SUNPRO_CC >= 0x5130)
# define CRYPTOPP_CXX11_STATIC_INIT 1
#endif // Dynamic Initialization compilers

// deleted functions: MS at VS2013 (18.00); GCC at 4.3; Clang at 2.9; Intel 12.1; SunCC 5.13.
#if (CRYPTOPP_MSC_VERSION >= 1800) || (CRYPTOPP_LLVM_CLANG_VERSION >= 20900) || \
	(CRYPTOPP_APPLE_CLANG_VERSION >= 40000) || (__INTEL_COMPILER >= 1210) || \
	(CRYPTOPP_GCC_VERSION >= 40300) || (__SUNPRO_CC >= 0x5130)
# define CRYPTOPP_CXX11_DELETED_FUNCTIONS 1
#endif // deleted functions

// alignof/alignas: MS at VS2015 (19.00); GCC at 4.8; Clang at 3.0; Intel 15.0; SunCC 5.13.
#if (CRYPTOPP_MSC_VERSION >= 1900) || __has_feature(cxx_alignas) || \
	(__INTEL_COMPILER >= 1500) || (CRYPTOPP_GCC_VERSION >= 40800) || (__SUNPRO_CC >= 0x5130)
#  define CRYPTOPP_CXX11_ALIGNAS 1
#endif // alignas

// alignof: MS at VS2015 (19.00); GCC at 4.5; Clang at 2.9; Intel 15.0; SunCC 5.13.
#if (CRYPTOPP_MSC_VERSION >= 1900) || __has_feature(cxx_alignof) || \
	(__INTEL_COMPILER >= 1500) || (CRYPTOPP_GCC_VERSION >= 40500) || (__SUNPRO_CC >= 0x5130)
#  define CRYPTOPP_CXX11_ALIGNOF 1
#endif // alignof

// initializer lists: MS at VS2013 (18.00); GCC at 4.4; Clang at 3.1; Intel 14.0; SunCC 5.13.
#if (CRYPTOPP_MSC_VERSION >= 1800) || (CRYPTOPP_LLVM_CLANG_VERSION >= 30100) || \
	(CRYPTOPP_APPLE_CLANG_VERSION >= 40000) || (__INTEL_COMPILER >= 1400) || \
	(CRYPTOPP_GCC_VERSION >= 40400) || (__SUNPRO_CC >= 0x5130)
#  define CRYPTOPP_CXX11_INITIALIZER_LIST 1
#endif // alignas

// lambdas: MS at VS2012 (17.00); GCC at 4.9; Clang at 3.3; Intel 12.0; SunCC 5.14.
#if (CRYPTOPP_MSC_VERSION >= 1700) || __has_feature(cxx_lambdas) || \
	(__INTEL_COMPILER >= 1200) || (CRYPTOPP_GCC_VERSION >= 40900) || (__SUNPRO_CC >= 0x5140)
#  define CRYPTOPP_CXX11_LAMBDA 1
#endif // lambdas

// noexcept: MS at VS2015 (19.00); GCC at 4.6; Clang at 3.0; Intel 14.0; SunCC 5.13.
#if (CRYPTOPP_MSC_VERSION >= 1900) || __has_feature(cxx_noexcept) || \
	(__INTEL_COMPILER >= 1400) || (CRYPTOPP_GCC_VERSION >= 40600) || (__SUNPRO_CC >= 0x5130)
# define CRYPTOPP_CXX11_NOEXCEPT 1
#endif // noexcept compilers

// variadic templates: MS at VS2013 (18.00); GCC at 4.3; Clang at 2.9; Intel 12.1; SunCC 5.13.
#if (__cpp_variadic_templates >= 200704) || __has_feature(cxx_variadic_templates) || \
	(CRYPTOPP_MSC_VERSION >= 1800) || (__INTEL_COMPILER >= 1210) || \
	(CRYPTOPP_GCC_VERSION >= 40300) || (__SUNPRO_CC >= 0x5130)
# define CRYPTOPP_CXX11_VARIADIC_TEMPLATES 1
#endif // variadic templates

// constexpr: MS at VS2015 (19.00); GCC at 4.6; Clang at 3.1; Intel 16.0; SunCC 5.13.
// Intel has mis-supported the feature since at least ICPC 13.00
#if (__cpp_constexpr >= 200704) || __has_feature(cxx_constexpr) || \
	(CRYPTOPP_MSC_VERSION >= 1900) || (__INTEL_COMPILER >= 1600) || \
	(CRYPTOPP_GCC_VERSION >= 40600) || (__SUNPRO_CC >= 0x5130)
# define CRYPTOPP_CXX11_CONSTEXPR 1
#endif // constexpr compilers

// strong typed enums: MS at VS2012 (17.00); GCC at 4.4; Clang at 3.3; Intel 14.0; SunCC 5.12.
// Mircorosft and Intel had partial support earlier, but we require full support.
#if (CRYPTOPP_MSC_VERSION >= 1700) || __has_feature(cxx_strong_enums) || \
	(__INTEL_COMPILER >= 1400) || (CRYPTOPP_GCC_VERSION >= 40400) || (__SUNPRO_CC >= 0x5120)
# define CRYPTOPP_CXX11_STRONG_ENUM 1
#endif // constexpr compilers

// nullptr_t: MS at VS2010 (16.00); GCC at 4.6; Clang at 3.3; Intel 10.0; SunCC 5.13.
#if (CRYPTOPP_MSC_VERSION >= 1600) || __has_feature(cxx_nullptr) || \
	(__INTEL_COMPILER >= 1000) || (CRYPTOPP_GCC_VERSION >= 40600) || \
    (__SUNPRO_CC >= 0x5130) || defined(__IBMCPP_NULLPTR)
# define CRYPTOPP_CXX11_NULLPTR 1
#endif // nullptr_t compilers

#endif // CRYPTOPP_CXX11

// ***************** C++14 and above ********************

#if defined(CRYPTOPP_CXX14)

// Extended static_assert with one argument
// Microsoft cannot handle the single argument static_assert as of VS2019 (cl.exe 19.00)
#if (__cpp_static_assert >= 201411)
# define CRYPTOPP_CXX17_STATIC_ASSERT 1
#endif // static_assert

#endif

// ***************** C++17 and above ********************

// C++17 is available
#if defined(CRYPTOPP_CXX17)

// C++17 uncaught_exceptions: MS at VS2015 (19.00); GCC at 6.0; Clang at 3.5; Intel 18.0.
// Clang and __EXCEPTIONS see http://releases.llvm.org/3.6.0/tools/clang/docs/ReleaseNotes.html
#if defined(__clang__)
# if __EXCEPTIONS && __has_feature(cxx_exceptions)
#  if __cpp_lib_uncaught_exceptions
#   define CRYPTOPP_CXX17_UNCAUGHT_EXCEPTIONS 1
#  endif
# endif
#elif (CRYPTOPP_MSC_VERSION >= 1900) || (__INTEL_COMPILER >= 1800) || \
      (CRYPTOPP_GCC_VERSION >= 60000) || (__cpp_lib_uncaught_exceptions)
# define CRYPTOPP_CXX17_UNCAUGHT_EXCEPTIONS 1
#endif // uncaught_exceptions compilers

#endif  // CRYPTOPP_CXX17

// ***************** C++ fixups ********************

#if defined(CRYPTOPP_CXX11_NOEXCEPT)
#  define CRYPTOPP_THROW noexcept(false)
#  define CRYPTOPP_NO_THROW noexcept(true)
#else
#  define CRYPTOPP_THROW
#  define CRYPTOPP_NO_THROW
#endif // CRYPTOPP_CXX11_NOEXCEPT

// Hack... C++11 nullptr_t type safety and analysis
#if defined(CRYPTOPP_CXX11_NULLPTR) && !defined(NULLPTR)
# define NULLPTR nullptr
#elif !defined(NULLPTR)
# define NULLPTR NULL
#endif // CRYPTOPP_CXX11_NULLPTR

#endif  // CRYPTOPP_CONFIG_CXX_H
