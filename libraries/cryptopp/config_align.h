// config_align.h - written and placed in public domain by Jeffrey Walton
//                  the bits that make up this source file are from the
//                  library's monolithic config.h.

/// \file config_align.h
/// \brief Library configuration file
/// \details <tt>config.h</tt> was split into components in May 2019 to better
///  integrate with Autoconf and its feature tests. The splitting occured so
///  users could continue to include <tt>config.h</tt> while allowing Autoconf
///  to write new <tt>config_asm.h</tt> and new <tt>config_cxx.h</tt> using
///  its feature tests.
/// \sa <A HREF="https://github.com/weidai11/cryptopp/issues/835">Issue 835</A>
/// \since Crypto++ 8.3

#ifndef CRYPTOPP_CONFIG_ALIGN_H
#define CRYPTOPP_CONFIG_ALIGN_H

#include "config_asm.h"  // CRYPTOPP_DISABLE_ASM
#include "config_cpu.h"  // X86, X32, X64, ARM32, ARM64, etc
#include "config_cxx.h"  // CRYPTOPP_CXX11_ALIGNAS
#include "config_ver.h"  // Compiler versions

// Nearly all Intel's and AMD's have SSE. Enable it independent of SSE ASM and intrinscs.
// ARM NEON and ARMv8 ASIMD only need natural alignment of an element in the vector.
// Altivec through POWER7 need vector alignment. POWER8 and POWER9 relax the requirement.
#if defined(CRYPTOPP_DISABLE_ASM)
	#define CRYPTOPP_BOOL_ALIGN16 0
#elif (CRYPTOPP_BOOL_X86 || CRYPTOPP_BOOL_X32 || CRYPTOPP_BOOL_X64 || \
       CRYPTOPP_BOOL_PPC32 || CRYPTOPP_BOOL_PPC64)
	#define CRYPTOPP_BOOL_ALIGN16 1
#else
	#define CRYPTOPP_BOOL_ALIGN16 0
#endif

// How to allocate 16-byte aligned memory (for SSE2)
// posix_memalign see https://forum.kde.org/viewtopic.php?p=66274
#if defined(_MSC_VER)
	#define CRYPTOPP_MM_MALLOC_AVAILABLE
#elif defined(__linux__) || defined(__sun__) || defined(__CYGWIN__)
	#define CRYPTOPP_MEMALIGN_AVAILABLE
#elif defined(__APPLE__) || defined(__NetBSD__) || defined(__OpenBSD__) || defined(__DragonFly__)
	#define CRYPTOPP_MALLOC_ALIGNMENT_IS_16
#elif (defined(_GNU_SOURCE) || ((_XOPEN_SOURCE + 0) >= 600)) && (_POSIX_ADVISORY_INFO > 0)
	#define CRYPTOPP_POSIX_MEMALIGN_AVAILABLE
#else
	#define CRYPTOPP_NO_ALIGNED_ALLOC
#endif

// Sun Studio Express 3 (December 2006) provides GCC-style attributes.
// IBM XL C/C++ alignment modifier per Optimization Guide, pp. 19-20.
// __IBM_ATTRIBUTES per XLC 12.1 AIX Compiler Manual, p. 473.
// CRYPTOPP_ALIGN_DATA may not be reliable on AIX.
#if defined(CRYPTOPP_CXX11_ALIGNAS)
	#define CRYPTOPP_ALIGN_DATA(x) alignas(x)
#elif defined(_MSC_VER)
	#define CRYPTOPP_ALIGN_DATA(x) __declspec(align(x))
#elif defined(__GNUC__) || defined(__clang__) || (__SUNPRO_CC >= 0x5100)
	#define CRYPTOPP_ALIGN_DATA(x) __attribute__((aligned(x)))
#elif defined(__xlc__) || defined(__xlC__)
	#define CRYPTOPP_ALIGN_DATA(x) __attribute__((aligned(x)))
#else
	#define CRYPTOPP_ALIGN_DATA(x)
#endif

#endif  // CRYPTOPP_CONFIG_ALIGN_H
