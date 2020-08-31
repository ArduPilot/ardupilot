// config_cpu.h - written and placed in public domain by Jeffrey Walton
//                the bits that make up this source file are from the
//                library's monolithic config.h.

/// \file config_cpu.h
/// \brief Library configuration file
/// \details <tt>config.h</tt> was split into components in May 2019 to better
///  integrate with Autoconf and its feature tests. The splitting occured so
///  users could continue to include <tt>config.h</tt> while allowing Autoconf
///  to write new <tt>config_asm.h</tt> and new <tt>config_cxx.h</tt> using
///  its feature tests.
/// \sa <A HREF="https://github.com/weidai11/cryptopp/issues/835">Issue 835</A>
///  <A HREF="https://sourceforge.net/p/predef/wiki/Architectures/">Sourceforge
///  Pre-defined Compiler Macros</A>
/// \since Crypto++ 8.3

#ifndef CRYPTOPP_CONFIG_CPU_H
#define CRYPTOPP_CONFIG_CPU_H

#include "config_ver.h"

#if (defined(__ILP32__) || defined(_ILP32)) && defined(__x86_64__)
	#define CRYPTOPP_BOOL_X32 1
#elif (defined(_M_X64) || defined(__x86_64__))
	#define CRYPTOPP_BOOL_X64 1
#elif (defined(_M_IX86) || defined(__i386__) || defined(__i386) || defined(_X86_) || defined(__I86__) || defined(__INTEL__))
	#define CRYPTOPP_BOOL_X86 1
#endif

// Microsoft added ARM64 define December 2017.
#if defined(__arm64__) || defined(__aarch32__) || defined(__aarch64__) || defined(_M_ARM64)
	#define CRYPTOPP_BOOL_ARMV8 1
#endif
#if defined(__arm64__) || defined(__aarch64__) || defined(_M_ARM64)
	#define CRYPTOPP_BOOL_ARM64 1
#elif defined(__arm__) || defined(_M_ARM)
	#define CRYPTOPP_BOOL_ARM32 1
#endif

// And PowerPC.
#if defined(__ppc64__) || defined(__powerpc64__) || defined(__PPC64__) || defined(_ARCH_PPC64)
	#define CRYPTOPP_BOOL_PPC64 1
#elif defined(__powerpc__) || defined(__ppc__) || defined(__PPC__) || defined(_ARCH_PPC)
	#define CRYPTOPP_BOOL_PPC32 1
#endif

// And MIPS. TODO: finish these defines
#if defined(__mips64__)
	#define CRYPTOPP_BOOL_MIPS64 1
#elif defined(__mips__)
	#define CRYPTOPP_BOOL_MIPS32 1
#endif

// And SPARC.
#if defined(__sparc64__) || defined(__sparc64) || defined(__sparcv9) || defined(__sparc_v9__)
	#define CRYPTOPP_BOOL_SPARC64 1
#elif defined(__sparc__) || defined(__sparc) || defined(__sparcv8) || defined(__sparc_v8__)
	#define CRYPTOPP_BOOL_SPARC32 1
#endif

// This should be a lower bound on the L1 cache line size.
// It's used for defense against timing attacks.
#ifndef CRYPTOPP_L1_CACHE_LINE_SIZE
	#if defined(CRYPTOPP_BOOL_X32) || defined(CRYPTOPP_BOOL_X64) || defined(CRYPTOPP_BOOL_ARMV8) || \
	    defined(CRYPTOPP_BOOL_PPC64) || defined(CRYPTOPP_BOOL_MIPS64) || defined(CRYPTOPP_BOOL_SPARC64)
		#define CRYPTOPP_L1_CACHE_LINE_SIZE 64
	#else
		// L1 cache line size is 32 on Pentium III and earlier
		#define CRYPTOPP_L1_CACHE_LINE_SIZE 32
	#endif
#endif

// The section attribute attempts to initialize CPU flags to avoid Valgrind findings above -O1
#if ((defined(__MACH__) && defined(__APPLE__)) && ((CRYPTOPP_LLVM_CLANG_VERSION >= 30600) || (CRYPTOPP_APPLE_CLANG_VERSION >= 70100) || (CRYPTOPP_GCC_VERSION >= 40300)))
	#define CRYPTOPP_SECTION_INIT __attribute__((section ("__DATA,__data")))
#elif (defined(__ELF__) && (CRYPTOPP_GCC_VERSION >= 40300))
	#define CRYPTOPP_SECTION_INIT __attribute__((section ("nocommon")))
#elif defined(__ELF__) && (defined(__xlC__) || defined(__ibmxl__))
	#define CRYPTOPP_SECTION_INIT __attribute__((section ("nocommon")))
#else
	#define CRYPTOPP_SECTION_INIT
#endif

// How to disable CPU feature probing. We determine machine
// capabilities by performing an os/platform *query* first,
// like getauxv(). If the *query* fails, we move onto a
// cpu *probe*. The cpu *probe* tries to exeute an instruction
// and then catches a SIGILL on Linux or the exception
// EXCEPTION_ILLEGAL_INSTRUCTION on Windows. Some OSes
// fail to hangle a SIGILL gracefully, like Apple OSes. Apple
// machines corrupt memory and variables around the probe.
#if defined(__APPLE__)
	#define CRYPTOPP_NO_CPU_FEATURE_PROBES 1
#endif

// Flavor of inline assembly language
#if defined(_MSC_VER) || defined(__BORLANDC__)
	#define CRYPTOPP_MS_STYLE_INLINE_ASSEMBLY 1
#else
	#define CRYPTOPP_GNU_STYLE_INLINE_ASSEMBLY 1
#endif

#endif
