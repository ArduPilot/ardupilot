// config_asm.h - written and placed in public domain by Jeffrey Walton
//                the bits that make up this source file are from the
//                library's monolithic config.h.

/// \file config_asm.h
/// \brief Library configuration file
/// \details <tt>config.h</tt> was split into components in May 2019 to better
///  integrate with Autoconf and its feature tests. The splitting occured so
///  users could continue to include <tt>config.h</tt> while allowing Autoconf
///  to write new <tt>config_asm.h</tt> and new <tt>config_cxx.h</tt> using
///  its feature tests.
/// \sa <A HREF="https://github.com/weidai11/cryptopp/issues/835">Issue 835</A>
/// \since Crypto++ 8.3

#ifndef CRYPTOPP_CONFIG_ASM_H
#define CRYPTOPP_CONFIG_ASM_H

#include "config_os.h"
#include "config_cpu.h"
#include "config_ver.h"

// Define this to disable ASM, intrinsics and built-ins. The library will be
// compiled using C++ only. The library code will not include SSE2 (and
// above), NEON, Aarch32, Aarch64, or Altivec (and above). Note the compiler
// may use higher ISAs depending on compiler options, but the library will not
// explictly use the ISAs. When disabling ASM, it is best to do it from
// config.h to ensure the library and all programs share the setting.
// #define CRYPTOPP_DISABLE_ASM 1

// https://github.com/weidai11/cryptopp/issues/719
#if defined(__native_client__)
# undef CRYPTOPP_DISABLE_ASM
# define CRYPTOPP_DISABLE_ASM 1
#endif

// Some Clang and SunCC cannot handle mixed asm with positional arguments,
// where the body is Intel style with no prefix and the templates are
// AT&T style. Define this if the Makefile misdetects the configuration.
// Also see https://bugs.llvm.org/show_bug.cgi?id=39895 .
// #define CRYPTOPP_DISABLE_MIXED_ASM 1

#if defined(__clang__)
# undef CRYPTOPP_DISABLE_MIXED_ASM
# define CRYPTOPP_DISABLE_MIXED_ASM 1
#endif

// CRYPTOPP_ALLOW_UNALIGNED_DATA_ACCESS is no longer honored. It
// was removed at https://github.com/weidai11/cryptopp/issues/682
// #define CRYPTOPP_ALLOW_UNALIGNED_DATA_ACCESS 1

// ***************** IA32 CPU features ********************

#if (CRYPTOPP_BOOL_X86 || CRYPTOPP_BOOL_X32 || CRYPTOPP_BOOL_X64)

// Apple Clang prior to 5.0 cannot handle SSE2
#if defined(CRYPTOPP_APPLE_CLANG_VERSION) && (CRYPTOPP_APPLE_CLANG_VERSION < 50000)
# define CRYPTOPP_DISABLE_ASM 1
#endif

// Sun Studio 12.1 provides GCC inline assembly
// http://blogs.oracle.com/x86be/entry/gcc_style_asm_inlining_support
#if defined(__SUNPRO_CC) && (__SUNPRO_CC < 0x5100)
# define CRYPTOPP_DISABLE_ASM 1
#endif

// Guard everything in CRYPTOPP_DISABLE_ASM
#if !defined(CRYPTOPP_DISABLE_ASM)

#if (defined(_MSC_VER) && defined(_M_IX86)) || ((defined(__GNUC__) && (defined(__i386__)) || defined(__x86_64__)))
	// C++Builder 2010 does not allow "call label" where label is defined within inline assembly
	#define CRYPTOPP_X86_ASM_AVAILABLE 1

	#if !defined(CRYPTOPP_DISABLE_SSE2) && (defined(_MSC_VER) || CRYPTOPP_GCC_VERSION >= 30300 || defined(__SSE2__))
		#define CRYPTOPP_SSE2_ASM_AVAILABLE 1
	#endif

	#if !defined(CRYPTOPP_DISABLE_SSSE3) && (_MSC_VER >= 1500 || CRYPTOPP_GCC_VERSION >= 40300 || defined(__SSSE3__))
		#define CRYPTOPP_SSSE3_ASM_AVAILABLE 1
	#endif
#endif

#if defined(_MSC_VER) && defined(_M_X64)
	#define CRYPTOPP_X64_MASM_AVAILABLE 1
#endif

#if defined(__GNUC__) && defined(__x86_64__)
	#define CRYPTOPP_X64_ASM_AVAILABLE 1
#endif

// 32-bit SunCC does not enable SSE2 by default.
#if !defined(CRYPTOPP_DISABLE_SSE2) && (defined(_MSC_VER) || CRYPTOPP_GCC_VERSION >= 30300 || defined(__SSE2__) || (__SUNPRO_CC >= 0x5100))
	#define CRYPTOPP_SSE2_INTRIN_AVAILABLE 1
#endif

#if !defined(CRYPTOPP_DISABLE_SSSE3)
# if defined(__SSSE3__) || (_MSC_VER >= 1500) || \
	(CRYPTOPP_GCC_VERSION >= 40300) || (__INTEL_COMPILER >= 1000) || (__SUNPRO_CC >= 0x5110) || \
	(CRYPTOPP_LLVM_CLANG_VERSION >= 20300) || (CRYPTOPP_APPLE_CLANG_VERSION >= 40000)
	#define CRYPTOPP_SSSE3_AVAILABLE 1
# endif
#endif

// Intrinsics availible in GCC 4.3 (http://gcc.gnu.org/gcc-4.3/changes.html) and
// MSVC 2008 (http://msdn.microsoft.com/en-us/library/bb892950%28v=vs.90%29.aspx)
// SunCC could generate SSE4 at 12.1, but the intrinsics are missing until 12.4.
#if !defined(CRYPTOPP_DISABLE_SSE4) && defined(CRYPTOPP_SSSE3_AVAILABLE) && \
	(defined(__SSE4_1__) || (CRYPTOPP_MSC_VERSION >= 1500) || \
	(CRYPTOPP_GCC_VERSION >= 40300) || (__INTEL_COMPILER >= 1000) || (__SUNPRO_CC >= 0x5110) || \
	(CRYPTOPP_LLVM_CLANG_VERSION >= 20300) || (CRYPTOPP_APPLE_CLANG_VERSION >= 40000))
	#define CRYPTOPP_SSE41_AVAILABLE 1
#endif

#if !defined(CRYPTOPP_DISABLE_SSE4) && defined(CRYPTOPP_SSSE3_AVAILABLE) && \
	(defined(__SSE4_2__) || (CRYPTOPP_MSC_VERSION >= 1500) || (__SUNPRO_CC >= 0x5110) || \
	(CRYPTOPP_GCC_VERSION >= 40300) || (__INTEL_COMPILER >= 1000) || \
	(CRYPTOPP_LLVM_CLANG_VERSION >= 20300) || (CRYPTOPP_APPLE_CLANG_VERSION >= 40000))
	#define CRYPTOPP_SSE42_AVAILABLE 1
#endif

// Couple to CRYPTOPP_DISABLE_AESNI, but use CRYPTOPP_CLMUL_AVAILABLE so we can selectively
//  disable for misbehaving platofrms and compilers, like Solaris or some Clang.
#if defined(CRYPTOPP_DISABLE_AESNI)
	#define CRYPTOPP_DISABLE_CLMUL 1
#endif

// Requires Sun Studio 12.3 (SunCC 0x5120) in theory.
#if !defined(CRYPTOPP_DISABLE_CLMUL) && defined(CRYPTOPP_SSE42_AVAILABLE) && \
	(defined(__PCLMUL__) || (_MSC_FULL_VER >= 150030729) || (__SUNPRO_CC >= 0x5120) || \
	(CRYPTOPP_GCC_VERSION >= 40300) || (__INTEL_COMPILER >= 1110) || \
	(CRYPTOPP_LLVM_CLANG_VERSION >= 30200) || (CRYPTOPP_APPLE_CLANG_VERSION >= 40300))
	#define CRYPTOPP_CLMUL_AVAILABLE 1
#endif

// Requires Sun Studio 12.3 (SunCC 0x5120)
#if !defined(CRYPTOPP_DISABLE_AESNI) && defined(CRYPTOPP_SSE42_AVAILABLE) && \
	(defined(__AES__) || (_MSC_FULL_VER >= 150030729) || (__SUNPRO_CC >= 0x5120) || \
	(CRYPTOPP_GCC_VERSION >= 40300) || (__INTEL_COMPILER >= 1110) || \
	(CRYPTOPP_LLVM_CLANG_VERSION >= 30200) || (CRYPTOPP_APPLE_CLANG_VERSION >= 40300))
	#define CRYPTOPP_AESNI_AVAILABLE 1
#endif

// Requires Binutils 2.24
#if !defined(CRYPTOPP_DISABLE_AVX) && defined(CRYPTOPP_SSE42_AVAILABLE) && \
	(defined(__AVX2__) || (CRYPTOPP_MSC_VERSION >= 1800) || (__SUNPRO_CC >= 0x5130) || \
	(CRYPTOPP_GCC_VERSION >= 40700) || (__INTEL_COMPILER >= 1400) || \
	(CRYPTOPP_LLVM_CLANG_VERSION >= 30100) || (CRYPTOPP_APPLE_CLANG_VERSION >= 40600))
#define CRYPTOPP_AVX_AVAILABLE 1
#endif

// Requires Binutils 2.24
#if !defined(CRYPTOPP_DISABLE_AVX2) && defined(CRYPTOPP_AVX_AVAILABLE) && \
	(defined(__AVX2__) || (CRYPTOPP_MSC_VERSION >= 1800) || (__SUNPRO_CC >= 0x5130) || \
	(CRYPTOPP_GCC_VERSION >= 40900) || (__INTEL_COMPILER >= 1400) || \
	(CRYPTOPP_LLVM_CLANG_VERSION >= 30100) || (CRYPTOPP_APPLE_CLANG_VERSION >= 40600))
#define CRYPTOPP_AVX2_AVAILABLE 1
#endif

// Guessing at SHA for SunCC. Its not in Sun Studio 12.6. Also see
// http://stackoverflow.com/questions/45872180/which-xarch-for-sha-extensions-on-solaris
#if !defined(CRYPTOPP_DISABLE_SHANI) && defined(CRYPTOPP_SSE42_AVAILABLE) && \
	(defined(__SHA__) || (CRYPTOPP_MSC_VERSION >= 1900) || (__SUNPRO_CC >= 0x5160) || \
	(CRYPTOPP_GCC_VERSION >= 40900) || (__INTEL_COMPILER >= 1300) || \
	(CRYPTOPP_LLVM_CLANG_VERSION >= 30400) || (CRYPTOPP_APPLE_CLANG_VERSION >= 50100))
	#define CRYPTOPP_SHANI_AVAILABLE 1
#endif

// RDRAND uses byte codes. All we need is x86 ASM for it.
// However tie it to AES-NI since SecureKey was available with it.
#if !defined(CRYPTOPP_DISABLE_RDRAND) && defined(CRYPTOPP_AESNI_AVAILABLE)
	#define CRYPTOPP_RDRAND_AVAILABLE 1
#endif

// RDSEED uses byte codes. All we need is x86 ASM for it.
// However tie it to AES-NI since SecureKey was available with it.
#if !defined(CRYPTOPP_DISABLE_RDSEED) && defined(CRYPTOPP_AESNI_AVAILABLE)
	#define CRYPTOPP_RDSEED_AVAILABLE 1
#endif

// PadlockRNG uses byte codes. All we need is x86 ASM for it.
#if !defined(CRYPTOPP_DISABLE_PADLOCK) && \
	!(defined(__ANDROID__) || defined(ANDROID) || defined(__APPLE__)) && \
	defined(CRYPTOPP_X86_ASM_AVAILABLE)
	#define CRYPTOPP_PADLOCK_AVAILABLE 1
	#define CRYPTOPP_PADLOCK_RNG_AVAILABLE 1
	#define CRYPTOPP_PADLOCK_ACE_AVAILABLE 1
	#define CRYPTOPP_PADLOCK_ACE2_AVAILABLE 1
	#define CRYPTOPP_PADLOCK_PHE_AVAILABLE 1
	#define CRYPTOPP_PADLOCK_PMM_AVAILABLE 1
#endif

// Fixup Android and SSE, Crypto. It may be enabled based on compiler version.
#if defined(__ANDROID__) || defined(ANDROID)
# if (CRYPTOPP_BOOL_X86)
#  undef CRYPTOPP_SSE41_AVAILABLE
#  undef CRYPTOPP_SSE42_AVAILABLE
#  undef CRYPTOPP_CLMUL_AVAILABLE
#  undef CRYPTOPP_AESNI_AVAILABLE
#  undef CRYPTOPP_SHANI_AVAILABLE
#  undef CRYPTOPP_RDRAND_AVAILABLE
#  undef CRYPTOPP_RDSEED_AVAILABLE
#  undef CRYPTOPP_AVX_AVAILABLE
#  undef CRYPTOPP_AVX2_AVAILABLE
# endif
# if (CRYPTOPP_BOOL_X64)
#  undef CRYPTOPP_CLMUL_AVAILABLE
#  undef CRYPTOPP_AESNI_AVAILABLE
#  undef CRYPTOPP_SHANI_AVAILABLE
#  undef CRYPTOPP_RDRAND_AVAILABLE
#  undef CRYPTOPP_RDSEED_AVAILABLE
#  undef CRYPTOPP_AVX_AVAILABLE
#  undef CRYPTOPP_AVX2_AVAILABLE
# endif
#endif

// Fixup for SunCC 12.1-12.4. Bad code generation in AES_Encrypt and friends.
#if defined(__SUNPRO_CC) && (__SUNPRO_CC <= 0x5130)
# undef CRYPTOPP_AESNI_AVAILABLE
#endif

// Fixup for SunCC 12.1-12.6. Compiler crash on GCM_Reduce_CLMUL.
// http://github.com/weidai11/cryptopp/issues/226
#if defined(__SUNPRO_CC) && (__SUNPRO_CC <= 0x5150)
# undef CRYPTOPP_CLMUL_AVAILABLE
#endif

#endif  // CRYPTOPP_DISABLE_ASM

#endif  // X86, X32, X64

// ***************** ARM CPU features ********************

#if (CRYPTOPP_BOOL_ARM32 || CRYPTOPP_BOOL_ARMV8)

// We don't have an ARM big endian test rig. Disable
// ARM-BE ASM and instrinsics until we can test it.
#if (CRYPTOPP_BIG_ENDIAN)
# define CRYPTOPP_DISABLE_ASM 1
#endif

// Guard everything in CRYPTOPP_DISABLE_ASM
#if !defined(CRYPTOPP_DISABLE_ASM)

// Requires ACLE 1.0. -mfpu=neon or above must be present
// Requires GCC 4.3, Clang 2.8 or Visual Studio 2012
// Do not use APPLE_CLANG_VERSION; use __ARM_FEATURE_XXX instead.
#if !defined(CRYPTOPP_ARM_NEON_AVAILABLE) && !defined(CRYPTOPP_DISABLE_ARM_NEON)
# if defined(__arm__) || defined(__ARM_NEON) || defined(__ARM_FEATURE_NEON) || defined(_M_ARM)
#  if (CRYPTOPP_GCC_VERSION >= 40300) || (CRYPTOPP_LLVM_CLANG_VERSION >= 20800) || \
      (CRYPTOPP_MSC_VERSION >= 1700)
#   define CRYPTOPP_ARM_NEON_AVAILABLE 1
#  endif  // Compilers
# endif  // Platforms
#endif

// ARMv8 and ASIMD. -march=armv8-a or above must be present
// Requires GCC 4.8, Clang 3.3 or Visual Studio 2017
// Do not use APPLE_CLANG_VERSION; use __ARM_FEATURE_XXX instead.
#if !defined(CRYPTOPP_ARM_ASIMD_AVAILABLE) && !defined(CRYPTOPP_DISABLE_ARM_ASIMD)
# if defined(__aarch32__) || defined(__aarch64__) || defined(_M_ARM64)
#  if defined(__ARM_NEON) || defined(__ARM_FEATURE_NEON) || defined(__ARM_FEATURE_ASIMD) || \
      (CRYPTOPP_GCC_VERSION >= 40800) || (CRYPTOPP_LLVM_CLANG_VERSION >= 30300) || \
      (CRYPTOPP_MSC_VERSION >= 1916)
#   define CRYPTOPP_ARM_NEON_AVAILABLE 1
#   define CRYPTOPP_ARM_ASIMD_AVAILABLE 1
#  endif  // Compilers
# endif  // Platforms
#endif

// ARMv8 and ASIMD. -march=armv8-a+crc or above must be present
// Requires GCC 4.8, Clang 3.3 or Visual Studio 2017
// Do not use APPLE_CLANG_VERSION; use __ARM_FEATURE_XXX instead.
#if !defined(CRYPTOPP_ARM_CRC32_AVAILABLE) && !defined(CRYPTOPP_DISABLE_ARM_CRC32)
# if defined(__aarch32__) || defined(__aarch64__) || defined(_M_ARM64)
#  if defined(__ARM_FEATURE_CRC32) || (CRYPTOPP_GCC_VERSION >= 40800) || \
      (CRYPTOPP_LLVM_CLANG_VERSION >= 30300) || (CRYPTOPP_MSC_VERSION >= 1916)
#   define CRYPTOPP_ARM_CRC32_AVAILABLE 1
#  endif  // Compilers
# endif  // Platforms
#endif

// ARMv8 and ASIMD. -march=armv8-a+crypto or above must be present
// Requires GCC 4.8, Clang 3.3 or Visual Studio 2017
// Do not use APPLE_CLANG_VERSION; use __ARM_FEATURE_XXX instead.
#if !defined(CRYPTOPP_ARM_PMULL_AVAILABLE) && !defined(CRYPTOPP_DISABLE_ARM_PMULL)
# if defined(__aarch32__) || defined(__aarch64__) || defined(_M_ARM64)
#  if defined(__ARM_FEATURE_CRYPTO) || (CRYPTOPP_GCC_VERSION >= 40800) || \
      (CRYPTOPP_LLVM_CLANG_VERSION >= 30300) || (CRYPTOPP_MSC_VERSION >= 1916)
#   define CRYPTOPP_ARM_PMULL_AVAILABLE 1
#  endif  // Compilers
# endif  // Platforms
#endif

// ARMv8 and AES. -march=armv8-a+crypto or above must be present
// Requires GCC 4.8, Clang 3.3 or Visual Studio 2017
// Do not use APPLE_CLANG_VERSION; use __ARM_FEATURE_XXX instead.
#if !defined(CRYPTOPP_ARM_AES_AVAILABLE) && !defined(CRYPTOPP_DISABLE_ARM_AES)
# if defined(__aarch32__) || defined(__aarch64__) || defined(_M_ARM64)
#  if defined(__ARM_FEATURE_CRYPTO) || (CRYPTOPP_GCC_VERSION >= 40800) || \
      (CRYPTOPP_LLVM_CLANG_VERSION >= 30300) || (CRYPTOPP_MSC_VERSION >= 1916)
#   define CRYPTOPP_ARM_AES_AVAILABLE 1
#  endif  // Compilers
# endif  // Platforms
#endif

// ARMv8 and SHA-1, SHA-256. -march=armv8-a+crypto or above must be present
// Requires GCC 4.8, Clang 3.3 or Visual Studio 2017
// Do not use APPLE_CLANG_VERSION; use __ARM_FEATURE_XXX instead.
#if !defined(CRYPTOPP_ARM_SHA_AVAILABLE) && !defined(CRYPTOPP_DISABLE_ARM_SHA)
# if defined(__aarch32__) || defined(__aarch64__) || defined(_M_ARM64)
#  if defined(__ARM_FEATURE_CRYPTO) || (CRYPTOPP_GCC_VERSION >= 40800) || \
      (CRYPTOPP_LLVM_CLANG_VERSION >= 30300) || (CRYPTOPP_MSC_VERSION >= 1916)
#   define CRYPTOPP_ARM_SHA1_AVAILABLE 1
#   define CRYPTOPP_ARM_SHA2_AVAILABLE 1
#  endif  // Compilers
# endif  // Platforms
#endif

// ARMv8 and SHA-512, SHA-3. -march=armv8.4-a+crypto or above must be present
// Requires GCC 8.0, Clang ??? or Visual Studio 20??
// Do not use APPLE_CLANG_VERSION; use __ARM_FEATURE_XXX instead.
#if !defined(CRYPTOPP_ARM_SHA3_AVAILABLE) && !defined(CRYPTOPP_DISABLE_ARM_SHA)
# if defined(__aarch32__) || defined(__aarch64__) || defined(_M_ARM64)
#  if defined(__ARM_FEATURE_SHA3) || (CRYPTOPP_GCC_VERSION >= 80000)
#   define CRYPTOPP_ARM_SHA512_AVAILABLE 1
#   define CRYPTOPP_ARM_SHA3_AVAILABLE 1
#  endif  // Compilers
# endif  // Platforms
#endif

// ARMv8 and SM3, SM4. -march=armv8.4-a+crypto or above must be present
// Requires GCC 8.0, Clang ??? or Visual Studio 20??
// Do not use APPLE_CLANG_VERSION; use __ARM_FEATURE_XXX instead.
#if !defined(CRYPTOPP_ARM_SM3_AVAILABLE) && !defined(CRYPTOPP_DISABLE_ARM_SM3)
# if defined(__aarch32__) || defined(__aarch64__) || defined(_M_ARM64)
#  if defined(__ARM_FEATURE_SM3) || (CRYPTOPP_GCC_VERSION >= 80000)
#   define CRYPTOPP_ARM_SM3_AVAILABLE 1
#   define CRYPTOPP_ARM_SM4_AVAILABLE 1
#  endif  // Compilers
# endif  // Platforms
#endif

// Limit the <arm_neon.h> include.
#if !defined(CRYPTOPP_ARM_NEON_HEADER)
# if defined(CRYPTOPP_ARM_NEON_AVAILABLE) || defined (CRYPTOPP_ARM_ASIMD_AVAILABLE)
#  if !defined(_M_ARM64)
#   define CRYPTOPP_ARM_NEON_HEADER 1
#  endif
# endif
#endif

// Limit the <arm_acle.h> include.
#if !defined(CRYPTOPP_ARM_ACLE_HEADER)
# if defined(__aarch32__) || defined(__aarch64__) || (__ARM_ARCH >= 8) || defined(__ARM_ACLE)
#  if !defined(__ANDROID__) && !defined(ANDROID) && !defined(__APPLE__)
#   define CRYPTOPP_ARM_ACLE_HEADER 1
#  endif
# endif
#endif

// Fixup Apple Clang and PMULL. Apple defines __ARM_FEATURE_CRYPTO for Xcode 6
// but does not provide PMULL. TODO: determine when PMULL is available.
#if defined(CRYPTOPP_APPLE_CLANG_VERSION) && (CRYPTOPP_APPLE_CLANG_VERSION < 70000)
# undef CRYPTOPP_ARM_PMULL_AVAILABLE
#endif

// Disable for Android. Android only offers the base Aarch64 architecture.
// https://developer.android.com/ndk/guides/abis
#if defined(__ANDROID__) || defined(ANDROID)
# undef CRYPTOPP_ARM_CRC32_AVAILABLE
# undef CRYPTOPP_ARM_PMULL_AVAILABLE
# undef CRYPTOPP_ARM_AES_AVAILABLE
# undef CRYPTOPP_ARM_SHA1_AVAILABLE
# undef CRYPTOPP_ARM_SHA2_AVAILABLE
# undef CRYPTOPP_ARM_SHA3_AVAILABLE
# undef CRYPTOPP_ARM_SHA512_AVAILABLE
# undef CRYPTOPP_ARM_SM3_AVAILABLE
# undef CRYPTOPP_ARM_SM4_AVAILABLE
#endif

// Cryptogams offers an ARM asm implementations for AES and SHA. Crypto++ does
// not provide an asm implementation. The Cryptogams AES implementation is
// about 50% faster than C/C++, and SHA implementation is about 30% faster
// than C/C++. Define this to use the Cryptogams AES and SHA implementations
// on GNU Linux systems. When defined, Crypto++ will use aes_armv4.S,
// sha1_armv4.S and sha256_armv4.S. https://www.cryptopp.com/wiki/Cryptogams.
#if defined(__arm__) && defined(__linux__)
# if defined(__GNUC__) || defined(__clang__)
#  define CRYPTOGAMS_ARM_AES      1
#  define CRYPTOGAMS_ARM_SHA1     1
#  define CRYPTOGAMS_ARM_SHA256   1
#  define CRYPTOGAMS_ARM_SHA512   1
# endif
#endif

#endif  // CRYPTOPP_DISABLE_ASM

#endif  // ARM32, ARM64

// ***************** AltiVec and Power8 ********************

#if (CRYPTOPP_BOOL_PPC32 || CRYPTOPP_BOOL_PPC64)

// Guard everything in CRYPTOPP_DISABLE_ASM
#if !defined(CRYPTOPP_DISABLE_ASM) && !defined(CRYPTOPP_DISABLE_ALTIVEC)

// An old Apple G5 with GCC 4.01 has AltiVec, but its only Power4 or so.
#if !defined(CRYPTOPP_ALTIVEC_AVAILABLE)
# if defined(_ARCH_PWR4) || defined(__ALTIVEC__) || \
	(CRYPTOPP_XLC_VERSION >= 100000) || (CRYPTOPP_GCC_VERSION >= 40001) || \
    (CRYPTOPP_LLVM_CLANG_VERSION >= 20900)
#  define CRYPTOPP_ALTIVEC_AVAILABLE 1
# endif
#endif

#if defined(CRYPTOPP_ALTIVEC_AVAILABLE)

// We need Power7 for unaligned loads and stores
#if !defined(CRYPTOPP_POWER7_AVAILABLE) && !defined(CRYPTOPP_DISABLE_POWER7)
# if defined(_ARCH_PWR7) || (CRYPTOPP_XLC_VERSION >= 100000) || \
    (CRYPTOPP_GCC_VERSION >= 40100) || (CRYPTOPP_LLVM_CLANG_VERSION >= 30100)
#  define CRYPTOPP_POWER7_AVAILABLE 1
# endif
#endif

#if defined(CRYPTOPP_POWER7_AVAILABLE)

// We need Power8 for in-core crypto and 64-bit vector types
#if !defined(CRYPTOPP_POWER8_AVAILABLE) && !defined(CRYPTOPP_DISABLE_POWER8)
# if defined(_ARCH_PWR8) || (CRYPTOPP_XLC_VERSION >= 130000) || \
    (CRYPTOPP_GCC_VERSION >= 40800) || (CRYPTOPP_LLVM_CLANG_VERSION >= 70000)
#  define CRYPTOPP_POWER8_AVAILABLE 1
# endif
#endif

#if !defined(CRYPTOPP_POWER8_AES_AVAILABLE) && !defined(CRYPTOPP_DISABLE_POWER8_AES) && defined(CRYPTOPP_POWER8_AVAILABLE)
# if defined(__CRYPTO__) || defined(_ARCH_PWR8) || (CRYPTOPP_XLC_VERSION >= 130000) || \
    (CRYPTOPP_GCC_VERSION >= 40800) || (CRYPTOPP_LLVM_CLANG_VERSION >= 70000)
//#  define CRYPTOPP_POWER8_CRC_AVAILABLE 1
#  define CRYPTOPP_POWER8_AES_AVAILABLE 1
#  define CRYPTOPP_POWER8_VMULL_AVAILABLE 1
#  define CRYPTOPP_POWER8_SHA_AVAILABLE 1
# endif
#endif

#if defined(CRYPTOPP_POWER8_AVAILABLE)

// Power9 for random numbers
#if !defined(CRYPTOPP_POWER9_AVAILABLE) && !defined(CRYPTOPP_DISABLE_POWER9)
# if defined(_ARCH_PWR9) || (CRYPTOPP_XLC_VERSION >= 130200) || \
    (CRYPTOPP_GCC_VERSION >= 70000) || (CRYPTOPP_LLVM_CLANG_VERSION >= 80000)
#  define CRYPTOPP_POWER9_AVAILABLE 1
# endif
#endif

#endif  // CRYPTOPP_POWER8_AVAILABLE
#endif  // CRYPTOPP_POWER7_AVAILABLE
#endif  // CRYPTOPP_ALTIVEC_AVAILABLE
#endif  // CRYPTOPP_DISABLE_ASM
#endif  // PPC32, PPC64

#endif  // CRYPTOPP_CONFIG_ASM_H
