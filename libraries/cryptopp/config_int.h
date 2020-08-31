// config_int.h - written and placed in public domain by Jeffrey Walton
//                the bits that make up this source file are from the
//                library's monolithic config.h.

/// \file config_int.h
/// \brief Library configuration file
/// \details <tt>config.h</tt> was split into components in May 2019 to better
///  integrate with Autoconf and its feature tests. The splitting occured so
///  users could continue to include <tt>config.h</tt> while allowing Autoconf
///  to write new <tt>config_asm.h</tt> and new <tt>config_cxx.h</tt> using
///  its feature tests.
/// \sa <A HREF="https://github.com/weidai11/cryptopp/issues/835">Issue 835</A>
/// \since Crypto++ 8.3

#ifndef CRYPTOPP_CONFIG_INT_H
#define CRYPTOPP_CONFIG_INT_H

#include "config_ns.h"
#include "config_ver.h"

// Originally in global namespace to avoid ambiguity with other byte typedefs.
// Moved to Crypto++ namespace due to C++17, std::byte and potential compile
// problems. Also see http://www.cryptopp.com/wiki/std::byte and
// http://github.com/weidai11/cryptopp/issues/442.
// typedef unsigned char byte;
#define CRYPTOPP_NO_GLOBAL_BYTE 1

NAMESPACE_BEGIN(CryptoPP)

// Signed words added at Issue 609 for early versions of and Visual Studio and
// the NaCl gear. Also see https://github.com/weidai11/cryptopp/issues/609.

typedef unsigned char byte;
typedef unsigned short word16;
typedef unsigned int word32;

typedef signed char sbyte;
typedef signed short sword16;
typedef signed int sword32;

#if defined(_MSC_VER) || defined(__BORLANDC__)
	typedef signed __int64 sword64;
	typedef unsigned __int64 word64;
	#define SW64LIT(x) x##i64
	#define W64LIT(x) x##ui64
#elif (_LP64 || __LP64__)
	typedef signed long sword64;
	typedef unsigned long word64;
	#define SW64LIT(x) x##L
	#define W64LIT(x) x##UL
#else
	typedef signed long long sword64;
	typedef unsigned long long word64;
	#define SW64LIT(x) x##LL
	#define W64LIT(x) x##ULL
#endif

// define large word type, used for file offsets and such
typedef word64 lword;
const lword LWORD_MAX = W64LIT(0xffffffffffffffff);

// define hword, word, and dword. these are used for multiprecision integer arithmetic
// Intel compiler won't have _umul128 until version 10.0. See http://softwarecommunity.intel.com/isn/Community/en-US/forums/thread/30231625.aspx
#if (defined(_MSC_VER) && (!defined(__INTEL_COMPILER) || __INTEL_COMPILER >= 1000) && (defined(_M_X64) || defined(_M_IA64))) || (defined(__DECCXX) && defined(__alpha__)) || (defined(__INTEL_COMPILER) && defined(__x86_64__)) || (defined(__SUNPRO_CC) && defined(__x86_64__))
	typedef word32 hword;
	typedef word64 word;
#else
	#define CRYPTOPP_NATIVE_DWORD_AVAILABLE 1
	#if defined(__alpha__) || defined(__ia64__) || defined(_ARCH_PPC64) || defined(__x86_64__) || defined(__mips64) || defined(__sparc64__) || defined(__aarch64__)
		#if ((CRYPTOPP_GCC_VERSION >= 30400) || (CRYPTOPP_LLVM_CLANG_VERSION >= 30000) || (CRYPTOPP_APPLE_CLANG_VERSION >= 40300)) && (__SIZEOF_INT128__ >= 16)
			// GCC 4.0.1 on MacOS X is missing __umodti3 and __udivti3
			// GCC 4.8.3 and bad uint128_t ops on PPC64/POWER7 (Issue 421)
			// mode(TI) division broken on amd64 with GCC earlier than GCC 3.4
			typedef word32 hword;
			typedef word64 word;
			typedef __uint128_t dword;
			typedef __uint128_t word128;
			#define CRYPTOPP_WORD128_AVAILABLE 1
		#else
			// if we're here, it means we're on a 64-bit CPU but we don't have a way to obtain 128-bit multiplication results
			typedef word16 hword;
			typedef word32 word;
			typedef word64 dword;
		#endif
	#else
		// being here means the native register size is probably 32 bits or less
		#define CRYPTOPP_BOOL_SLOW_WORD64 1
		typedef word16 hword;
		typedef word32 word;
		typedef word64 dword;
	#endif
#endif

#ifndef CRYPTOPP_BOOL_SLOW_WORD64
# define CRYPTOPP_BOOL_SLOW_WORD64 0
#endif

const unsigned int WORD_SIZE = sizeof(word);
const unsigned int WORD_BITS = WORD_SIZE * 8;

NAMESPACE_END

#endif  // CRYPTOPP_CONFIG_INT_H
