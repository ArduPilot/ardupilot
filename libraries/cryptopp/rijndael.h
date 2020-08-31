// rijndael.h - originally written and placed in the public domain by Wei Dai

/// \file rijndael.h
/// \brief Classes for Rijndael encryption algorithm
/// \details All key sizes are supported. The library only provides Rijndael with 128-bit blocks,
///   and not 192-bit or 256-bit blocks
/// \since Rijndael since Crypto++ 3.1, Intel AES-NI since Crypto++ 5.6.1, ARMv8 AES since Crypto++ 6.0,
///   Power8 AES since Crypto++ 6.0, ARMv7 AES since Crypto++ 8.0

#ifndef CRYPTOPP_RIJNDAEL_H
#define CRYPTOPP_RIJNDAEL_H

#include "seckey.h"
#include "secblock.h"

// Clang 3.3 integrated assembler crash on Linux. Clang 3.4 due to compiler
// error with .intel_syntax, http://llvm.org/bugs/show_bug.cgi?id=24232
#if CRYPTOPP_BOOL_X32 || defined(CRYPTOPP_DISABLE_MIXED_ASM)
# define CRYPTOPP_DISABLE_RIJNDAEL_ASM 1
#endif

#if CRYPTOPP_BOOL_X64 || CRYPTOPP_BOOL_X32 || CRYPTOPP_BOOL_X86 || CRYPTOPP_BOOL_ARM32 || \
	CRYPTOPP_BOOL_ARMV8 || CRYPTOPP_BOOL_PPC32 || CRYPTOPP_BOOL_PPC64
# define CRYPTOPP_RIJNDAEL_ADVANCED_PROCESS_BLOCKS 1
#endif

NAMESPACE_BEGIN(CryptoPP)

/// \brief Rijndael block cipher information
/// \details All key sizes are supported. The library only provides Rijndael with 128-bit blocks,
///   and not 192-bit or 256-bit blocks
/// \since Rijndael since Crypto++ 3.1, Intel AES-NI since Crypto++ 5.6.1, ARMv8 AES since Crypto++ 6.0,
///   Power8 AES since Crypto++ 6.0, ARMv7 AES since Crypto++ 8.0
struct Rijndael_Info : public FixedBlockSize<16>, public VariableKeyLength<16, 16, 32, 8>
{
	CRYPTOPP_DLL static const char * CRYPTOPP_API StaticAlgorithmName() {return "AES";}
};

/// \brief Rijndael block cipher
/// \details All key sizes are supported. The library only provides Rijndael with 128-bit blocks,
///   and not 192-bit or 256-bit blocks
/// \since Rijndael since Crypto++ 3.1, Intel AES-NI since Crypto++ 5.6.1, ARMv8 AES since Crypto++ 6.0,
///   Power8 AES since Crypto++ 6.0, ARMv7 AES since Crypto++ 8.0
/// \sa <a href="http://www.cryptopp.com/wiki/Rijndael">Rijndael</a>
class CRYPTOPP_DLL Rijndael : public Rijndael_Info, public BlockCipherDocumentation
{
	/// \brief Rijndael block cipher transformation functions
	/// \details Provides implementation common to encryption and decryption
	class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE Base : public BlockCipherImpl<Rijndael_Info>
	{
	public:
		void UncheckedSetKey(const byte *userKey, unsigned int keyLength, const NameValuePairs &params);
		std::string AlgorithmProvider() const;
		unsigned int OptimalDataAlignment() const;

	protected:
		static void FillEncTable();
		static void FillDecTable();

		// VS2005 workaround: have to put these on separate lines, or error C2487 is triggered in DLL build
		static const byte Se[256];
		static const byte Sd[256];

		static const word32 rcon[];

		unsigned int m_rounds;
		SecBlock<word32, AllocatorWithCleanup<word32, true> > m_key;
		mutable SecByteBlock m_aliasBlock;
	};

	/// \brief Encryption transformation
	/// \details Enc provides implementation for encryption transformation. All key sizes are supported.
	///   The library only provides Rijndael with 128-bit blocks, and not 192-bit or 256-bit blocks
	/// \since Rijndael since Crypto++ 3.1, Intel AES-NI since Crypto++ 5.6.1, ARMv8 AES since Crypto++ 6.0,
	///   Power8 AES since Crypto++ 6.0, ARMv7 AES since Crypto++ 8.0
	class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE Enc : public Base
	{
	public:
		void ProcessAndXorBlock(const byte *inBlock, const byte *xorBlock, byte *outBlock) const;
#if CRYPTOPP_RIJNDAEL_ADVANCED_PROCESS_BLOCKS
		size_t AdvancedProcessBlocks(const byte *inBlocks, const byte *xorBlocks, byte *outBlocks, size_t length, word32 flags) const;
#endif
	};

	/// \brief Decryption transformation
	/// \details Dec provides implementation for decryption transformation. All key sizes are supported.
	///   The library only provides Rijndael with 128-bit blocks, and not 192-bit or 256-bit blocks
	/// \since Rijndael since Crypto++ 3.1, Intel AES-NI since Crypto++ 5.6.1, ARMv8 AES since Crypto++ 6.0,
	///   Power8 AES since Crypto++ 6.0, ARMv7 AES since Crypto++ 8.0
	class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE Dec : public Base
	{
	public:
		void ProcessAndXorBlock(const byte *inBlock, const byte *xorBlock, byte *outBlock) const;
#if CRYPTOPP_RIJNDAEL_ADVANCED_PROCESS_BLOCKS
		size_t AdvancedProcessBlocks(const byte *inBlocks, const byte *xorBlocks, byte *outBlocks, size_t length, word32 flags) const;
#endif
	};

public:
	typedef BlockCipherFinal<ENCRYPTION, Enc> Encryption;
	typedef BlockCipherFinal<DECRYPTION, Dec> Decryption;
};

typedef Rijndael::Encryption RijndaelEncryption;
typedef Rijndael::Decryption RijndaelDecryption;

NAMESPACE_END

#endif
