// mdc.h - originally written and placed in the public domain by Wei Dai

/// \file mdc.h
/// \brief Classes for the MDC message digest

#ifndef CRYPTOPP_MDC_H
#define CRYPTOPP_MDC_H

#include "seckey.h"
#include "secblock.h"
#include "misc.h"

// GCC cast warning
#define HashWordPtr(x) ((HashWordType*)(void*)(x))
#define ConstHashWordPtr(x) ((const HashWordType*)(const void*)(x))

NAMESPACE_BEGIN(CryptoPP)

/// \tparam B BlockCipher derived class
/// \brief MDC_Info cipher information
template <class B>
struct MDC_Info : public FixedBlockSize<B::DIGESTSIZE>, public FixedKeyLength<B::BLOCKSIZE>
{
	static std::string StaticAlgorithmName() {return std::string("MDC/")+B::StaticAlgorithmName();}
};

/// \brief MDC cipher
/// \tparam H HashTransformation derived class
/// \details MDC() is a construction by Peter Gutmann to turn an iterated hash function into a PRF
/// \sa <a href="http://www.cryptopp.com/wiki/MDC">MDC</a>
template <class H>
class MDC : public MDC_Info<H>
{
	/// \brief MDC cipher encryption operation
	class CRYPTOPP_NO_VTABLE Enc : public BlockCipherImpl<MDC_Info<H> >
	{
		typedef typename H::HashWordType HashWordType;

	public:
		void UncheckedSetKey(const byte *userKey, unsigned int length, const NameValuePairs &params)
		{
			CRYPTOPP_UNUSED(params);
			this->AssertValidKeyLength(length);
			ConditionalByteReverse(BIG_ENDIAN_ORDER, Key(), ConstHashWordPtr(userKey), this->KEYLENGTH);
		}

		void ProcessAndXorBlock(const byte *inBlock, const byte *xorBlock, byte *outBlock) const
		{
			ConditionalByteReverse(BIG_ENDIAN_ORDER, Buffer(), ConstHashWordPtr(inBlock), this->BLOCKSIZE);
			H::Transform(Buffer(), Key());

			if (xorBlock)
			{
				ConditionalByteReverse(BIG_ENDIAN_ORDER, Buffer(), Buffer(), this->BLOCKSIZE);
				xorbuf(outBlock, xorBlock, m_buffer, this->BLOCKSIZE);
			}
			else
			{
				ConditionalByteReverse(BIG_ENDIAN_ORDER, HashWordPtr(outBlock), Buffer(), this->BLOCKSIZE);
			}
		}

		bool IsPermutation() const {return false;}

		unsigned int OptimalDataAlignment() const {return sizeof(HashWordType);}

	private:
		HashWordType *Key() {return HashWordPtr(m_key.data());}
		const HashWordType *Key() const {return ConstHashWordPtr(m_key.data());}
		HashWordType *Buffer() const {return HashWordPtr(m_buffer.data());}

		// VC60 workaround: bug triggered if using FixedSizeAllocatorWithCleanup
		FixedSizeSecBlock<byte, MDC_Info<H>::KEYLENGTH, AllocatorWithCleanup<byte> > m_key;
		mutable FixedSizeSecBlock<byte, MDC_Info<H>::BLOCKSIZE, AllocatorWithCleanup<byte> > m_buffer;
	};

public:
	// use BlockCipher interface
	typedef BlockCipherFinal<ENCRYPTION, Enc> Encryption;
};

NAMESPACE_END

#endif
