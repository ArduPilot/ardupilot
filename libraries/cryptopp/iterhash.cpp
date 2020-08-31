// iterhash.cpp - originally written and placed in the public domain by Wei Dai

#ifndef __GNUC__
#define CRYPTOPP_MANUALLY_INSTANTIATE_TEMPLATES
#endif

#include "iterhash.h"
#include "misc.h"
#include "cpu.h"

NAMESPACE_BEGIN(CryptoPP)

template <class T, class BASE> void IteratedHashBase<T, BASE>::Update(const byte *input, size_t length)
{
	CRYPTOPP_ASSERT(!(input == NULLPTR && length != 0));
	if (length == 0) { return; }

	HashWordType oldCountLo = m_countLo, oldCountHi = m_countHi;
	if ((m_countLo = oldCountLo + HashWordType(length)) < oldCountLo)
		m_countHi++;             // carry from low to high
	m_countHi += (HashWordType)SafeRightShift<8*sizeof(HashWordType)>(length);
	if (m_countHi < oldCountHi || SafeRightShift<2*8*sizeof(HashWordType)>(length) != 0)
		throw HashInputTooLong(this->AlgorithmName());

	const unsigned int blockSize = this->BlockSize();
	unsigned int num = ModPowerOf2(oldCountLo, blockSize);

	T* dataBuf = this->DataBuf();
	byte* data = (byte *)dataBuf;

	if (num != 0)	// process left over data
	{
		if (num+length >= blockSize)
		{
			if (input)
				{std::memcpy(data+num, input, blockSize-num);}

			HashBlock(dataBuf);
			input += (blockSize-num);
			length -= (blockSize-num);
			num = 0;
			// drop through and do the rest
		}
		else
		{
			if (input && length)
				{std::memcpy(data+num, input, length);}
			return;
		}
	}

	// now process the input data in blocks of blockSize bytes and save the leftovers to m_data
	if (length >= blockSize)
	{
		if (input == data)
		{
			CRYPTOPP_ASSERT(length == blockSize);
			HashBlock(dataBuf);
			return;
		}
		else if (IsAligned<T>(input))
		{
			size_t leftOver = HashMultipleBlocks((T *)(void*)input, length);
			input += (length - leftOver);
			length = leftOver;
		}
		else
		{
			do
			{   // copy input first if it's not aligned correctly
				if (input)
					{ std::memcpy(data, input, blockSize); }

				HashBlock(dataBuf);
				input+=blockSize;
				length-=blockSize;
			} while (length >= blockSize);
		}
	}

	if (input && data != input)
		std::memcpy(data, input, length);
}

template <class T, class BASE> byte * IteratedHashBase<T, BASE>::CreateUpdateSpace(size_t &size)
{
	unsigned int blockSize = this->BlockSize();
	unsigned int num = ModPowerOf2(m_countLo, blockSize);
	size = blockSize - num;
	return (byte *)DataBuf() + num;
}

template <class T, class BASE> size_t IteratedHashBase<T, BASE>::HashMultipleBlocks(const T *input, size_t length)
{
	const unsigned int blockSize = this->BlockSize();
	bool noReverse = NativeByteOrderIs(this->GetByteOrder());
	T* dataBuf = this->DataBuf();

	// Alignment checks due to http://github.com/weidai11/cryptopp/issues/690.
	// Sparc requires 8-byte aligned buffer when HashWordType is word64.
	// We also had to provide a GetAlignmentOf specialization for word64 on Sparc.

	do
	{
		if (noReverse)
		{
			if (IsAligned<HashWordType>(input))
			{
				// Sparc bus error with non-aligned input.
				this->HashEndianCorrectedBlock(input);
			}
			else
			{
				std::memcpy(dataBuf, input, blockSize);
				this->HashEndianCorrectedBlock(dataBuf);
			}
		}
		else
		{
			if (IsAligned<HashWordType>(input))
			{
				// Sparc bus error with non-aligned input.
				ByteReverse(dataBuf, input, blockSize);
				this->HashEndianCorrectedBlock(dataBuf);
			}
			else
			{
				std::memcpy(dataBuf, input, blockSize);
				ByteReverse(dataBuf, dataBuf, blockSize);
				this->HashEndianCorrectedBlock(dataBuf);
			}
		}

		input += blockSize/sizeof(T);
		length -= blockSize;
	}
	while (length >= blockSize);
	return length;
}

template <class T, class BASE> void IteratedHashBase<T, BASE>::PadLastBlock(unsigned int lastBlockSize, byte padFirst)
{
	unsigned int blockSize = this->BlockSize();
	unsigned int num = ModPowerOf2(m_countLo, blockSize);
	T* dataBuf = this->DataBuf();
	byte* data = (byte *)dataBuf;

	data[num++] = padFirst;
	if (num <= lastBlockSize)
		memset(data+num, 0, lastBlockSize-num);
	else
	{
		memset(data+num, 0, blockSize-num);
		HashBlock(dataBuf);
		memset(data, 0, lastBlockSize);
	}
}

template <class T, class BASE> void IteratedHashBase<T, BASE>::Restart()
{
	m_countLo = m_countHi = 0;
	Init();
}

template <class T, class BASE> void IteratedHashBase<T, BASE>::TruncatedFinal(byte *digest, size_t size)
{
	CRYPTOPP_ASSERT(digest != NULLPTR);
	this->ThrowIfInvalidTruncatedSize(size);

	T* dataBuf = this->DataBuf();
	T* stateBuf = this->StateBuf();
	unsigned int blockSize = this->BlockSize();
	ByteOrder order = this->GetByteOrder();

	PadLastBlock(blockSize - 2*sizeof(HashWordType));
	dataBuf[blockSize/sizeof(T)-2+order] = ConditionalByteReverse(order, this->GetBitCountLo());
	dataBuf[blockSize/sizeof(T)-1-order] = ConditionalByteReverse(order, this->GetBitCountHi());

	HashBlock(dataBuf);

	if (IsAligned<HashWordType>(digest) && size%sizeof(HashWordType)==0)
		ConditionalByteReverse<HashWordType>(order, (HashWordType *)(void*)digest, stateBuf, size);
	else
	{
		ConditionalByteReverse<HashWordType>(order, stateBuf, stateBuf, this->DigestSize());
		std::memcpy(digest, stateBuf, size);
	}

	this->Restart();		// reinit for next use
}

#if defined(__GNUC__) || defined(__clang__)
	template class IteratedHashBase<word64, HashTransformation>;
	template class IteratedHashBase<word64, MessageAuthenticationCode>;

	template class IteratedHashBase<word32, HashTransformation>;
	template class IteratedHashBase<word32, MessageAuthenticationCode>;
#endif

NAMESPACE_END
