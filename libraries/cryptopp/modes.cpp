// modes.cpp - originally written and placed in the public domain by Wei Dai

#include "pch.h"

#ifndef CRYPTOPP_IMPORTS

#include "modes.h"
#include "misc.h"

#if defined(CRYPTOPP_DEBUG)
#include "des.h"
#endif

NAMESPACE_BEGIN(CryptoPP)

#if defined(CRYPTOPP_DEBUG) && !defined(CRYPTOPP_DOXYGEN_PROCESSING)
void Modes_TestInstantiations()
{
	CFB_Mode<DES>::Encryption m0;
	CFB_Mode<DES>::Decryption m1;
	OFB_Mode<DES>::Encryption m2;
	CTR_Mode<DES>::Encryption m3;
	ECB_Mode<DES>::Encryption m4;
	CBC_Mode<DES>::Encryption m5;
}
#endif

void CipherModeBase::ResizeBuffers()
{
	m_register.New(m_cipher->BlockSize());
}

void CFB_ModePolicy::Iterate(byte *output, const byte *input, CipherDir dir, size_t iterationCount)
{
	CRYPTOPP_ASSERT(input);	CRYPTOPP_ASSERT(output);
	CRYPTOPP_ASSERT(m_cipher->IsForwardTransformation());
	CRYPTOPP_ASSERT(m_register.size() == BlockSize());
	CRYPTOPP_ASSERT(m_temp.size() == BlockSize());
	CRYPTOPP_ASSERT(iterationCount > 0);

	const unsigned int s = BlockSize();
	if (dir == ENCRYPTION)
	{
		m_cipher->ProcessAndXorBlock(m_register, input, output);
		if (iterationCount > 1)
			m_cipher->AdvancedProcessBlocks(output, PtrAdd(input,s), PtrAdd(output,s), (iterationCount-1)*s, 0);
		memcpy(m_register, PtrAdd(output,(iterationCount-1)*s), s);
	}
	else
	{
		// make copy first in case of in-place decryption
		memcpy(m_temp, PtrAdd(input,(iterationCount-1)*s), s);
		if (iterationCount > 1)
			m_cipher->AdvancedProcessBlocks(input, PtrAdd(input,s), PtrAdd(output,s), (iterationCount-1)*s, BlockTransformation::BT_ReverseDirection);
		m_cipher->ProcessAndXorBlock(m_register, input, output);
		memcpy(m_register, m_temp, s);
	}
}

void CFB_ModePolicy::TransformRegister()
{
	CRYPTOPP_ASSERT(m_cipher->IsForwardTransformation());
	CRYPTOPP_ASSERT(m_register.size() == BlockSize());
	CRYPTOPP_ASSERT(m_temp.size() == BlockSize());

	const ptrdiff_t updateSize = BlockSize()-m_feedbackSize;
	m_cipher->ProcessBlock(m_register, m_temp);
	memmove_s(m_register, m_register.size(), PtrAdd(m_register.begin(),m_feedbackSize), updateSize);
	memcpy_s(PtrAdd(m_register.begin(),updateSize), m_register.size()-updateSize, m_temp, m_feedbackSize);
}

void CFB_ModePolicy::CipherResynchronize(const byte *iv, size_t length)
{
	CRYPTOPP_ASSERT(length == BlockSize());
	CRYPTOPP_ASSERT(m_register.size() == BlockSize());

	CopyOrZero(m_register, m_register.size(), iv, length);
	TransformRegister();
}

void CFB_ModePolicy::SetFeedbackSize(unsigned int feedbackSize)
{
	if (feedbackSize > BlockSize())
		throw InvalidArgument("CFB_Mode: invalid feedback size");
	m_feedbackSize = feedbackSize ? feedbackSize : BlockSize();
}

void CFB_ModePolicy::ResizeBuffers()
{
	CipherModeBase::ResizeBuffers();
	m_temp.New(BlockSize());
}

byte* CFB_ModePolicy::GetRegisterBegin()
{
	CRYPTOPP_ASSERT(!m_register.empty());
	CRYPTOPP_ASSERT(BlockSize() >= m_feedbackSize);
	return PtrAdd(m_register.begin(), BlockSize() - m_feedbackSize);
}

void OFB_ModePolicy::WriteKeystream(byte *keystreamBuffer, size_t iterationCount)
{
	CRYPTOPP_ASSERT(m_cipher->IsForwardTransformation());
	CRYPTOPP_ASSERT(m_register.size() == BlockSize());
	CRYPTOPP_ASSERT(iterationCount > 0);

	const unsigned int s = BlockSize();
	m_cipher->ProcessBlock(m_register, keystreamBuffer);
	if (iterationCount > 1)
		m_cipher->AdvancedProcessBlocks(keystreamBuffer, NULLPTR, PtrAdd(keystreamBuffer, s), s*(iterationCount-1), 0);
	memcpy(m_register, PtrAdd(keystreamBuffer, (iterationCount-1)*s), s);
}

void OFB_ModePolicy::CipherResynchronize(byte *keystreamBuffer, const byte *iv, size_t length)
{
	CRYPTOPP_UNUSED(keystreamBuffer), CRYPTOPP_UNUSED(length);
	CRYPTOPP_ASSERT(m_register.size() == BlockSize());
	CRYPTOPP_ASSERT(length == BlockSize());

	CopyOrZero(m_register, m_register.size(), iv, length);
}

void CTR_ModePolicy::SeekToIteration(lword iterationCount)
{
	int carry=0;
	for (int i=BlockSize()-1; i>=0; i--)
	{
		unsigned int sum = m_register[i] + (byte)iterationCount + carry;
		m_counterArray[i] = byte(sum & 0xff);
		carry = sum >> 8;
		iterationCount >>= 8;
	}
}

void CTR_ModePolicy::IncrementCounterBy256()
{
	IncrementCounterByOne(m_counterArray, BlockSize()-1);
}

void CTR_ModePolicy::OperateKeystream(KeystreamOperation /*operation*/, byte *output, const byte *input, size_t iterationCount)
{
	CRYPTOPP_ASSERT(m_cipher->IsForwardTransformation());
	CRYPTOPP_ASSERT(m_counterArray.size() == BlockSize());

	const unsigned int s = BlockSize();
	const unsigned int inputIncrement = input ? s : 0;

	while (iterationCount)
	{
		const byte lsb = m_counterArray[s-1];
		const size_t blocks = UnsignedMin(iterationCount, 256U-lsb);

		m_cipher->AdvancedProcessBlocks(m_counterArray, input, output, blocks*s, BlockTransformation::BT_InBlockIsCounter|BlockTransformation::BT_AllowParallel);
		if ((m_counterArray[s-1] = byte(lsb + blocks)) == 0)
			IncrementCounterBy256();

		output = PtrAdd(output, blocks*s);
		input = PtrAdd(input, blocks*inputIncrement);
		iterationCount -= blocks;
	}
}

void CTR_ModePolicy::CipherResynchronize(byte *keystreamBuffer, const byte *iv, size_t length)
{
	CRYPTOPP_UNUSED(keystreamBuffer), CRYPTOPP_UNUSED(length);
	CRYPTOPP_ASSERT(m_register.size() == BlockSize());
	CRYPTOPP_ASSERT(length == BlockSize());

	CopyOrZero(m_register, m_register.size(), iv, length);
	m_counterArray.Assign(m_register.begin(), m_register.size());
}

void BlockOrientedCipherModeBase::UncheckedSetKey(const byte *key, unsigned int length, const NameValuePairs &params)
{
	m_cipher->SetKey(key, length, params);
	ResizeBuffers();
	if (IsResynchronizable())
	{
		size_t ivLength;
		const byte *iv = GetIVAndThrowIfInvalid(params, ivLength);
		Resynchronize(iv, (int)ivLength);
	}
}

void BlockOrientedCipherModeBase::ResizeBuffers()
{
	CipherModeBase::ResizeBuffers();
	m_buffer.New(BlockSize());
}

void ECB_OneWay::ProcessData(byte *outString, const byte *inString, size_t length)
{
	CRYPTOPP_ASSERT(length%BlockSize()==0);
	m_cipher->AdvancedProcessBlocks(inString, NULLPTR, outString, length, BlockTransformation::BT_AllowParallel);
}

void CBC_Encryption::ProcessData(byte *outString, const byte *inString, size_t length)
{
	CRYPTOPP_ASSERT(length%BlockSize()==0);
	CRYPTOPP_ASSERT(m_register.size() == BlockSize());
	if (!length) return;

	const unsigned int blockSize = BlockSize();
	m_cipher->AdvancedProcessBlocks(inString, m_register, outString, blockSize, BlockTransformation::BT_XorInput);
	if (length > blockSize)
		m_cipher->AdvancedProcessBlocks(PtrAdd(inString,blockSize), outString, PtrAdd(outString,blockSize), length-blockSize, BlockTransformation::BT_XorInput);
	memcpy(m_register, PtrAdd(outString, length - blockSize), blockSize);
}

size_t CBC_CTS_Encryption::ProcessLastBlock(byte *outString, size_t outLength, const byte *inString, size_t inLength)
{
	CRYPTOPP_UNUSED(outLength);
	const size_t used = inLength;
	const unsigned int blockSize = BlockSize();

	if (inLength <= blockSize)
	{
		if (!m_stolenIV)
			throw InvalidArgument("CBC_Encryption: message is too short for ciphertext stealing");

		// steal from IV
		memcpy(outString, m_register, inLength);
		outString = m_stolenIV;
	}
	else
	{
		// steal from next to last block
		xorbuf(m_register, inString, blockSize);
		m_cipher->ProcessBlock(m_register);
		inString = PtrAdd(inString, blockSize);
		inLength -= blockSize;
		memcpy(PtrAdd(outString, blockSize), m_register, inLength);
	}

	// output last full ciphertext block
	xorbuf(m_register, inString, inLength);
	m_cipher->ProcessBlock(m_register);
	memcpy(outString, m_register, blockSize);

	return used;
}

void CBC_Decryption::ResizeBuffers()
{
	BlockOrientedCipherModeBase::ResizeBuffers();
	m_temp.New(BlockSize());
}

void CBC_Decryption::ProcessData(byte *outString, const byte *inString, size_t length)
{
	CRYPTOPP_ASSERT(length%BlockSize()==0);
	if (!length) {return;}

	// save copy now in case of in-place decryption
	const unsigned int blockSize = BlockSize();
	memcpy(m_temp, PtrAdd(inString,length-blockSize), blockSize);
	if (length > blockSize)
		m_cipher->AdvancedProcessBlocks(PtrAdd(inString,blockSize), inString, PtrAdd(outString,blockSize), length-blockSize, BlockTransformation::BT_ReverseDirection|BlockTransformation::BT_AllowParallel);
	m_cipher->ProcessAndXorBlock(inString, m_register, outString);
	m_register.swap(m_temp);
}

size_t CBC_CTS_Decryption::ProcessLastBlock(byte *outString, size_t outLength, const byte *inString, size_t inLength)
{
	CRYPTOPP_UNUSED(outLength);
	const byte *pn1, *pn2;
	const size_t used = inLength;
	const bool stealIV = inLength <= BlockSize();
	const unsigned int blockSize = BlockSize();

	if (stealIV)
	{
		pn1 = inString;
		pn2 = m_register;
	}
	else
	{
		pn1 = PtrAdd(inString, blockSize);
		pn2 = inString;
		inLength -= blockSize;
	}

	// decrypt last partial plaintext block
	memcpy(m_temp, pn2, blockSize);
	m_cipher->ProcessBlock(m_temp);
	xorbuf(m_temp, pn1, inLength);

	if (stealIV)
	{
		memcpy(outString, m_temp, inLength);
	}
	else
	{
		memcpy(PtrAdd(outString, blockSize), m_temp, inLength);
		// decrypt next to last plaintext block
		memcpy(m_temp, pn1, inLength);
		m_cipher->ProcessBlock(m_temp);
		xorbuf(outString, m_temp, m_register, blockSize);
	}

	return used;
}

NAMESPACE_END

#endif
