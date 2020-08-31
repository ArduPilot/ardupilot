// asn.cpp - originally written and placed in the public domain by Wei Dai
//           CryptoPP::Test namespace added by JW in February 2017

#include "pch.h"
#include "config.h"

#ifndef CRYPTOPP_IMPORTS

#include "cryptlib.h"
#include "asn.h"
#include "misc.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <time.h>

NAMESPACE_BEGIN(CryptoPP)

size_t DERLengthEncode(BufferedTransformation &bt, lword length)
{
	size_t i=0;
	if (length <= 0x7f)
	{
		bt.Put(byte(length));
		i++;
	}
	else
	{
		bt.Put(byte(BytePrecision(length) | 0x80));
		i++;
		for (int j=BytePrecision(length); j; --j)
		{
			bt.Put(byte(length >> (j-1)*8));
			i++;
		}
	}
	return i;
}

bool BERLengthDecode(BufferedTransformation &bt, lword &length, bool &definiteLength)
{
	byte b;

	if (!bt.Get(b))
		return false;

	if (!(b & 0x80))
	{
		definiteLength = true;
		length = b;
	}
	else
	{
		unsigned int lengthBytes = b & 0x7f;

		if (lengthBytes == 0)
		{
			definiteLength = false;
			return true;
		}

		definiteLength = true;
		length = 0;
		while (lengthBytes--)
		{
			if (length >> (8*(sizeof(length)-1)))
				BERDecodeError();	// length about to overflow

			if (!bt.Get(b))
				return false;

			length = (length << 8) | b;
		}
	}
	return true;
}

bool BERLengthDecode(BufferedTransformation &bt, size_t &length)
{
	lword lw = 0;
	bool definiteLength = false;
	if (!BERLengthDecode(bt, lw, definiteLength))
		BERDecodeError();
	if (!SafeConvert(lw, length))
		BERDecodeError();
	return definiteLength;
}

void DEREncodeNull(BufferedTransformation &out)
{
	out.Put(TAG_NULL);
	out.Put(0);
}

void BERDecodeNull(BufferedTransformation &in)
{
	byte b;
	if (!in.Get(b) || b != TAG_NULL)
		BERDecodeError();
	size_t length;
	if (!BERLengthDecode(in, length) || length != 0)
		BERDecodeError();
}

/// ASN Strings
size_t DEREncodeOctetString(BufferedTransformation &bt, const byte *str, size_t strLen)
{
	bt.Put(OCTET_STRING);
	size_t lengthBytes = DERLengthEncode(bt, strLen);
	bt.Put(str, strLen);
	return 1+lengthBytes+strLen;
}

size_t DEREncodeOctetString(BufferedTransformation &bt, const SecByteBlock &str)
{
	return DEREncodeOctetString(bt, ConstBytePtr(str), BytePtrSize(str));
}

size_t BERDecodeOctetString(BufferedTransformation &bt, SecByteBlock &str)
{
	byte b;
	if (!bt.Get(b) || b != OCTET_STRING)
		BERDecodeError();

	size_t bc;
	if (!BERLengthDecode(bt, bc))
		BERDecodeError();
	if (bc > bt.MaxRetrievable()) // Issue 346
		BERDecodeError();

	str.New(bc);
	if (bc != bt.Get(BytePtr(str), bc))
		BERDecodeError();
	return bc;
}

size_t BERDecodeOctetString(BufferedTransformation &bt, BufferedTransformation &str)
{
	byte b;
	if (!bt.Get(b) || b != OCTET_STRING)
		BERDecodeError();

	size_t bc;
	if (!BERLengthDecode(bt, bc))
		BERDecodeError();
	if (bc > bt.MaxRetrievable()) // Issue 346
		BERDecodeError();

	bt.TransferTo(str, bc);
	return bc;
}

size_t DEREncodeTextString(BufferedTransformation &bt, const byte* str, size_t strLen, byte asnTag)
{
	bt.Put(asnTag);
	size_t lengthBytes = DERLengthEncode(bt, strLen);
	bt.Put(str, strLen);
	return 1+lengthBytes+strLen;
}

size_t DEREncodeTextString(BufferedTransformation &bt, const SecByteBlock &str, byte asnTag)
{
	return DEREncodeTextString(bt, ConstBytePtr(str), BytePtrSize(str), asnTag);
}

size_t DEREncodeTextString(BufferedTransformation &bt, const std::string &str, byte asnTag)
{
	return DEREncodeTextString(bt, ConstBytePtr(str), BytePtrSize(str), asnTag);
}

size_t BERDecodeTextString(BufferedTransformation &bt, SecByteBlock &str, byte asnTag)
{
	byte b;
	if (!bt.Get(b) || b != asnTag)
		BERDecodeError();

	size_t bc;
	if (!BERLengthDecode(bt, bc))
		BERDecodeError();
	if (bc > bt.MaxRetrievable()) // Issue 346
		BERDecodeError();

	str.resize(bc);
	if (bc != bt.Get(BytePtr(str), BytePtrSize(str)))
		BERDecodeError();

	return bc;
}

size_t BERDecodeTextString(BufferedTransformation &bt, std::string &str, byte asnTag)
{
	byte b;
	if (!bt.Get(b) || b != asnTag)
		BERDecodeError();

	size_t bc;
	if (!BERLengthDecode(bt, bc))
		BERDecodeError();
	if (bc > bt.MaxRetrievable()) // Issue 346
		BERDecodeError();

	str.resize(bc);
	if (bc != bt.Get(BytePtr(str), BytePtrSize(str)))
		BERDecodeError();

	return bc;
}

size_t DEREncodeDate(BufferedTransformation &bt, const SecByteBlock &str, byte asnTag)
{
	bt.Put(asnTag);
	size_t lengthBytes = DERLengthEncode(bt, str.size());
	bt.Put(ConstBytePtr(str), BytePtrSize(str));
	return 1+lengthBytes+str.size();
}

size_t BERDecodeDate(BufferedTransformation &bt, SecByteBlock &str, byte asnTag)
{
	byte b;
	if (!bt.Get(b) || b != asnTag)
		BERDecodeError();

	size_t bc;
	if (!BERLengthDecode(bt, bc))
		BERDecodeError();
	if (bc > bt.MaxRetrievable()) // Issue 346
		BERDecodeError();

	str.resize(bc);
	if (bc != bt.Get(BytePtr(str), BytePtrSize(str)))
		BERDecodeError();

	return bc;
}

size_t DEREncodeBitString(BufferedTransformation &bt, const byte *str, size_t strLen, unsigned int unusedBits)
{
	bt.Put(BIT_STRING);
	size_t lengthBytes = DERLengthEncode(bt, strLen+1);
	bt.Put((byte)unusedBits);
	bt.Put(str, strLen);
	return 2+lengthBytes+strLen;
}

size_t BERDecodeBitString(BufferedTransformation &bt, SecByteBlock &str, unsigned int &unusedBits)
{
	byte b;
	if (!bt.Get(b) || b != BIT_STRING)
		BERDecodeError();

	size_t bc;
	if (!BERLengthDecode(bt, bc))
		BERDecodeError();
	if (bc == 0)
		BERDecodeError();
	if (bc > bt.MaxRetrievable()) // Issue 346
		BERDecodeError();

	// X.690, 8.6.2.2: "The number [of unused bits] shall be in the range zero to seven"
	byte unused;
	if (!bt.Get(unused) || unused > 7)
		BERDecodeError();
	unusedBits = unused;
	str.resize(bc-1);
	if ((bc-1) != bt.Get(BytePtr(str), bc-1))
		BERDecodeError();
	return bc-1;
}

void DERReencode(BufferedTransformation &source, BufferedTransformation &dest)
{
	byte tag;
	source.Peek(tag);
	BERGeneralDecoder decoder(source, tag);
	DERGeneralEncoder encoder(dest, tag);
	if (decoder.IsDefiniteLength())
		decoder.TransferTo(encoder, decoder.RemainingLength());
	else
	{
		while (!decoder.EndReached())
			DERReencode(decoder, encoder);
	}
	decoder.MessageEnd();
	encoder.MessageEnd();
}

size_t BERDecodePeekLength(const BufferedTransformation &bt)
{
	lword count = (std::min)(bt.MaxRetrievable(), static_cast<lword>(16));
	if (count == 0) return 0;

	ByteQueue tagAndLength;
	bt.CopyTo(tagAndLength, count);

	// Skip tag
	tagAndLength.Skip(1);

	// BERLengthDecode fails for indefinite length.
	size_t length;
	if (!BERLengthDecode(tagAndLength, length))
		return 0;

	return length;
}

void OID::EncodeValue(BufferedTransformation &bt, word32 v)
{
	for (unsigned int i=RoundUpToMultipleOf(STDMAX(7U,BitPrecision(v)), 7U)-7; i != 0; i-=7)
		bt.Put((byte)(0x80 | ((v >> i) & 0x7f)));
	bt.Put((byte)(v & 0x7f));
}

size_t OID::DecodeValue(BufferedTransformation &bt, word32 &v)
{
	byte b;
	size_t i=0;
	v = 0;
	while (true)
	{
		if (!bt.Get(b))
			BERDecodeError();
		i++;
		if (v >> (8*sizeof(v)-7))	// v about to overflow
			BERDecodeError();
		v <<= 7;
		v += b & 0x7f;
		if (!(b & 0x80))
			return i;
	}
}

void OID::DEREncode(BufferedTransformation &bt) const
{
	CRYPTOPP_ASSERT(m_values.size() >= 2);
	ByteQueue temp;
	temp.Put(byte(m_values[0] * 40 + m_values[1]));
	for (size_t i=2; i<m_values.size(); i++)
		EncodeValue(temp, m_values[i]);
	bt.Put(OBJECT_IDENTIFIER);
	DERLengthEncode(bt, temp.CurrentSize());
	temp.TransferTo(bt);
}

void OID::BERDecode(BufferedTransformation &bt)
{
	byte b;
	if (!bt.Get(b) || b != OBJECT_IDENTIFIER)
		BERDecodeError();

	size_t length;
	if (!BERLengthDecode(bt, length) || length < 1)
		BERDecodeError();

	if (!bt.Get(b))
		BERDecodeError();

	length--;
	m_values.resize(2);
	m_values[0] = b / 40;
	m_values[1] = b % 40;

	while (length > 0)
	{
		word32 v;
		size_t valueLen = DecodeValue(bt, v);
		if (valueLen > length)
			BERDecodeError();
		m_values.push_back(v);
		length -= valueLen;
	}
}

void OID::BERDecodeAndCheck(BufferedTransformation &bt) const
{
	OID oid(bt);
	if (*this != oid)
		BERDecodeError();
}

std::ostream& OID::Print(std::ostream& out) const
{
	std::ostringstream oss;
	for (size_t i = 0; i < m_values.size(); ++i)
	{
		oss << m_values[i];
		if (i+1 < m_values.size())
			oss << ".";
	}
	return out << oss.str();
}

inline BufferedTransformation & EncodedObjectFilter::CurrentTarget()
{
	if (m_flags & PUT_OBJECTS)
		return *AttachedTransformation();
	else
		return TheBitBucket();
}

void EncodedObjectFilter::Put(const byte *inString, size_t length)
{
	if (m_nCurrentObject == m_nObjects)
	{
		AttachedTransformation()->Put(inString, length);
		return;
	}

	LazyPutter lazyPutter(m_queue, inString, length);

	while (m_queue.AnyRetrievable())
	{
		switch (m_state)
		{
		case IDENTIFIER:
			if (!m_queue.Get(m_id))
				return;
			m_queue.TransferTo(CurrentTarget(), 1);
			m_state = LENGTH;
		// fall through
		case LENGTH:
		{
			byte b;
			if (m_level > 0 && m_id == 0 && m_queue.Peek(b) && b == 0)
			{
				m_queue.TransferTo(CurrentTarget(), 1);
				m_level--;
				m_state = IDENTIFIER;
				break;
			}
			ByteQueue::Walker walker(m_queue);
			bool definiteLength = false;
			if (!BERLengthDecode(walker, m_lengthRemaining, definiteLength))
				return;
			m_queue.TransferTo(CurrentTarget(), walker.GetCurrentPosition());
			if (!((m_id & CONSTRUCTED) || definiteLength))
				BERDecodeError();
			if (!definiteLength)
			{
				if (!(m_id & CONSTRUCTED))
					BERDecodeError();
				m_level++;
				m_state = IDENTIFIER;
				break;
			}
			m_state = BODY;
		}
		// fall through
		case BODY:
			m_lengthRemaining -= m_queue.TransferTo(CurrentTarget(), m_lengthRemaining);

			if (m_lengthRemaining == 0)
				m_state = IDENTIFIER;
		// fall through
		case TAIL:
		case ALL_DONE:
		default: ;
		}

		if (m_state == IDENTIFIER && m_level == 0)
		{
			// just finished processing a level 0 object
			++m_nCurrentObject;

			if (m_flags & PUT_MESSANGE_END_AFTER_EACH_OBJECT)
				AttachedTransformation()->MessageEnd();

			if (m_nCurrentObject == m_nObjects)
			{
				if (m_flags & PUT_MESSANGE_END_AFTER_ALL_OBJECTS)
					AttachedTransformation()->MessageEnd();

				if (m_flags & PUT_MESSANGE_SERIES_END_AFTER_ALL_OBJECTS)
					AttachedTransformation()->MessageSeriesEnd();

				m_queue.TransferAllTo(*AttachedTransformation());
				return;
			}
		}
	}
}

BERGeneralDecoder::BERGeneralDecoder(BufferedTransformation &inQueue)
	: m_inQueue(inQueue), m_length(0), m_finished(false)
{
	Init(DefaultTag);
}

BERGeneralDecoder::BERGeneralDecoder(BufferedTransformation &inQueue, byte asnTag)
	: m_inQueue(inQueue), m_length(0), m_finished(false)
{
	Init(asnTag);
}

BERGeneralDecoder::BERGeneralDecoder(BERGeneralDecoder &inQueue, byte asnTag)
	: m_inQueue(inQueue), m_length(0), m_finished(false)
{
	Init(asnTag);
}

void BERGeneralDecoder::Init(byte asnTag)
{
	byte b;
	if (!m_inQueue.Get(b) || b != asnTag)
		BERDecodeError();

	if (!BERLengthDecode(m_inQueue, m_length, m_definiteLength))
		BERDecodeError();

	if (!m_definiteLength && !(asnTag & CONSTRUCTED))
		BERDecodeError();	// cannot be primitive and have indefinite length
}

BERGeneralDecoder::~BERGeneralDecoder()
{
	try	// avoid throwing in destructor
	{
		if (!m_finished)
			MessageEnd();
	}
	catch (const Exception&)
	{
		// CRYPTOPP_ASSERT(0);
	}
}

bool BERGeneralDecoder::EndReached() const
{
	if (m_definiteLength)
		return m_length == 0;
	else
	{	// check end-of-content octets
		word16 i;
		return (m_inQueue.PeekWord16(i)==2 && i==0);
	}
}

byte BERGeneralDecoder::PeekByte() const
{
	byte b;
	if (!Peek(b))
		BERDecodeError();
	return b;
}

void BERGeneralDecoder::CheckByte(byte check)
{
	byte b;
	if (!Get(b) || b != check)
		BERDecodeError();
}

void BERGeneralDecoder::MessageEnd()
{
	m_finished = true;
	if (m_definiteLength)
	{
		if (m_length != 0)
			BERDecodeError();
	}
	else
	{	// remove end-of-content octets
		word16 i;
		if (m_inQueue.GetWord16(i) != 2 || i != 0)
			BERDecodeError();
	}
}

size_t BERGeneralDecoder::TransferTo2(BufferedTransformation &target, lword &transferBytes, const std::string &channel, bool blocking)
{
	if (m_definiteLength && transferBytes > m_length)
		transferBytes = m_length;
	size_t blockedBytes = m_inQueue.TransferTo2(target, transferBytes, channel, blocking);
	ReduceLength(transferBytes);
	return blockedBytes;
}

size_t BERGeneralDecoder::CopyRangeTo2(BufferedTransformation &target, lword &begin, lword end, const std::string &channel, bool blocking) const
{
	if (m_definiteLength)
		end = STDMIN(m_length, end);
	return m_inQueue.CopyRangeTo2(target, begin, end, channel, blocking);
}

lword BERGeneralDecoder::ReduceLength(lword delta)
{
	if (m_definiteLength)
	{
		if (m_length < delta)
			BERDecodeError();
		m_length -= delta;
	}
	return delta;
}

DERGeneralEncoder::DERGeneralEncoder(BufferedTransformation &outQueue)
	: m_outQueue(outQueue), m_asnTag(DefaultTag), m_finished(false)
{
}

DERGeneralEncoder::DERGeneralEncoder(BufferedTransformation &outQueue, byte asnTag)
	: m_outQueue(outQueue), m_asnTag(asnTag), m_finished(false)
{
}

DERGeneralEncoder::DERGeneralEncoder(DERGeneralEncoder &outQueue, byte asnTag)
	: m_outQueue(outQueue), m_asnTag(asnTag), m_finished(false)
{
}

DERGeneralEncoder::~DERGeneralEncoder()
{
	try	// avoid throwing in constructor
	{
		if (!m_finished)
			MessageEnd();
	}
	catch (const Exception&)
	{
		CRYPTOPP_ASSERT(0);
	}
}

void DERGeneralEncoder::MessageEnd()
{
	m_finished = true;
	lword length = CurrentSize();
	m_outQueue.Put(m_asnTag);
	DERLengthEncode(m_outQueue, length);
	TransferTo(m_outQueue);
}

// *************************************************************

void X509PublicKey::BERDecode(BufferedTransformation &bt)
{
	BERSequenceDecoder subjectPublicKeyInfo(bt);
		BERSequenceDecoder algorithm(subjectPublicKeyInfo);
			GetAlgorithmID().BERDecodeAndCheck(algorithm);
			bool parametersPresent = algorithm.EndReached() ? false : BERDecodeAlgorithmParameters(algorithm);
		algorithm.MessageEnd();

		BERGeneralDecoder subjectPublicKey(subjectPublicKeyInfo, BIT_STRING);
			subjectPublicKey.CheckByte(0);	// unused bits
			BERDecodePublicKey(subjectPublicKey, parametersPresent, (size_t)subjectPublicKey.RemainingLength());
		subjectPublicKey.MessageEnd();
	subjectPublicKeyInfo.MessageEnd();
}

void X509PublicKey::DEREncode(BufferedTransformation &bt) const
{
	DERSequenceEncoder subjectPublicKeyInfo(bt);

		DERSequenceEncoder algorithm(subjectPublicKeyInfo);
			GetAlgorithmID().DEREncode(algorithm);
			DEREncodeAlgorithmParameters(algorithm);
		algorithm.MessageEnd();

		DERGeneralEncoder subjectPublicKey(subjectPublicKeyInfo, BIT_STRING);
			subjectPublicKey.Put(0);	// unused bits
			DEREncodePublicKey(subjectPublicKey);
		subjectPublicKey.MessageEnd();

	subjectPublicKeyInfo.MessageEnd();
}

void PKCS8PrivateKey::BERDecode(BufferedTransformation &bt)
{
	BERSequenceDecoder privateKeyInfo(bt);
		word32 version;
		BERDecodeUnsigned<word32>(privateKeyInfo, version, INTEGER, 0, 0);	// check version

		BERSequenceDecoder algorithm(privateKeyInfo);
			GetAlgorithmID().BERDecodeAndCheck(algorithm);
			bool parametersPresent = algorithm.EndReached() ? false : BERDecodeAlgorithmParameters(algorithm);
		algorithm.MessageEnd();

		BERGeneralDecoder octetString(privateKeyInfo, OCTET_STRING);
			BERDecodePrivateKey(octetString, parametersPresent, (size_t)privateKeyInfo.RemainingLength());
		octetString.MessageEnd();

		if (!privateKeyInfo.EndReached())
			BERDecodeOptionalAttributes(privateKeyInfo);
	privateKeyInfo.MessageEnd();
}

void PKCS8PrivateKey::DEREncode(BufferedTransformation &bt) const
{
	DERSequenceEncoder privateKeyInfo(bt);
		DEREncodeUnsigned<word32>(privateKeyInfo, 0);	// version

		DERSequenceEncoder algorithm(privateKeyInfo);
			GetAlgorithmID().DEREncode(algorithm);
			DEREncodeAlgorithmParameters(algorithm);
		algorithm.MessageEnd();

		DERGeneralEncoder octetString(privateKeyInfo, OCTET_STRING);
			DEREncodePrivateKey(octetString);
		octetString.MessageEnd();

		DEREncodeOptionalAttributes(privateKeyInfo);
	privateKeyInfo.MessageEnd();
}

void PKCS8PrivateKey::BERDecodeOptionalAttributes(BufferedTransformation &bt)
{
	DERReencode(bt, m_optionalAttributes);
}

void PKCS8PrivateKey::DEREncodeOptionalAttributes(BufferedTransformation &bt) const
{
	m_optionalAttributes.CopyTo(bt);
}

NAMESPACE_END

#endif
