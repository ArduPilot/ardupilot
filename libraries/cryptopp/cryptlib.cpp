// cryptlib.cpp - originally written and placed in the public domain by Wei Dai

#include "pch.h"
#include "config.h"

#if CRYPTOPP_MSC_VERSION
# pragma warning(disable: 4127 4189 4459)
#endif

#if CRYPTOPP_GCC_DIAGNOSTIC_AVAILABLE
# pragma GCC diagnostic ignored "-Wunused-value"
# pragma GCC diagnostic ignored "-Wunused-variable"
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#ifndef CRYPTOPP_IMPORTS

#include "cryptlib.h"
#include "filters.h"
#include "algparam.h"
#include "fips140.h"
#include "argnames.h"
#include "fltrimpl.h"
#include "osrng.h"
#include "secblock.h"
#include "smartptr.h"
#include "stdcpp.h"
#include "misc.h"

NAMESPACE_BEGIN(CryptoPP)

CRYPTOPP_COMPILE_ASSERT(sizeof(byte) == 1);
CRYPTOPP_COMPILE_ASSERT(sizeof(word16) == 2);
CRYPTOPP_COMPILE_ASSERT(sizeof(word32) == 4);
CRYPTOPP_COMPILE_ASSERT(sizeof(word64) == 8);
#ifdef CRYPTOPP_NATIVE_DWORD_AVAILABLE
CRYPTOPP_COMPILE_ASSERT(sizeof(dword) == 2*sizeof(word));
#endif

BufferedTransformation & TheBitBucket()
{
	static BitBucket bitBucket;
	return bitBucket;
}

Algorithm::Algorithm(bool checkSelfTestStatus)
{
	if (checkSelfTestStatus && FIPS_140_2_ComplianceEnabled())
	{
		if (GetPowerUpSelfTestStatus() == POWER_UP_SELF_TEST_NOT_DONE && !PowerUpSelfTestInProgressOnThisThread())
			throw SelfTestFailure("Cryptographic algorithms are disabled before the power-up self tests are performed.");

		if (GetPowerUpSelfTestStatus() == POWER_UP_SELF_TEST_FAILED)
			throw SelfTestFailure("Cryptographic algorithms are disabled after a power-up self test failed.");
	}
}

void SimpleKeyingInterface::SetKey(const byte *key, size_t length, const NameValuePairs &params)
{
	this->ThrowIfInvalidKeyLength(length);
	this->UncheckedSetKey(key, static_cast<unsigned int>(length), params);
}

void SimpleKeyingInterface::SetKeyWithRounds(const byte *key, size_t length, int rounds)
{
	SetKey(key, length, MakeParameters(Name::Rounds(), rounds));
}

void SimpleKeyingInterface::SetKeyWithIV(const byte *key, size_t length, const byte *iv, size_t ivLength)
{
	SetKey(key, length, MakeParameters(Name::IV(), ConstByteArrayParameter(iv, ivLength)));
}

void SimpleKeyingInterface::ThrowIfInvalidKeyLength(size_t length)
{
	if (!IsValidKeyLength(length))
		throw InvalidKeyLength(GetAlgorithm().AlgorithmName(), length);
}

void SimpleKeyingInterface::ThrowIfResynchronizable()
{
	if (IsResynchronizable())
		throw InvalidArgument(GetAlgorithm().AlgorithmName() + ": this object requires an IV");
}

void SimpleKeyingInterface::ThrowIfInvalidIV(const byte *iv)
{
	if (!iv && IVRequirement() == UNPREDICTABLE_RANDOM_IV)
		throw InvalidArgument(GetAlgorithm().AlgorithmName() + ": this object cannot use a null IV");
}

size_t SimpleKeyingInterface::ThrowIfInvalidIVLength(int length)
{
	size_t size = 0;
	if (length < 0)
		size = static_cast<size_t>(IVSize());
	else if ((size_t)length < MinIVLength())
		throw InvalidArgument(GetAlgorithm().AlgorithmName() + ": IV length " + IntToString(length) + " is less than the minimum of " + IntToString(MinIVLength()));
	else if ((size_t)length > MaxIVLength())
		throw InvalidArgument(GetAlgorithm().AlgorithmName() + ": IV length " + IntToString(length) + " exceeds the maximum of " + IntToString(MaxIVLength()));
	else
		size = static_cast<size_t>(length);

	return size;
}

const byte * SimpleKeyingInterface::GetIVAndThrowIfInvalid(const NameValuePairs &params, size_t &size)
{
	ConstByteArrayParameter ivWithLength;
	const byte *iv = NULLPTR;
	bool found = false;

	try {found = params.GetValue(Name::IV(), ivWithLength);}
	catch (const NameValuePairs::ValueTypeMismatch &) {}

	if (found)
	{
		iv = ivWithLength.begin();
		ThrowIfInvalidIV(iv);
		size = ThrowIfInvalidIVLength(static_cast<int>(ivWithLength.size()));
	}
	else if (params.GetValue(Name::IV(), iv))
	{
		ThrowIfInvalidIV(iv);
		size = static_cast<size_t>(IVSize());
	}
	else
	{
		ThrowIfResynchronizable();
		size = 0;
	}

	return iv;
}

void SimpleKeyingInterface::GetNextIV(RandomNumberGenerator &rng, byte *iv)
{
	rng.GenerateBlock(iv, IVSize());
}

size_t BlockTransformation::AdvancedProcessBlocks(const byte *inBlocks, const byte *xorBlocks, byte *outBlocks, size_t length, word32 flags) const
{
	CRYPTOPP_ASSERT(inBlocks);
	CRYPTOPP_ASSERT(outBlocks);
	CRYPTOPP_ASSERT(length);

	const unsigned int blockSize = BlockSize();
	size_t inIncrement = (flags & (BT_InBlockIsCounter|BT_DontIncrementInOutPointers)) ? 0 : blockSize;
	size_t xorIncrement = xorBlocks ? blockSize : 0;
	size_t outIncrement = (flags & BT_DontIncrementInOutPointers) ? 0 : blockSize;

	if (flags & BT_ReverseDirection)
	{
		inBlocks = PtrAdd(inBlocks, length - blockSize);
		xorBlocks = PtrAdd(xorBlocks, length - blockSize);
		outBlocks = PtrAdd(outBlocks, length - blockSize);
		inIncrement = 0-inIncrement;
		xorIncrement = 0-xorIncrement;
		outIncrement = 0-outIncrement;
	}

	// Coverity finding.
	const bool xorFlag = xorBlocks && (flags & BT_XorInput);
	while (length >= blockSize)
	{
		if (xorFlag)
		{
			// xorBlocks non-NULL and with BT_XorInput.
			xorbuf(outBlocks, xorBlocks, inBlocks, blockSize);
			ProcessBlock(outBlocks);
		}
		else
		{
			// xorBlocks may be non-NULL and without BT_XorInput.
			ProcessAndXorBlock(inBlocks, xorBlocks, outBlocks);
		}

		if (flags & BT_InBlockIsCounter)
			const_cast<byte *>(inBlocks)[blockSize-1]++;

		inBlocks = PtrAdd(inBlocks, inIncrement);
		outBlocks = PtrAdd(outBlocks, outIncrement);
		xorBlocks = PtrAdd(xorBlocks, xorIncrement);
		length -= blockSize;
	}

	return length;
}

unsigned int BlockTransformation::OptimalDataAlignment() const
{
	return GetAlignmentOf<word32>();
}

unsigned int StreamTransformation::OptimalDataAlignment() const
{
	return GetAlignmentOf<word32>();
}

unsigned int HashTransformation::OptimalDataAlignment() const
{
	return GetAlignmentOf<word32>();
}

#if 0
void StreamTransformation::ProcessLastBlock(byte *outString, const byte *inString, size_t length)
{
	CRYPTOPP_ASSERT(MinLastBlockSize() == 0);	// this function should be overridden otherwise

	if (length == MandatoryBlockSize())
		ProcessData(outString, inString, length);
	else if (length != 0)
		throw NotImplemented(AlgorithmName() + ": this object doesn't support a special last block");
}
#endif

size_t StreamTransformation::ProcessLastBlock(byte *outString, size_t outLength, const byte *inString, size_t inLength)
{
	// this function should be overridden otherwise
	CRYPTOPP_ASSERT(MinLastBlockSize() == 0);

	if (inLength == MandatoryBlockSize())
	{
		outLength = inLength; // squash unused warning
		ProcessData(outString, inString, inLength);
	}
	else if (inLength != 0)
		throw NotImplemented(AlgorithmName() + ": this object doesn't support a special last block");

	return outLength;
}

void AuthenticatedSymmetricCipher::SpecifyDataLengths(lword headerLength, lword messageLength, lword footerLength)
{
	if (headerLength > MaxHeaderLength())
		throw InvalidArgument(GetAlgorithm().AlgorithmName() + ": header length " + IntToString(headerLength) + " exceeds the maximum of " + IntToString(MaxHeaderLength()));

	if (messageLength > MaxMessageLength())
		throw InvalidArgument(GetAlgorithm().AlgorithmName() + ": message length " + IntToString(messageLength) + " exceeds the maximum of " + IntToString(MaxMessageLength()));

	if (footerLength > MaxFooterLength())
		throw InvalidArgument(GetAlgorithm().AlgorithmName() + ": footer length " + IntToString(footerLength) + " exceeds the maximum of " + IntToString(MaxFooterLength()));

	UncheckedSpecifyDataLengths(headerLength, messageLength, footerLength);
}

void AuthenticatedSymmetricCipher::EncryptAndAuthenticate(byte *ciphertext, byte *mac, size_t macSize, const byte *iv, int ivLength, const byte *header, size_t headerLength, const byte *message, size_t messageLength)
{
	Resynchronize(iv, ivLength);
	SpecifyDataLengths(headerLength, messageLength);
	Update(header, headerLength);
	ProcessString(ciphertext, message, messageLength);
	TruncatedFinal(mac, macSize);
}

bool AuthenticatedSymmetricCipher::DecryptAndVerify(byte *message, const byte *mac, size_t macLength, const byte *iv, int ivLength, const byte *header, size_t headerLength, const byte *ciphertext, size_t ciphertextLength)
{
	Resynchronize(iv, ivLength);
	SpecifyDataLengths(headerLength, ciphertextLength);
	Update(header, headerLength);
	ProcessString(message, ciphertext, ciphertextLength);
	return TruncatedVerify(mac, macLength);
}

std::string AuthenticatedSymmetricCipher::AlgorithmName() const
{
	// Squash C4505 on Visual Studio 2008 and friends
	return "Unknown";
}

unsigned int RandomNumberGenerator::GenerateBit()
{
	return GenerateByte() & 1;
}

byte RandomNumberGenerator::GenerateByte()
{
	byte b;
	GenerateBlock(&b, 1);
	return b;
}

word32 RandomNumberGenerator::GenerateWord32(word32 min, word32 max)
{
	const word32 range = max-min;
	const unsigned int maxBits = BitPrecision(range);

	word32 value;

	do
	{
		GenerateBlock((byte *)&value, sizeof(value));
		value = Crop(value, maxBits);
	} while (value > range);

	return value+min;
}

// Stack recursion below... GenerateIntoBufferedTransformation calls GenerateBlock,
// and GenerateBlock calls GenerateIntoBufferedTransformation. Ad infinitum. Also
// see http://github.com/weidai11/cryptopp/issues/38.
//
// According to Wei, RandomNumberGenerator is an interface, and it should not
// be instantiable. Its now spilt milk, and we are going to CRYPTOPP_ASSERT it in Debug
// builds to alert the programmer and throw in Release builds. Developers have
// a reference implementation in case its needed. If a programmer
// unintentionally lands here, then they should ensure use of a
// RandomNumberGenerator pointer or reference so polymorphism can provide the
// proper runtime dispatching.

void RandomNumberGenerator::GenerateBlock(byte *output, size_t size)
{
	CRYPTOPP_UNUSED(output), CRYPTOPP_UNUSED(size);

	ArraySink s(output, size);
	GenerateIntoBufferedTransformation(s, DEFAULT_CHANNEL, size);
}

void RandomNumberGenerator::DiscardBytes(size_t n)
{
	GenerateIntoBufferedTransformation(TheBitBucket(), DEFAULT_CHANNEL, n);
}

void RandomNumberGenerator::GenerateIntoBufferedTransformation(BufferedTransformation &target, const std::string &channel, lword length)
{
	FixedSizeSecBlock<byte, 256> buffer;
	while (length)
	{
		size_t len = UnsignedMin(buffer.size(), length);
		GenerateBlock(buffer, len);
		(void)target.ChannelPut(channel, buffer, len);
		length -= len;
	}
}

size_t KeyDerivationFunction::MinDerivedKeyLength() const
{
	return 0;
}

size_t KeyDerivationFunction::MaxDerivedKeyLength() const
{
	return static_cast<size_t>(-1);
}

void KeyDerivationFunction::ThrowIfInvalidDerivedKeyLength(size_t length) const
{
	if (!IsValidDerivedLength(length))
		throw InvalidDerivedKeyLength(GetAlgorithm().AlgorithmName(), length);
}

void KeyDerivationFunction::SetParameters(const NameValuePairs& params) {
	CRYPTOPP_UNUSED(params);
}

/// \brief Random Number Generator that does not produce random numbers
/// \details ClassNullRNG can be used for functions that require a RandomNumberGenerator
///   but don't actually use it. The class throws NotImplemented when a generation function is called.
/// \sa NullRNG()
class ClassNullRNG : public RandomNumberGenerator
{
public:
	/// \brief The name of the generator
	/// \returns the string \a NullRNGs
	std::string AlgorithmName() const {return "NullRNG";}

#if defined(CRYPTOPP_DOXYGEN_PROCESSING)
	/// \brief An implementation that throws NotImplemented
	byte GenerateByte () {}
	/// \brief An implementation that throws NotImplemented
	unsigned int GenerateBit () {}
	/// \brief An implementation that throws NotImplemented
	word32 GenerateWord32 (word32 min, word32 max) {}
#endif

	/// \brief An implementation that throws NotImplemented
	void GenerateBlock(byte *output, size_t size)
	{
		CRYPTOPP_UNUSED(output); CRYPTOPP_UNUSED(size);
		throw NotImplemented("NullRNG: NullRNG should only be passed to functions that don't need to generate random bytes");
	}

#if defined(CRYPTOPP_DOXYGEN_PROCESSING)
	/// \brief An implementation that throws NotImplemented
	void GenerateIntoBufferedTransformation (BufferedTransformation &target, const std::string &channel, lword length) {}
	/// \brief An implementation that throws NotImplemented
	void IncorporateEntropy (const byte *input, size_t length) {}
	/// \brief An implementation that returns \p false
	bool CanIncorporateEntropy () const {}
	/// \brief An implementation that does nothing
	void DiscardBytes (size_t n) {}
	/// \brief An implementation that does nothing
	void Shuffle (IT begin, IT end) {}

private:
	Clonable* Clone () const { return NULLPTR; }
#endif
};

RandomNumberGenerator & NullRNG()
{
	static ClassNullRNG s_nullRNG;
	return s_nullRNG;
}

bool HashTransformation::TruncatedVerify(const byte *digest, size_t digestLength)
{
	// Allocate at least 1 for calculated to avoid triggering diagnostics
	ThrowIfInvalidTruncatedSize(digestLength);
	SecByteBlock calculated(digestLength ? digestLength : 1);
	TruncatedFinal(calculated, digestLength);
	return VerifyBufsEqual(calculated, digest, digestLength);
}

void HashTransformation::ThrowIfInvalidTruncatedSize(size_t size) const
{
	if (size > DigestSize())
		throw InvalidArgument("HashTransformation: can't truncate a " + IntToString(DigestSize()) + " byte digest to " + IntToString(size) + " bytes");
}

unsigned int BufferedTransformation::GetMaxWaitObjectCount() const
{
	const BufferedTransformation *t = AttachedTransformation();
	return t ? t->GetMaxWaitObjectCount() : 0;
}

void BufferedTransformation::GetWaitObjects(WaitObjectContainer &container, CallStack const& callStack)
{
	BufferedTransformation *t = AttachedTransformation();
	if (t)
		t->GetWaitObjects(container, callStack);  // reduce clutter by not adding to stack here
}

void BufferedTransformation::Initialize(const NameValuePairs &parameters, int propagation)
{
	CRYPTOPP_UNUSED(propagation);
	CRYPTOPP_ASSERT(!AttachedTransformation());
	IsolatedInitialize(parameters);
}

bool BufferedTransformation::Flush(bool hardFlush, int propagation, bool blocking)
{
	CRYPTOPP_UNUSED(propagation);
	CRYPTOPP_ASSERT(!AttachedTransformation());
	return IsolatedFlush(hardFlush, blocking);
}

bool BufferedTransformation::MessageSeriesEnd(int propagation, bool blocking)
{
	CRYPTOPP_UNUSED(propagation);
	CRYPTOPP_ASSERT(!AttachedTransformation());
	return IsolatedMessageSeriesEnd(blocking);
}

byte * BufferedTransformation::ChannelCreatePutSpace(const std::string &channel, size_t &size)
{
	byte* space = NULLPTR;
	if (channel.empty())
		space = CreatePutSpace(size);
	else
		throw NoChannelSupport(AlgorithmName());
	return space;
}

size_t BufferedTransformation::ChannelPut2(const std::string &channel, const byte *inString, size_t length, int messageEnd, bool blocking)
{
	size_t size = 0;
	if (channel.empty())
		size = Put2(inString, length, messageEnd, blocking);
	else
		throw NoChannelSupport(AlgorithmName());
	return size;
}

size_t BufferedTransformation::ChannelPutModifiable2(const std::string &channel, byte *inString, size_t length, int messageEnd, bool blocking)
{
	size_t size = 0;
	if (channel.empty())
		size = PutModifiable2(inString, length, messageEnd, blocking);
	else
		size = ChannelPut2(channel, inString, length, messageEnd, blocking);
	return size;
}

bool BufferedTransformation::ChannelFlush(const std::string &channel, bool hardFlush, int propagation, bool blocking)
{
	bool result = 0;
	if (channel.empty())
		result = Flush(hardFlush, propagation, blocking);
	else
		throw NoChannelSupport(AlgorithmName());
	return result;
}

bool BufferedTransformation::ChannelMessageSeriesEnd(const std::string &channel, int propagation, bool blocking)
{
	bool result = false;
	if (channel.empty())
		result = MessageSeriesEnd(propagation, blocking);
	else
		throw NoChannelSupport(AlgorithmName());
	return result;
}

lword BufferedTransformation::MaxRetrievable() const
{
	lword size = 0;
	if (AttachedTransformation())
		size = AttachedTransformation()->MaxRetrievable();
	else
		size = CopyTo(TheBitBucket());
	return size;
}

bool BufferedTransformation::AnyRetrievable() const
{
	bool result = false;
	if (AttachedTransformation())
		result = AttachedTransformation()->AnyRetrievable();
	else
	{
		byte b;
		result = Peek(b) != 0;
	}
	return result;
}

size_t BufferedTransformation::Get(byte &outByte)
{
	size_t size = 0;
	if (AttachedTransformation())
		size = AttachedTransformation()->Get(outByte);
	else
		size = Get(&outByte, 1);
	return size;
}

size_t BufferedTransformation::Get(byte *outString, size_t getMax)
{
	size_t size = 0;
	if (AttachedTransformation())
		size = AttachedTransformation()->Get(outString, getMax);
	else
	{
		ArraySink arraySink(outString, getMax);
		size = (size_t)TransferTo(arraySink, getMax);
	}
	return size;
}

size_t BufferedTransformation::Peek(byte &outByte) const
{
	size_t size = 0;
	if (AttachedTransformation())
		size = AttachedTransformation()->Peek(outByte);
	else
		size = Peek(&outByte, 1);
	return size;
}

size_t BufferedTransformation::Peek(byte *outString, size_t peekMax) const
{
	size_t size = 0;
	if (AttachedTransformation())
		size = AttachedTransformation()->Peek(outString, peekMax);
	else
	{
		ArraySink arraySink(outString, peekMax);
		size = (size_t)CopyTo(arraySink, peekMax);
	}
	return size;
}

lword BufferedTransformation::Skip(lword skipMax)
{
	lword size = 0;
	if (AttachedTransformation())
		size = AttachedTransformation()->Skip(skipMax);
	else
		size = TransferTo(TheBitBucket(), skipMax);
	return size;
}

lword BufferedTransformation::TotalBytesRetrievable() const
{
	lword size = 0;
	if (AttachedTransformation())
		size = AttachedTransformation()->TotalBytesRetrievable();
	else
		size = MaxRetrievable();
	return size;
}

unsigned int BufferedTransformation::NumberOfMessages() const
{
	unsigned int size = 0;
	if (AttachedTransformation())
		size = AttachedTransformation()->NumberOfMessages();
	else
		size = CopyMessagesTo(TheBitBucket());
	return size;
}

bool BufferedTransformation::AnyMessages() const
{
	bool result = false;
	if (AttachedTransformation())
		result = AttachedTransformation()->AnyMessages();
	else
		result = NumberOfMessages() != 0;
	return result;
}

bool BufferedTransformation::GetNextMessage()
{
	bool result = false;
	if (AttachedTransformation())
		result = AttachedTransformation()->GetNextMessage();
	else
	{
		CRYPTOPP_ASSERT(!AnyMessages());
	}
	return result;
}

unsigned int BufferedTransformation::SkipMessages(unsigned int count)
{
	unsigned int size = 0;
	if (AttachedTransformation())
		size = AttachedTransformation()->SkipMessages(count);
	else
		size = TransferMessagesTo(TheBitBucket(), count);
	return size;
}

size_t BufferedTransformation::TransferMessagesTo2(BufferedTransformation &target, unsigned int &messageCount, const std::string &channel, bool blocking)
{
	if (AttachedTransformation())
		return AttachedTransformation()->TransferMessagesTo2(target, messageCount, channel, blocking);
	else
	{
		unsigned int maxMessages = messageCount;
		for (messageCount=0; messageCount < maxMessages && AnyMessages(); messageCount++)
		{
			size_t blockedBytes;
			lword transferredBytes;

			while (AnyRetrievable())
			{
				transferredBytes = LWORD_MAX;
				blockedBytes = TransferTo2(target, transferredBytes, channel, blocking);
				if (blockedBytes > 0)
					return blockedBytes;
			}

			if (target.ChannelMessageEnd(channel, GetAutoSignalPropagation(), blocking))
				return 1;

			bool result = GetNextMessage();
			CRYPTOPP_UNUSED(result); CRYPTOPP_ASSERT(result);
		}
		return 0;
	}
}

unsigned int BufferedTransformation::CopyMessagesTo(BufferedTransformation &target, unsigned int count, const std::string &channel) const
{
	unsigned int size = 0;
	if (AttachedTransformation())
		size = AttachedTransformation()->CopyMessagesTo(target, count, channel);
	return size;
}

void BufferedTransformation::SkipAll()
{
	if (AttachedTransformation())
		AttachedTransformation()->SkipAll();
	else
	{
		while (SkipMessages()) {}
		while (Skip()) {}
	}
}

size_t BufferedTransformation::TransferAllTo2(BufferedTransformation &target, const std::string &channel, bool blocking)
{
	if (AttachedTransformation())
		return AttachedTransformation()->TransferAllTo2(target, channel, blocking);
	else
	{
		CRYPTOPP_ASSERT(!NumberOfMessageSeries());

		unsigned int messageCount;
		do
		{
			messageCount = UINT_MAX;
			size_t blockedBytes = TransferMessagesTo2(target, messageCount, channel, blocking);
			if (blockedBytes)
				return blockedBytes;
		}
		while (messageCount != 0);

		lword byteCount;
		do
		{
			byteCount = ULONG_MAX;
			size_t blockedBytes = TransferTo2(target, byteCount, channel, blocking);
			if (blockedBytes)
				return blockedBytes;
		}
		while (byteCount != 0);

		return 0;
	}
}

void BufferedTransformation::CopyAllTo(BufferedTransformation &target, const std::string &channel) const
{
	if (AttachedTransformation())
		AttachedTransformation()->CopyAllTo(target, channel);
	else
	{
		CRYPTOPP_ASSERT(!NumberOfMessageSeries());
		while (CopyMessagesTo(target, UINT_MAX, channel)) {}
	}
}

void BufferedTransformation::SetRetrievalChannel(const std::string &channel)
{
	if (AttachedTransformation())
		AttachedTransformation()->SetRetrievalChannel(channel);
}

size_t BufferedTransformation::ChannelPutWord16(const std::string &channel, word16 value, ByteOrder order, bool blocking)
{
	PutWord(false, order, m_buf, value);
	return ChannelPut(channel, m_buf, 2, blocking);
}

size_t BufferedTransformation::ChannelPutWord32(const std::string &channel, word32 value, ByteOrder order, bool blocking)
{
	PutWord(false, order, m_buf, value);
	return ChannelPut(channel, m_buf, 4, blocking);
}

size_t BufferedTransformation::ChannelPutWord64(const std::string &channel, word64 value, ByteOrder order, bool blocking)
{
	PutWord(false, order, m_buf, value);
	return ChannelPut(channel, m_buf, 8, blocking);
}

size_t BufferedTransformation::PutWord16(word16 value, ByteOrder order, bool blocking)
{
	return ChannelPutWord16(DEFAULT_CHANNEL, value, order, blocking);
}

size_t BufferedTransformation::PutWord32(word32 value, ByteOrder order, bool blocking)
{
	return ChannelPutWord32(DEFAULT_CHANNEL, value, order, blocking);
}

size_t BufferedTransformation::PutWord64(word64 value, ByteOrder order, bool blocking)
{
	return ChannelPutWord64(DEFAULT_CHANNEL, value, order, blocking);
}

size_t BufferedTransformation::PeekWord16(word16 &value, ByteOrder order) const
{
	byte buf[2] = {0, 0};
	size_t len = Peek(buf, 2);

	if (order == BIG_ENDIAN_ORDER)
		value = word16((buf[0] << 8) | buf[1]);
	else
		value = word16((buf[1] << 8) | buf[0]);

	return len;
}

size_t BufferedTransformation::PeekWord32(word32 &value, ByteOrder order) const
{
	byte buf[4] = {0, 0, 0, 0};
	size_t len = Peek(buf, 4);

	if (order == BIG_ENDIAN_ORDER)
		value = word32((buf[0] << 24) | (buf[1] << 16) |
		               (buf[2] << 8)  | (buf[3] << 0));
	else
		value = word32((buf[3] << 24) | (buf[2] << 16) |
		               (buf[1] << 8)  | (buf[0] << 0));

	return len;
}

size_t BufferedTransformation::PeekWord64(word64 &value, ByteOrder order) const
{
	byte buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	size_t len = Peek(buf, 8);

	if (order == BIG_ENDIAN_ORDER)
		value = ((word64)buf[0] << 56) | ((word64)buf[1] << 48) | ((word64)buf[2] << 40) |
		        ((word64)buf[3] << 32) | ((word64)buf[4] << 24) | ((word64)buf[5] << 16) |
		        ((word64)buf[6] << 8)  |  (word64)buf[7];
	else
		value = ((word64)buf[7] << 56) | ((word64)buf[6] << 48) | ((word64)buf[5] << 40) |
		        ((word64)buf[4] << 32) | ((word64)buf[3] << 24) | ((word64)buf[2] << 16) |
		        ((word64)buf[1] << 8)  |  (word64)buf[0];

	return len;
}

size_t BufferedTransformation::GetWord16(word16 &value, ByteOrder order)
{
	return (size_t)Skip(PeekWord16(value, order));
}

size_t BufferedTransformation::GetWord32(word32 &value, ByteOrder order)
{
	return (size_t)Skip(PeekWord32(value, order));
}

size_t BufferedTransformation::GetWord64(word64 &value, ByteOrder order)
{
	return (size_t)Skip(PeekWord64(value, order));
}

void BufferedTransformation::Attach(BufferedTransformation *newAttachment)
{
	if (AttachedTransformation() && AttachedTransformation()->Attachable())
		AttachedTransformation()->Attach(newAttachment);
	else
		Detach(newAttachment);
}

void GeneratableCryptoMaterial::GenerateRandomWithKeySize(RandomNumberGenerator &rng, unsigned int keySize)
{
	GenerateRandom(rng, MakeParameters("KeySize", (int)keySize));
}

class PK_DefaultEncryptionFilter : public Unflushable<Filter>
{
public:
	PK_DefaultEncryptionFilter(RandomNumberGenerator &rng, const PK_Encryptor &encryptor, BufferedTransformation *attachment, const NameValuePairs &parameters)
		: m_rng(rng), m_encryptor(encryptor), m_parameters(parameters)
	{
		Detach(attachment);
	}

	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking)
	{
		FILTER_BEGIN;
		m_plaintextQueue.Put(inString, length);

		if (messageEnd)
		{
			{
			size_t plaintextLength;
			if (!SafeConvert(m_plaintextQueue.CurrentSize(), plaintextLength))
				throw InvalidArgument("PK_DefaultEncryptionFilter: plaintext too long");
			size_t ciphertextLength = m_encryptor.CiphertextLength(plaintextLength);

			SecByteBlock plaintext(plaintextLength);
			m_plaintextQueue.Get(plaintext, plaintextLength);
			m_ciphertext.resize(ciphertextLength);
			m_encryptor.Encrypt(m_rng, plaintext, plaintextLength, m_ciphertext, m_parameters);
			}

			FILTER_OUTPUT(1, m_ciphertext, m_ciphertext.size(), messageEnd);
		}
		FILTER_END_NO_MESSAGE_END;
	}

	RandomNumberGenerator &m_rng;
	const PK_Encryptor &m_encryptor;
	const NameValuePairs &m_parameters;
	ByteQueue m_plaintextQueue;
	SecByteBlock m_ciphertext;
};

BufferedTransformation * PK_Encryptor::CreateEncryptionFilter(RandomNumberGenerator &rng, BufferedTransformation *attachment, const NameValuePairs &parameters) const
{
	return new PK_DefaultEncryptionFilter(rng, *this, attachment, parameters);
}

class PK_DefaultDecryptionFilter : public Unflushable<Filter>
{
public:
	PK_DefaultDecryptionFilter(RandomNumberGenerator &rng, const PK_Decryptor &decryptor, BufferedTransformation *attachment, const NameValuePairs &parameters)
		: m_rng(rng), m_decryptor(decryptor), m_parameters(parameters)
	{
		Detach(attachment);
	}

	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking)
	{
		FILTER_BEGIN;
		m_ciphertextQueue.Put(inString, length);

		if (messageEnd)
		{
			{
			size_t ciphertextLength;
			if (!SafeConvert(m_ciphertextQueue.CurrentSize(), ciphertextLength))
				throw InvalidArgument("PK_DefaultDecryptionFilter: ciphertext too long");
			size_t maxPlaintextLength = m_decryptor.MaxPlaintextLength(ciphertextLength);

			SecByteBlock ciphertext(ciphertextLength);
			m_ciphertextQueue.Get(ciphertext, ciphertextLength);
			m_plaintext.resize(maxPlaintextLength);
			m_result = m_decryptor.Decrypt(m_rng, ciphertext, ciphertextLength, m_plaintext, m_parameters);
			if (!m_result.isValidCoding)
				throw InvalidCiphertext(m_decryptor.AlgorithmName() + ": invalid ciphertext");
			}

			FILTER_OUTPUT(1, m_plaintext, m_result.messageLength, messageEnd);
		}
		FILTER_END_NO_MESSAGE_END;
	}

	RandomNumberGenerator &m_rng;
	const PK_Decryptor &m_decryptor;
	const NameValuePairs &m_parameters;
	ByteQueue m_ciphertextQueue;
	SecByteBlock m_plaintext;
	DecodingResult m_result;
};

BufferedTransformation * PK_Decryptor::CreateDecryptionFilter(RandomNumberGenerator &rng, BufferedTransformation *attachment, const NameValuePairs &parameters) const
{
	return new PK_DefaultDecryptionFilter(rng, *this, attachment, parameters);
}

size_t PK_Signer::Sign(RandomNumberGenerator &rng, PK_MessageAccumulator *messageAccumulator, byte *signature) const
{
	member_ptr<PK_MessageAccumulator> m(messageAccumulator);
	return SignAndRestart(rng, *m, signature, false);
}

size_t PK_Signer::SignMessage(RandomNumberGenerator &rng, const byte *message, size_t messageLen, byte *signature) const
{
	member_ptr<PK_MessageAccumulator> m(NewSignatureAccumulator(rng));
	m->Update(message, messageLen);
	return SignAndRestart(rng, *m, signature, false);
}

size_t PK_Signer::SignMessageWithRecovery(RandomNumberGenerator &rng, const byte *recoverableMessage, size_t recoverableMessageLength,
	const byte *nonrecoverableMessage, size_t nonrecoverableMessageLength, byte *signature) const
{
	member_ptr<PK_MessageAccumulator> m(NewSignatureAccumulator(rng));
	InputRecoverableMessage(*m, recoverableMessage, recoverableMessageLength);
	m->Update(nonrecoverableMessage, nonrecoverableMessageLength);
	return SignAndRestart(rng, *m, signature, false);
}

bool PK_Verifier::Verify(PK_MessageAccumulator *messageAccumulator) const
{
	member_ptr<PK_MessageAccumulator> m(messageAccumulator);
	return VerifyAndRestart(*m);
}

bool PK_Verifier::VerifyMessage(const byte *message, size_t messageLen, const byte *signature, size_t signatureLen) const
{
	member_ptr<PK_MessageAccumulator> m(NewVerificationAccumulator());
	InputSignature(*m, signature, signatureLen);
	m->Update(message, messageLen);
	return VerifyAndRestart(*m);
}

DecodingResult PK_Verifier::Recover(byte *recoveredMessage, PK_MessageAccumulator *messageAccumulator) const
{
	member_ptr<PK_MessageAccumulator> m(messageAccumulator);
	return RecoverAndRestart(recoveredMessage, *m);
}

DecodingResult PK_Verifier::RecoverMessage(byte *recoveredMessage,
	const byte *nonrecoverableMessage, size_t nonrecoverableMessageLength,
	const byte *signature, size_t signatureLength) const
{
	member_ptr<PK_MessageAccumulator> m(NewVerificationAccumulator());
	InputSignature(*m, signature, signatureLength);
	m->Update(nonrecoverableMessage, nonrecoverableMessageLength);
	return RecoverAndRestart(recoveredMessage, *m);
}

void SimpleKeyAgreementDomain::GenerateKeyPair(RandomNumberGenerator &rng, byte *privateKey, byte *publicKey) const
{
	GeneratePrivateKey(rng, privateKey);
	GeneratePublicKey(rng, privateKey, publicKey);
}

void AuthenticatedKeyAgreementDomain::GenerateStaticKeyPair(RandomNumberGenerator &rng, byte *privateKey, byte *publicKey) const
{
	GenerateStaticPrivateKey(rng, privateKey);
	GenerateStaticPublicKey(rng, privateKey, publicKey);
}

void AuthenticatedKeyAgreementDomain::GenerateEphemeralKeyPair(RandomNumberGenerator &rng, byte *privateKey, byte *publicKey) const
{
	GenerateEphemeralPrivateKey(rng, privateKey);
	GenerateEphemeralPublicKey(rng, privateKey, publicKey);
}

// Allow a distro or packager to override the build-time version
//  http://github.com/weidai11/cryptopp/issues/371
#ifndef CRYPTOPP_BUILD_VERSION
# define CRYPTOPP_BUILD_VERSION CRYPTOPP_VERSION
#endif
int LibraryVersion(CRYPTOPP_NOINLINE_DOTDOTDOT)
{
	return CRYPTOPP_BUILD_VERSION;
}

class NullNameValuePairs : public NameValuePairs
{
public:
	NullNameValuePairs() {}    //  Clang complains a default ctor must be avilable
	bool GetVoidValue(const char *name, const std::type_info &valueType, void *pValue) const
		{CRYPTOPP_UNUSED(name); CRYPTOPP_UNUSED(valueType); CRYPTOPP_UNUSED(pValue); return false;}
};

#if HAVE_GCC_INIT_PRIORITY
  const std::string DEFAULT_CHANNEL __attribute__ ((init_priority (CRYPTOPP_INIT_PRIORITY + 25))) = "";
  const std::string AAD_CHANNEL __attribute__ ((init_priority (CRYPTOPP_INIT_PRIORITY + 26))) = "AAD";
  const NullNameValuePairs s_nullNameValuePairs __attribute__ ((init_priority (CRYPTOPP_INIT_PRIORITY + 27)));
  const NameValuePairs& g_nullNameValuePairs = s_nullNameValuePairs;
#elif HAVE_MSC_INIT_PRIORITY
  #pragma warning(disable: 4073)
  #pragma init_seg(lib)
  const std::string DEFAULT_CHANNEL = "";
  const std::string AAD_CHANNEL = "AAD";
  const NullNameValuePairs s_nullNameValuePairs;
  const NameValuePairs& g_nullNameValuePairs = s_nullNameValuePairs;
  #pragma warning(default: 4073)
#elif HAVE_XLC_INIT_PRIORITY
  #pragma priority(260)
  const std::string DEFAULT_CHANNEL = "";
  const std::string AAD_CHANNEL = "AAD";
  const NullNameValuePairs s_nullNameValuePairs;
  const NameValuePairs& g_nullNameValuePairs = s_nullNameValuePairs;
#else
  const std::string DEFAULT_CHANNEL = "";
  const std::string AAD_CHANNEL = "AAD";
  const simple_ptr<NullNameValuePairs> s_pNullNameValuePairs(new NullNameValuePairs);
  const NameValuePairs &g_nullNameValuePairs = *s_pNullNameValuePairs.m_p;
#endif

NAMESPACE_END  // CryptoPP

#endif  // CRYPTOPP_IMPORTS
