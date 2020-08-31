// asn.h - originally written and placed in the public domain by Wei Dai

/// \file asn.h
/// \brief Classes and functions for working with ANS.1 objects

#ifndef CRYPTOPP_ASN_H
#define CRYPTOPP_ASN_H

#include "cryptlib.h"
#include "filters.h"
#include "smartptr.h"
#include "stdcpp.h"
#include "queue.h"
#include "misc.h"

#include <iosfwd>

// Issue 340
#if CRYPTOPP_GCC_DIAGNOSTIC_AVAILABLE
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wconversion"
# pragma GCC diagnostic ignored "-Wsign-conversion"
#endif

NAMESPACE_BEGIN(CryptoPP)

/// \brief ASN.1 types
/// \note These tags are not complete
enum ASNTag
{
	/// \brief ASN.1 Boolean
	BOOLEAN 			= 0x01,
	/// \brief ASN.1 Integer
	INTEGER 			= 0x02,
	/// \brief ASN.1 Bit string
	BIT_STRING			= 0x03,
	/// \brief ASN.1 Octet string
	OCTET_STRING		= 0x04,
	/// \brief ASN.1 Null
	TAG_NULL			= 0x05,
	/// \brief ASN.1 Object identifier
	OBJECT_IDENTIFIER	= 0x06,
	/// \brief ASN.1 Object descriptor
	OBJECT_DESCRIPTOR	= 0x07,
	/// \brief ASN.1 External reference
	EXTERNAL			= 0x08,
	/// \brief ASN.1 Real integer
	REAL				= 0x09,
	/// \brief ASN.1 Enumerated value
	ENUMERATED			= 0x0a,
	/// \brief ASN.1 UTF-8 string
	UTF8_STRING			= 0x0c,
	/// \brief ASN.1 Sequence
	SEQUENCE			= 0x10,
	/// \brief ASN.1 Set
	SET 				= 0x11,
	/// \brief ASN.1 Numeric string
	NUMERIC_STRING		= 0x12,
	/// \brief ASN.1 Printable string
	PRINTABLE_STRING 	= 0x13,
	/// \brief ASN.1 T61 string
	T61_STRING			= 0x14,
	/// \brief ASN.1 Videotext string
	VIDEOTEXT_STRING 	= 0x15,
	/// \brief ASN.1 IA5 string
	IA5_STRING			= 0x16,
	/// \brief ASN.1 UTC time
	UTC_TIME 			= 0x17,
	/// \brief ASN.1 Generalized time
	GENERALIZED_TIME 	= 0x18,
	/// \brief ASN.1 Graphic string
	GRAPHIC_STRING		= 0x19,
	/// \brief ASN.1 Visible string
	VISIBLE_STRING		= 0x1a,
	/// \brief ASN.1 General string
	GENERAL_STRING		= 0x1b,
	/// \brief ASN.1 Universal string
	UNIVERSAL_STRING	= 0x1c,
	/// \brief ASN.1 BMP string
	BMP_STRING  		= 0x1e
};

/// \brief ASN.1 flags
/// \note These flags are not complete
enum ASNIdFlag
{
	/// \brief ASN.1 Universal class
	UNIVERSAL           = 0x00,
	// DATA           = 0x01,
	// HEADER           = 0x02,
	/// \brief ASN.1 Primitive flag
	PRIMITIVE           = 0x00,
	/// \brief ASN.1 Constructed flag
	CONSTRUCTED         = 0x20,
	/// \brief ASN.1 Application class
	APPLICATION         = 0x40,
	/// \brief ASN.1 Context specific class
	CONTEXT_SPECIFIC    = 0x80,
	/// \brief ASN.1 Private class
	PRIVATE             = 0xc0
};

/// \brief Raises a BERDecodeErr
inline void BERDecodeError() {throw BERDecodeErr();}

/// \brief Exception thrown when an unknown object identifier is encountered
class CRYPTOPP_DLL UnknownOID : public BERDecodeErr
{
public:
	/// \brief Construct an UnknownOID
	UnknownOID() : BERDecodeErr("BER decode error: unknown object identifier") {}
	/// \brief Construct an UnknownOID
	/// \param err error message to use for the execption
	UnknownOID(const char *err) : BERDecodeErr(err) {}
};

/// \brief DER encode a length
/// \param bt BufferedTransformation object for writing
/// \param length the size to encode
/// \returns the number of octets used for the encoding
CRYPTOPP_DLL size_t CRYPTOPP_API DERLengthEncode(BufferedTransformation &bt, lword length);

/// \brief BER decode a length
/// \param bt BufferedTransformation object for reading
/// \param length the decoded size
/// \returns true if the value was decoded
/// \throws BERDecodeError if the value fails to decode or is too large for size_t
/// \details BERLengthDecode() returns false if the encoding is indefinite length.
CRYPTOPP_DLL bool CRYPTOPP_API BERLengthDecode(BufferedTransformation &bt, size_t &length);

/// \brief DER encode NULL
/// \param bt BufferedTransformation object for writing
CRYPTOPP_DLL void CRYPTOPP_API DEREncodeNull(BufferedTransformation &bt);

/// \brief BER decode NULL
/// \param bt BufferedTransformation object for reading
CRYPTOPP_DLL void CRYPTOPP_API BERDecodeNull(BufferedTransformation &bt);

/// \brief DER encode octet string
/// \param bt BufferedTransformation object for writing
/// \param str the string to encode
/// \param strLen the length of the string
/// \returns the number of octets used for the encoding
CRYPTOPP_DLL size_t CRYPTOPP_API DEREncodeOctetString(BufferedTransformation &bt, const byte *str, size_t strLen);

/// \brief DER encode octet string
/// \param bt BufferedTransformation object for reading
/// \param str the string to encode
/// \returns the number of octets used for the encoding
CRYPTOPP_DLL size_t CRYPTOPP_API DEREncodeOctetString(BufferedTransformation &bt, const SecByteBlock &str);

/// \brief BER decode octet string
/// \param bt BufferedTransformation object for reading
/// \param str the decoded string
/// \returns the number of octets used for the encoding
CRYPTOPP_DLL size_t CRYPTOPP_API BERDecodeOctetString(BufferedTransformation &bt, SecByteBlock &str);

/// \brief BER decode octet string
/// \param bt BufferedTransformation object for reading
/// \param str the decoded string
/// \returns the number of octets used for the encoding
CRYPTOPP_DLL size_t CRYPTOPP_API BERDecodeOctetString(BufferedTransformation &bt, BufferedTransformation &str);

/// \brief DER encode text string
/// \param bt BufferedTransformation object for writing
/// \param str the string to encode
/// \param strLen the length of the string, in bytes
/// \param asnTag the ASN.1 identifier
/// \returns the number of octets used for the encoding
/// \details DEREncodeTextString() can be used for UTF8_STRING, PRINTABLE_STRING, and IA5_STRING
/// \since Crypto++ 8.3
CRYPTOPP_DLL size_t CRYPTOPP_API DEREncodeTextString(BufferedTransformation &bt, const byte* str, size_t strLen, byte asnTag);

/// \brief DER encode text string
/// \param bt BufferedTransformation object for writing
/// \param str the string to encode
/// \param asnTag the ASN.1 identifier
/// \returns the number of octets used for the encoding
/// \details DEREncodeTextString() can be used for UTF8_STRING, PRINTABLE_STRING, and IA5_STRING
/// \since Crypto++ 8.3
CRYPTOPP_DLL size_t CRYPTOPP_API DEREncodeTextString(BufferedTransformation &bt, const SecByteBlock &str, byte asnTag);

/// \brief DER encode text string
/// \param bt BufferedTransformation object for writing
/// \param str the string to encode
/// \param asnTag the ASN.1 identifier
/// \returns the number of octets used for the encoding
/// \details DEREncodeTextString() can be used for UTF8_STRING, PRINTABLE_STRING, and IA5_STRING
/// \since Crypto++ 6.0
CRYPTOPP_DLL size_t CRYPTOPP_API DEREncodeTextString(BufferedTransformation &bt, const std::string &str, byte asnTag);

/// \brief BER decode text string
/// \param bt BufferedTransformation object for reading
/// \param str the string to decode
/// \param asnTag the ASN.1 identifier
/// \details BERDecodeTextString() can be used for UTF8_STRING, PRINTABLE_STRING, and IA5_STRING
/// \since Crypto++ 8.3
CRYPTOPP_DLL size_t CRYPTOPP_API BERDecodeTextString(BufferedTransformation &bt, SecByteBlock &str, byte asnTag);

/// \brief BER decode text string
/// \param bt BufferedTransformation object for reading
/// \param str the string to decode
/// \param asnTag the ASN.1 identifier
/// \details BERDecodeTextString() can be used for UTF8_STRING, PRINTABLE_STRING, and IA5_STRING
/// \since Crypto++ 6.0
CRYPTOPP_DLL size_t CRYPTOPP_API BERDecodeTextString(BufferedTransformation &bt, std::string &str, byte asnTag);

/// \brief DER encode date
/// \param bt BufferedTransformation object for writing
/// \param str the date to encode
/// \param asnTag the ASN.1 identifier
/// \returns the number of octets used for the encoding
/// \details BERDecodeDate() can be used for UTC_TIME and GENERALIZED_TIME
/// \since Crypto++ 8.3
CRYPTOPP_DLL size_t CRYPTOPP_API DEREncodeDate(BufferedTransformation &bt, const SecByteBlock &str, byte asnTag);

/// \brief BER decode date
/// \param bt BufferedTransformation object for reading
/// \param str the date to decode
/// \param asnTag the ASN.1 identifier
/// \details BERDecodeDate() can be used for UTC_TIME and GENERALIZED_TIME
/// \since Crypto++ 8.3
CRYPTOPP_DLL size_t CRYPTOPP_API BERDecodeDate(BufferedTransformation &bt, SecByteBlock &str, byte asnTag);

/// \brief DER encode bit string
/// \param bt BufferedTransformation object for writing
/// \param str the string to encode
/// \param strLen the length of the string
/// \param unusedBits the number of unused bits
/// \returns the number of octets used for the encoding
/// \details The caller is responsible for shifting octets if unusedBits is
///  not 0. For example, to DER encode a web server X.509 key usage, the 101b
///  bit mask is often used (digitalSignature and keyEncipherment). In this
///  case <tt>str</tt> is one octet with a value=0xa0 and unusedBits=5. The
///  value 0xa0 is <tt>101b << 5</tt>.
CRYPTOPP_DLL size_t CRYPTOPP_API DEREncodeBitString(BufferedTransformation &bt, const byte *str, size_t strLen, unsigned int unusedBits=0);

/// \brief DER decode bit string
/// \param bt BufferedTransformation object for reading
/// \param str the decoded string
/// \param unusedBits the number of unused bits
/// \details The caller is responsible for shifting octets if unusedBits is
///  not 0. For example, to DER encode a web server X.509 key usage, the 101b
///  bit mask is often used (digitalSignature and keyEncipherment). In this
///  case <tt>str</tt> is one octet with a value=0xa0 and unusedBits=5. The
///  value 0xa0 is <tt>101b << 5</tt>.
CRYPTOPP_DLL size_t CRYPTOPP_API BERDecodeBitString(BufferedTransformation &bt, SecByteBlock &str, unsigned int &unusedBits);

/// \brief BER decode and DER re-encode
/// \param bt BufferedTransformation object for writing
/// \param dest BufferedTransformation object
CRYPTOPP_DLL void CRYPTOPP_API DERReencode(BufferedTransformation &bt, BufferedTransformation &dest);

/// \brief BER decode size
/// \param bt BufferedTransformation object for reading
/// \returns the length of the ASN.1 value, in bytes
/// \details BERDecodePeekLength() determines the length of a value without
///  consuming octets in the stream. The stream must use definite length encoding.
///  If indefinite length encoding is used or an error occurs, then 0 is returned.
/// \since Crypto++ 8.3
CRYPTOPP_DLL size_t CRYPTOPP_API BERDecodePeekLength(const BufferedTransformation &bt);

/// \brief Object Identifier
class CRYPTOPP_DLL OID
{
public:
	virtual ~OID() {}

	/// \brief Construct an OID
	OID() {}

	/// \brief Construct an OID
	/// \param v value to initialize the OID
	OID(word32 v) : m_values(1, v) {}

	/// \brief Construct an OID
	/// \param bt BufferedTransformation object
	OID(BufferedTransformation &bt) {
		BERDecode(bt);
	}

	/// \brief Append a value to an OID
	/// \param rhs the value to append
	inline OID & operator+=(word32 rhs) {
		m_values.push_back(rhs); return *this;
	}

	/// \brief DER encode this OID
	/// \param bt BufferedTransformation object
	void DEREncode(BufferedTransformation &bt) const;

	/// \brief BER decode an OID
	/// \param bt BufferedTransformation object
	void BERDecode(BufferedTransformation &bt);

	/// \brief BER decode an OID
	/// \param bt BufferedTransformation object
	/// \throws BERDecodeErr() if decoded value doesn't match an expected OID
	/// \details BERDecodeAndCheck() can be used to parse an OID and verify it matches an expected.
	/// <pre>
	///   BERSequenceDecoder key(bt);
	///   ...
	///   BERSequenceDecoder algorithm(key);
	///   GetAlgorithmID().BERDecodeAndCheck(algorithm);
	/// </pre>
	void BERDecodeAndCheck(BufferedTransformation &bt) const;

	/// \brief Determine if OID is empty
	/// \returns true if OID has 0 elements, false otherwise
	/// \since Crypto++ 8.0
	bool Empty() const {
		return m_values.empty();
	}

	/// \brief Retrieve OID value array
	/// \returns OID value vector
	/// \since Crypto++ 8.0
	const std::vector<word32>& GetValues() const {
		return m_values;
	}

	/// \brief Print an OID
	/// \param out ostream object
	/// \returns ostream reference
	/// \details Print() writes the OID in a customary format, like
	///  1.2.840.113549.1.1.11. The caller is reposnsible to convert the
	///  OID to a friendly name, like sha256WithRSAEncryption.
	/// \since Crypto++ 8.3
	std::ostream& Print(std::ostream& out) const;

protected:
	friend bool operator==(const OID &lhs, const OID &rhs);
	friend bool operator!=(const OID &lhs, const OID &rhs);
	friend bool operator<(const OID &lhs, const OID &rhs);
	friend bool operator<=(const OID &lhs, const OID &rhs);
	friend bool operator>=(const OID &lhs, const OID &rhs);

	std::vector<word32> m_values;

private:
	static void EncodeValue(BufferedTransformation &bt, word32 v);
	static size_t DecodeValue(BufferedTransformation &bt, word32 &v);
};

/// \brief ASN.1 encoded object filter
class EncodedObjectFilter : public Filter
{
public:
	enum Flag {PUT_OBJECTS=1, PUT_MESSANGE_END_AFTER_EACH_OBJECT=2, PUT_MESSANGE_END_AFTER_ALL_OBJECTS=4, PUT_MESSANGE_SERIES_END_AFTER_ALL_OBJECTS=8};
	enum State {IDENTIFIER, LENGTH, BODY, TAIL, ALL_DONE} m_state;

	virtual ~EncodedObjectFilter() {}

	/// \brief Construct an EncodedObjectFilter
	/// \param attachment a BufferedTrasformation to attach to this object
	/// \param nObjects the number of objects
	/// \param flags bitwise OR of EncodedObjectFilter::Flag
	EncodedObjectFilter(BufferedTransformation *attachment = NULLPTR, unsigned int nObjects = 1, word32 flags = 0);

	/// \brief Input a byte buffer for processing
	/// \param inString the byte buffer to process
	/// \param length the size of the string, in bytes
	void Put(const byte *inString, size_t length);

	unsigned int GetNumberOfCompletedObjects() const {return m_nCurrentObject;}
	unsigned long GetPositionOfObject(unsigned int i) const {return m_positions[i];}

private:
	BufferedTransformation & CurrentTarget();

	ByteQueue m_queue;
	std::vector<unsigned int> m_positions;
	lword m_lengthRemaining;
	word32 m_nObjects, m_nCurrentObject, m_level, m_flags;
	byte m_id;
};

/// \brief BER General Decoder
class CRYPTOPP_DLL BERGeneralDecoder : public Store
{
public:
	/// \brief Default ASN.1 tag
	enum {DefaultTag = SEQUENCE | CONSTRUCTED};

	virtual ~BERGeneralDecoder();

	/// \brief Construct an ASN.1 decoder
	/// \param inQueue input byte queue
	/// \details BERGeneralDecoder uses DefaultTag
	explicit BERGeneralDecoder(BufferedTransformation &inQueue);

	/// \brief Construct an ASN.1 decoder
	/// \param inQueue input byte queue
	/// \param asnTag ASN.1 tag
	explicit BERGeneralDecoder(BufferedTransformation &inQueue, byte asnTag);

	/// \brief Construct an ASN.1 decoder
	/// \param inQueue input byte queue
	/// \param asnTag ASN.1 tag
	explicit BERGeneralDecoder(BERGeneralDecoder &inQueue, byte asnTag);

	/// \brief Determine length encoding
	/// \returns true if the ASN.1 object is definite length encoded, false otherwise
	bool IsDefiniteLength() const {
		return m_definiteLength;
	}

	/// \brief Determine remaining length
	/// \returns number of octets that remain to be consumed
	/// \details RemainingLength() is only valid if IsDefiniteLength()
	///  returns true.
	lword RemainingLength() const {
		CRYPTOPP_ASSERT(m_definiteLength);
		return IsDefiniteLength() ? m_length : 0;
	}

	/// \brief Determine end of stream
	/// \returns true if all octets have been consumed, false otherwise
	bool EndReached() const;

	/// \brief Determine next octet
	/// \returns next octet in the stream
	/// \details PeekByte does not consume the octet.
	/// \throws BERDecodeError if there are no octets remaining
	byte PeekByte() const;

	/// \brief Determine next octet
	/// \details CheckByte reads the next byte in the stream and verifies
	///  the octet matches b.
	/// \throws BERDecodeError if the next octet is not b
	void CheckByte(byte b);

	/// \brief Transfer bytes to another BufferedTransformation
	/// \param target the destination BufferedTransformation
	/// \param transferBytes the number of bytes to transfer
	/// \param channel the channel on which the transfer should occur
	/// \param blocking specifies whether the object should block when
	///  processing input
	/// \return the number of bytes that remain in the transfer block
	///  (i.e., bytes not transferred)
	/// \details TransferTo2() removes bytes and moves
	///  them to the destination. Transfer begins at the index position
	///  in the current stream, and not from an absolute position in the
	///  stream.
	/// \details transferBytes is an \a IN and \a OUT parameter. When
	///  the call is made, transferBytes is the requested size of the
	///  transfer. When the call returns, transferBytes is the number
	///  of bytes that were transferred.
	size_t TransferTo2(BufferedTransformation &target, lword &transferBytes, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true);

	/// \brief Copy bytes to another BufferedTransformation
	/// \param target the destination BufferedTransformation
	/// \param begin the 0-based index of the first byte to copy in
	///  the stream
	/// \param end the 0-based index of the last byte to copy in
	///  the stream
	/// \param channel the channel on which the transfer should occur
	/// \param blocking specifies whether the object should block when
	///  processing input
	/// \return the number of bytes that remain in the copy block
	///  (i.e., bytes not copied)
	/// \details CopyRangeTo2 copies bytes to the
	///  destination. The bytes are not removed from this object. Copying
	///  begins at the index position in the current stream, and not from
	///  an absolute position in the stream.
	/// \details begin is an \a IN and \a OUT parameter. When the call is
	///  made, begin is the starting position of the copy. When the call
	///  returns, begin is the position of the first byte that was \a not
	///  copied (which may be different than end). begin can be used for
	///  subsequent calls to CopyRangeTo2().
	size_t CopyRangeTo2(BufferedTransformation &target, lword &begin, lword end=LWORD_MAX, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true) const;

	/// \brief Signals the end of messages to the object
	/// \details Call this to denote end of sequence
	void MessageEnd();

protected:
	BufferedTransformation &m_inQueue;
	lword m_length;
	bool m_finished, m_definiteLength;

private:
	void Init(byte asnTag);
	void StoreInitialize(const NameValuePairs &parameters)
		{CRYPTOPP_UNUSED(parameters); CRYPTOPP_ASSERT(false);}
	lword ReduceLength(lword delta);
};

/// \brief DER General Encoder
class CRYPTOPP_DLL DERGeneralEncoder : public ByteQueue
{
public:
	/// \brief Default ASN.1 tag
	enum {DefaultTag = SEQUENCE | CONSTRUCTED};

	virtual ~DERGeneralEncoder();

	/// \brief Construct an ASN.1 encoder
	/// \param outQueue output byte queue
	/// \details DERGeneralEncoder uses DefaultTag
	explicit DERGeneralEncoder(BufferedTransformation &outQueue);

	/// \brief Construct an ASN.1 encoder
	/// \param outQueue output byte queue
	/// \param asnTag ASN.1 tag
	explicit DERGeneralEncoder(BufferedTransformation &outQueue, byte asnTag);

	/// \brief Construct an ASN.1 encoder
	/// \param outQueue output byte queue
	/// \param asnTag ASN.1 tag
	explicit DERGeneralEncoder(DERGeneralEncoder &outQueue, byte asnTag);

	/// \brief Signals the end of messages to the object
	/// \details Call this to denote end of sequence
	void MessageEnd();

private:
	BufferedTransformation &m_outQueue;
	byte m_asnTag;
	bool m_finished;
};

/// \brief BER Sequence Decoder
class CRYPTOPP_DLL BERSequenceDecoder : public BERGeneralDecoder
{
public:
	/// \brief Default ASN.1 tag
	enum {DefaultTag = SEQUENCE | CONSTRUCTED};

	/// \brief Construct an ASN.1 decoder
	/// \param inQueue input byte queue
	/// \details BERSequenceDecoder uses DefaultTag
	explicit BERSequenceDecoder(BufferedTransformation &inQueue)
		: BERGeneralDecoder(inQueue, DefaultTag) {}

	/// \brief Construct an ASN.1 decoder
	/// \param inQueue input byte queue
	/// \param asnTag ASN.1 tag
	explicit BERSequenceDecoder(BufferedTransformation &inQueue, byte asnTag)
		: BERGeneralDecoder(inQueue, asnTag) {}

	/// \brief Construct an ASN.1 decoder
	/// \param inQueue input byte queue
	/// \details BERSequenceDecoder uses DefaultTag
	explicit BERSequenceDecoder(BERSequenceDecoder &inQueue)
		: BERGeneralDecoder(inQueue, DefaultTag) {}

	/// \brief Construct an ASN.1 decoder
	/// \param inQueue input byte queue
	/// \param asnTag ASN.1 tag
	explicit BERSequenceDecoder(BERSequenceDecoder &inQueue, byte asnTag)
		: BERGeneralDecoder(inQueue, asnTag) {}
};

/// \brief DER Sequence Encoder
class CRYPTOPP_DLL DERSequenceEncoder : public DERGeneralEncoder
{
public:
	/// \brief Default ASN.1 tag
	enum {DefaultTag = SEQUENCE | CONSTRUCTED};

	/// \brief Construct an ASN.1 encoder
	/// \param outQueue output byte queue
	/// \details DERSequenceEncoder uses DefaultTag
	explicit DERSequenceEncoder(BufferedTransformation &outQueue)
		: DERGeneralEncoder(outQueue, DefaultTag) {}

	/// \brief Construct an ASN.1 encoder
	/// \param outQueue output byte queue
	/// \param asnTag ASN.1 tag
	explicit DERSequenceEncoder(BufferedTransformation &outQueue, byte asnTag)
		: DERGeneralEncoder(outQueue, asnTag) {}

	/// \brief Construct an ASN.1 encoder
	/// \param outQueue output byte queue
	/// \details DERSequenceEncoder uses DefaultTag
	explicit DERSequenceEncoder(DERSequenceEncoder &outQueue)
		: DERGeneralEncoder(outQueue, DefaultTag) {}

	/// \brief Construct an ASN.1 encoder
	/// \param outQueue output byte queue
	/// \param asnTag ASN.1 tag
	explicit DERSequenceEncoder(DERSequenceEncoder &outQueue, byte asnTag)
		: DERGeneralEncoder(outQueue, asnTag) {}
};

/// \brief BER Set Decoder
class CRYPTOPP_DLL BERSetDecoder : public BERGeneralDecoder
{
public:
	/// \brief Default ASN.1 tag
	enum {DefaultTag = SET | CONSTRUCTED};

	/// \brief Construct an ASN.1 decoder
	/// \param inQueue input byte queue
	/// \details BERSetDecoder uses DefaultTag
	explicit BERSetDecoder(BufferedTransformation &inQueue)
		: BERGeneralDecoder(inQueue, DefaultTag) {}

	/// \brief Construct an ASN.1 decoder
	/// \param inQueue input byte queue
	/// \param asnTag ASN.1 tag
	explicit BERSetDecoder(BufferedTransformation &inQueue, byte asnTag)
		: BERGeneralDecoder(inQueue, asnTag) {}

	/// \brief Construct an ASN.1 decoder
	/// \param inQueue input byte queue
	/// \details BERSetDecoder uses DefaultTag
	explicit BERSetDecoder(BERSetDecoder &inQueue)
		: BERGeneralDecoder(inQueue, DefaultTag) {}

	/// \brief Construct an ASN.1 decoder
	/// \param inQueue input byte queue
	/// \param asnTag ASN.1 tag
	explicit BERSetDecoder(BERSetDecoder &inQueue, byte asnTag)
		: BERGeneralDecoder(inQueue, asnTag) {}
};

/// \brief DER Set Encoder
class CRYPTOPP_DLL DERSetEncoder : public DERGeneralEncoder
{
public:
	/// \brief Default ASN.1 tag
	enum {DefaultTag = SET | CONSTRUCTED};

	/// \brief Construct an ASN.1 encoder
	/// \param outQueue output byte queue
	/// \details DERSetEncoder uses DefaultTag
	explicit DERSetEncoder(BufferedTransformation &outQueue)
		: DERGeneralEncoder(outQueue, DefaultTag) {}

	/// \brief Construct an ASN.1 encoder
	/// \param outQueue output byte queue
	/// \param asnTag ASN.1 tag
	explicit DERSetEncoder(BufferedTransformation &outQueue, byte asnTag)
		: DERGeneralEncoder(outQueue, asnTag) {}

	/// \brief Construct an ASN.1 encoder
	/// \param outQueue output byte queue
	/// \details DERSetEncoder uses DefaultTag
	explicit DERSetEncoder(DERSetEncoder &outQueue)
		: DERGeneralEncoder(outQueue, DefaultTag) {}

	/// \brief Construct an ASN.1 encoder
	/// \param outQueue output byte queue
	/// \param asnTag ASN.1 tag
	explicit DERSetEncoder(DERSetEncoder &outQueue, byte asnTag)
		: DERGeneralEncoder(outQueue, asnTag) {}
};

/// \brief Optional data encoder and decoder
/// \tparam T class or type
template <class T>
class ASNOptional : public member_ptr<T>
{
public:
	/// \brief BER decode optional data
	/// \param seqDecoder sequence with the optional ASN.1 data
	/// \param tag ASN.1 tag to match as optional data
	/// \param mask the mask to apply when matching the tag
	/// \sa ASNTag and ASNIdFlag
	void BERDecode(BERSequenceDecoder &seqDecoder, byte tag, byte mask = ~CONSTRUCTED)
	{
		byte b;
		if (seqDecoder.Peek(b) && (b & mask) == tag)
			reset(new T(seqDecoder));
	}

	/// \brief DER encode optional data
	/// \param out BufferedTransformation object
	void DEREncode(BufferedTransformation &out)
	{
		if (this->get() != NULLPTR)
			this->get()->DEREncode(out);
	}
};

/// \brief Encode and decode ASN.1 objects with additional information
/// \tparam BASE base class or type
/// \details Encodes and decodes public keys, private keys and group
///   parameters with OID identifying the algorithm or scheme.
template <class BASE>
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE ASN1CryptoMaterial : public ASN1Object, public BASE
{
public:
	/// \brief DER encode ASN.1 object
	/// \param bt BufferedTransformation object
	/// \details Save() will write the OID associated with algorithm or scheme.
	///   In the case of public and private keys, this function writes the
	///   subjectPubicKeyInfo and privateKeyInfo parts.
	void Save(BufferedTransformation &bt) const
		{BEREncode(bt);}

	/// \brief BER decode ASN.1 object
	/// \param bt BufferedTransformation object
	void Load(BufferedTransformation &bt)
		{BERDecode(bt);}
};

/// \brief Encodes and decodes subjectPublicKeyInfo
class CRYPTOPP_DLL X509PublicKey : public ASN1CryptoMaterial<PublicKey>
{
public:
	virtual ~X509PublicKey() {}

	void BERDecode(BufferedTransformation &bt);
	void DEREncode(BufferedTransformation &bt) const;

	/// \brief Retrieves the OID of the algorithm
	/// \returns OID of the algorithm
	virtual OID GetAlgorithmID() const =0;

	/// \brief Decode algorithm parameters
	/// \param bt BufferedTransformation object
	/// \sa BERDecodePublicKey, <A HREF="http://www.ietf.org/rfc/rfc2459.txt">RFC
	///  2459, section 7.3.1</A>
	virtual bool BERDecodeAlgorithmParameters(BufferedTransformation &bt)
		{BERDecodeNull(bt); return false;}

	/// \brief Encode algorithm parameters
	/// \param bt BufferedTransformation object
	/// \sa DEREncodePublicKey, <A HREF="http://www.ietf.org/rfc/rfc2459.txt">RFC
	///  2459, section 7.3.1</A>
	virtual bool DEREncodeAlgorithmParameters(BufferedTransformation &bt) const
		{DEREncodeNull(bt); return false;}

	/// \brief Decode subjectPublicKey part of subjectPublicKeyInfo
	/// \param bt BufferedTransformation object
	/// \param parametersPresent flag indicating if algorithm parameters are present
	/// \param size number of octets to read for the parameters, in bytes
	/// \details BERDecodePublicKey() the decodes subjectPublicKey part of
	///  subjectPublicKeyInfo, without the BIT STRING header.
	/// \details When <tt>parametersPresent = true</tt> then BERDecodePublicKey() calls
	///  BERDecodeAlgorithmParameters() to parse algorithm parameters.
	/// \sa BERDecodeAlgorithmParameters
	virtual void BERDecodePublicKey(BufferedTransformation &bt, bool parametersPresent, size_t size) =0;

	/// \brief Encode subjectPublicKey part of subjectPublicKeyInfo
	/// \param bt BufferedTransformation object
	/// \details DEREncodePublicKey() encodes the subjectPublicKey part of
	///  subjectPublicKeyInfo, without the BIT STRING header.
	/// \sa DEREncodeAlgorithmParameters
	virtual void DEREncodePublicKey(BufferedTransformation &bt) const =0;
};

/// \brief Encodes and Decodes privateKeyInfo
class CRYPTOPP_DLL PKCS8PrivateKey : public ASN1CryptoMaterial<PrivateKey>
{
public:
	virtual ~PKCS8PrivateKey() {}

	void BERDecode(BufferedTransformation &bt);
	void DEREncode(BufferedTransformation &bt) const;

	/// \brief Retrieves the OID of the algorithm
	/// \returns OID of the algorithm
	virtual OID GetAlgorithmID() const =0;

	/// \brief Decode optional parameters
	/// \param bt BufferedTransformation object
	/// \sa BERDecodePrivateKey, <A HREF="http://www.ietf.org/rfc/rfc2459.txt">RFC
	///  2459, section 7.3.1</A>
	virtual bool BERDecodeAlgorithmParameters(BufferedTransformation &bt)
		{BERDecodeNull(bt); return false;}

	/// \brief Encode optional parameters
	/// \param bt BufferedTransformation object
	/// \sa DEREncodePrivateKey, <A HREF="http://www.ietf.org/rfc/rfc2459.txt">RFC
	///  2459, section 7.3.1</A>
	virtual bool DEREncodeAlgorithmParameters(BufferedTransformation &bt) const
		{DEREncodeNull(bt); return false;}

	/// \brief Decode privateKey part of privateKeyInfo
	/// \param bt BufferedTransformation object
	/// \param parametersPresent flag indicating if algorithm parameters are present
	/// \param size number of octets to read for the parameters, in bytes
	/// \details BERDecodePrivateKey() the decodes privateKey part of privateKeyInfo,
	///  without the OCTET STRING header.
	/// \details When <tt>parametersPresent = true</tt> then BERDecodePrivateKey() calls
	///  BERDecodeAlgorithmParameters() to parse algorithm parameters.
	/// \sa BERDecodeAlgorithmParameters
	virtual void BERDecodePrivateKey(BufferedTransformation &bt, bool parametersPresent, size_t size) =0;

	/// \brief Encode privateKey part of privateKeyInfo
	/// \param bt BufferedTransformation object
	/// \details DEREncodePrivateKey() encodes the privateKey part of privateKeyInfo,
	///  without the OCTET STRING header.
	/// \sa DEREncodeAlgorithmParameters
	virtual void DEREncodePrivateKey(BufferedTransformation &bt) const =0;

	/// \brief Decode optional attributes
	/// \param bt BufferedTransformation object
	/// \details BERDecodeOptionalAttributes() decodes optional attributes including
	///  context-specific tag.
	/// \sa BERDecodeAlgorithmParameters, DEREncodeOptionalAttributes
	/// \note default implementation stores attributes to be output using
	///  DEREncodeOptionalAttributes
	virtual void BERDecodeOptionalAttributes(BufferedTransformation &bt);

	/// \brief Encode optional attributes
	/// \param bt BufferedTransformation object
	/// \details DEREncodeOptionalAttributes() encodes optional attributes including
	///  context-specific tag.
	/// \sa BERDecodeAlgorithmParameters
	virtual void DEREncodeOptionalAttributes(BufferedTransformation &bt) const;

protected:
	ByteQueue m_optionalAttributes;
};

// ********************************************************

/// \brief DER Encode unsigned value
/// \tparam T class or type
/// \param out BufferedTransformation object
/// \param w unsigned value to encode
/// \param asnTag the ASN.1 identifier
/// \details DEREncodeUnsigned() can be used with INTEGER, BOOLEAN, and ENUM
template <class T>
size_t DEREncodeUnsigned(BufferedTransformation &out, T w, byte asnTag = INTEGER)
{
	byte buf[sizeof(w)+1];
	unsigned int bc;
	if (asnTag == BOOLEAN)
	{
		buf[sizeof(w)] = w ? 0xff : 0;
		bc = 1;
	}
	else
	{
		buf[0] = 0;
		for (unsigned int i=0; i<sizeof(w); i++)
			buf[i+1] = byte(w >> (sizeof(w)-1-i)*8);
		bc = sizeof(w);
		while (bc > 1 && buf[sizeof(w)+1-bc] == 0)
			--bc;
		if (buf[sizeof(w)+1-bc] & 0x80)
			++bc;
	}
	out.Put(asnTag);
	size_t lengthBytes = DERLengthEncode(out, bc);
	out.Put(buf+sizeof(w)+1-bc, bc);
	return 1+lengthBytes+bc;
}

/// \brief BER Decode unsigned value
/// \tparam T fundamental C++ type
/// \param in BufferedTransformation object
/// \param w the decoded value
/// \param asnTag the ASN.1 identifier
/// \param minValue the minimum expected value
/// \param maxValue the maximum expected value
/// \throws BERDecodeErr() if the value cannot be parsed or the decoded value is not within range.
/// \details DEREncodeUnsigned() can be used with INTEGER, BOOLEAN, and ENUM
template <class T>
void BERDecodeUnsigned(BufferedTransformation &in, T &w, byte asnTag = INTEGER,
					   T minValue = 0, T maxValue = T(0xffffffff))
{
	byte b;
	if (!in.Get(b) || b != asnTag)
		BERDecodeError();

	size_t bc;
	bool definite = BERLengthDecode(in, bc);
	if (!definite)
		BERDecodeError();
	if (bc > in.MaxRetrievable())  // Issue 346
		BERDecodeError();
	if (asnTag == BOOLEAN && bc != 1) // X.690, 8.2.1
		BERDecodeError();
	if ((asnTag == INTEGER || asnTag == ENUMERATED) && bc == 0) // X.690, 8.3.1 and 8.4
		BERDecodeError();

	SecByteBlock buf(bc);

	if (bc != in.Get(buf, bc))
		BERDecodeError();

	// This consumes leading 0 octets. According to X.690, 8.3.2, it could be non-conforming behavior.
	//  X.690, 8.3.2 says "the bits of the first octet and bit 8 of the second octet ... (a) shall
	//  not all be ones and (b) shall not all be zeros ... These rules ensure that an integer value
	//  is always encoded in the smallest possible number of octet".
	// We invented AER (Alternate Encoding Rules), which is more relaxed than BER, CER, and DER.
	const byte *ptr = buf;
	while (bc > sizeof(w) && *ptr == 0)
	{
		bc--;
		ptr++;
	}
	if (bc > sizeof(w))
		BERDecodeError();

	w = 0;
	for (unsigned int i=0; i<bc; i++)
		w = (w << 8) | ptr[i];

	if (w < minValue || w > maxValue)
		BERDecodeError();
}

#ifdef CRYPTOPP_DOXYGEN_PROCESSING
/// \brief Compare two OIDs for equality
/// \param lhs the first OID
/// \param rhs the second OID
/// \returns true if the OIDs are equal, false otherwise
inline bool operator==(const OID &lhs, const OID &rhs);
/// \brief Compare two OIDs for inequality
/// \param lhs the first OID
/// \param rhs the second OID
/// \returns true if the OIDs are not equal, false otherwise
inline bool operator!=(const OID &lhs, const OID &rhs);
/// \brief Compare two OIDs for ordering
/// \param lhs the first OID
/// \param rhs the second OID
/// \returns true if the first OID is less than the second OID, false otherwise
/// \details operator<() calls std::lexicographical_compare() on each element in the array of values.
inline bool operator<(const OID &lhs, const OID &rhs);
/// \brief Compare two OIDs for ordering
/// \param lhs the first OID
/// \param rhs the second OID
/// \returns true if the first OID is less than or equal to the second OID, false otherwise
/// \details operator<=() is implemented in terms of operator==() and operator<().
/// \since Crypto++ 8.3
inline bool operator<=(const OID &lhs, const OID &rhs);
/// \brief Compare two OIDs for ordering
/// \param lhs the first OID
/// \param rhs the second OID
/// \returns true if the first OID is greater than or equal to the second OID, false otherwise
/// \details operator>=() is implemented in terms of operator<().
/// \since Crypto++ 8.3
inline bool operator>=(const OID &lhs, const OID &rhs);
/// \brief Append a value to an OID
/// \param lhs the OID
/// \param rhs the value to append
inline OID operator+(const OID &lhs, unsigned long rhs);
/// \brief Print a OID value
/// \param out the output stream
/// \param oid the OID
inline std::ostream& operator<<(std::ostream& out, const OID &oid)
	{ return oid.Print(out); }
#else
inline bool operator==(const ::CryptoPP::OID &lhs, const ::CryptoPP::OID &rhs)
	{return lhs.m_values == rhs.m_values;}
inline bool operator!=(const ::CryptoPP::OID &lhs, const ::CryptoPP::OID &rhs)
	{return lhs.m_values != rhs.m_values;}
inline bool operator<(const ::CryptoPP::OID &lhs, const ::CryptoPP::OID &rhs)
	{return std::lexicographical_compare(lhs.m_values.begin(), lhs.m_values.end(), rhs.m_values.begin(), rhs.m_values.end());}
inline bool operator<=(const ::CryptoPP::OID &lhs, const ::CryptoPP::OID &rhs)
	{return lhs<rhs || lhs==rhs;}
inline bool operator>=(const ::CryptoPP::OID &lhs, const ::CryptoPP::OID &rhs)
	{return ! (lhs<rhs);}
inline ::CryptoPP::OID operator+(const ::CryptoPP::OID &lhs, unsigned long rhs)
	{return ::CryptoPP::OID(lhs)+=rhs;}
inline std::ostream& operator<<(std::ostream& out, const OID &oid)
	{ return oid.Print(out); }
#endif

NAMESPACE_END

// Issue 340
#if CRYPTOPP_GCC_DIAGNOSTIC_AVAILABLE
# pragma GCC diagnostic pop
#endif

#endif
