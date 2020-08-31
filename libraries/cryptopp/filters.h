// filters.h - originally written and placed in the public domain by Wei Dai

/// \file filters.h
/// \brief Implementation of BufferedTransformation's attachment interface.

#ifndef CRYPTOPP_FILTERS_H
#define CRYPTOPP_FILTERS_H

#include "config.h"

#if CRYPTOPP_MSC_VERSION
# pragma warning(push)
# pragma warning(disable: 4127 4189 4231 4275 4514)
#endif

#include "cryptlib.h"
#include "simple.h"
#include "secblock.h"
#include "misc.h"
#include "smartptr.h"
#include "queue.h"
#include "algparam.h"
#include "stdcpp.h"

NAMESPACE_BEGIN(CryptoPP)

/// \brief Implementation of BufferedTransformation's attachment interface
/// \details Filter is a cornerstone of the Pipeline trinitiy. Data flows from
///  Sources, through Filters, and then terminates in Sinks. The difference
///  between a Source and Filter is a Source \a pumps data, while a Filter does
///  not. The difference between a Filter and a Sink is a Filter allows an
///  attached transformation, while a Sink does not.
/// \details See the discussion of BufferedTransformation in cryptlib.h for
///  more details.
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE Filter : public BufferedTransformation, public NotCopyable
{
public:
	virtual ~Filter() {}

	///	\name ATTACHMENT
	//@{

	/// \brief Construct a Filter
	/// \param attachment an optional attached transformation
	/// \details attachment can be NULL.
	Filter(BufferedTransformation *attachment = NULLPTR);

	/// \brief Determine if attachable
	/// \return true if the object allows attached transformations, false otherwise.
	/// \note Source and Filter offer attached transformations; while Sink does not.
	bool Attachable() {return true;}

	/// \brief Retrieve attached transformation
	/// \return pointer to a BufferedTransformation if there is an attached transformation, NULL otherwise.
	BufferedTransformation *AttachedTransformation();

	/// \brief Retrieve attached transformation
	/// \return pointer to a BufferedTransformation if there is an attached transformation, NULL otherwise.
	const BufferedTransformation *AttachedTransformation() const;

	/// \brief Replace an attached transformation
	/// \param newAttachment an optional attached transformation
	/// \details newAttachment can be a single filter, a chain of filters or NULL.
	///  Pass NULL to remove an existing BufferedTransformation or chain of filters
	void Detach(BufferedTransformation *newAttachment = NULLPTR);

	//@}

	///	\name RETRIEVAL OF ONE MESSAGE
	//@{

	// BufferedTransformation in cryptlib.h
	size_t TransferTo2(BufferedTransformation &target, lword &transferBytes, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true);
	size_t CopyRangeTo2(BufferedTransformation &target, lword &begin, lword end=LWORD_MAX, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true) const;

	//@}

	///	\name SIGNALS
	//@{

	// BufferedTransformation in cryptlib.h
	void Initialize(const NameValuePairs &parameters=g_nullNameValuePairs, int propagation=-1);
	bool Flush(bool hardFlush, int propagation=-1, bool blocking=true);
	bool MessageSeriesEnd(int propagation=-1, bool blocking=true);

	//@}

protected:
	virtual BufferedTransformation * NewDefaultAttachment() const;
	void Insert(Filter *nextFilter);	// insert filter after this one

	virtual bool ShouldPropagateMessageEnd() const {return true;}
	virtual bool ShouldPropagateMessageSeriesEnd() const {return true;}

	void PropagateInitialize(const NameValuePairs &parameters, int propagation);

	/// \brief Forward processed data on to attached transformation
	/// \param outputSite unknown, system crash between keyboard and chair...
	/// \param inString the byte buffer to process
	/// \param length the size of the string, in bytes
	/// \param messageEnd means how many filters to signal MessageEnd() to, including this one
	/// \param blocking specifies whether the object should block when processing input
	/// \param channel the channel to process the data
	/// \return the number of bytes that remain to be processed (i.e., bytes not processed).
	///  0 indicates all bytes were processed.
	size_t Output(int outputSite, const byte *inString, size_t length, int messageEnd, bool blocking, const std::string &channel=DEFAULT_CHANNEL);

	/// \brief Output multiple bytes that may be modified by callee.
	/// \param outputSite unknown, system crash between keyboard and chair...
	/// \param inString the byte buffer to process
	/// \param length the size of the string, in bytes
	/// \param messageEnd means how many filters to signal MessageEnd() to, including this one
	/// \param blocking specifies whether the object should block when processing input
	/// \param channel the channel to process the data
	/// \return the number of bytes that remain to be processed (i.e., bytes not processed).
	///  0 indicates all bytes were processed.
	size_t OutputModifiable(int outputSite, byte *inString, size_t length, int messageEnd, bool blocking, const std::string &channel=DEFAULT_CHANNEL);

	/// \brief Signals the end of messages to the object
	/// \param outputSite unknown, system crash between keyboard and chair...
	/// \param propagation the number of attached transformations the  MessageEnd() signal should be passed
	/// \param blocking specifies whether the object should block when processing input
	/// \param channel the channel to process the data
	/// \return true is the MessageEnd signal was successful, false otherwise.
	/// \details propagation count includes this object. Setting  propagation to <tt>1</tt> means this
	///  object only. Setting propagation to <tt>-1</tt> means unlimited propagation.
	bool OutputMessageEnd(int outputSite, int propagation, bool blocking, const std::string &channel=DEFAULT_CHANNEL);

	/// \brief Flush buffered input and/or output, with signal propagation
	/// \param outputSite unknown, system crash between keyboard and chair...
	/// \param hardFlush is used to indicate whether all data should be flushed
	/// \param propagation the number of attached transformations the  Flush() signal should be passed
	/// \param blocking specifies whether the object should block when processing input
	/// \param channel the channel to process the data
	/// \return true is the Flush signal was successful, false otherwise.
	/// \details propagation count includes this object. Setting  propagation to <tt>1</tt> means this
	///  object only. Setting  propagation to <tt>-1</tt> means unlimited propagation.
	/// \note Hard flushes must be used with care. It means try to process and output everything, even if
	///  there may not be enough data to complete the action. For example, hard flushing a  HexDecoder
	///  would cause an error if you do it after inputing an odd number of hex encoded characters.
	/// \note For some types of filters, like  ZlibDecompressor, hard flushes can only
	///  be done at "synchronization points". These synchronization points are positions in the data
	///  stream that are created by hard flushes on the corresponding reverse filters, in this
	///  example ZlibCompressor. This is useful when zlib compressed data is moved across a
	///  network in packets and compression state is preserved across packets, as in the SSH2 protocol.
	bool OutputFlush(int outputSite, bool hardFlush, int propagation, bool blocking, const std::string &channel=DEFAULT_CHANNEL);

	/// \brief Marks the end of a series of messages, with signal propagation
	/// \param outputSite unknown, system crash between keyboard and chair...
	/// \param propagation the number of attached transformations the  MessageSeriesEnd() signal should be passed
	/// \param blocking specifies whether the object should block when processing input
	/// \param channel the channel to process the data
	/// \return true is the MessageEnd signal was successful, false otherwise.
	/// \details Each object that receives the signal will perform its processing, decrement
	///  propagation, and then pass the signal on to attached transformations if the value is not 0.
	/// \details propagation count includes this object. Setting  propagation to <tt>1</tt> means this
	///  object only. Setting  propagation to <tt>-1</tt> means unlimited propagation.
	/// \note There should be a MessageEnd() immediately before MessageSeriesEnd().
	bool OutputMessageSeriesEnd(int outputSite, int propagation, bool blocking, const std::string &channel=DEFAULT_CHANNEL);

private:
	member_ptr<BufferedTransformation> m_attachment;

protected:
	size_t m_inputPosition;
	int m_continueAt;
};

/// \brief Create a working space in a BufferedTransformation
struct CRYPTOPP_DLL FilterPutSpaceHelper
{
	virtual ~FilterPutSpaceHelper() {}

	/// \brief Create a working space in a BufferedTransformation
	/// \param target BufferedTransformation for the working space
	/// \param channel channel for the working space
	/// \param minSize minimum size of the allocation, in bytes
	/// \param desiredSize preferred size of the allocation, in bytes
	/// \param bufferSize actual size of the allocation, in bytes
	/// \pre <tt>desiredSize >= minSize</tt> and <tt>bufferSize >= minSize</tt>.
	/// \details bufferSize is an IN and OUT parameter. If HelpCreatePutSpace() returns a non-NULL value, then
	///  bufferSize is valid and provides the size of the working space created for the caller.
	/// \details Internally, HelpCreatePutSpace() calls \ref BufferedTransformation::ChannelCreatePutSpace
	///  "ChannelCreatePutSpace()" using desiredSize. If the target returns desiredSize with a size less
	///  than minSize (i.e., the request could not be fulfilled), then an internal SecByteBlock
	///  called m_tempSpace is resized and used for the caller.
	byte *HelpCreatePutSpace(BufferedTransformation &target, const std::string &channel, size_t minSize, size_t desiredSize, size_t &bufferSize)
	{
		CRYPTOPP_ASSERT(desiredSize >= minSize && bufferSize >= minSize);
		if (m_tempSpace.size() < minSize)
		{
			byte *result = target.ChannelCreatePutSpace(channel, desiredSize);
			if (desiredSize >= minSize)
			{
				bufferSize = desiredSize;
				return result;
			}
			m_tempSpace.New(bufferSize);
		}

		bufferSize = m_tempSpace.size();
		return m_tempSpace.begin();
	}

	/// \brief Create a working space in a BufferedTransformation
	/// \param target the BufferedTransformation for the working space
	/// \param channel channel for the working space
	/// \param minSize minimum size of the allocation, in bytes
	/// \return pointer to the created space
	/// \details Internally, the overload calls HelpCreatePutSpace() using minSize for missing arguments.
	/// \details The filter will delete the space. The caller does not need to delete the space.
	byte *HelpCreatePutSpace(BufferedTransformation &target, const std::string &channel, size_t minSize)
		{return HelpCreatePutSpace(target, channel, minSize, minSize, minSize);}

	/// \brief Create a working space in a BufferedTransformation
	/// \param target the BufferedTransformation for the working space
	/// \param channel channel for the working space
	/// \param minSize minimum size of the allocation, in bytes
	/// \param bufferSize the actual size of the allocation, in bytes
	/// \details Internally, the overload calls HelpCreatePutSpace() using minSize for missing arguments.
	/// \details The filter will delete the space. The caller does not need to delete the space.
	byte *HelpCreatePutSpace(BufferedTransformation &target, const std::string &channel, size_t minSize, size_t bufferSize)
		{return HelpCreatePutSpace(target, channel, minSize, minSize, bufferSize);}

	/// \brief Temporay working space
	SecByteBlock m_tempSpace;
};

/// \brief Measure how many bytes and messages pass through the filter
/// \details measure how many bytes and messages pass through the filter. The filter also serves as valve by
///  maintaining a list of ranges to skip during processing.
class CRYPTOPP_DLL MeterFilter : public Bufferless<Filter>
{
public:
	virtual ~MeterFilter() {}

	/// \brief Construct a MeterFilter
	/// \param attachment an optional attached transformation
	/// \param transparent flag indicating if the filter should function transparently
	/// \details attachment can be NULL. The filter is transparent by default. If the filter is
	///  transparent, then PutMaybeModifiable() does not process a request and always returns 0.
	MeterFilter(BufferedTransformation *attachment=NULLPTR, bool transparent=true)
		: m_transparent(transparent), m_currentMessageBytes(0), m_totalBytes(0)
		, m_currentSeriesMessages(0), m_totalMessages(0), m_totalMessageSeries(0)
		, m_begin(NULLPTR), m_length(0) {Detach(attachment); ResetMeter();}

	/// \brief Set or change the transparent mode of this object
	/// \param transparent the new transparent mode
	void SetTransparent(bool transparent) {m_transparent = transparent;}

	/// \brief Adds a range to skip during processing
	/// \param message the message to apply the range
	/// \param position the 0-based index in the current stream
	/// \param size the length of the range
	/// \param sortNow flag indicating whether the range should be sorted
	/// \details Internally, MeterFilter maitains a deque of ranges to skip. As messages are processed,
	///  ranges of bytes are skipped according to the list of ranges.
	void AddRangeToSkip(unsigned int message, lword position, lword size, bool sortNow = true);

	/// \brief Resets the meter
	/// \details ResetMeter() reinitializes the meter by setting counters to 0 and removing previous
	///  skip ranges.
	void ResetMeter();

	// BufferedTransformation in cryptlib.h
	void IsolatedInitialize(const NameValuePairs &parameters)
		{CRYPTOPP_UNUSED(parameters); ResetMeter();}

	/// \brief Number of bytes in the current message
	/// \return GetCurrentMessageBytes() returns the number of bytes in the current message
	lword GetCurrentMessageBytes() const {return m_currentMessageBytes;}

	/// \brief Number of bytes processed by the filter
	/// \return GetTotalBytes() returns the number of bytes processed by the filter
	lword GetTotalBytes() const {return m_totalBytes;}

	/// \brief Message number in the series
	/// \return GetCurrentSeriesMessages() returns the message number in the series
	unsigned int GetCurrentSeriesMessages() const {return m_currentSeriesMessages;}

	/// \brief Number of messages in the message series
	/// \return GetTotalMessages() returns the number of messages in the message series
	unsigned int GetTotalMessages() const {return m_totalMessages;}

	/// \brief Number of messages processed by the filter
	/// \return GetTotalMessageSeries() returns the number of messages processed by the filter
	unsigned int GetTotalMessageSeries() const {return m_totalMessageSeries;}

	// BufferedTransformation in cryptlib.h
	byte * CreatePutSpace(size_t &size) {return AttachedTransformation()->CreatePutSpace(size);}
	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking);
	size_t PutModifiable2(byte *inString, size_t length, int messageEnd, bool blocking);
	bool IsolatedMessageSeriesEnd(bool blocking);

private:
	size_t PutMaybeModifiable(byte *inString, size_t length, int messageEnd, bool blocking, bool modifiable);
	bool ShouldPropagateMessageEnd() const {return m_transparent;}
	bool ShouldPropagateMessageSeriesEnd() const {return m_transparent;}

	struct MessageRange
	{
		inline bool operator<(const MessageRange &b) const	// BCB2006 workaround: this has to be a member function
			{return message < b.message || (message == b.message && position < b.position);}
		unsigned int message; lword position; lword size;
	};

	bool m_transparent;
	lword m_currentMessageBytes, m_totalBytes;
	unsigned int m_currentSeriesMessages, m_totalMessages, m_totalMessageSeries;
	std::deque<MessageRange> m_rangesToSkip;
	byte *m_begin;
	size_t m_length;
};

/// \brief A transparent MeterFilter
/// \sa MeterFilter, OpaqueFilter
class CRYPTOPP_DLL TransparentFilter : public MeterFilter
{
public:
	/// \brief Construct a TransparentFilter
	/// \param attachment an optional attached transformation
	TransparentFilter(BufferedTransformation *attachment=NULLPTR) : MeterFilter(attachment, true) {}
};

/// \brief A non-transparent MeterFilter
/// \sa MeterFilter, TransparentFilter
class CRYPTOPP_DLL OpaqueFilter : public MeterFilter
{
public:
	/// \brief Construct an OpaqueFilter
	/// \param attachment an optional attached transformation
	OpaqueFilter(BufferedTransformation *attachment=NULLPTR) : MeterFilter(attachment, false) {}
};

/// \brief Divides an input stream into discrete blocks
/// \details FilterWithBufferedInput divides the input stream into a first block, a number of
///  middle blocks, and a last block. First and last blocks are optional, and middle blocks may
///  be a stream instead (i.e. <tt>blockSize == 1</tt>).
/// \sa AuthenticatedEncryptionFilter, AuthenticatedDecryptionFilter, HashVerificationFilter,
///  SignatureVerificationFilter, StreamTransformationFilter
class CRYPTOPP_DLL FilterWithBufferedInput : public Filter
{
public:
	virtual ~FilterWithBufferedInput() {}

	/// \brief Construct a FilterWithBufferedInput with an attached transformation
	/// \param attachment an attached transformation
	FilterWithBufferedInput(BufferedTransformation *attachment);

	/// \brief Construct a FilterWithBufferedInput with an attached transformation
	/// \param firstSize the size of the first block
	/// \param blockSize the size of middle blocks
	/// \param lastSize the size of the last block
	/// \param attachment an attached transformation
	/// \details firstSize and lastSize may be 0. blockSize must be at least 1.
	FilterWithBufferedInput(size_t firstSize, size_t blockSize, size_t lastSize, BufferedTransformation *attachment);

	void IsolatedInitialize(const NameValuePairs &parameters);
	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking)
	{
		return PutMaybeModifiable(const_cast<byte *>(inString), length, messageEnd, blocking, false);
	}

	size_t PutModifiable2(byte *inString, size_t length, int messageEnd, bool blocking)
	{
		return PutMaybeModifiable(inString, length, messageEnd, blocking, true);
	}

	/// \brief Flushes data buffered by this object, without signal propagation
	/// \param hardFlush indicates whether all data should be flushed
	/// \param blocking specifies whether the object should block when processing input
	/// \return true if the Flush was successful, false otherwise
	/// \details IsolatedFlush() calls ForceNextPut() if hardFlush is true
	/// \note  hardFlush must be used with care
	bool IsolatedFlush(bool hardFlush, bool blocking);

	/// \brief Flushes data buffered by this object
	/// \details The input buffer may contain more than blockSize bytes if <tt>lastSize != 0</tt>.
	///  ForceNextPut() forces a call to NextPut() if this is the case.
	void ForceNextPut();

protected:
	virtual bool DidFirstPut() const {return m_firstInputDone;}
	virtual size_t GetFirstPutSize() const {return m_firstSize;}
	virtual size_t GetBlockPutSize() const {return m_blockSize;}
	virtual size_t GetLastPutSize() const {return m_lastSize;}

	virtual void InitializeDerivedAndReturnNewSizes(const NameValuePairs &parameters, size_t &firstSize, size_t &blockSize, size_t &lastSize)
		{CRYPTOPP_UNUSED(parameters); CRYPTOPP_UNUSED(firstSize); CRYPTOPP_UNUSED(blockSize); CRYPTOPP_UNUSED(lastSize); InitializeDerived(parameters);}
	virtual void InitializeDerived(const NameValuePairs &parameters)
		{CRYPTOPP_UNUSED(parameters);}
	// FirstPut() is called if (firstSize != 0 and totalLength >= firstSize)
	// or (firstSize == 0 and (totalLength > 0 or a MessageEnd() is received)).
	// inString is m_firstSize in length.
	virtual void FirstPut(const byte *inString) =0;
	// NextPut() is called if totalLength >= firstSize+blockSize+lastSize
	virtual void NextPutSingle(const byte *inString)
		{CRYPTOPP_UNUSED(inString); CRYPTOPP_ASSERT(false);}
	// Same as NextPut() except length can be a multiple of blockSize
	// Either NextPut() or NextPutMultiple() must be overridden
	virtual void NextPutMultiple(const byte *inString, size_t length);
	// Same as NextPutMultiple(), but inString can be modified
	virtual void NextPutModifiable(byte *inString, size_t length)
		{NextPutMultiple(inString, length);}
	/// \brief Input the last block of data
	/// \param inString the input byte buffer
	/// \param length the size of the input buffer, in bytes
	/// \details LastPut() processes the last block of data and signals attached filters to do the same.
	///  LastPut() is always called. The pseudo algorithm for the logic is:
	/// <pre>
	///     if totalLength < firstSize then length == totalLength
	///     else if totalLength <= firstSize+lastSize then length == totalLength-firstSize
	///     else lastSize <= length < lastSize+blockSize
	/// </pre>
	virtual void LastPut(const byte *inString, size_t length) =0;
	virtual void FlushDerived() {}

protected:
	size_t PutMaybeModifiable(byte *begin, size_t length, int messageEnd, bool blocking, bool modifiable);
	void NextPutMaybeModifiable(byte *inString, size_t length, bool modifiable)
	{
		if (modifiable) NextPutModifiable(inString, length);
		else NextPutMultiple(inString, length);
	}

	// This function should no longer be used, put this here to cause a compiler error
	// if someone tries to override NextPut().
	virtual int NextPut(const byte *inString, size_t length)
		{CRYPTOPP_UNUSED(inString); CRYPTOPP_UNUSED(length); CRYPTOPP_ASSERT(false); return 0;}

	class BlockQueue
	{
	public:
		void ResetQueue(size_t blockSize, size_t maxBlocks);
		byte *GetBlock();
		byte *GetContigousBlocks(size_t &numberOfBytes);
		size_t GetAll(byte *outString);
		void Put(const byte *inString, size_t length);
		size_t CurrentSize() const {return m_size;}
		size_t MaxSize() const {return m_buffer.size();}

	private:
		SecByteBlock m_buffer;
		size_t m_blockSize, m_maxBlocks, m_size;
		byte *m_begin;
	};

	size_t m_firstSize, m_blockSize, m_lastSize;
	bool m_firstInputDone;
	BlockQueue m_queue;
};

/// \brief A filter that buffers input using a ByteQueue
/// \details FilterWithInputQueue will buffer input using a ByteQueue. When the filter receives
///  a \ref BufferedTransformation::MessageEnd() "MessageEnd()" signal it will pass the data
///  on to its attached transformation.
class CRYPTOPP_DLL FilterWithInputQueue : public Filter
{
public:
	virtual ~FilterWithInputQueue() {}

	/// \brief Construct a FilterWithInputQueue
	/// \param attachment an optional attached transformation
	FilterWithInputQueue(BufferedTransformation *attachment=NULLPTR) : Filter(attachment) {}

	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking)
	{
		if (!blocking)
			throw BlockingInputOnly("FilterWithInputQueue");

		m_inQueue.Put(inString, length);
		if (messageEnd)
		{
			IsolatedMessageEnd(blocking);
			Output(0, NULLPTR, 0, messageEnd, blocking);
		}
		return 0;
	}

protected:
	virtual bool IsolatedMessageEnd(bool blocking) =0;
	void IsolatedInitialize(const NameValuePairs &parameters)
		{CRYPTOPP_UNUSED(parameters); m_inQueue.Clear();}

	ByteQueue m_inQueue;
};

/// \struct BlockPaddingSchemeDef
/// \brief Padding schemes used for block ciphers
/// \since Crypto++ 5.0
struct BlockPaddingSchemeDef
{
	/// \enum BlockPaddingScheme
	/// \brief Padding schemes used for block ciphers.
	/// \details DEFAULT_PADDING means PKCS_PADDING if <tt>cipher.MandatoryBlockSize() > 1 &&
	///  cipher.MinLastBlockSize() == 0</tt>, which holds for ECB or CBC mode. Otherwise,
	///  NO_PADDING for modes like OFB, CFB, CTR, CBC-CTS.
	/// \sa <A HREF="http://www.weidai.com/scan-mirror/csp.html">Block Cipher Padding</A> for
	///  additional details.
	/// \since Crypto++ 5.0
	enum BlockPaddingScheme {
		/// \brief No padding added to a block
		/// \since Crypto++ 5.0
		NO_PADDING,
		/// \brief 0's padding added to a block
		/// \since Crypto++ 5.0
		ZEROS_PADDING,
		/// \brief PKCS padding added to a block
		/// \since Crypto++ 5.0
		PKCS_PADDING,
		/// \brief 1 and 0's padding added to a block
		/// \since Crypto++ 5.0
		ONE_AND_ZEROS_PADDING,
		/// \brief W3C padding added to a block
		/// \sa <A HREF="http://www.w3.org/TR/2002/REC-xmlenc-core-20021210/Overview.html">XML
		///  Encryption Syntax and Processing</A>
		/// \since Crypto++ 6.0
		W3C_PADDING,
		/// \brief Default padding scheme
		/// \since Crypto++ 5.0
		DEFAULT_PADDING
	};
};

/// \brief Filter wrapper for StreamTransformation
/// \details StreamTransformationFilter() is a filter wrapper for StreamTransformation(). It is used when
///  pipelining data for stream ciphers and confidentiality-only block ciphers. The filter will optionally
///  handle padding and unpadding when needed. If you are using an authenticated encryption mode of operation,
///  then use AuthenticatedEncryptionFilter() and AuthenticatedDecryptionFilter()
/// \since Crypto++ 5.0
class CRYPTOPP_DLL StreamTransformationFilter : public FilterWithBufferedInput, public BlockPaddingSchemeDef, private FilterPutSpaceHelper
{
public:
	virtual ~StreamTransformationFilter() {}

	/// \brief Construct a StreamTransformationFilter
	/// \param c reference to a StreamTransformation
	/// \param attachment an optional attached transformation
	/// \param padding the \ref BlockPaddingSchemeDef "padding scheme"
	/// \details This contructor creates a StreamTransformationFilter() for stream ciphers and
	///  confidentiality-only block cipher modes of operation. If you are using an authenticated
	///  encryption mode of operation, then use either AuthenticatedEncryptionFilter() or
	///  AuthenticatedDecryptionFilter().
	/// \sa AuthenticatedEncryptionFilter() and AuthenticatedDecryptionFilter()
	StreamTransformationFilter(StreamTransformation &c, BufferedTransformation *attachment = NULLPTR, BlockPaddingScheme padding = DEFAULT_PADDING);

	std::string AlgorithmName() const {return m_cipher.AlgorithmName();}

protected:

	friend class AuthenticatedEncryptionFilter;
	friend class AuthenticatedDecryptionFilter;

	/// \brief Construct a StreamTransformationFilter
	/// \param c reference to a StreamTransformation
	/// \param attachment an optional attached transformation
	/// \param padding the \ref BlockPaddingSchemeDef "padding scheme"
	/// \param authenticated flag indicating whether the filter should allow authenticated encryption schemes
	/// \details This constructor is used for authenticated encryption mode of operation and by
	///  AuthenticatedEncryptionFilter() and AuthenticatedDecryptionFilter().
	StreamTransformationFilter(StreamTransformation &c, BufferedTransformation *attachment, BlockPaddingScheme padding, bool authenticated);

	void InitializeDerivedAndReturnNewSizes(const NameValuePairs &parameters, size_t &firstSize, size_t &blockSize, size_t &lastSize);
	void FirstPut(const byte *inString);
	void NextPutMultiple(const byte *inString, size_t length);
	void NextPutModifiable(byte *inString, size_t length);
	void LastPut(const byte *inString, size_t length);

	static size_t LastBlockSize(StreamTransformation &c, BlockPaddingScheme padding);

	StreamTransformation &m_cipher;
	BlockPaddingScheme m_padding;
	unsigned int m_mandatoryBlockSize;
	unsigned int m_optimalBufferSize;
	unsigned int m_reservedBufferSize;
	bool m_isSpecial;
};

/// \brief Filter wrapper for HashTransformation
/// \since Crypto++ 1.0
class CRYPTOPP_DLL HashFilter : public Bufferless<Filter>, private FilterPutSpaceHelper
{
public:
	virtual ~HashFilter() {}

	/// \brief Construct a HashFilter
	/// \param hm reference to a HashTransformation
	/// \param attachment an optional attached transformation
	/// \param putMessage flag indicating whether the original message should be passed to an attached transformation
	/// \param truncatedDigestSize the size of the digest
	/// \param messagePutChannel the channel on which the message should be output
	/// \param hashPutChannel the channel on which the digest should be output
	HashFilter(HashTransformation &hm, BufferedTransformation *attachment = NULLPTR, bool putMessage=false, int truncatedDigestSize=-1, const std::string &messagePutChannel=DEFAULT_CHANNEL, const std::string &hashPutChannel=DEFAULT_CHANNEL);

	std::string AlgorithmName() const {return m_hashModule.AlgorithmName();}
	void IsolatedInitialize(const NameValuePairs &parameters);
	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking);
	byte * CreatePutSpace(size_t &size) {return m_hashModule.CreateUpdateSpace(size);}

private:
	HashTransformation &m_hashModule;
	bool m_putMessage;
	unsigned int m_digestSize;
	byte *m_space;
	std::string m_messagePutChannel, m_hashPutChannel;
};

/// \brief Filter wrapper for HashTransformation
/// \since Crypto++ 4.0
class CRYPTOPP_DLL HashVerificationFilter : public FilterWithBufferedInput
{
public:
	virtual ~HashVerificationFilter() {}

	/// \brief Exception thrown when a data integrity check failure is encountered
	class HashVerificationFailed : public Exception
	{
	public:
		HashVerificationFailed()
			: Exception(DATA_INTEGRITY_CHECK_FAILED, "HashVerificationFilter: message hash or MAC not valid") {}
	};

	/// \enum Flags
	/// \brief Flags controlling filter behavior.
	/// \details The flags are a bitmask and can be OR'd together.
	enum Flags {
		/// \brief The hash is at the end of the message (i.e., concatenation of message+hash)
		HASH_AT_END=0,
		/// \brief The hash is at the beginning of the message (i.e., concatenation of hash+message)
		HASH_AT_BEGIN=1,
		/// \brief The message should be passed to an attached transformation
		PUT_MESSAGE=2,
		/// \brief The hash should be passed to an attached transformation
		PUT_HASH=4,
		/// \brief The result of the verification should be passed to an attached transformation
		PUT_RESULT=8,
		/// \brief The filter should throw a HashVerificationFailed if a failure is encountered
		THROW_EXCEPTION=16,
		/// \brief Default flags using HASH_AT_BEGIN and PUT_RESULT
		DEFAULT_FLAGS = HASH_AT_BEGIN | PUT_RESULT
	};

	/// \brief Construct a HashVerificationFilter
	/// \param hm reference to a HashTransformation
	/// \param attachment an optional attached transformation
	/// \param flags flags indicating behaviors for the filter
	/// \param truncatedDigestSize the size of the digest
	/// \details <tt>truncatedDigestSize = -1</tt> indicates \ref HashTransformation::DigestSize() "DigestSize" should be used.
	HashVerificationFilter(HashTransformation &hm, BufferedTransformation *attachment = NULLPTR, word32 flags = DEFAULT_FLAGS, int truncatedDigestSize=-1);

	std::string AlgorithmName() const {return m_hashModule.AlgorithmName();}
	bool GetLastResult() const {return m_verified;}

protected:
	void InitializeDerivedAndReturnNewSizes(const NameValuePairs &parameters, size_t &firstSize, size_t &blockSize, size_t &lastSize);
	void FirstPut(const byte *inString);
	void NextPutMultiple(const byte *inString, size_t length);
	void LastPut(const byte *inString, size_t length);

private:
	friend class AuthenticatedDecryptionFilter;

	HashTransformation &m_hashModule;
	word32 m_flags;
	unsigned int m_digestSize;
	bool m_verified;
	SecByteBlock m_expectedHash;
};

/// \brief Filter wrapper for encrypting with AuthenticatedSymmetricCipher
/// \details AuthenticatedEncryptionFilter() is a wrapper for encrypting with
///  AuthenticatedSymmetricCipher(), optionally handling padding/unpadding when needed.
/// \details AuthenticatedDecryptionFilter() for Crypto++ 8.2 and earlier
///  had a bug where a FileSource() would cause an exception, but a StringSource()
///  was OK. Also see <A HREF=
///  "https://github.com/weidai11/cryptopp/issues/817">Issue 817</A> and <A HREF=
///  "https://github.com/weidai11/cryptopp/commit/ff110c6e183e">Commit ff110c6e183e</A>.
/// \sa AuthenticatedSymmetricCipher, AuthenticatedDecryptionFilter, EAX, CCM, GCM,
///  and <A HREF="https://www.cryptopp.com/wiki/AadSource">AadSource</A> on the
///  Crypto++ wiki.
/// \since Crypto++ 5.6.0
class CRYPTOPP_DLL AuthenticatedEncryptionFilter : public StreamTransformationFilter
{
public:
	virtual ~AuthenticatedEncryptionFilter() {}

	/// \brief Construct a AuthenticatedEncryptionFilter
	/// \param c reference to a AuthenticatedSymmetricCipher
	/// \param attachment an optional attached transformation
	/// \param putAAD flag indicating whether the AAD should be passed to an attached transformation
	/// \param truncatedDigestSize the size of the digest
	/// \param macChannel the channel on which the MAC should be output
	/// \param padding the \ref BlockPaddingSchemeDef "padding scheme"
	/// \details <tt>truncatedDigestSize = -1</tt> indicates \ref HashTransformation::DigestSize() "DigestSize" should be used.
	/// \since Crypto++ 5.6.0
	AuthenticatedEncryptionFilter(AuthenticatedSymmetricCipher &c, BufferedTransformation *attachment = NULLPTR, bool putAAD=false, int truncatedDigestSize=-1, const std::string &macChannel=DEFAULT_CHANNEL, BlockPaddingScheme padding = DEFAULT_PADDING);

	void IsolatedInitialize(const NameValuePairs &parameters);
	byte * ChannelCreatePutSpace(const std::string &channel, size_t &size);
	size_t ChannelPut2(const std::string &channel, const byte *begin, size_t length, int messageEnd, bool blocking);

	/// \brief Input the last block of data
	/// \param inString the input byte buffer
	/// \param length the size of the input buffer, in bytes
	/// \details LastPut() processes the last block of data and signals attached filters to do the same.
	///  LastPut() is always called. The pseudo algorithm for the logic is:
	/// <pre>
	///     if totalLength < firstSize then length == totalLength
	///     else if totalLength <= firstSize+lastSize then length == totalLength-firstSize
	///     else lastSize <= length < lastSize+blockSize
	/// </pre>
	void LastPut(const byte *inString, size_t length);

protected:
	HashFilter m_hf;
};

/// \brief Filter wrapper for decrypting with AuthenticatedSymmetricCipher
/// \details AuthenticatedDecryptionFilter() is a wrapper for decrypting with
///  AuthenticatedSymmetricCipher(), optionally handling padding/unpadding when
///  needed.
/// \details AuthenticatedDecryptionFilter() for Crypto++ 8.2 and earlier
///  had a bug where a FileSource() would cause an exception, but a StringSource()
///  was OK. Also see <A HREF=
///  "https://github.com/weidai11/cryptopp/issues/817">Issue 817</A> and <A HREF=
///  "https://github.com/weidai11/cryptopp/commit/ff110c6e183e">Commit ff110c6e183e</A>.
/// \sa AuthenticatedSymmetricCipher, AuthenticatedEncryptionFilter, EAX, CCM, GCM,
///  and <A HREF="https://www.cryptopp.com/wiki/AadSource">AadSource</A> on the
///  Crypto++ wiki.
/// \since Crypto++ 5.6.0
class CRYPTOPP_DLL AuthenticatedDecryptionFilter : public FilterWithBufferedInput, public BlockPaddingSchemeDef
{
public:
	/// \enum Flags
	/// \brief Flags controlling filter behavior.
	/// \details The flags are a bitmask and can be OR'd together.
	enum Flags {
		/// \brief The MAC is at the end of the message (i.e., concatenation of message+mac)
		MAC_AT_END=0,
		/// \brief The MAC is at the beginning of the message (i.e., concatenation of mac+message)
		MAC_AT_BEGIN=1,
		/// \brief The filter should throw a HashVerificationFailed if a failure is encountered
		THROW_EXCEPTION=16,
		/// \brief Default flags using THROW_EXCEPTION
		DEFAULT_FLAGS = THROW_EXCEPTION
	};

	virtual ~AuthenticatedDecryptionFilter() {}

	/// \brief Construct a AuthenticatedDecryptionFilter
	/// \param c reference to a AuthenticatedSymmetricCipher
	/// \param attachment an optional attached transformation
	/// \param flags flags indicating behaviors for the filter
	/// \param truncatedDigestSize the size of the digest
	/// \param padding the \ref BlockPaddingSchemeDef "padding scheme"
	/// \details Additional authenticated data should be given in channel "AAD".
	/// \details <tt>truncatedDigestSize = -1</tt> indicates \ref HashTransformation::DigestSize() "DigestSize" should be used.
	/// \since Crypto++ 5.6.0
	AuthenticatedDecryptionFilter(AuthenticatedSymmetricCipher &c, BufferedTransformation *attachment = NULLPTR, word32 flags = DEFAULT_FLAGS, int truncatedDigestSize=-1, BlockPaddingScheme padding = DEFAULT_PADDING);

	std::string AlgorithmName() const {return m_hashVerifier.AlgorithmName();}
	byte * ChannelCreatePutSpace(const std::string &channel, size_t &size);
	size_t ChannelPut2(const std::string &channel, const byte *begin, size_t length, int messageEnd, bool blocking);
	size_t ChannelPutModifiable2(const std::string &channel, byte *begin, size_t length, int messageEnd, bool blocking)
		{ return ChannelPut2(channel, begin, length, messageEnd, blocking); }
	/// \brief Get verifier result
	/// \return true if the digest on the previosus message was valid, false otherwise
	bool GetLastResult() const {return m_hashVerifier.GetLastResult();}

protected:
	void InitializeDerivedAndReturnNewSizes(const NameValuePairs &parameters, size_t &firstSize, size_t &blockSize, size_t &lastSize);
	void FirstPut(const byte *inString);
	void NextPutMultiple(const byte *inString, size_t length);

	/// \brief Input the last block of data
	/// \param inString the input byte buffer
	/// \param length the size of the input buffer, in bytes
	/// \details LastPut() processes the last block of data and signals attached filters to do the same.
	///  LastPut() is always called. The pseudo algorithm for the logic is:
	/// <pre>
	///     if totalLength < firstSize then length == totalLength
	///     else if totalLength <= firstSize+lastSize then length == totalLength-firstSize
	///     else lastSize <= length < lastSize+blockSize
	/// </pre>
	void LastPut(const byte *inString, size_t length);

	HashVerificationFilter m_hashVerifier;
	StreamTransformationFilter m_streamFilter;
};

/// \brief Filter wrapper for PK_Signer
/// \since Crypto++ 4.0
class CRYPTOPP_DLL SignerFilter : public Unflushable<Filter>
{
public:
	virtual ~SignerFilter() {}

	/// \brief Construct a SignerFilter
	/// \param rng a RandomNumberGenerator derived class
	/// \param signer a PK_Signer derived class
	/// \param attachment an optional attached transformation
	/// \param putMessage flag indicating whether the original message should be passed to an attached transformation
	SignerFilter(RandomNumberGenerator &rng, const PK_Signer &signer, BufferedTransformation *attachment = NULLPTR, bool putMessage=false)
		: m_rng(rng), m_signer(signer), m_messageAccumulator(signer.NewSignatureAccumulator(rng)), m_putMessage(putMessage) {Detach(attachment);}

	std::string AlgorithmName() const {return m_signer.AlgorithmName();}

	void IsolatedInitialize(const NameValuePairs &parameters);
	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking);

private:
	RandomNumberGenerator &m_rng;
	const PK_Signer &m_signer;
	member_ptr<PK_MessageAccumulator> m_messageAccumulator;
	bool m_putMessage;
	SecByteBlock m_buf;
};

/// \brief Filter wrapper for PK_Verifier
/// \details This filter was formerly named <tt>VerifierFilter</tt>. The name changed at Crypto++ 5.0.
/// \since Crypto++ 4.0
class CRYPTOPP_DLL SignatureVerificationFilter : public FilterWithBufferedInput
{
public:
	/// \brief Exception thrown when an invalid signature is encountered
	class SignatureVerificationFailed : public Exception
	{
	public:
		SignatureVerificationFailed()
			: Exception(DATA_INTEGRITY_CHECK_FAILED, "VerifierFilter: digital signature not valid") {}
	};

	/// \enum Flags
	/// \brief Flags controlling filter behavior.
	/// \details The flags are a bitmask and can be OR'd together.
	enum Flags {
		/// \brief The signature is at the end of the message (i.e., concatenation of message+signature)
		SIGNATURE_AT_END=0,
		/// \brief The signature is at the beginning of the message (i.e., concatenation of signature+message)
		SIGNATURE_AT_BEGIN=1,
		/// \brief The message should be passed to an attached transformation
		PUT_MESSAGE=2,
		/// \brief The signature should be passed to an attached transformation
		PUT_SIGNATURE=4,
		/// \brief The result of the verification should be passed to an attached transformation
		PUT_RESULT=8,
		/// \brief The filter should throw a HashVerificationFailed if a failure is encountered
		THROW_EXCEPTION=16,
		/// \brief Default flags using SIGNATURE_AT_BEGIN and PUT_RESULT
		DEFAULT_FLAGS = SIGNATURE_AT_BEGIN | PUT_RESULT
	};

	virtual ~SignatureVerificationFilter() {}

	/// \brief Construct a SignatureVerificationFilter
	/// \param verifier a PK_Verifier derived class
	/// \param attachment an optional attached transformation
	/// \param flags flags indicating behaviors for the filter
	SignatureVerificationFilter(const PK_Verifier &verifier, BufferedTransformation *attachment = NULLPTR, word32 flags = DEFAULT_FLAGS);

	std::string AlgorithmName() const {return m_verifier.AlgorithmName();}

	/// \brief Retrieves the result of the last verification
	/// \return true if the signature on the previosus message was valid, false otherwise
	bool GetLastResult() const {return m_verified;}

protected:
	void InitializeDerivedAndReturnNewSizes(const NameValuePairs &parameters, size_t &firstSize, size_t &blockSize, size_t &lastSize);
	void FirstPut(const byte *inString);
	void NextPutMultiple(const byte *inString, size_t length);
	void LastPut(const byte *inString, size_t length);

private:
	const PK_Verifier &m_verifier;
	member_ptr<PK_MessageAccumulator> m_messageAccumulator;
	word32 m_flags;
	SecByteBlock m_signature;
	bool m_verified;
};

/// \brief Redirect input to another BufferedTransformation without owning it
/// \since Crypto++ 4.0
class CRYPTOPP_DLL Redirector : public CustomSignalPropagation<Sink>
{
public:
	/// \enum Behavior
	/// \brief Controls signal propagation behavior
	enum Behavior
	{
		/// \brief Pass data only
		DATA_ONLY = 0x00,
		/// \brief Pass signals
		PASS_SIGNALS = 0x01,
		/// \brief Pass wait events
		PASS_WAIT_OBJECTS = 0x02,
		/// \brief Pass everything
		/// \details PASS_EVERYTHING is default
		PASS_EVERYTHING = PASS_SIGNALS | PASS_WAIT_OBJECTS
	};

	virtual ~Redirector() {}

	/// \brief Construct a Redirector
	Redirector() : m_target(NULLPTR), m_behavior(PASS_EVERYTHING) {}

	/// \brief Construct a Redirector
	/// \param target the destination BufferedTransformation
	/// \param behavior Behavior "flags" specifying signal propagation
	Redirector(BufferedTransformation &target, Behavior behavior=PASS_EVERYTHING)
		: m_target(&target), m_behavior(behavior) {}

	/// \brief Redirect input to another BufferedTransformation
	/// \param target the destination BufferedTransformation
	void Redirect(BufferedTransformation &target) {m_target = &target;}
	/// \brief Stop redirecting input
	void StopRedirection() {m_target = NULLPTR;}

	/// \brief Retrieve signal propagation behavior
	/// \return the current signal propagation behavior
	Behavior GetBehavior() {return static_cast<Behavior>(m_behavior);}
	/// \brief Set signal propagation behavior
	/// \param behavior the new signal propagation behavior
	void SetBehavior(Behavior behavior) {m_behavior=behavior;}
	/// \brief Retrieve signal propagation behavior
	/// \return true if the Redirector passes signals, false otherwise.
	bool GetPassSignals() const {return (m_behavior & PASS_SIGNALS) != 0;}
	/// \brief Set signal propagation behavior
	/// \param pass flag indicating if the Redirector should pass signals
	void SetPassSignals(bool pass) { if (pass) m_behavior |= PASS_SIGNALS; else m_behavior &= ~static_cast<word32>(PASS_SIGNALS); }
	/// \brief Retrieve signal propagation behavior
	/// \return true if the Redirector passes wait objects, false otherwise.
	bool GetPassWaitObjects() const {return (m_behavior & PASS_WAIT_OBJECTS) != 0;}
	/// \brief Set signal propagation behavior
	/// \param pass flag indicating if the Redirector should pass wait objects
	void SetPassWaitObjects(bool pass) { if (pass) m_behavior |= PASS_WAIT_OBJECTS; else m_behavior &= ~static_cast<word32>(PASS_WAIT_OBJECTS); }

	bool CanModifyInput() const
		{return m_target ? m_target->CanModifyInput() : false;}

	void Initialize(const NameValuePairs &parameters, int propagation);
	byte * CreatePutSpace(size_t &size)
	{
		if (m_target)
			return m_target->CreatePutSpace(size);
		else
		{
			size = 0;
			return NULLPTR;
		}
	}
	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking)
		{return m_target ? m_target->Put2(inString, length, GetPassSignals() ? messageEnd : 0, blocking) : 0;}
	bool Flush(bool hardFlush, int propagation=-1, bool blocking=true)
		{return m_target && GetPassSignals() ? m_target->Flush(hardFlush, propagation, blocking) : false;}
	bool MessageSeriesEnd(int propagation=-1, bool blocking=true)
		{return m_target && GetPassSignals() ? m_target->MessageSeriesEnd(propagation, blocking) : false;}

	byte * ChannelCreatePutSpace(const std::string &channel, size_t &size)
	{
		if (m_target)
			return m_target->ChannelCreatePutSpace(channel, size);
		else
		{
			size = 0;
			return NULLPTR;
		}
	}
	size_t ChannelPut2(const std::string &channel, const byte *begin, size_t length, int messageEnd, bool blocking)
		{return m_target ? m_target->ChannelPut2(channel, begin, length, GetPassSignals() ? messageEnd : 0, blocking) : 0;}
	size_t ChannelPutModifiable2(const std::string &channel, byte *begin, size_t length, int messageEnd, bool blocking)
		{return m_target ? m_target->ChannelPutModifiable2(channel, begin, length, GetPassSignals() ? messageEnd : 0, blocking) : 0;}
	bool ChannelFlush(const std::string &channel, bool completeFlush, int propagation=-1, bool blocking=true)
		{return m_target && GetPassSignals() ? m_target->ChannelFlush(channel, completeFlush, propagation, blocking) : false;}
	bool ChannelMessageSeriesEnd(const std::string &channel, int propagation=-1, bool blocking=true)
		{return m_target && GetPassSignals() ? m_target->ChannelMessageSeriesEnd(channel, propagation, blocking) : false;}

	unsigned int GetMaxWaitObjectCount() const
		{ return m_target && GetPassWaitObjects() ? m_target->GetMaxWaitObjectCount() : 0; }
	void GetWaitObjects(WaitObjectContainer &container, CallStack const& callStack)
		{ if (m_target && GetPassWaitObjects()) m_target->GetWaitObjects(container, callStack); }

private:
	BufferedTransformation *m_target;
	word32 m_behavior;
};

/// \brief Filter class that is a proxy for a sink
/// \details Used By ProxyFilter
/// \since Crypto++ 4.0
class CRYPTOPP_DLL OutputProxy : public CustomSignalPropagation<Sink>
{
public:
	virtual ~OutputProxy() {}

	/// \brief Construct an OutputProxy
	/// \param owner the owning transformation
	/// \param passSignal flag indicating if signals should be passed
	OutputProxy(BufferedTransformation &owner, bool passSignal) : m_owner(owner), m_passSignal(passSignal) {}

	/// \brief Retrieve passSignal flag
	/// \return flag indicating if signals should be passed
	bool GetPassSignal() const {return m_passSignal;}
	/// \brief Set passSignal flag
	/// \param passSignal flag indicating if signals should be passed
	void SetPassSignal(bool passSignal) {m_passSignal = passSignal;}

	byte * CreatePutSpace(size_t &size)
		{return m_owner.AttachedTransformation()->CreatePutSpace(size);}
	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking)
		{return m_owner.AttachedTransformation()->Put2(inString, length, m_passSignal ? messageEnd : 0, blocking);}
	size_t PutModifiable2(byte *begin, size_t length, int messageEnd, bool blocking)
		{return m_owner.AttachedTransformation()->PutModifiable2(begin, length, m_passSignal ? messageEnd : 0, blocking);}
	void Initialize(const NameValuePairs &parameters=g_nullNameValuePairs, int propagation=-1)
		{if (m_passSignal) m_owner.AttachedTransformation()->Initialize(parameters, propagation);}
	bool Flush(bool hardFlush, int propagation=-1, bool blocking=true)
		{return m_passSignal ? m_owner.AttachedTransformation()->Flush(hardFlush, propagation, blocking) : false;}
	bool MessageSeriesEnd(int propagation=-1, bool blocking=true)
		{return m_passSignal ? m_owner.AttachedTransformation()->MessageSeriesEnd(propagation, blocking) : false;}

	byte * ChannelCreatePutSpace(const std::string &channel, size_t &size)
		{return m_owner.AttachedTransformation()->ChannelCreatePutSpace(channel, size);}
	size_t ChannelPut2(const std::string &channel, const byte *begin, size_t length, int messageEnd, bool blocking)
		{return m_owner.AttachedTransformation()->ChannelPut2(channel, begin, length, m_passSignal ? messageEnd : 0, blocking);}
	size_t ChannelPutModifiable2(const std::string &channel, byte *begin, size_t length, int messageEnd, bool blocking)
		{return m_owner.AttachedTransformation()->ChannelPutModifiable2(channel, begin, length, m_passSignal ? messageEnd : 0, blocking);}
	bool ChannelFlush(const std::string &channel, bool completeFlush, int propagation=-1, bool blocking=true)
		{return m_passSignal ? m_owner.AttachedTransformation()->ChannelFlush(channel, completeFlush, propagation, blocking) : false;}
	bool ChannelMessageSeriesEnd(const std::string &channel, int propagation=-1, bool blocking=true)
		{return m_passSignal ? m_owner.AttachedTransformation()->ChannelMessageSeriesEnd(channel, propagation, blocking) : false;}

private:
	BufferedTransformation &m_owner;
	bool m_passSignal;
};

/// \brief Base class for Filter classes that are proxies for a chain of other filters
/// \since Crypto++ 4.0
class CRYPTOPP_DLL ProxyFilter : public FilterWithBufferedInput
{
public:
	virtual ~ProxyFilter() {}

	/// \brief Construct a ProxyFilter
	/// \param filter an output filter
	/// \param firstSize the first Put size
	/// \param lastSize the last Put size
	/// \param attachment an attached transformation
	ProxyFilter(BufferedTransformation *filter, size_t firstSize, size_t lastSize, BufferedTransformation *attachment);

	bool IsolatedFlush(bool hardFlush, bool blocking);

	/// \brief Sets the OutputProxy filter
	/// \param filter an OutputProxy filter
	void SetFilter(Filter *filter);
	void NextPutMultiple(const byte *s, size_t len);
	void NextPutModifiable(byte *inString, size_t length);

protected:
	member_ptr<BufferedTransformation> m_filter;
};

/// \brief Proxy filter that doesn't modify the underlying filter's input or output
/// \since Crypto++ 5.0
class CRYPTOPP_DLL SimpleProxyFilter : public ProxyFilter
{
public:
	/// \brief Construct a SimpleProxyFilter
	/// \param filter an output filter
	/// \param attachment an attached transformation
	SimpleProxyFilter(BufferedTransformation *filter, BufferedTransformation *attachment)
		: ProxyFilter(filter, 0, 0, attachment) {}

	void FirstPut(const byte * inString)
		{CRYPTOPP_UNUSED(inString);}

	/// \brief Input the last block of data
	/// \param inString the input byte buffer
	/// \param length the size of the input buffer, in bytes
	/// \details LastPut() processes the last block of data and signals attached filters to do the same.
	///  LastPut() is always called. The pseudo algorithm for the logic is:
	/// <pre>
	///     if totalLength < firstSize then length == totalLength
	///     else if totalLength <= firstSize+lastSize then length == totalLength-firstSize
	///     else lastSize <= length < lastSize+blockSize
	/// </pre>
	void LastPut(const byte *inString, size_t length)
		{CRYPTOPP_UNUSED(inString), CRYPTOPP_UNUSED(length); m_filter->MessageEnd();}
};

/// \brief Filter wrapper for PK_Encryptor
/// \details PK_DecryptorFilter is a proxy for the filter created by PK_Encryptor::CreateEncryptionFilter.
///  This class provides symmetry with VerifierFilter.
/// \since Crypto++ 5.0
class CRYPTOPP_DLL PK_EncryptorFilter : public SimpleProxyFilter
{
public:
	/// \brief Construct a PK_EncryptorFilter
	/// \param rng a RandomNumberGenerator derived class
	/// \param encryptor a PK_Encryptor derived class
	/// \param attachment an optional attached transformation
	PK_EncryptorFilter(RandomNumberGenerator &rng, const PK_Encryptor &encryptor, BufferedTransformation *attachment = NULLPTR)
		: SimpleProxyFilter(encryptor.CreateEncryptionFilter(rng), attachment) {}
};

/// \brief Filter wrapper for PK_Decryptor
/// \details PK_DecryptorFilter is a proxy for the filter created by PK_Decryptor::CreateDecryptionFilter.
///  This class provides symmetry with SignerFilter.
/// \since Crypto++ 5.0
class CRYPTOPP_DLL PK_DecryptorFilter : public SimpleProxyFilter
{
public:
	/// \brief Construct a PK_DecryptorFilter
	/// \param rng a RandomNumberGenerator derived class
	/// \param decryptor a PK_Decryptor derived class
	/// \param attachment an optional attached transformation
	PK_DecryptorFilter(RandomNumberGenerator &rng, const PK_Decryptor &decryptor, BufferedTransformation *attachment = NULLPTR)
		: SimpleProxyFilter(decryptor.CreateDecryptionFilter(rng), attachment) {}
};

/// \brief Append input to a string object
/// \tparam T std::basic_string<char> type
/// \details StringSinkTemplate is a StringSinkTemplate typedef
/// \since Crypto++ 5.0
template <class T>
class StringSinkTemplate : public Bufferless<Sink>
{
public:
	typedef typename T::value_type value_type;
	virtual ~StringSinkTemplate() {}

	/// \brief Construct a StringSinkTemplate
	/// \param output std::basic_string<char> or std::vector<byte> type
	StringSinkTemplate(T &output)
		: m_output(&output) {CRYPTOPP_ASSERT(sizeof(value_type)==1);}

	void IsolatedInitialize(const NameValuePairs &parameters)
		{if (!parameters.GetValue("OutputStringPointer", m_output)) throw InvalidArgument("StringSink: OutputStringPointer not specified");}

	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking)
	{
		CRYPTOPP_UNUSED(messageEnd); CRYPTOPP_UNUSED(blocking);
		if (length > 0)
		{
			typename T::size_type size = m_output->size();
			if (length < size && size + length > m_output->capacity())
				m_output->reserve(2*size);
			m_output->insert(m_output->end(), (const value_type *)inString, (const value_type *)inString+length);
		}
		return 0;
	}

private:
	T *m_output;
};

/// \brief Append input to a string object
/// \details StringSink is a typedef for StringSinkTemplate<std::string>.
/// \sa ArraySink, ArrayXorSink
/// \since Crypto++ 4.0
DOCUMENTED_TYPEDEF(StringSinkTemplate<std::string>, StringSink);
CRYPTOPP_DLL_TEMPLATE_CLASS StringSinkTemplate<std::string>;

/// \brief Append input to a std::vector<byte> object
/// \details VectorSink is a typedef for StringSinkTemplate<std::vector<byte> >.
/// \since Crypto++ 8.0
DOCUMENTED_TYPEDEF(StringSinkTemplate<std::vector<byte> >, VectorSink);
CRYPTOPP_DLL_TEMPLATE_CLASS StringSinkTemplate<std::vector<byte> >;

/// \brief Incorporates input into RNG as additional entropy
/// \since Crypto++ 4.0
class RandomNumberSink : public Bufferless<Sink>
{
public:
	virtual ~RandomNumberSink() {}

	/// \brief Construct a RandomNumberSink
	RandomNumberSink()
		: m_rng(NULLPTR) {}

	/// \brief Construct a RandomNumberSink
	/// \param rng a RandomNumberGenerator derived class
	RandomNumberSink(RandomNumberGenerator &rng)
		: m_rng(&rng) {}

	void IsolatedInitialize(const NameValuePairs &parameters);
	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking);

private:
	RandomNumberGenerator *m_rng;
};

/// \brief Copy input to a memory buffer
/// \details ArraySink wraps a fixed size buffer. The buffer is full once Put returns non-0.
///  When used in a pipleline, ArraySink silently discards input if the buffer is full.
///  AvailableSize() can be used to determine how much space remains in the buffer.
///  TotalPutLength() can be used to determine how many bytes were processed.
/// \sa StringSink, ArrayXorSink
/// \since Crypto++ 4.0
class CRYPTOPP_DLL ArraySink : public Bufferless<Sink>
{
public:
	virtual ~ArraySink() {}

	/// \brief Construct an ArraySink
	/// \param parameters a set of NameValuePairs to initialize this object
	/// \details Name::OutputBuffer() is a mandatory parameter using this constructor.
	ArraySink(const NameValuePairs &parameters = g_nullNameValuePairs)
		: m_buf(NULLPTR), m_size(0), m_total(0) {IsolatedInitialize(parameters);}

	/// \brief Construct an ArraySink
	/// \param buf pointer to a memory buffer
	/// \param size length of the memory buffer
	ArraySink(byte *buf, size_t size)
		: m_buf(buf), m_size(size), m_total(0) {}

	/// \brief Provides the size remaining in the Sink
	/// \return size remaining in the Sink, in bytes
	size_t AvailableSize() {return SaturatingSubtract(m_size, m_total);}

	/// \brief Provides the number of bytes written to the Sink
	/// \return number of bytes written to the Sink, in bytes
	lword TotalPutLength() {return m_total;}

	void IsolatedInitialize(const NameValuePairs &parameters);
	byte * CreatePutSpace(size_t &size);
	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking);

protected:
	byte *m_buf;
	size_t m_size;
	lword m_total;
};

/// \brief Xor input to a memory buffer
/// \details ArrayXorSink wraps a fixed size buffer. The buffer is full once Put returns non-0.
///  When used in a pipleline, ArrayXorSink silently discards input if the buffer is full.
///  AvailableSize() can be used to determine how much space remains in the buffer.
///  TotalPutLength() can be used to determine how many bytes were processed.
/// \sa StringSink, ArraySink
/// \since Crypto++ 4.0
class CRYPTOPP_DLL ArrayXorSink : public ArraySink
{
public:
	virtual ~ArrayXorSink() {}

	/// \brief Construct an ArrayXorSink
	/// \param buf pointer to a memory buffer
	/// \param size length of the memory buffer
	ArrayXorSink(byte *buf, size_t size)
		: ArraySink(buf, size) {}

	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking);
	byte * CreatePutSpace(size_t &size) {return BufferedTransformation::CreatePutSpace(size);}
};

/// \brief String-based implementation of Store interface
/// \since Crypto++ 4.0
class StringStore : public Store
{
public:
	/// \brief Construct a StringStore
	/// \param string pointer to a C-String
	StringStore(const char *string = NULLPTR)
		{StoreInitialize(MakeParameters("InputBuffer", ConstByteArrayParameter(string)));}

	/// \brief Construct a StringStore
	/// \param string pointer to a memory buffer
	/// \param length size of the memory buffer
	StringStore(const byte *string, size_t length)
		{StoreInitialize(MakeParameters("InputBuffer", ConstByteArrayParameter(string, length)));}

	/// \brief Construct a StringStore
	/// \tparam T std::basic_string<char> type
	/// \param string reference to a std::basic_string<char> type
	template <class T> StringStore(const T &string)
		{StoreInitialize(MakeParameters("InputBuffer", ConstByteArrayParameter(string)));}

	CRYPTOPP_DLL size_t TransferTo2(BufferedTransformation &target, lword &transferBytes, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true);
	CRYPTOPP_DLL size_t CopyRangeTo2(BufferedTransformation &target, lword &begin, lword end=LWORD_MAX, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true) const;

private:
	CRYPTOPP_DLL void StoreInitialize(const NameValuePairs &parameters);

	const byte *m_store;
	size_t m_length, m_count;
};

/// \brief RNG-based implementation of Source interface
/// \since Crypto++ 4.0
class CRYPTOPP_DLL RandomNumberStore : public Store
{
public:
	virtual ~RandomNumberStore() {}

	RandomNumberStore()
		: m_rng(NULLPTR), m_length(0), m_count(0) {}

	RandomNumberStore(RandomNumberGenerator &rng, lword length)
		: m_rng(&rng), m_length(length), m_count(0) {}

	bool AnyRetrievable() const {return MaxRetrievable() != 0;}
	lword MaxRetrievable() const {return m_length-m_count;}

	size_t TransferTo2(BufferedTransformation &target, lword &transferBytes, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true);
	size_t CopyRangeTo2(BufferedTransformation &target, lword &begin, lword end=LWORD_MAX, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true) const
	{
		CRYPTOPP_UNUSED(target); CRYPTOPP_UNUSED(begin); CRYPTOPP_UNUSED(end); CRYPTOPP_UNUSED(channel); CRYPTOPP_UNUSED(blocking);
		throw NotImplemented("RandomNumberStore: CopyRangeTo2() is not supported by this store");
	}

private:
	void StoreInitialize(const NameValuePairs &parameters);

	RandomNumberGenerator *m_rng;
	lword m_length, m_count;
};

/// \brief Empty store
/// \since Crypto++ 5.0
class CRYPTOPP_DLL NullStore : public Store
{
public:
	NullStore(lword size = ULONG_MAX) : m_size(size) {}
	void StoreInitialize(const NameValuePairs &parameters)
		{CRYPTOPP_UNUSED(parameters);}
	lword MaxRetrievable() const {return m_size;}
	size_t TransferTo2(BufferedTransformation &target, lword &transferBytes, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true);
	size_t CopyRangeTo2(BufferedTransformation &target, lword &begin, lword end=LWORD_MAX, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true) const;

private:
	lword m_size;
};

/// \brief Implementation of BufferedTransformation's attachment interface
/// \details Source is a cornerstone of the Pipeline trinitiy. Data flows from
///  Sources, through Filters, and then terminates in Sinks. The difference
///  between a Source and Filter is a Source \a pumps data, while a Filter does
///  not. The difference between a Filter and a Sink is a Filter allows an
///  attached transformation, while a Sink does not.
/// \details See the discussion of BufferedTransformation in cryptlib.h for
///  more details.
/// \sa Store and SourceTemplate
/// \since Crypto++ 1.0
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE Source : public InputRejecting<Filter>
{
public:
	virtual ~Source() {}

	/// \brief Construct a Source
	/// \param attachment an optional attached transformation
	Source(BufferedTransformation *attachment = NULLPTR)
		{Source::Detach(attachment);}

	///	\name PIPELINE
	//@{

	/// \brief Pump data to attached transformation
	/// \param pumpMax the maximum number of bytes to pump
	/// \return the number of bytes that remain to be processed (i.e., bytes not processed).
	///  0 indicates all bytes were processed.
	/// \details Internally, Pump() calls Pump2().
	/// \note pumpMax is a <tt>lword</tt>, which is a 64-bit value that typically uses
	///  <tt>LWORD_MAX</tt>. The default argument is <tt>SIZE_MAX</tt>, and it can be
	///  32-bits or 64-bits.
	/// \sa Pump2, PumpAll, AnyRetrievable, MaxRetrievable
	lword Pump(lword pumpMax=SIZE_MAX)
		{Pump2(pumpMax); return pumpMax;}

	/// \brief Pump messages to attached transformation
	/// \param count the maximum number of messages to pump
	/// \return TODO
	/// \details Internally, PumpMessages() calls PumpMessages2().
	unsigned int PumpMessages(unsigned int count=UINT_MAX)
		{PumpMessages2(count); return count;}

	/// \brief Pump all data to attached transformation
	/// \details Pumps all data to the attached transformation and signal the end of the current
	///  message. To avoid the MessageEnd() signal call \ref Pump "Pump(LWORD_MAX)" or \ref Pump2
	///  "Pump2(LWORD_MAX, bool)".
	/// \details Internally, PumpAll() calls PumpAll2(), which calls PumpMessages().
	/// \sa Pump, Pump2, AnyRetrievable, MaxRetrievable
	void PumpAll()
		{PumpAll2();}

	/// \brief Pump data to attached transformation
	/// \param byteCount the maximum number of bytes to pump
	/// \param blocking specifies whether the object should block when processing input
	/// \return the number of bytes that remain to be processed (i.e., bytes not processed).
	///  0 indicates all bytes were processed.
	/// \details byteCount is an \a IN and \a OUT parameter. When the call is made, byteCount is the
	///  requested size of the pump. When the call returns, byteCount is the number of bytes that
	///  were pumped.
	/// \sa Pump, PumpAll, AnyRetrievable, MaxRetrievable
	virtual size_t Pump2(lword &byteCount, bool blocking=true) =0;

	/// \brief Pump messages to attached transformation
	/// \param messageCount the maximum number of messages to pump
	/// \param blocking specifies whether the object should block when processing input
	/// \details messageCount is an IN and OUT parameter.
	virtual size_t PumpMessages2(unsigned int &messageCount, bool blocking=true) =0;

	/// \brief Pump all data to attached transformation
	/// \param blocking specifies whether the object should block when processing input
	/// \return the number of bytes that remain to be processed (i.e., bytes not processed).
	///  0 indicates all bytes were processed.
	/// \sa Pump, Pump2, AnyRetrievable, MaxRetrievable
	virtual size_t PumpAll2(bool blocking=true);

	/// \brief Determines if the Source is exhausted
	/// \return true if the source has been exhausted
	virtual bool SourceExhausted() const =0;

	//@}

protected:
	void SourceInitialize(bool pumpAll, const NameValuePairs &parameters)
	{
		IsolatedInitialize(parameters);
		if (pumpAll)
			PumpAll();
	}
};

/// \brief Transform a Store into a Source
/// \tparam T the class or type
/// \since Crypto++ 5.0
template <class T>
class SourceTemplate : public Source
{
public:
	virtual ~SourceTemplate() {}

	/// \brief Construct a SourceTemplate
	/// \param attachment an attached transformation
	SourceTemplate<T>(BufferedTransformation *attachment)
		: Source(attachment) {}
	void IsolatedInitialize(const NameValuePairs &parameters)
		{m_store.IsolatedInitialize(parameters);}
	size_t Pump2(lword &byteCount, bool blocking=true)
		{return m_store.TransferTo2(*AttachedTransformation(), byteCount, DEFAULT_CHANNEL, blocking);}
	size_t PumpMessages2(unsigned int &messageCount, bool blocking=true)
		{return m_store.TransferMessagesTo2(*AttachedTransformation(), messageCount, DEFAULT_CHANNEL, blocking);}
	size_t PumpAll2(bool blocking=true)
		{return m_store.TransferAllTo2(*AttachedTransformation(), DEFAULT_CHANNEL, blocking);}
	bool SourceExhausted() const
		{return !m_store.AnyRetrievable() && !m_store.AnyMessages();}
	void SetAutoSignalPropagation(int propagation)
		{m_store.SetAutoSignalPropagation(propagation);}
	int GetAutoSignalPropagation() const
		{return m_store.GetAutoSignalPropagation();}

protected:
	T m_store;
};

/// \brief String-based implementation of the Source interface
/// \since Crypto++ 4.0
class CRYPTOPP_DLL StringSource : public SourceTemplate<StringStore>
{
public:
	/// \brief Construct a StringSource
	/// \param attachment an optional attached transformation
	StringSource(BufferedTransformation *attachment = NULLPTR)
		: SourceTemplate<StringStore>(attachment) {}

	/// \brief Construct a StringSource
	/// \param string C-String
	/// \param pumpAll flag indicating if source data should be pumped to its attached transformation
	/// \param attachment an optional attached transformation
	StringSource(const char *string, bool pumpAll, BufferedTransformation *attachment = NULLPTR)
		: SourceTemplate<StringStore>(attachment) {SourceInitialize(pumpAll, MakeParameters("InputBuffer", ConstByteArrayParameter(string)));}

	/// \brief Construct a StringSource
	/// \param string binary byte array
	/// \param length size of the byte array
	/// \param pumpAll flag indicating if source data should be pumped to its attached transformation
	/// \param attachment an optional attached transformation
	StringSource(const byte *string, size_t length, bool pumpAll, BufferedTransformation *attachment = NULLPTR)
		: SourceTemplate<StringStore>(attachment) {SourceInitialize(pumpAll, MakeParameters("InputBuffer", ConstByteArrayParameter(string, length)));}

	/// \brief Construct a StringSource
	/// \param string std::string
	/// \param pumpAll flag indicating if source data should be pumped to its attached transformation
	/// \param attachment an optional attached transformation
	StringSource(const std::string &string, bool pumpAll, BufferedTransformation *attachment = NULLPTR)
		: SourceTemplate<StringStore>(attachment) {SourceInitialize(pumpAll, MakeParameters("InputBuffer", ConstByteArrayParameter(string)));}
};

/// \brief Pointer-based implementation of the Source interface
/// \details ArraySource is a typedef for StringSource. Use the third constructor for an array source.
///  The third constructor takes a pointer and length.
/// \since Crypto++ 5.6.0
DOCUMENTED_TYPEDEF(StringSource, ArraySource);

/// \brief std::vector-based implementation of the Source interface
/// \since Crypto++ 8.0
class CRYPTOPP_DLL VectorSource : public SourceTemplate<StringStore>
{
public:
	/// \brief Construct a VectorSource
	/// \param attachment an optional attached transformation
	VectorSource(BufferedTransformation *attachment = NULLPTR)
		: SourceTemplate<StringStore>(attachment) {}

	/// \brief Construct a VectorSource
	/// \param vec vector of bytes
	/// \param pumpAll flag indicating if source data should be pumped to its attached transformation
	/// \param attachment an optional attached transformation
	VectorSource(const std::vector<byte> &vec, bool pumpAll, BufferedTransformation *attachment = NULLPTR)
		: SourceTemplate<StringStore>(attachment) {SourceInitialize(pumpAll, MakeParameters("InputBuffer", ConstByteArrayParameter(vec)));}
};

/// \brief RNG-based implementation of Source interface
/// \since Crypto++ 4.0
class CRYPTOPP_DLL RandomNumberSource : public SourceTemplate<RandomNumberStore>
{
public:
	RandomNumberSource(RandomNumberGenerator &rng, int length, bool pumpAll, BufferedTransformation *attachment = NULLPTR)
		: SourceTemplate<RandomNumberStore>(attachment)
		{SourceInitialize(pumpAll, MakeParameters("RandomNumberGeneratorPointer", &rng)("RandomNumberStoreSize", length));}
};

NAMESPACE_END

#if CRYPTOPP_MSC_VERSION
# pragma warning(pop)
#endif

#endif
