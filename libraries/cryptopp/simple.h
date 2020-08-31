// simple.h - originally written and placed in the public domain by Wei Dai

/// \file simple.h
/// \brief Classes providing basic library services.

#ifndef CRYPTOPP_SIMPLE_H
#define CRYPTOPP_SIMPLE_H

#include "config.h"

#if CRYPTOPP_MSC_VERSION
# pragma warning(push)
# pragma warning(disable: 4127 4189)
#endif

#include "cryptlib.h"
#include "misc.h"

NAMESPACE_BEGIN(CryptoPP)

/// \brief Base class for identifying alogorithm
/// \tparam BASE base class from which to derive
/// \tparam DERIVED class which to clone
template <class DERIVED, class BASE>
class CRYPTOPP_NO_VTABLE ClonableImpl : public BASE
{
public:
	/// \brief Create a copy of this object
	/// \return a copy of this object
	/// \details The caller is responsible for freeing the object.
	Clonable * Clone() const {return new DERIVED(*static_cast<const DERIVED *>(this));}
};

/// \brief Base class information
/// \tparam BASE an Algorithm derived class
/// \tparam ALGORITHM_INFO an Algorithm derived class
/// \details AlgorithmImpl provides StaticAlgorithmName from the template parameter BASE
template <class BASE, class ALGORITHM_INFO=BASE>
class CRYPTOPP_NO_VTABLE AlgorithmImpl : public BASE
{
public:
	/// \brief The algorithm name
	/// \return the algorithm name
	/// \details StaticAlgorithmName returns the algorithm's name as a static member function.
	///  The name is taken from information provided by BASE.
	static std::string CRYPTOPP_API StaticAlgorithmName() {return ALGORITHM_INFO::StaticAlgorithmName();}
	/// \brief The algorithm name
	/// \return the algorithm name
	/// \details AlgorithmName returns the algorithm's name as a member function.
	///  The name is is acquired by calling StaticAlgorithmName.
	std::string AlgorithmName() const {return ALGORITHM_INFO::StaticAlgorithmName();}
};

/// \brief Exception thrown when an invalid key length is encountered
class CRYPTOPP_DLL InvalidKeyLength : public InvalidArgument
{
public:
	/// \brief Construct an InvalidKeyLength
	/// \param algorithm the Algorithm associated with the exception
	/// \param length the key size associated with the exception
	explicit InvalidKeyLength(const std::string &algorithm, size_t length) : InvalidArgument(algorithm + ": " + IntToString(length) + " is not a valid key length") {}
};

/// \brief Exception thrown when an invalid number of rounds is encountered
class CRYPTOPP_DLL InvalidRounds : public InvalidArgument
{
public:
	/// \brief Construct an InvalidRounds
	/// \param algorithm the Algorithm associated with the exception
	/// \param rounds the number of rounds associated with the exception
	explicit InvalidRounds(const std::string &algorithm, unsigned int rounds) : InvalidArgument(algorithm + ": " + IntToString(rounds) + " is not a valid number of rounds") {}
};

/// \brief Exception thrown when an invalid block size is encountered
class CRYPTOPP_DLL InvalidBlockSize : public InvalidArgument
{
public:
	/// \brief Construct an InvalidBlockSize
	/// \param algorithm the Algorithm associated with the exception
	/// \param length the block size associated with the exception
	explicit InvalidBlockSize(const std::string &algorithm, size_t length) : InvalidArgument(algorithm + ": " + IntToString(length) + " is not a valid block size") {}
};

/// \brief Exception thrown when an invalid derived key length is encountered
class CRYPTOPP_DLL InvalidDerivedKeyLength : public InvalidArgument
{
public:
	/// \brief Construct an InvalidDerivedKeyLength
	/// \param algorithm the Algorithm associated with the exception
	/// \param length the size associated with the exception
	explicit InvalidDerivedKeyLength(const std::string &algorithm, size_t length) : InvalidArgument(algorithm + ": " + IntToString(length) + " is not a valid derived key length") {}
};

/// \brief Exception thrown when an invalid personalization string length is encountered
class CRYPTOPP_DLL InvalidPersonalizationLength : public InvalidArgument
{
public:
	/// \brief Construct an InvalidPersonalizationLength
	/// \param algorithm the Algorithm associated with the exception
	/// \param length the personalization size associated with the exception
	explicit InvalidPersonalizationLength(const std::string &algorithm, size_t length) : InvalidArgument(algorithm + ": " + IntToString(length) + " is not a valid salt length") {}
};

/// \brief Exception thrown when an invalid salt length is encountered
class CRYPTOPP_DLL InvalidSaltLength : public InvalidArgument
{
public:
	/// \brief Construct an InvalidSaltLength
	/// \param algorithm the Algorithm associated with the exception
	/// \param length the salt size associated with the exception
	explicit InvalidSaltLength(const std::string &algorithm, size_t length) : InvalidArgument(algorithm + ": " + IntToString(length) + " is not a valid salt length") {}
};

// *****************************

/// \brief Base class for bufferless filters
/// \tparam T the class or type
template <class T>
class CRYPTOPP_NO_VTABLE Bufferless : public T
{
public:
	/// \brief Flushes data buffered by this object, without signal propagation
	/// \param hardFlush indicates whether all data should be flushed
	/// \param blocking specifies whether the object should block when processing input
	/// \note hardFlush must be used with care
	bool IsolatedFlush(bool hardFlush, bool blocking)
		{CRYPTOPP_UNUSED(hardFlush); CRYPTOPP_UNUSED(blocking); return false;}
};

/// \brief Base class for unflushable filters
/// \tparam T the class or type
template <class T>
class CRYPTOPP_NO_VTABLE Unflushable : public T
{
public:
	/// \brief Flush buffered input and/or output, with signal propagation
	/// \param completeFlush is used to indicate whether all data should be flushed
	/// \param propagation the number of attached transformations the Flush()
	///  signal should be passed
	/// \param blocking specifies whether the object should block when processing
	///  input
	/// \details propagation count includes this object. Setting propagation to
	///  <tt>1</tt> means this object only. Setting propagation to <tt>-1</tt>
	///  means unlimited propagation.
	/// \note Hard flushes must be used with care. It means try to process and
	///  output everything, even if there may not be enough data to complete the
	///  action. For example, hard flushing a HexDecoder would cause an error if
	///  you do it after inputing an odd number of hex encoded characters.
	/// \note For some types of filters, like  ZlibDecompressor, hard flushes can
	///  only be done at "synchronization points". These synchronization points
	///  are positions in the data stream that are created by hard flushes on the
	///  corresponding reverse filters, in this example ZlibCompressor. This is
	///  useful when zlib compressed data is moved across a network in packets
	///  and compression state is preserved across packets, as in the SSH2 protocol.
	bool Flush(bool completeFlush, int propagation=-1, bool blocking=true)
		{return ChannelFlush(DEFAULT_CHANNEL, completeFlush, propagation, blocking);}

	/// \brief Flushes data buffered by this object, without signal propagation
	/// \param hardFlush indicates whether all data should be flushed
	/// \param blocking specifies whether the object should block when processing input
	/// \note hardFlush must be used with care
	bool IsolatedFlush(bool hardFlush, bool blocking)
		{CRYPTOPP_UNUSED(hardFlush); CRYPTOPP_UNUSED(blocking); CRYPTOPP_ASSERT(false); return false;}

	/// \brief Flush buffered input and/or output on a channel
	/// \param channel the channel to flush the data
	/// \param hardFlush is used to indicate whether all data should be flushed
	/// \param propagation the number of attached transformations the ChannelFlush()
	///  signal should be passed
	/// \param blocking specifies whether the object should block when processing input
	/// \return true of the Flush was successful
	/// \details propagation count includes this object. Setting propagation to
	///  <tt>1</tt> means this object only. Setting propagation to <tt>-1</tt> means
	///  unlimited propagation.
	bool ChannelFlush(const std::string &channel, bool hardFlush, int propagation=-1, bool blocking=true)
	{
		if (hardFlush && !InputBufferIsEmpty())
			throw CannotFlush("Unflushable<T>: this object has buffered input that cannot be flushed");
		else
		{
			BufferedTransformation *attached = this->AttachedTransformation();
			return attached && propagation ? attached->ChannelFlush(channel, hardFlush, propagation-1, blocking) : false;
		}
	}

protected:
	virtual bool InputBufferIsEmpty() const {return false;}
};

/// \brief Base class for input rejecting filters
/// \tparam T the class or type
/// \details T should be a BufferedTransformation derived class
template <class T>
class CRYPTOPP_NO_VTABLE InputRejecting : public T
{
public:
	struct InputRejected : public NotImplemented
		{InputRejected() : NotImplemented("BufferedTransformation: this object doesn't allow input") {}};

	///	\name INPUT
	//@{

	/// \brief Input a byte array for processing
	/// \param inString the byte array to process
	/// \param length the size of the string, in bytes
	/// \param messageEnd means how many filters to signal MessageEnd() to, including this one
	/// \param blocking specifies whether the object should block when processing input
	/// \throws InputRejected
	/// \return the number of bytes that remain to be processed (i.e., bytes not processed)
	/// \details Internally, the default implementation throws InputRejected.
	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking)
		{CRYPTOPP_UNUSED(inString); CRYPTOPP_UNUSED(length); CRYPTOPP_UNUSED(messageEnd); CRYPTOPP_UNUSED(blocking); throw InputRejected();}
	//@}

	///	\name SIGNALS
	//@{

	/// \brief Flushes data buffered by this object, without signal propagation
	/// \param hardFlush indicates whether all data should be flushed
	/// \param blocking specifies whether the object should block when processing input
	/// \note hardFlush must be used with care
	bool IsolatedFlush(bool hardFlush, bool blocking)
		{CRYPTOPP_UNUSED(hardFlush); CRYPTOPP_UNUSED(blocking); return false;}

	/// \brief Marks the end of a series of messages, without signal propagation
	/// \param blocking specifies whether the object should block when completing the processing on
	///  the current series of messages
	/// \return true if the message was successful, false otherwise
	bool IsolatedMessageSeriesEnd(bool blocking)
		{CRYPTOPP_UNUSED(blocking); throw InputRejected();}

	/// \brief Input multiple bytes for processing on a channel.
	/// \param channel the channel to process the data.
	/// \param inString the byte buffer to process.
	/// \param length the size of the string, in bytes.
	/// \param messageEnd means how many filters to signal MessageEnd() to, including this one.
	/// \param blocking specifies whether the object should block when processing input.
	/// \return the number of bytes that remain to be processed (i.e., bytes not processed)
	size_t ChannelPut2(const std::string &channel, const byte *inString, size_t length, int messageEnd, bool blocking)
		{CRYPTOPP_UNUSED(channel); CRYPTOPP_UNUSED(inString); CRYPTOPP_UNUSED(length);
		 CRYPTOPP_UNUSED(messageEnd); CRYPTOPP_UNUSED(blocking); throw InputRejected();}

	/// \brief Marks the end of a series of messages on a channel
	/// \param channel the channel to signal the end of a series of messages
	/// \param messageEnd the number of attached transformations the ChannelMessageSeriesEnd() signal should be passed
	/// \param blocking specifies whether the object should block when processing input
	/// \return true if the message was successful, false otherwise
	/// \details Each object that receives the signal will perform its processing, decrement
	///  propagation, and then pass the signal on to attached transformations if the value is not 0.
	/// \details propagation count includes this object. Setting propagation to <tt>1</tt> means this
	///  object only. Setting propagation to <tt>-1</tt> means unlimited propagation.
	/// \note There should be a MessageEnd() immediately before MessageSeriesEnd().
	bool ChannelMessageSeriesEnd(const std::string& channel, int messageEnd, bool blocking)
		{CRYPTOPP_UNUSED(channel); CRYPTOPP_UNUSED(messageEnd); CRYPTOPP_UNUSED(blocking); throw InputRejected();}
	//@}
};

/// \brief Interface for custom flush signals propagation
/// \tparam T BufferedTransformation derived class
template <class T>
class CRYPTOPP_NO_VTABLE CustomFlushPropagation : public T
{
public:
	///	\name SIGNALS
	//@{

	/// \brief Flush buffered input and/or output, with signal propagation
	/// \param hardFlush is used to indicate whether all data should be flushed
	/// \param propagation the number of attached transformations the  Flush() signal should be passed
	/// \param blocking specifies whether the object should block when processing input
	/// \details propagation count includes this object. Setting propagation to <tt>1</tt> means this
	///  object only. Setting propagation to <tt>-1</tt> means unlimited propagation.
	/// \note Hard flushes must be used with care. It means try to process and output everything, even if
	///  there may not be enough data to complete the action. For example, hard flushing a HexDecoder
	///  would cause an error if you do it after inputing an odd number of hex encoded characters.
	/// \note For some types of filters, like  ZlibDecompressor, hard flushes can only
	///  be done at "synchronization points". These synchronization points are positions in the data
	///  stream that are created by hard flushes on the corresponding reverse filters, in this
	///  example ZlibCompressor. This is useful when zlib compressed data is moved across a
	///  network in packets and compression state is preserved across packets, as in the SSH2 protocol.
	virtual bool Flush(bool hardFlush, int propagation=-1, bool blocking=true) =0;

	//@}

private:
	bool IsolatedFlush(bool hardFlush, bool blocking)
		{CRYPTOPP_UNUSED(hardFlush); CRYPTOPP_UNUSED(blocking); CRYPTOPP_ASSERT(false); return false;}
};

/// \brief Interface for custom flush signals
/// \tparam T BufferedTransformation derived class
template <class T>
class CRYPTOPP_NO_VTABLE CustomSignalPropagation : public CustomFlushPropagation<T>
{
public:
	/// \brief Initialize or reinitialize this object, with signal propagation
	/// \param parameters a set of NameValuePairs to initialize or reinitialize this object
	/// \param propagation the number of attached transformations the Initialize() signal should be passed
	/// \details Initialize() is used to initialize or reinitialize an object using a variable number of
	///  arbitrarily typed arguments. The function avoids the need for multiple constructors providing
	///  all possible combintations of configurable parameters.
	/// \details propagation count includes this object. Setting propagation to <tt>1</tt> means this
	///  object only. Setting propagation to <tt>-1</tt> means unlimited propagation.
	virtual void Initialize(const NameValuePairs &parameters=g_nullNameValuePairs, int propagation=-1) =0;

private:
	void IsolatedInitialize(const NameValuePairs &parameters)
		{CRYPTOPP_UNUSED(parameters); CRYPTOPP_ASSERT(false);}
};

/// \brief Multiple channels support for custom signal processing
/// \tparam T the class or type
/// \details T should be a BufferedTransformation derived class
template <class T>
class CRYPTOPP_NO_VTABLE Multichannel : public CustomFlushPropagation<T>
{
public:
	bool Flush(bool hardFlush, int propagation=-1, bool blocking=true)
		{return this->ChannelFlush(DEFAULT_CHANNEL, hardFlush, propagation, blocking);}

	/// \brief Marks the end of a series of messages, with signal propagation
	/// \param propagation the number of attached transformations the  MessageSeriesEnd() signal should be passed
	/// \param blocking specifies whether the object should block when processing input
	/// \details Each object that receives the signal will perform its processing, decrement
	///  propagation, and then pass the signal on to attached transformations if the value is not 0.
	/// \details propagation count includes this object. Setting propagation to <tt>1</tt> means this
	///  object only. Setting propagation to <tt>-1</tt> means unlimited propagation.
	/// \note There should be a MessageEnd() immediately before MessageSeriesEnd().
	bool MessageSeriesEnd(int propagation=-1, bool blocking=true)
		{return this->ChannelMessageSeriesEnd(DEFAULT_CHANNEL, propagation, blocking);}

	/// \brief Request space which can be written into by the caller
	/// \param size the requested size of the buffer
	/// \details The purpose of this method is to help avoid extra memory allocations.
	/// \details size is an \a IN and \a OUT parameter and used as a hint. When the call is made,
	///  size is the requested size of the buffer. When the call returns,  size is the size of
	///  the array returned to the caller.
	/// \details The base class implementation sets  size to 0 and returns  NULL.
	/// \note Some objects, like ArraySink, cannot create a space because its fixed. In the case of
	///  an ArraySink, the pointer to the array is returned and the  size is remaining size.
	byte * CreatePutSpace(size_t &size)
		{return this->ChannelCreatePutSpace(DEFAULT_CHANNEL, size);}

	/// \brief Input multiple bytes for processing
	/// \param inString the byte buffer to process
	/// \param length the size of the string, in bytes
	/// \param messageEnd means how many filters to signal MessageEnd() to, including this one
	/// \param blocking specifies whether the object should block when processing input
	/// \return the number of bytes that remain to be processed (i.e., bytes not processed)
	/// \details Derived classes must implement Put2().
	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking)
		{return this->ChannelPut2(DEFAULT_CHANNEL, inString, length, messageEnd, blocking);}

	/// \brief Input multiple bytes that may be modified by callee.
	/// \param inString the byte buffer to process.
	/// \param length the size of the string, in bytes.
	/// \param messageEnd means how many filters to signal MessageEnd() to, including this one.
	/// \param blocking specifies whether the object should block when processing input.
	/// \return the number of bytes that remain to be processed (i.e., bytes not processed)
	/// \details Internally, PutModifiable2() calls Put2().
	size_t PutModifiable2(byte *inString, size_t length, int messageEnd, bool blocking)
		{return this->ChannelPutModifiable2(DEFAULT_CHANNEL, inString, length, messageEnd, blocking);}

	//	void ChannelMessageSeriesEnd(const std::string &channel, int propagation=-1)
	//		{PropagateMessageSeriesEnd(propagation, channel);}

	/// \brief Request space which can be written into by the caller
	/// \param channel the channel to process the data
	/// \param size the requested size of the buffer
	/// \return a pointer to a memory block with length size
	/// \details The purpose of this method is to help avoid extra memory allocations.
	/// \details size is an \a IN and \a OUT parameter and used as a hint. When the call is made,
	///  size is the requested size of the buffer. When the call returns, size is the size of
	///  the array returned to the caller.
	/// \details The base class implementation sets size to 0 and returns NULL.
	/// \note Some objects, like ArraySink(), cannot create a space because its fixed. In the case of
	///  an ArraySink(), the pointer to the array is returned and the size is remaining size.
	byte * ChannelCreatePutSpace(const std::string &channel, size_t &size)
		{CRYPTOPP_UNUSED(channel); size = 0; return NULLPTR;}

	/// \brief Input multiple bytes that may be modified by callee on a channel
	/// \param channel the channel to process the data.
	/// \param inString the byte buffer to process
	/// \param length the size of the string, in bytes
	/// \return true if all bytes were processed, false otherwise.
	bool ChannelPutModifiable(const std::string &channel, byte *inString, size_t length)
		{this->ChannelPut(channel, inString, length); return false;}

	/// \brief Input multiple bytes for processing on a channel.
	/// \param channel the channel to process the data.
	/// \param begin the byte buffer to process.
	/// \param length the size of the string, in bytes.
	/// \param messageEnd means how many filters to signal MessageEnd() to, including this one.
	/// \param blocking specifies whether the object should block when processing input.
	/// \return the number of bytes that remain to be processed (i.e., bytes not processed)
	virtual size_t ChannelPut2(const std::string &channel, const byte *begin, size_t length, int messageEnd, bool blocking) =0;

	/// \brief Input multiple bytes that may be modified by callee on a channel
	/// \param channel the channel to process the data
	/// \param begin the byte buffer to process
	/// \param length the size of the string, in bytes
	/// \param messageEnd means how many filters to signal MessageEnd() to, including this one
	/// \param blocking specifies whether the object should block when processing input
	/// \return the number of bytes that remain to be processed (i.e., bytes not processed)
	size_t ChannelPutModifiable2(const std::string &channel, byte *begin, size_t length, int messageEnd, bool blocking)
		{return ChannelPut2(channel, begin, length, messageEnd, blocking);}

	/// \brief Flush buffered input and/or output on a channel
	/// \param channel the channel to flush the data
	/// \param hardFlush is used to indicate whether all data should be flushed
	/// \param propagation the number of attached transformations the ChannelFlush() signal should be passed
	/// \param blocking specifies whether the object should block when processing input
	/// \return true of the Flush was successful
	/// \details propagation count includes this object. Setting propagation to <tt>1</tt> means this
	///  object only. Setting propagation to <tt>-1</tt> means unlimited propagation.
	virtual bool ChannelFlush(const std::string &channel, bool hardFlush, int propagation=-1, bool blocking=true) =0;
};

/// \brief Provides auto signaling support
/// \tparam T BufferedTransformation derived class
template <class T>
class CRYPTOPP_NO_VTABLE AutoSignaling : public T
{
public:
	/// \brief Construct an AutoSignaling
	/// \param propagation the propagation count
	AutoSignaling(int propagation=-1) : m_autoSignalPropagation(propagation) {}

	/// \brief Set propagation of automatically generated and transferred signals
	/// \param propagation then new value
	/// \details Setting propagation to <tt>0</tt> means do not automatically generate signals. Setting
	///  propagation to <tt>-1</tt> means unlimited propagation.
	void SetAutoSignalPropagation(int propagation)
		{m_autoSignalPropagation = propagation;}

	/// \brief Retrieve automatic signal propagation value
	/// \return the number of attached transformations the signal is propagated to. 0 indicates
	///  the signal is only witnessed by this object
	int GetAutoSignalPropagation() const
		{return m_autoSignalPropagation;}

private:
	int m_autoSignalPropagation;
};

/// \brief Acts as a Source for pre-existing, static data
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE Store : public AutoSignaling<InputRejecting<BufferedTransformation> >
{
public:
	/// \brief Construct a Store
	Store() : m_messageEnd(false) {}

	void IsolatedInitialize(const NameValuePairs &parameters)
	{
		m_messageEnd = false;
		StoreInitialize(parameters);
	}

	unsigned int NumberOfMessages() const {return m_messageEnd ? 0 : 1;}
	bool GetNextMessage();
	unsigned int CopyMessagesTo(BufferedTransformation &target, unsigned int count=UINT_MAX, const std::string &channel=DEFAULT_CHANNEL) const;

protected:
	virtual void StoreInitialize(const NameValuePairs &parameters) =0;

	bool m_messageEnd;
};

/// \brief Implementation of BufferedTransformation's attachment interface
/// \details Sink is a cornerstone of the Pipeline trinitiy. Data flows from
///  Sources, through Filters, and then terminates in Sinks. The difference
///  between a Source and Filter is a Source \a pumps data, while a Filter does
///  not. The difference between a Filter and a Sink is a Filter allows an
///  attached transformation, while a Sink does not.
/// \details A Sink doesnot produce any retrievable output.
/// \details See the discussion of BufferedTransformation in cryptlib.h for
///  more details.
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE Sink : public BufferedTransformation
{
public:
	size_t TransferTo2(BufferedTransformation &target, lword &transferBytes, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true)
		{CRYPTOPP_UNUSED(target); CRYPTOPP_UNUSED(transferBytes); CRYPTOPP_UNUSED(channel); CRYPTOPP_UNUSED(blocking); transferBytes = 0; return 0;}
	size_t CopyRangeTo2(BufferedTransformation &target, lword &begin, lword end=LWORD_MAX, const std::string &channel=DEFAULT_CHANNEL, bool blocking=true) const
		{CRYPTOPP_UNUSED(target); CRYPTOPP_UNUSED(begin); CRYPTOPP_UNUSED(end); CRYPTOPP_UNUSED(channel); CRYPTOPP_UNUSED(blocking); return 0;}
};

/// \brief Acts as an input discarding Filter or Sink
/// \details The BitBucket discards all input and returns 0 to the caller
///  to indicate all data was processed.
class CRYPTOPP_DLL BitBucket : public Bufferless<Sink>
{
public:
	std::string AlgorithmName() const {return "BitBucket";}
	void IsolatedInitialize(const NameValuePairs &params)
		{CRYPTOPP_UNUSED(params);}
	size_t Put2(const byte *inString, size_t length, int messageEnd, bool blocking)
		{CRYPTOPP_UNUSED(inString); CRYPTOPP_UNUSED(length); CRYPTOPP_UNUSED(messageEnd); CRYPTOPP_UNUSED(blocking); return 0;}
};

NAMESPACE_END

#if CRYPTOPP_MSC_VERSION
# pragma warning(pop)
#endif

#endif
