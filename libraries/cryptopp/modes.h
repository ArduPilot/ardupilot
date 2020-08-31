// modes.h - originally written and placed in the public domain by Wei Dai

/// \file modes.h
/// \brief Classes for block cipher modes of operation

#ifndef CRYPTOPP_MODES_H
#define CRYPTOPP_MODES_H

#include "cryptlib.h"
#include "secblock.h"
#include "misc.h"
#include "strciphr.h"
#include "argnames.h"
#include "algparam.h"

// Issue 340
#if CRYPTOPP_GCC_DIAGNOSTIC_AVAILABLE
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wconversion"
# pragma GCC diagnostic ignored "-Wsign-conversion"
#endif

#if CRYPTOPP_MSC_VERSION
# pragma warning(push)
# pragma warning(disable: 4231 4275)
# if (CRYPTOPP_MSC_VERSION >= 1400)
#  pragma warning(disable: 6011 6386 28193)
# endif
#endif

NAMESPACE_BEGIN(CryptoPP)

/// \brief Block cipher mode of operation information
/// \details Each class derived from this one defines two types, Encryption and Decryption,
///   both of which implement the SymmetricCipher interface.
///   For each mode there are two classes, one of which is a template class,
///   and the other one has a name that ends in "_ExternalCipher".
///   The "external cipher" mode objects hold a reference to the underlying block cipher,
///   instead of holding an instance of it. The reference must be passed in to the constructor.
///   For the "cipher holder" classes, the CIPHER template parameter should be a class
///   derived from BlockCipherDocumentation, for example DES or AES.
/// \details See NIST SP 800-38A for definitions of these modes. See
///   AuthenticatedSymmetricCipherDocumentation for authenticated encryption modes.
struct CipherModeDocumentation : public SymmetricCipherDocumentation
{
};

/// \brief Block cipher mode of operation information
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE CipherModeBase : public SymmetricCipher
{
public:
	virtual ~CipherModeBase() {}

	// Algorithm class
	std::string AlgorithmProvider() const {
		return m_cipher != NULLPTR ? m_cipher->AlgorithmProvider() : "C++";
	}

	/// \brief Returns smallest valid key length
	/// \returns the minimum key length, in bytes
	size_t MinKeyLength() const {return m_cipher->MinKeyLength();}

	/// \brief Returns largest valid key length
	/// \returns the maximum key length, in bytes
	size_t MaxKeyLength() const {return m_cipher->MaxKeyLength();}

	/// \brief Returns default key length
	/// \returns the default key length, in bytes
	size_t DefaultKeyLength() const {return m_cipher->DefaultKeyLength();}

	/// \brief Returns a valid key length for the algorithm
	/// \param keylength the size of the key, in bytes
	/// \returns the valid key length, in bytes
	/// \details keylength is provided in bytes, not bits. If keylength is less than MIN_KEYLENGTH,
	///   then the function returns MIN_KEYLENGTH. If keylength is greater than MAX_KEYLENGTH,
	///   then the function returns MAX_KEYLENGTH. if If keylength is a multiple of KEYLENGTH_MULTIPLE,
	///   then keylength is returned. Otherwise, the function returns a \a lower multiple of
	///   KEYLENGTH_MULTIPLE.
	size_t GetValidKeyLength(size_t keylength) const {return m_cipher->GetValidKeyLength(keylength);}

	/// \brief Returns whether keylength is a valid key length
	/// \param keylength the requested keylength
	/// \return true if keylength is valid, false otherwise
	/// \details Internally the function calls GetValidKeyLength()
	bool IsValidKeyLength(size_t keylength) const {return m_cipher->IsValidKeyLength(keylength);}

	/// \brief Provides input and output data alignment for optimal performance.
	/// \return the input data alignment that provides optimal performance
	/// \sa GetAlignment() and OptimalBlockSize()
	unsigned int OptimalDataAlignment() const {return m_cipher->OptimalDataAlignment();}

	/// \brief Returns length of the IV accepted by this object
	/// \return the size of an IV, in bytes
	/// \throws NotImplemented() if the object does not support resynchronization
	/// \details The default implementation throws NotImplemented
	unsigned int IVSize() const {return BlockSize();}

	/// \brief Minimal requirement for secure IVs
	/// \return the secure IV requirement of the algorithm
	virtual IV_Requirement IVRequirement() const =0;

	/// \brief Set external block cipher
	/// \param cipher An external block cipher
	/// \details The cipher should be keyed.
	void SetCipher(BlockCipher &cipher)
	{
		this->ThrowIfResynchronizable();
		this->m_cipher = &cipher;
		this->ResizeBuffers();
	}

	/// \brief Set external block cipher and IV
	/// \param cipher An external block cipher
	/// \param iv a byte array used to resynchronize the cipher
	/// \param feedbackSize the feedback size, in bytes
	/// \details The cipher should be keyed.
	void SetCipherWithIV(BlockCipher &cipher, const byte *iv, int feedbackSize = 0)
	{
		this->ThrowIfInvalidIV(iv);
		this->m_cipher = &cipher;
		this->ResizeBuffers();
		this->SetFeedbackSize(feedbackSize);
		if (this->IsResynchronizable())
			this->Resynchronize(iv);
	}

protected:
	CipherModeBase() : m_cipher(NULLPTR) {}
	inline unsigned int BlockSize() const
	{
		CRYPTOPP_ASSERT(m_register.size() > 0);
		return static_cast<unsigned int>(m_register.size());
	}
	virtual void SetFeedbackSize(unsigned int feedbackSize)
	{
		if (!(feedbackSize == 0 || feedbackSize == BlockSize()))
			throw InvalidArgument("CipherModeBase: feedback size cannot be specified for this cipher mode");
	}

	virtual void ResizeBuffers();

	BlockCipher *m_cipher;
	SecByteBlock m_register;
};

/// \brief Block cipher mode of operation common operations
/// \tparam POLICY_INTERFACE common operations
template <class POLICY_INTERFACE>
class CRYPTOPP_NO_VTABLE ModePolicyCommonTemplate : public CipherModeBase, public POLICY_INTERFACE
{
	unsigned int GetAlignment() const {return m_cipher->OptimalDataAlignment();}
	void CipherSetKey(const NameValuePairs &params, const byte *key, size_t length);
};

template <class POLICY_INTERFACE>
void ModePolicyCommonTemplate<POLICY_INTERFACE>::CipherSetKey(const NameValuePairs &params, const byte *key, size_t length)
{
	m_cipher->SetKey(key, length, params);
	ResizeBuffers();
	int feedbackSize = params.GetIntValueWithDefault(Name::FeedbackSize(), 0);
	SetFeedbackSize(feedbackSize);
}

/// \brief CFB block cipher mode of operation
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE CFB_ModePolicy : public ModePolicyCommonTemplate<CFB_CipherAbstractPolicy>
{
public:
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "CFB";}

	virtual ~CFB_ModePolicy() {}
	CFB_ModePolicy() : m_feedbackSize(0) {}
	IV_Requirement IVRequirement() const {return RANDOM_IV;}

protected:
	unsigned int GetBytesPerIteration() const {return m_feedbackSize;}
	bool CanIterate() const {return m_feedbackSize == BlockSize();}
	void Iterate(byte *output, const byte *input, CipherDir dir, size_t iterationCount);
	void TransformRegister();
	void CipherResynchronize(const byte *iv, size_t length);
	void SetFeedbackSize(unsigned int feedbackSize);
	void ResizeBuffers();
	byte * GetRegisterBegin();

	SecByteBlock m_temp;
	unsigned int m_feedbackSize;
};

/// \brief Initialize a block of memory
/// \param dest the destination block of memory
/// \param dsize the size of the destination block, in bytes
/// \param src the source block of memory
/// \param ssize the size of the source block, in bytes
/// \details CopyOrZero copies ssize bytes from source to destination if
///   src is not NULL. If src is NULL then dest is zero'd. Bounds are not
///   checked at runtime. Debug builds assert if ssize exceeds dsize.
inline void CopyOrZero(void *dest, size_t dsize, const void *src, size_t ssize)
{
	CRYPTOPP_ASSERT(dest);
	CRYPTOPP_ASSERT(dsize >= ssize);

	if (src != NULLPTR)
		memcpy_s(dest, dsize, src, ssize);
	else
		memset(dest, 0, dsize);
}

/// \brief OFB block cipher mode of operation
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE OFB_ModePolicy : public ModePolicyCommonTemplate<AdditiveCipherAbstractPolicy>
{
public:
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "OFB";}

	bool CipherIsRandomAccess() const {return false;}
	IV_Requirement IVRequirement() const {return UNIQUE_IV;}

protected:
	unsigned int GetBytesPerIteration() const {return BlockSize();}
	unsigned int GetIterationsToBuffer() const {return m_cipher->OptimalNumberOfParallelBlocks();}
	void WriteKeystream(byte *keystreamBuffer, size_t iterationCount);
	void CipherResynchronize(byte *keystreamBuffer, const byte *iv, size_t length);
};

/// \brief CTR block cipher mode of operation
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE CTR_ModePolicy : public ModePolicyCommonTemplate<AdditiveCipherAbstractPolicy>
{
public:
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "CTR";}

	virtual ~CTR_ModePolicy() {}
	bool CipherIsRandomAccess() const {return true;}
	IV_Requirement IVRequirement() const {return RANDOM_IV;}

protected:
	virtual void IncrementCounterBy256();
	unsigned int GetAlignment() const {return m_cipher->OptimalDataAlignment();}
	unsigned int GetBytesPerIteration() const {return BlockSize();}
	unsigned int GetIterationsToBuffer() const {return m_cipher->OptimalNumberOfParallelBlocks();}
	void WriteKeystream(byte *buffer, size_t iterationCount)
		{OperateKeystream(WRITE_KEYSTREAM, buffer, NULLPTR, iterationCount);}
	bool CanOperateKeystream() const {return true;}
	void OperateKeystream(KeystreamOperation operation, byte *output, const byte *input, size_t iterationCount);
	void CipherResynchronize(byte *keystreamBuffer, const byte *iv, size_t length);
	void SeekToIteration(lword iterationCount);

	// adv_simd.h increments the counter
	mutable SecByteBlock m_counterArray;
};

/// \brief Block cipher mode of operation default implementation
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE BlockOrientedCipherModeBase : public CipherModeBase
{
public:
	virtual ~BlockOrientedCipherModeBase() {}
	void UncheckedSetKey(const byte *key, unsigned int length, const NameValuePairs &params);
	unsigned int MandatoryBlockSize() const {return BlockSize();}
	bool IsRandomAccess() const {return false;}
	bool IsSelfInverting() const {return false;}
	bool IsForwardTransformation() const
		{return m_cipher->IsForwardTransformation();}
	void Resynchronize(const byte *iv, int length=-1)
		{memcpy_s(m_register, m_register.size(), iv, ThrowIfInvalidIVLength(length));}

protected:
	bool RequireAlignedInput() const {return true;}
	virtual void ResizeBuffers();

	SecByteBlock m_buffer;
};

/// \brief ECB block cipher mode of operation default implementation
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE ECB_OneWay : public BlockOrientedCipherModeBase
{
public:
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "ECB";}

	void SetKey(const byte *key, size_t length, const NameValuePairs &params = g_nullNameValuePairs)
		{m_cipher->SetKey(key, length, params); BlockOrientedCipherModeBase::ResizeBuffers();}
	IV_Requirement IVRequirement() const {return NOT_RESYNCHRONIZABLE;}
	unsigned int OptimalBlockSize() const {return static_cast<unsigned int>(BlockSize() * m_cipher->OptimalNumberOfParallelBlocks());}
	void ProcessData(byte *outString, const byte *inString, size_t length);
};

/// \brief CBC block cipher mode of operation default implementation
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE CBC_ModeBase : public BlockOrientedCipherModeBase
{
public:
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "CBC";}

	IV_Requirement IVRequirement() const {return UNPREDICTABLE_RANDOM_IV;}
	bool RequireAlignedInput() const {return false;}
	unsigned int MinLastBlockSize() const {return 0;}
};

/// \brief CBC block cipher mode of operation encryption operation
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE CBC_Encryption : public CBC_ModeBase
{
public:
	void ProcessData(byte *outString, const byte *inString, size_t length);
};

/// \brief CBC-CTS block cipher mode of operation encryption operation
/// \since Crypto++ 3.0
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE CBC_CTS_Encryption : public CBC_Encryption
{
public:
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "CBC/CTS";}

	void SetStolenIV(byte *iv) {m_stolenIV = iv;}
	unsigned int MinLastBlockSize() const {return BlockSize()+1;}
	size_t ProcessLastBlock(byte *outString, size_t outLength, const byte *inString, size_t inLength);

protected:
	void UncheckedSetKey(const byte *key, unsigned int length, const NameValuePairs &params)
	{
		CBC_Encryption::UncheckedSetKey(key, length, params);
		m_stolenIV = params.GetValueWithDefault(Name::StolenIV(), static_cast<byte *>(NULLPTR));
	}

	byte *m_stolenIV;
};

/// \brief CBC block cipher mode of operation decryption operation
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE CBC_Decryption : public CBC_ModeBase
{
public:
	virtual ~CBC_Decryption() {}
	void ProcessData(byte *outString, const byte *inString, size_t length);

protected:
	virtual void ResizeBuffers();

	SecByteBlock m_temp;
};

/// \brief CBC-CTS block cipher mode of operation decryption operation
/// \since Crypto++ 3.0
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE CBC_CTS_Decryption : public CBC_Decryption
{
public:
	unsigned int MinLastBlockSize() const {return BlockSize()+1;}
	size_t ProcessLastBlock(byte *outString, size_t outLength, const byte *inString, size_t inLength);
};

/// \brief Block cipher mode of operation aggregate
template <class CIPHER, class BASE>
class CipherModeFinalTemplate_CipherHolder : protected ObjectHolder<CIPHER>, public AlgorithmImpl<BASE, CipherModeFinalTemplate_CipherHolder<CIPHER, BASE> >
{
public:
	/// \brief Provides the name of this algorithm
	/// \return the standard algorithm name
	/// \details The standard algorithm name can be a name like \a AES or \a AES/GCM. Some algorithms
	///   do not have standard names yet. For example, there is no standard algorithm name for
	///   Shoup's ECIES.
	static std::string CRYPTOPP_API StaticAlgorithmName()
		{return CIPHER::StaticAlgorithmName() + "/" + BASE::StaticAlgorithmName();}

	/// \brief Construct a CipherModeFinalTemplate
	CipherModeFinalTemplate_CipherHolder()
	{
		this->m_cipher = &this->m_object;
		this->ResizeBuffers();
	}

	/// \brief Construct a CipherModeFinalTemplate
	/// \param key a byte array used to key the cipher
	/// \param length size of the key in bytes
	/// \details key must be at least DEFAULT_KEYLENGTH in length. Internally, the function calls
	///    SimpleKeyingInterface::SetKey.
	CipherModeFinalTemplate_CipherHolder(const byte *key, size_t length)
	{
		this->m_cipher = &this->m_object;
		this->SetKey(key, length);
	}

	/// \brief Construct a CipherModeFinalTemplate
	/// \param key a byte array used to key the cipher
	/// \param length size of the key in bytes
	/// \param iv a byte array used to resynchronize the cipher
	/// \details key must be at least DEFAULT_KEYLENGTH in length. iv must be IVSize() or
	///    BLOCKSIZE in length. Internally, the function calls SimpleKeyingInterface::SetKey.
	CipherModeFinalTemplate_CipherHolder(const byte *key, size_t length, const byte *iv)
	{
		this->m_cipher = &this->m_object;
		this->SetKey(key, length, MakeParameters(Name::IV(), ConstByteArrayParameter(iv, this->m_cipher->BlockSize())));
	}

	/// \brief Construct a CipherModeFinalTemplate
	/// \param key a byte array used to key the cipher
	/// \param length size of the key in bytes
	/// \param iv a byte array used to resynchronize the cipher
	/// \param feedbackSize the feedback size, in bytes
	/// \details key must be at least DEFAULT_KEYLENGTH in length. iv must be IVSize() or
	///    BLOCKSIZE in length. Internally, the function calls SimpleKeyingInterface::SetKey.
	CipherModeFinalTemplate_CipherHolder(const byte *key, size_t length, const byte *iv, int feedbackSize)
	{
		this->m_cipher = &this->m_object;
		this->SetKey(key, length, MakeParameters(Name::IV(), ConstByteArrayParameter(iv, this->m_cipher->BlockSize()))(Name::FeedbackSize(), feedbackSize));
	}

	// Algorithm class
	std::string AlgorithmProvider() const {
		return this->m_cipher->AlgorithmProvider();
	}
};

/// \tparam BASE CipherModeFinalTemplate_CipherHolder base class
/// \details Base class for external mode cipher combinations
template <class BASE>
class CipherModeFinalTemplate_ExternalCipher : public BASE
{
public:
	/// \brief Construct a default CipherModeFinalTemplate
	/// \details The cipher is not keyed.
	CipherModeFinalTemplate_ExternalCipher() {}

	/// \brief Construct a CipherModeFinalTemplate
	/// \param cipher An external block cipher
	/// \details The cipher should be keyed.
	CipherModeFinalTemplate_ExternalCipher(BlockCipher &cipher)
		{this->SetCipher(cipher);}

	/// \brief Construct a CipherModeFinalTemplate
	/// \param cipher An external block cipher
	/// \param iv a byte array used to resynchronize the cipher
	/// \param feedbackSize the feedback size, in bytes
	/// \details The cipher should be keyed.
	CipherModeFinalTemplate_ExternalCipher(BlockCipher &cipher, const byte *iv, int feedbackSize = 0)
		{this->SetCipherWithIV(cipher, iv, feedbackSize);}

	/// \brief Provides the name of this algorithm
	/// \return the standard algorithm name
	/// \details The standard algorithm name can be a name like \a AES or \a AES/GCM. Some algorithms
	///   do not have standard names yet. For example, there is no standard algorithm name for
	///   Shoup's ECIES.
	/// \note  AlgorithmName is not universally implemented yet
	std::string AlgorithmName() const
		{return (this->m_cipher ? this->m_cipher->AlgorithmName() + "/" : std::string("")) + BASE::StaticAlgorithmName();}

	// Algorithm class
	std::string AlgorithmProvider() const
		{return this->m_cipher->AlgorithmProvider();}
};

CRYPTOPP_DLL_TEMPLATE_CLASS CFB_CipherTemplate<AbstractPolicyHolder<CFB_CipherAbstractPolicy, CFB_ModePolicy> >;
CRYPTOPP_DLL_TEMPLATE_CLASS CFB_EncryptionTemplate<AbstractPolicyHolder<CFB_CipherAbstractPolicy, CFB_ModePolicy> >;
CRYPTOPP_DLL_TEMPLATE_CLASS CFB_DecryptionTemplate<AbstractPolicyHolder<CFB_CipherAbstractPolicy, CFB_ModePolicy> >;

/// \brief CFB block cipher mode of operation
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
template <class CIPHER>
struct CFB_Mode : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Encryption, ConcretePolicyHolder<Empty, CFB_EncryptionTemplate<AbstractPolicyHolder<CFB_CipherAbstractPolicy, CFB_ModePolicy> > > > Encryption;
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Encryption, ConcretePolicyHolder<Empty, CFB_DecryptionTemplate<AbstractPolicyHolder<CFB_CipherAbstractPolicy, CFB_ModePolicy> > > > Decryption;
};

/// \brief CFB mode, external cipher.
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
struct CFB_Mode_ExternalCipher : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_ExternalCipher<ConcretePolicyHolder<Empty, CFB_EncryptionTemplate<AbstractPolicyHolder<CFB_CipherAbstractPolicy, CFB_ModePolicy> > > > Encryption;
	typedef CipherModeFinalTemplate_ExternalCipher<ConcretePolicyHolder<Empty, CFB_DecryptionTemplate<AbstractPolicyHolder<CFB_CipherAbstractPolicy, CFB_ModePolicy> > > > Decryption;
};

/// \brief CFB block cipher mode of operation providing FIPS validated cryptography.
/// \details Requires full block plaintext according to FIPS 800-38A
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
template <class CIPHER>
struct CFB_FIPS_Mode : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Encryption, ConcretePolicyHolder<Empty, CFB_RequireFullDataBlocks<CFB_EncryptionTemplate<AbstractPolicyHolder<CFB_CipherAbstractPolicy, CFB_ModePolicy> > > > > Encryption;
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Encryption, ConcretePolicyHolder<Empty, CFB_RequireFullDataBlocks<CFB_DecryptionTemplate<AbstractPolicyHolder<CFB_CipherAbstractPolicy, CFB_ModePolicy> > > > > Decryption;
};

/// \brief CFB mode, external cipher, providing FIPS validated cryptography.
/// \details Requires full block plaintext according to FIPS 800-38A
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
struct CFB_FIPS_Mode_ExternalCipher : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_ExternalCipher<ConcretePolicyHolder<Empty, CFB_RequireFullDataBlocks<CFB_EncryptionTemplate<AbstractPolicyHolder<CFB_CipherAbstractPolicy, CFB_ModePolicy> > > > > Encryption;
	typedef CipherModeFinalTemplate_ExternalCipher<ConcretePolicyHolder<Empty, CFB_RequireFullDataBlocks<CFB_DecryptionTemplate<AbstractPolicyHolder<CFB_CipherAbstractPolicy, CFB_ModePolicy> > > > > Decryption;
};

CRYPTOPP_DLL_TEMPLATE_CLASS AdditiveCipherTemplate<AbstractPolicyHolder<AdditiveCipherAbstractPolicy, OFB_ModePolicy> >;

/// \brief OFB block cipher mode of operation
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
template <class CIPHER>
struct OFB_Mode : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Encryption, ConcretePolicyHolder<Empty, AdditiveCipherTemplate<AbstractPolicyHolder<AdditiveCipherAbstractPolicy, OFB_ModePolicy> > > > Encryption;
	typedef Encryption Decryption;
};

/// \brief OFB mode, external cipher.
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
struct OFB_Mode_ExternalCipher : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_ExternalCipher<ConcretePolicyHolder<Empty, AdditiveCipherTemplate<AbstractPolicyHolder<AdditiveCipherAbstractPolicy, OFB_ModePolicy> > > > Encryption;
	typedef Encryption Decryption;
};

CRYPTOPP_DLL_TEMPLATE_CLASS AdditiveCipherTemplate<AbstractPolicyHolder<AdditiveCipherAbstractPolicy, CTR_ModePolicy> >;
CRYPTOPP_DLL_TEMPLATE_CLASS CipherModeFinalTemplate_ExternalCipher<ConcretePolicyHolder<Empty, AdditiveCipherTemplate<AbstractPolicyHolder<AdditiveCipherAbstractPolicy, CTR_ModePolicy> > > >;

/// \brief CTR block cipher mode of operation
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
template <class CIPHER>
struct CTR_Mode : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Encryption, ConcretePolicyHolder<Empty, AdditiveCipherTemplate<AbstractPolicyHolder<AdditiveCipherAbstractPolicy, CTR_ModePolicy> > > > Encryption;
	typedef Encryption Decryption;
};

/// \brief CTR mode, external cipher.
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
struct CTR_Mode_ExternalCipher : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_ExternalCipher<ConcretePolicyHolder<Empty, AdditiveCipherTemplate<AbstractPolicyHolder<AdditiveCipherAbstractPolicy, CTR_ModePolicy> > > > Encryption;
	typedef Encryption Decryption;
};

/// \brief ECB block cipher mode of operation
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
template <class CIPHER>
struct ECB_Mode : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Encryption, ECB_OneWay> Encryption;
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Decryption, ECB_OneWay> Decryption;
};

CRYPTOPP_DLL_TEMPLATE_CLASS CipherModeFinalTemplate_ExternalCipher<ECB_OneWay>;

/// \brief ECB mode, external cipher.
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
struct ECB_Mode_ExternalCipher : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_ExternalCipher<ECB_OneWay> Encryption;
	typedef Encryption Decryption;
};

/// \brief CBC block cipher mode of operation
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
template <class CIPHER>
struct CBC_Mode : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Encryption, CBC_Encryption> Encryption;
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Decryption, CBC_Decryption> Decryption;
};

CRYPTOPP_DLL_TEMPLATE_CLASS CipherModeFinalTemplate_ExternalCipher<CBC_Encryption>;
CRYPTOPP_DLL_TEMPLATE_CLASS CipherModeFinalTemplate_ExternalCipher<CBC_Decryption>;

/// \brief CBC mode, external cipher
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
struct CBC_Mode_ExternalCipher : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_ExternalCipher<CBC_Encryption> Encryption;
	typedef CipherModeFinalTemplate_ExternalCipher<CBC_Decryption> Decryption;
};

/// \brief CBC-CTS block cipher mode of operation
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
/// \since Crypto++ 3.0
template <class CIPHER>
struct CBC_CTS_Mode : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Encryption, CBC_CTS_Encryption> Encryption;
	typedef CipherModeFinalTemplate_CipherHolder<typename CIPHER::Decryption, CBC_CTS_Decryption> Decryption;
};

CRYPTOPP_DLL_TEMPLATE_CLASS CipherModeFinalTemplate_ExternalCipher<CBC_CTS_Encryption>;
CRYPTOPP_DLL_TEMPLATE_CLASS CipherModeFinalTemplate_ExternalCipher<CBC_CTS_Decryption>;

/// \brief CBC mode with ciphertext stealing, external cipher
/// \sa <A HREF="http://www.cryptopp.com/wiki/Modes_of_Operation">Modes of Operation</A>
///   on the Crypto++ wiki.
/// \since Crypto++ 3.0
struct CBC_CTS_Mode_ExternalCipher : public CipherModeDocumentation
{
	typedef CipherModeFinalTemplate_ExternalCipher<CBC_CTS_Encryption> Encryption;
	typedef CipherModeFinalTemplate_ExternalCipher<CBC_CTS_Decryption> Decryption;
};

NAMESPACE_END

// Issue 340
#if CRYPTOPP_MSC_VERSION
# pragma warning(pop)
#endif

#if CRYPTOPP_GCC_DIAGNOSTIC_AVAILABLE
# pragma GCC diagnostic pop
#endif

#endif
