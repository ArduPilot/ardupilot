// pubkey.h - originally written and placed in the public domain by Wei Dai

/// \file pubkey.h
/// \brief This file contains helper classes/functions for implementing public key algorithms.
/// \details The class hierachies in this header file tend to look like this:
///
/// <pre>
///                   x1
///                  +--+
///                  |  |
///                 y1  z1
///                  |  |
///             x2<y1>  x2<z1>
///                  |  |
///                 y2  z2
///                  |  |
///             x3<y2>  x3<z2>
///                  |  |
///                 y3  z3
/// </pre>
///
/// <ul>
///   <li>x1, y1, z1 are abstract interface classes defined in cryptlib.h
///   <li>x2, y2, z2 are implementations of the interfaces using "abstract policies", which
/// 	  are pure virtual functions that should return interfaces to interchangeable algorithms.
/// 	  These classes have Base suffixes.
///   <li>x3, y3, z3 hold actual algorithms and implement those virtual functions.
/// 	  These classes have Impl suffixes.
/// </ul>
///
/// \details The TF_ prefix means an implementation using trapdoor functions on integers.
/// \details The DL_ prefix means an implementation using group operations in groups where discrete log is hard.

#ifndef CRYPTOPP_PUBKEY_H
#define CRYPTOPP_PUBKEY_H

#include "config.h"

#if CRYPTOPP_MSC_VERSION
# pragma warning(push)
# pragma warning(disable: 4702)
#endif

#include "cryptlib.h"
#include "integer.h"
#include "algebra.h"
#include "modarith.h"
#include "filters.h"
#include "eprecomp.h"
#include "fips140.h"
#include "argnames.h"
#include "smartptr.h"
#include "stdcpp.h"

#if defined(__SUNPRO_CC)
# define MAYBE_RETURN(x) return x
#else
# define MAYBE_RETURN(x) CRYPTOPP_UNUSED(x)
#endif

NAMESPACE_BEGIN(CryptoPP)

/// \brief Provides range for plaintext and ciphertext lengths
/// \details A trapdoor function is a function that is easy to compute in one direction,
///   but difficult to compute in the opposite direction without special knowledge.
///   The special knowledge is usually the private key.
/// \details Trapdoor functions only handle messages of a limited length or size.
///   MaxPreimage is the plaintext's maximum length, and MaxImage is the
///   ciphertext's maximum length.
/// \sa TrapdoorFunctionBounds(), RandomizedTrapdoorFunction(), TrapdoorFunction(),
///   RandomizedTrapdoorFunctionInverse() and TrapdoorFunctionInverse()
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE TrapdoorFunctionBounds
{
public:
	virtual ~TrapdoorFunctionBounds() {}

	/// \brief Returns the maximum size of a message before the trapdoor function is applied
	/// \returns the maximum size of a message before the trapdoor function is applied
	/// \details Derived classes must implement PreimageBound().
	virtual Integer PreimageBound() const =0;
	/// \brief Returns the maximum size of a message after the trapdoor function is applied
	/// \returns the maximum size of a message after the trapdoor function is applied
	/// \details Derived classes must implement ImageBound().
	virtual Integer ImageBound() const =0;
	/// \brief Returns the maximum size of a message before the trapdoor function is applied bound to a public key
	/// \returns the maximum size of a message before the trapdoor function is applied bound to a public key
	/// \details The default implementation returns <tt>PreimageBound() - 1</tt>.
	virtual Integer MaxPreimage() const {return --PreimageBound();}
	/// \brief Returns the maximum size of a message after the trapdoor function is applied bound to a public key
	/// \returns the the maximum size of a message after the trapdoor function is applied bound to a public key
	/// \details The default implementation returns <tt>ImageBound() - 1</tt>.
	virtual Integer MaxImage() const {return --ImageBound();}
};

/// \brief Applies the trapdoor function, using random data if required
/// \details ApplyFunction() is the foundation for encrypting a message under a public key.
///   Derived classes will override it at some point.
/// \sa TrapdoorFunctionBounds(), RandomizedTrapdoorFunction(), TrapdoorFunction(),
///   RandomizedTrapdoorFunctionInverse() and TrapdoorFunctionInverse()
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE RandomizedTrapdoorFunction : public TrapdoorFunctionBounds
{
public:
	virtual ~RandomizedTrapdoorFunction() {}

	/// \brief Applies the trapdoor function, using random data if required
	/// \param rng a RandomNumberGenerator derived class
	/// \param x the message on which the encryption function is applied
	/// \returns the message x encrypted under the public key
	/// \details ApplyRandomizedFunction is a generalization of encryption under a public key
	///    cryptosystem. The RandomNumberGenerator may (or may not) be required.
	///    Derived classes must implement it.
	virtual Integer ApplyRandomizedFunction(RandomNumberGenerator &rng, const Integer &x) const =0;

	/// \brief Determines if the encryption algorithm is randomized
	/// \returns true if the encryption algorithm is randomized, false otherwise
	/// \details If IsRandomized() returns false, then NullRNG() can be used.
	virtual bool IsRandomized() const {return true;}
};

/// \brief Applies the trapdoor function
/// \details ApplyFunction() is the foundation for encrypting a message under a public key.
///    Derived classes will override it at some point.
/// \sa TrapdoorFunctionBounds(), RandomizedTrapdoorFunction(), TrapdoorFunction(),
///   RandomizedTrapdoorFunctionInverse() and TrapdoorFunctionInverse()
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE TrapdoorFunction : public RandomizedTrapdoorFunction
{
public:
	virtual ~TrapdoorFunction() {}

	/// \brief Applies the trapdoor function
	/// \param rng a RandomNumberGenerator derived class
	/// \param x the message on which the encryption function is applied
	/// \details ApplyRandomizedFunction is a generalization of encryption under a public key
	///    cryptosystem. The RandomNumberGenerator may (or may not) be required.
	/// \details Internally, ApplyRandomizedFunction() calls ApplyFunction() \a
	///   without the RandomNumberGenerator.
	Integer ApplyRandomizedFunction(RandomNumberGenerator &rng, const Integer &x) const
		{CRYPTOPP_UNUSED(rng); return ApplyFunction(x);}
	bool IsRandomized() const {return false;}

	/// \brief Applies the trapdoor
	/// \param x the message on which the encryption function is applied
	/// \returns the message x encrypted under the public key
	/// \details ApplyFunction is a generalization of encryption under a public key
	///    cryptosystem. Derived classes must implement it.
	virtual Integer ApplyFunction(const Integer &x) const =0;
};

/// \brief Applies the inverse of the trapdoor function, using random data if required
/// \details CalculateInverse() is the foundation for decrypting a message under a private key
///   in a public key cryptosystem. Derived classes will override it at some point.
/// \sa TrapdoorFunctionBounds(), RandomizedTrapdoorFunction(), TrapdoorFunction(),
///   RandomizedTrapdoorFunctionInverse() and TrapdoorFunctionInverse()
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE RandomizedTrapdoorFunctionInverse
{
public:
	virtual ~RandomizedTrapdoorFunctionInverse() {}

	/// \brief Applies the inverse of the trapdoor function, using random data if required
	/// \param rng a RandomNumberGenerator derived class
	/// \param x the message on which the decryption function is applied
	/// \returns the message x decrypted under the private key
	/// \details CalculateRandomizedInverse is a generalization of decryption using the private key
	///    The RandomNumberGenerator may (or may not) be required. Derived classes must implement it.
	virtual Integer CalculateRandomizedInverse(RandomNumberGenerator &rng, const Integer &x) const =0;

	/// \brief Determines if the decryption algorithm is randomized
	/// \returns true if the decryption algorithm is randomized, false otherwise
	/// \details If IsRandomized() returns false, then NullRNG() can be used.
	virtual bool IsRandomized() const {return true;}
};

/// \brief Applies the inverse of the trapdoor function
/// \details CalculateInverse() is the foundation for decrypting a message under a private key
///   in a public key cryptosystem. Derived classes will override it at some point.
/// \sa TrapdoorFunctionBounds(), RandomizedTrapdoorFunction(), TrapdoorFunction(),
///   RandomizedTrapdoorFunctionInverse() and TrapdoorFunctionInverse()
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE TrapdoorFunctionInverse : public RandomizedTrapdoorFunctionInverse
{
public:
	virtual ~TrapdoorFunctionInverse() {}

	/// \brief Applies the inverse of the trapdoor function
	/// \param rng a RandomNumberGenerator derived class
	/// \param x the message on which the decryption function is applied
	/// \returns the message x decrypted under the private key
	/// \details CalculateRandomizedInverse is a generalization of decryption using the private key
	/// \details Internally, CalculateRandomizedInverse() calls CalculateInverse() \a
	///   without the RandomNumberGenerator.
	Integer CalculateRandomizedInverse(RandomNumberGenerator &rng, const Integer &x) const
		{return CalculateInverse(rng, x);}

	/// \brief Determines if the decryption algorithm is randomized
	/// \returns true if the decryption algorithm is randomized, false otherwise
	/// \details If IsRandomized() returns false, then NullRNG() can be used.
	bool IsRandomized() const {return false;}

	/// \brief Calculates the inverse of an element
	/// \param rng a RandomNumberGenerator derived class
	/// \param x the element
	/// \returns the inverse of the element in the group
	virtual Integer CalculateInverse(RandomNumberGenerator &rng, const Integer &x) const =0;
};

// ********************************************************

/// \brief Message encoding method for public key encryption
class CRYPTOPP_NO_VTABLE PK_EncryptionMessageEncodingMethod
{
public:
	virtual ~PK_EncryptionMessageEncodingMethod() {}

	virtual bool ParameterSupported(const char *name) const
		{CRYPTOPP_UNUSED(name); return false;}

	/// max size of unpadded message in bytes, given max size of padded message in bits (1 less than size of modulus)
	virtual size_t MaxUnpaddedLength(size_t paddedLength) const =0;

	virtual void Pad(RandomNumberGenerator &rng, const byte *raw, size_t inputLength, byte *padded, size_t paddedBitLength, const NameValuePairs &parameters) const =0;

	virtual DecodingResult Unpad(const byte *padded, size_t paddedBitLength, byte *raw, const NameValuePairs &parameters) const =0;
};

// ********************************************************

/// \brief The base for trapdoor based cryptosystems
/// \tparam TFI trapdoor function interface derived class
/// \tparam MEI message encoding interface derived class
template <class TFI, class MEI>
class CRYPTOPP_NO_VTABLE TF_Base
{
protected:
	virtual ~TF_Base() {}

	virtual const TrapdoorFunctionBounds & GetTrapdoorFunctionBounds() const =0;

	typedef TFI TrapdoorFunctionInterface;
	virtual const TrapdoorFunctionInterface & GetTrapdoorFunctionInterface() const =0;

	typedef MEI MessageEncodingInterface;
	virtual const MessageEncodingInterface & GetMessageEncodingInterface() const =0;
};

// ********************************************************

/// \brief Public key trapdoor function default implementation
/// \tparam BASE public key cryptosystem with a fixed length
template <class BASE>
class CRYPTOPP_NO_VTABLE PK_FixedLengthCryptoSystemImpl : public BASE
{
public:
	virtual ~PK_FixedLengthCryptoSystemImpl() {}

	size_t MaxPlaintextLength(size_t ciphertextLength) const
		{return ciphertextLength == FixedCiphertextLength() ? FixedMaxPlaintextLength() : 0;}
	size_t CiphertextLength(size_t plaintextLength) const
		{return plaintextLength <= FixedMaxPlaintextLength() ? FixedCiphertextLength() : 0;}

	virtual size_t FixedMaxPlaintextLength() const =0;
	virtual size_t FixedCiphertextLength() const =0;
};

/// \brief Trapdoor function cryptosystem base class
/// \tparam INTFACE public key cryptosystem base interface
/// \tparam BASE public key cryptosystem implementation base
template <class INTFACE, class BASE>
class CRYPTOPP_NO_VTABLE TF_CryptoSystemBase : public PK_FixedLengthCryptoSystemImpl<INTFACE>, protected BASE
{
public:
	virtual ~TF_CryptoSystemBase() {}

	bool ParameterSupported(const char *name) const {return this->GetMessageEncodingInterface().ParameterSupported(name);}
	size_t FixedMaxPlaintextLength() const {return this->GetMessageEncodingInterface().MaxUnpaddedLength(PaddedBlockBitLength());}
	size_t FixedCiphertextLength() const {return this->GetTrapdoorFunctionBounds().MaxImage().ByteCount();}

protected:
	size_t PaddedBlockByteLength() const {return BitsToBytes(PaddedBlockBitLength());}
	// Coverity finding on potential overflow/underflow.
	size_t PaddedBlockBitLength() const {return SaturatingSubtract(this->GetTrapdoorFunctionBounds().PreimageBound().BitCount(),1U);}
};

/// \brief Trapdoor function cryptosystems decryption base class
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE TF_DecryptorBase : public TF_CryptoSystemBase<PK_Decryptor, TF_Base<TrapdoorFunctionInverse, PK_EncryptionMessageEncodingMethod> >
{
public:
	virtual ~TF_DecryptorBase() {}

	DecodingResult Decrypt(RandomNumberGenerator &rng, const byte *ciphertext, size_t ciphertextLength, byte *plaintext, const NameValuePairs &parameters = g_nullNameValuePairs) const;
};

/// \brief Trapdoor function cryptosystems encryption base class
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE TF_EncryptorBase : public TF_CryptoSystemBase<PK_Encryptor, TF_Base<RandomizedTrapdoorFunction, PK_EncryptionMessageEncodingMethod> >
{
public:
	virtual ~TF_EncryptorBase() {}

	void Encrypt(RandomNumberGenerator &rng, const byte *plaintext, size_t plaintextLength, byte *ciphertext, const NameValuePairs &parameters = g_nullNameValuePairs) const;
};

// ********************************************************

// Typedef change due to Clang, http://github.com/weidai11/cryptopp/issues/300
typedef std::pair<const byte *, unsigned int> HashIdentifier;

/// \brief Interface for message encoding method for public key signature schemes.
/// \details PK_SignatureMessageEncodingMethod provides interfaces for message
///   encoding method for public key signature schemes. The methods support both
///   trapdoor functions (<tt>TF_*</tt>) and discrete logarithm (<tt>DL_*</tt>)
///   based schemes.
class CRYPTOPP_NO_VTABLE PK_SignatureMessageEncodingMethod
{
public:
	virtual ~PK_SignatureMessageEncodingMethod() {}

	virtual size_t MinRepresentativeBitLength(size_t hashIdentifierLength, size_t digestLength) const
		{CRYPTOPP_UNUSED(hashIdentifierLength); CRYPTOPP_UNUSED(digestLength); return 0;}
	virtual size_t MaxRecoverableLength(size_t representativeBitLength, size_t hashIdentifierLength, size_t digestLength) const
		{CRYPTOPP_UNUSED(representativeBitLength); CRYPTOPP_UNUSED(representativeBitLength); CRYPTOPP_UNUSED(hashIdentifierLength); CRYPTOPP_UNUSED(digestLength); return 0;}

	/// \brief Determines whether an encoding method requires a random number generator
	/// \return true if the encoding method requires a RandomNumberGenerator()
	/// \details if IsProbabilistic() returns false, then NullRNG() can be passed to functions that take
	///   RandomNumberGenerator().
	/// \sa Bellare and Rogaway<a href="http://grouper.ieee.org/groups/1363/P1363a/contributions/pss-submission.pdf">PSS:
	///   Provably Secure Encoding Method for Digital Signatures</a>
	bool IsProbabilistic() const
		{return true;}
	bool AllowNonrecoverablePart() const
		{throw NotImplemented("PK_MessageEncodingMethod: this signature scheme does not support message recovery");}
	virtual bool RecoverablePartFirst() const
		{throw NotImplemented("PK_MessageEncodingMethod: this signature scheme does not support message recovery");}

	// for verification, DL
	virtual void ProcessSemisignature(HashTransformation &hash, const byte *semisignature, size_t semisignatureLength) const
		{CRYPTOPP_UNUSED(hash); CRYPTOPP_UNUSED(semisignature); CRYPTOPP_UNUSED(semisignatureLength);}

	// for signature
	virtual void ProcessRecoverableMessage(HashTransformation &hash,
		const byte *recoverableMessage, size_t recoverableMessageLength,
		const byte *presignature, size_t presignatureLength,
		SecByteBlock &semisignature) const
	{
		CRYPTOPP_UNUSED(hash);CRYPTOPP_UNUSED(recoverableMessage); CRYPTOPP_UNUSED(recoverableMessageLength);
		CRYPTOPP_UNUSED(presignature); CRYPTOPP_UNUSED(presignatureLength); CRYPTOPP_UNUSED(semisignature);
		if (RecoverablePartFirst())
			CRYPTOPP_ASSERT(!"ProcessRecoverableMessage() not implemented");
	}

	virtual void ComputeMessageRepresentative(RandomNumberGenerator &rng,
		const byte *recoverableMessage, size_t recoverableMessageLength,
		HashTransformation &hash, HashIdentifier hashIdentifier, bool messageEmpty,
		byte *representative, size_t representativeBitLength) const =0;

	virtual bool VerifyMessageRepresentative(
		HashTransformation &hash, HashIdentifier hashIdentifier, bool messageEmpty,
		byte *representative, size_t representativeBitLength) const =0;

	virtual DecodingResult RecoverMessageFromRepresentative(	// for TF
		HashTransformation &hash, HashIdentifier hashIdentifier, bool messageEmpty,
		byte *representative, size_t representativeBitLength,
		byte *recoveredMessage) const
		{CRYPTOPP_UNUSED(hash);CRYPTOPP_UNUSED(hashIdentifier); CRYPTOPP_UNUSED(messageEmpty);
		CRYPTOPP_UNUSED(representative); CRYPTOPP_UNUSED(representativeBitLength); CRYPTOPP_UNUSED(recoveredMessage);
		throw NotImplemented("PK_MessageEncodingMethod: this signature scheme does not support message recovery");}

	virtual DecodingResult RecoverMessageFromSemisignature(		// for DL
		HashTransformation &hash, HashIdentifier hashIdentifier,
		const byte *presignature, size_t presignatureLength,
		const byte *semisignature, size_t semisignatureLength,
		byte *recoveredMessage) const
		{CRYPTOPP_UNUSED(hash);CRYPTOPP_UNUSED(hashIdentifier); CRYPTOPP_UNUSED(presignature); CRYPTOPP_UNUSED(presignatureLength);
		CRYPTOPP_UNUSED(semisignature); CRYPTOPP_UNUSED(semisignatureLength); CRYPTOPP_UNUSED(recoveredMessage);
		throw NotImplemented("PK_MessageEncodingMethod: this signature scheme does not support message recovery");}

	// VC60 workaround
	struct HashIdentifierLookup
	{
		template <class H> struct HashIdentifierLookup2
		{
			static HashIdentifier CRYPTOPP_API Lookup()
			{
				return HashIdentifier(static_cast<const byte *>(NULLPTR), 0);
			}
		};
	};
};

/// \brief Interface for message encoding method for public key signature schemes.
/// \details PK_DeterministicSignatureMessageEncodingMethod provides interfaces
///   for message encoding method for public key signature schemes.
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE PK_DeterministicSignatureMessageEncodingMethod : public PK_SignatureMessageEncodingMethod
{
public:
	bool VerifyMessageRepresentative(
		HashTransformation &hash, HashIdentifier hashIdentifier, bool messageEmpty,
		byte *representative, size_t representativeBitLength) const;
};

/// \brief Interface for message encoding method for public key signature schemes.
/// \details PK_RecoverableSignatureMessageEncodingMethod provides interfaces
///   for message encoding method for public key signature schemes.
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE PK_RecoverableSignatureMessageEncodingMethod : public PK_SignatureMessageEncodingMethod
{
public:
	bool VerifyMessageRepresentative(
		HashTransformation &hash, HashIdentifier hashIdentifier, bool messageEmpty,
		byte *representative, size_t representativeBitLength) const;
};

/// \brief Interface for message encoding method for public key signature schemes.
/// \details DL_SignatureMessageEncodingMethod_DSA provides interfaces
///   for message encoding method for DSA.
class CRYPTOPP_DLL DL_SignatureMessageEncodingMethod_DSA : public PK_DeterministicSignatureMessageEncodingMethod
{
public:
	void ComputeMessageRepresentative(RandomNumberGenerator &rng,
		const byte *recoverableMessage, size_t recoverableMessageLength,
		HashTransformation &hash, HashIdentifier hashIdentifier, bool messageEmpty,
		byte *representative, size_t representativeBitLength) const;
};

/// \brief Interface for message encoding method for public key signature schemes.
/// \details DL_SignatureMessageEncodingMethod_NR provides interfaces
///   for message encoding method for Nyberg-Rueppel.
class CRYPTOPP_DLL DL_SignatureMessageEncodingMethod_NR : public PK_DeterministicSignatureMessageEncodingMethod
{
public:
	void ComputeMessageRepresentative(RandomNumberGenerator &rng,
		const byte *recoverableMessage, size_t recoverableMessageLength,
		HashTransformation &hash, HashIdentifier hashIdentifier, bool messageEmpty,
		byte *representative, size_t representativeBitLength) const;
};

#if 0
/// \brief Interface for message encoding method for public key signature schemes.
/// \details DL_SignatureMessageEncodingMethod_SM2 provides interfaces
///   for message encoding method for SM2.
class CRYPTOPP_DLL DL_SignatureMessageEncodingMethod_SM2 : public PK_DeterministicSignatureMessageEncodingMethod
{
public:
	void ComputeMessageRepresentative(RandomNumberGenerator &rng,
		const byte *recoverableMessage, size_t recoverableMessageLength,
		HashTransformation &hash, HashIdentifier hashIdentifier, bool messageEmpty,
		byte *representative, size_t representativeBitLength) const;
};
#endif

/// \brief Interface for message encoding method for public key signature schemes.
/// \details PK_MessageAccumulatorBase provides interfaces
///   for message encoding method.
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE PK_MessageAccumulatorBase : public PK_MessageAccumulator
{
public:
	PK_MessageAccumulatorBase() : m_empty(true) {}

	virtual HashTransformation & AccessHash() =0;

	void Update(const byte *input, size_t length)
	{
		AccessHash().Update(input, length);
		m_empty = m_empty && length == 0;
	}

	SecByteBlock m_recoverableMessage, m_representative, m_presignature, m_semisignature;
	Integer m_k, m_s;
	bool m_empty;
};

/// \brief Interface for message encoding method for public key signature schemes.
/// \details PK_MessageAccumulatorBase provides interfaces
///   for message encoding method.
template <class HASH_ALGORITHM>
class PK_MessageAccumulatorImpl : public PK_MessageAccumulatorBase, protected ObjectHolder<HASH_ALGORITHM>
{
public:
	HashTransformation & AccessHash() {return this->m_object;}
};

/// \brief Trapdoor Function (TF) Signature Scheme base class
/// \tparam INTFACE interface
/// \tparam BASE base class
template <class INTFACE, class BASE>
class CRYPTOPP_NO_VTABLE TF_SignatureSchemeBase : public INTFACE, protected BASE
{
public:
	virtual ~TF_SignatureSchemeBase() {}

	size_t SignatureLength() const
		{return this->GetTrapdoorFunctionBounds().MaxPreimage().ByteCount();}
	size_t MaxRecoverableLength() const
		{return this->GetMessageEncodingInterface().MaxRecoverableLength(MessageRepresentativeBitLength(), GetHashIdentifier().second, GetDigestSize());}
	size_t MaxRecoverableLengthFromSignatureLength(size_t signatureLength) const
		{CRYPTOPP_UNUSED(signatureLength); return this->MaxRecoverableLength();}

	bool IsProbabilistic() const
		{return this->GetTrapdoorFunctionInterface().IsRandomized() || this->GetMessageEncodingInterface().IsProbabilistic();}
	bool AllowNonrecoverablePart() const
		{return this->GetMessageEncodingInterface().AllowNonrecoverablePart();}
	bool RecoverablePartFirst() const
		{return this->GetMessageEncodingInterface().RecoverablePartFirst();}

protected:
	size_t MessageRepresentativeLength() const {return BitsToBytes(MessageRepresentativeBitLength());}
	// Coverity finding on potential overflow/underflow.
	size_t MessageRepresentativeBitLength() const {return SaturatingSubtract(this->GetTrapdoorFunctionBounds().ImageBound().BitCount(),1U);}
	virtual HashIdentifier GetHashIdentifier() const =0;
	virtual size_t GetDigestSize() const =0;
};

/// \brief Trapdoor Function (TF) Signer base class
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE TF_SignerBase : public TF_SignatureSchemeBase<PK_Signer, TF_Base<RandomizedTrapdoorFunctionInverse, PK_SignatureMessageEncodingMethod> >
{
public:
	virtual ~TF_SignerBase() {}

	void InputRecoverableMessage(PK_MessageAccumulator &messageAccumulator, const byte *recoverableMessage, size_t recoverableMessageLength) const;
	size_t SignAndRestart(RandomNumberGenerator &rng, PK_MessageAccumulator &messageAccumulator, byte *signature, bool restart=true) const;
};

/// \brief Trapdoor Function (TF) Verifier base class
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE TF_VerifierBase : public TF_SignatureSchemeBase<PK_Verifier, TF_Base<TrapdoorFunction, PK_SignatureMessageEncodingMethod> >
{
public:
	virtual ~TF_VerifierBase() {}

	void InputSignature(PK_MessageAccumulator &messageAccumulator, const byte *signature, size_t signatureLength) const;
	bool VerifyAndRestart(PK_MessageAccumulator &messageAccumulator) const;
	DecodingResult RecoverAndRestart(byte *recoveredMessage, PK_MessageAccumulator &recoveryAccumulator) const;
};

// ********************************************************

/// \brief Trapdoor Function (TF) scheme options
/// \tparam T1 algorithm info class
/// \tparam T2 keys class with public and private key
/// \tparam T3 message encoding class
template <class T1, class T2, class T3>
struct TF_CryptoSchemeOptions
{
	typedef T1 AlgorithmInfo;
	typedef T2 Keys;
	typedef typename Keys::PrivateKey PrivateKey;
	typedef typename Keys::PublicKey PublicKey;
	typedef T3 MessageEncodingMethod;
};

/// \brief Trapdoor Function (TF) signature scheme options
/// \tparam T1 algorithm info class
/// \tparam T2 keys class with public and private key
/// \tparam T3 message encoding class
/// \tparam T4 HashTransformation class
template <class T1, class T2, class T3, class T4>
struct TF_SignatureSchemeOptions : public TF_CryptoSchemeOptions<T1, T2, T3>
{
	typedef T4 HashFunction;
};

/// \brief Trapdoor Function (TF) base implementation
/// \tparam BASE base class
/// \tparam SCHEME_OPTIONS scheme options class
/// \tparam KEY_CLASS key class
template <class BASE, class SCHEME_OPTIONS, class KEY_CLASS>
class CRYPTOPP_NO_VTABLE TF_ObjectImplBase : public AlgorithmImpl<BASE, typename SCHEME_OPTIONS::AlgorithmInfo>
{
public:
	typedef SCHEME_OPTIONS SchemeOptions;
	typedef KEY_CLASS KeyClass;

	virtual ~TF_ObjectImplBase() {}

	PublicKey & AccessPublicKey() {return AccessKey();}
	const PublicKey & GetPublicKey() const {return GetKey();}

	PrivateKey & AccessPrivateKey() {return AccessKey();}
	const PrivateKey & GetPrivateKey() const {return GetKey();}

	virtual const KeyClass & GetKey() const =0;
	virtual KeyClass & AccessKey() =0;

	const KeyClass & GetTrapdoorFunction() const {return GetKey();}

	PK_MessageAccumulator * NewSignatureAccumulator(RandomNumberGenerator &rng) const
	{
		CRYPTOPP_UNUSED(rng);
		return new PK_MessageAccumulatorImpl<typename SCHEME_OPTIONS::HashFunction>;
	}
	PK_MessageAccumulator * NewVerificationAccumulator() const
	{
		return new PK_MessageAccumulatorImpl<typename SCHEME_OPTIONS::HashFunction>;
	}

protected:
	const typename BASE::MessageEncodingInterface & GetMessageEncodingInterface() const
		{return Singleton<typename SCHEME_OPTIONS::MessageEncodingMethod>().Ref();}
	const TrapdoorFunctionBounds & GetTrapdoorFunctionBounds() const
		{return GetKey();}
	const typename BASE::TrapdoorFunctionInterface & GetTrapdoorFunctionInterface() const
		{return GetKey();}

	// for signature scheme
	HashIdentifier GetHashIdentifier() const
	{
        typedef typename SchemeOptions::MessageEncodingMethod::HashIdentifierLookup::template HashIdentifierLookup2<typename SchemeOptions::HashFunction> L;
        return L::Lookup();
	}
	size_t GetDigestSize() const
	{
		typedef typename SchemeOptions::HashFunction H;
		return H::DIGESTSIZE;
	}
};

/// \brief Trapdoor Function (TF) signature with external reference
/// \tparam BASE base class
/// \tparam SCHEME_OPTIONS scheme options class
/// \tparam KEY key class
/// \details TF_ObjectImplExtRef() holds a pointer to an external key structure
template <class BASE, class SCHEME_OPTIONS, class KEY>
class TF_ObjectImplExtRef : public TF_ObjectImplBase<BASE, SCHEME_OPTIONS, KEY>
{
public:
	virtual ~TF_ObjectImplExtRef() {}

	TF_ObjectImplExtRef(const KEY *pKey = NULLPTR) : m_pKey(pKey) {}
	void SetKeyPtr(const KEY *pKey) {m_pKey = pKey;}

	const KEY & GetKey() const {return *m_pKey;}
	KEY & AccessKey() {throw NotImplemented("TF_ObjectImplExtRef: cannot modify refererenced key");}

private:
	const KEY * m_pKey;
};

/// \brief Trapdoor Function (TF) signature scheme options
/// \tparam BASE base class
/// \tparam SCHEME_OPTIONS scheme options class
/// \tparam KEY_CLASS key class
/// \details TF_ObjectImpl() holds a reference to a trapdoor function
template <class BASE, class SCHEME_OPTIONS, class KEY_CLASS>
class CRYPTOPP_NO_VTABLE TF_ObjectImpl : public TF_ObjectImplBase<BASE, SCHEME_OPTIONS, KEY_CLASS>
{
public:
	typedef KEY_CLASS KeyClass;

	virtual ~TF_ObjectImpl() {}

	const KeyClass & GetKey() const {return m_trapdoorFunction;}
	KeyClass & AccessKey() {return m_trapdoorFunction;}

private:
	KeyClass m_trapdoorFunction;
};

/// \brief Trapdoor Function (TF) decryptor options
/// \tparam SCHEME_OPTIONS scheme options class
template <class SCHEME_OPTIONS>
class TF_DecryptorImpl : public TF_ObjectImpl<TF_DecryptorBase, SCHEME_OPTIONS, typename SCHEME_OPTIONS::PrivateKey>
{
};

/// \brief Trapdoor Function (TF) encryptor options
/// \tparam SCHEME_OPTIONS scheme options class
template <class SCHEME_OPTIONS>
class TF_EncryptorImpl : public TF_ObjectImpl<TF_EncryptorBase, SCHEME_OPTIONS, typename SCHEME_OPTIONS::PublicKey>
{
};

/// \brief Trapdoor Function (TF) encryptor options
/// \tparam SCHEME_OPTIONS scheme options class
template <class SCHEME_OPTIONS>
class TF_SignerImpl : public TF_ObjectImpl<TF_SignerBase, SCHEME_OPTIONS, typename SCHEME_OPTIONS::PrivateKey>
{
};

/// \brief Trapdoor Function (TF) encryptor options
/// \tparam SCHEME_OPTIONS scheme options class
template <class SCHEME_OPTIONS>
class TF_VerifierImpl : public TF_ObjectImpl<TF_VerifierBase, SCHEME_OPTIONS, typename SCHEME_OPTIONS::PublicKey>
{
};

// ********************************************************

/// \brief Mask generation function interface
/// \sa P1363_KDF2, P1363_MGF1
/// \since Crypto++ 2.0
class CRYPTOPP_NO_VTABLE MaskGeneratingFunction
{
public:
	virtual ~MaskGeneratingFunction() {}

	/// \brief Generate and apply mask
	/// \param hash HashTransformation derived class
	/// \param output the destination byte array
	/// \param outputLength the size fo the the destination byte array
	/// \param input the message to hash
	/// \param inputLength the size of the message
	/// \param mask flag indicating whether to apply the mask
	virtual void GenerateAndMask(HashTransformation &hash, byte *output, size_t outputLength, const byte *input, size_t inputLength, bool mask = true) const =0;
};

/// \fn P1363_MGF1KDF2_Common
/// \brief P1363 mask generation function
/// \param hash HashTransformation derived class
/// \param output the destination byte array
/// \param outputLength the size fo the the destination byte array
/// \param input the message to hash
/// \param inputLength the size of the message
/// \param derivationParams additional derivation parameters
/// \param derivationParamsLength the size of the additional derivation parameters
/// \param mask flag indicating whether to apply the mask
/// \param counterStart starting counter value used in generation function
CRYPTOPP_DLL void CRYPTOPP_API P1363_MGF1KDF2_Common(HashTransformation &hash, byte *output, size_t outputLength, const byte *input, size_t inputLength, const byte *derivationParams, size_t derivationParamsLength, bool mask, unsigned int counterStart);

/// \brief P1363 mask generation function
/// \sa P1363_KDF2, MaskGeneratingFunction
/// \since Crypto++ 2.0
class P1363_MGF1 : public MaskGeneratingFunction
{
public:
	/// \brief The algorithm name
	/// \returns the algorithm name
	/// \details StaticAlgorithmName returns the algorithm's name as a static
	///   member function.
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "MGF1";}

	/// \brief P1363 mask generation function
	/// \param hash HashTransformation derived class
	/// \param output the destination byte array
	/// \param outputLength the size fo the the destination byte array
	/// \param input the message to hash
	/// \param inputLength the size of the message
	/// \param mask flag indicating whether to apply the mask
	void GenerateAndMask(HashTransformation &hash, byte *output, size_t outputLength, const byte *input, size_t inputLength, bool mask = true) const
	{
		P1363_MGF1KDF2_Common(hash, output, outputLength, input, inputLength, NULLPTR, 0, mask, 0);
	}
};

// ********************************************************

/// \brief P1363 key derivation function
/// \tparam H hash function used in the derivation
/// \sa P1363_MGF1, KeyDerivationFunction, <A
///  HREF="https://www.cryptopp.com/wiki/P1363_KDF2">P1363_KDF2</A>
///  on the Crypto++ wiki
/// \since Crypto++ 2.0
template <class H>
class P1363_KDF2
{
public:
	/// \brief P1363 key derivation function
	/// \param output the destination byte array
	/// \param outputLength the size fo the the destination byte array
	/// \param input the message to hash
	/// \param inputLength the size of the message
	/// \param derivationParams additional derivation parameters
	/// \param derivationParamsLength the size of the additional derivation parameters
	/// \details DeriveKey calls P1363_MGF1KDF2_Common
	static void CRYPTOPP_API DeriveKey(byte *output, size_t outputLength, const byte *input, size_t inputLength, const byte *derivationParams, size_t derivationParamsLength)
	{
		H h;
		P1363_MGF1KDF2_Common(h, output, outputLength, input, inputLength, derivationParams, derivationParamsLength, false, 1);
	}
};

// ********************************************************

/// \brief Exception thrown when an invalid group element is encountered
/// \details Thrown by DecodeElement and AgreeWithStaticPrivateKey
class DL_BadElement : public InvalidDataFormat
{
public:
	DL_BadElement() : InvalidDataFormat("CryptoPP: invalid group element") {}
};

/// \brief Interface for Discrete Log (DL) group parameters
/// \tparam T element in the group
/// \details The element is usually an Integer, \ref ECP "ECP::Point" or \ref EC2N "EC2N::Point"
template <class T>
class CRYPTOPP_NO_VTABLE DL_GroupParameters : public CryptoParameters
{
	typedef DL_GroupParameters<T> ThisClass;

public:
	typedef T Element;

	virtual ~DL_GroupParameters() {}

	DL_GroupParameters() : m_validationLevel(0) {}

	// CryptoMaterial
	bool Validate(RandomNumberGenerator &rng, unsigned int level) const
	{
		if (!GetBasePrecomputation().IsInitialized())
			return false;

		if (m_validationLevel > level)
			return true;

		CRYPTOPP_ASSERT(ValidateGroup(rng, level));
		bool pass = ValidateGroup(rng, level);
		CRYPTOPP_ASSERT(ValidateElement(level, GetSubgroupGenerator(), &GetBasePrecomputation()));
		pass = pass && ValidateElement(level, GetSubgroupGenerator(), &GetBasePrecomputation());

		m_validationLevel = pass ? level+1 : 0;

		return pass;
	}

	bool GetVoidValue(const char *name, const std::type_info &valueType, void *pValue) const
	{
		return GetValueHelper(this, name, valueType, pValue)
			CRYPTOPP_GET_FUNCTION_ENTRY(SubgroupOrder)
			CRYPTOPP_GET_FUNCTION_ENTRY(SubgroupGenerator)
			;
	}

	/// \brief Determines whether the object supports precomputation
	/// \return true if the object supports precomputation, false otherwise
	/// \sa Precompute()
	bool SupportsPrecomputation() const {return true;}

	/// \brief Perform precomputation
	/// \param precomputationStorage the suggested number of objects for the precompute table
	/// \throws NotImplemented
	/// \details The exact semantics of Precompute() varies, but it typically means calculate
	///   a table of n objects that can be used later to speed up computation.
	/// \details If a derived class does not override Precompute(), then the base class throws
	///   NotImplemented.
	/// \sa SupportsPrecomputation(), LoadPrecomputation(), SavePrecomputation()
	void Precompute(unsigned int precomputationStorage=16)
	{
		AccessBasePrecomputation().Precompute(GetGroupPrecomputation(), GetSubgroupOrder().BitCount(), precomputationStorage);
	}

	/// \brief Retrieve previously saved precomputation
	/// \param storedPrecomputation BufferedTransformation with the saved precomputation
	/// \throws NotImplemented
	/// \sa SupportsPrecomputation(), Precompute()
	void LoadPrecomputation(BufferedTransformation &storedPrecomputation)
	{
		AccessBasePrecomputation().Load(GetGroupPrecomputation(), storedPrecomputation);
		m_validationLevel = 0;
	}

	/// \brief Save precomputation for later use
	/// \param storedPrecomputation BufferedTransformation to write the precomputation
	/// \throws NotImplemented
	/// \sa SupportsPrecomputation(), Precompute()
	void SavePrecomputation(BufferedTransformation &storedPrecomputation) const
	{
		GetBasePrecomputation().Save(GetGroupPrecomputation(), storedPrecomputation);
	}

	/// \brief Retrieves the subgroup generator
	/// \return the subgroup generator
	/// \details The subgroup generator is retrieved from the base precomputation
	virtual const Element & GetSubgroupGenerator() const {return GetBasePrecomputation().GetBase(GetGroupPrecomputation());}

	/// \brief Sets the subgroup generator
	/// \param base the new subgroup generator
	/// \details The subgroup generator is set in the base precomputation
	virtual void SetSubgroupGenerator(const Element &base) {AccessBasePrecomputation().SetBase(GetGroupPrecomputation(), base);}

	/// \brief Exponentiates the base
	/// \return the element after exponentiation
	/// \details ExponentiateBase() calls GetBasePrecomputation() and then exponentiates.
	virtual Element ExponentiateBase(const Integer &exponent) const
	{
		return GetBasePrecomputation().Exponentiate(GetGroupPrecomputation(), exponent);
	}

	/// \brief Exponentiates an element
	/// \param base the base elemenet
	/// \param exponent the exponent to raise the base
	/// \return the result of the exponentiation
	/// \details Internally, ExponentiateElement() calls SimultaneousExponentiate().
	virtual Element ExponentiateElement(const Element &base, const Integer &exponent) const
	{
		Element result;
		SimultaneousExponentiate(&result, base, &exponent, 1);
		return result;
	}

	/// \brief Retrieves the group precomputation
	/// \return a const reference to the group precomputation
	virtual const DL_GroupPrecomputation<Element> & GetGroupPrecomputation() const =0;

	/// \brief Retrieves the group precomputation
	/// \return a const reference to the group precomputation using a fixed base
	virtual const DL_FixedBasePrecomputation<Element> & GetBasePrecomputation() const =0;

	/// \brief Retrieves the group precomputation
	/// \return a non-const reference to the group precomputation using a fixed base
	virtual DL_FixedBasePrecomputation<Element> & AccessBasePrecomputation() =0;

	/// \brief Retrieves the subgroup order
	/// \return the order of subgroup generated by the base element
	virtual const Integer & GetSubgroupOrder() const =0;

	/// \brief Retrieves the maximum exponent for the group
	/// \return the maximum exponent for the group
	virtual Integer GetMaxExponent() const =0;

	/// \brief Retrieves the order of the group
	/// \return the order of the group
	/// \details Either GetGroupOrder() or GetCofactor() must be overridden in a derived class.
	virtual Integer GetGroupOrder() const {return GetSubgroupOrder()*GetCofactor();}

	/// \brief Retrieves the cofactor
	/// \return the cofactor
	/// \details Either GetGroupOrder() or GetCofactor() must be overridden in a derived class.
	virtual Integer GetCofactor() const {return GetGroupOrder()/GetSubgroupOrder();}

	/// \brief Retrieves the encoded element's size
	/// \param reversible flag indicating the encoding format
	/// \return encoded element's size, in bytes
	/// \details The format of the encoded element varies by the underlying type of the element and the
	///   reversible flag. GetEncodedElementSize() must be implemented in a derived class.
	/// \sa GetEncodedElementSize(), EncodeElement(), DecodeElement()
	virtual unsigned int GetEncodedElementSize(bool reversible) const =0;

	/// \brief Encodes the element
	/// \param reversible flag indicating the encoding format
	/// \param element reference to the element to encode
	/// \param encoded destination byte array for the encoded element
	/// \details EncodeElement() must be implemented in a derived class.
	/// \pre <tt>COUNTOF(encoded) == GetEncodedElementSize()</tt>
	virtual void EncodeElement(bool reversible, const Element &element, byte *encoded) const =0;

	/// \brief Decodes the element
	/// \param encoded byte array with the encoded element
	/// \param checkForGroupMembership flag indicating if the element should be validated
	/// \return Element after decoding
	/// \details DecodeElement() must be implemented in a derived class.
	/// \pre <tt>COUNTOF(encoded) == GetEncodedElementSize()</tt>
	virtual Element DecodeElement(const byte *encoded, bool checkForGroupMembership) const =0;

	/// \brief Converts an element to an Integer
	/// \param element the element to convert to an Integer
	/// \return Element after converting to an Integer
	/// \details ConvertElementToInteger() must be implemented in a derived class.
	virtual Integer ConvertElementToInteger(const Element &element) const =0;

	/// \brief Check the group for errors
	/// \param rng RandomNumberGenerator for objects which use randomized testing
	/// \param level level of thoroughness
	/// \return true if the tests succeed, false otherwise
	/// \details There are four levels of thoroughness:
	///   <ul>
	///   <li>0 - using this object won't cause a crash or exception
	///   <li>1 - this object will probably function, and encrypt, sign, other operations correctly
	///   <li>2 - ensure this object will function correctly, and perform reasonable security checks
	///   <li>3 - perform reasonable security checks, and do checks that may take a long time
	///   </ul>
	/// \details Level 0 does not require a RandomNumberGenerator. A NullRNG() can be used for level 0.
	///   Level 1 may not check for weak keys and such. Levels 2 and 3 are recommended.
	/// \details ValidateGroup() must be implemented in a derived class.
	virtual bool ValidateGroup(RandomNumberGenerator &rng, unsigned int level) const =0;

	/// \brief Check the element for errors
	/// \param level level of thoroughness
	/// \param element element to check
	/// \param precomp optional pointer to DL_FixedBasePrecomputation
	/// \return true if the tests succeed, false otherwise
	/// \details There are four levels of thoroughness:
	///   <ul>
	///   <li>0 - using this object won't cause a crash or exception
	///   <li>1 - this object will probably function, and encrypt, sign, other operations correctly
	///   <li>2 - ensure this object will function correctly, and perform reasonable security checks
	///   <li>3 - perform reasonable security checks, and do checks that may take a long time
	///   </ul>
	/// \details Level 0 performs group membership checks. Level 1 may not check for weak keys and such.
	///   Levels 2 and 3 are recommended.
	/// \details ValidateElement() must be implemented in a derived class.
	virtual bool ValidateElement(unsigned int level, const Element &element, const DL_FixedBasePrecomputation<Element> *precomp) const =0;

	virtual bool FastSubgroupCheckAvailable() const =0;

	/// \brief Determines if an element is an identity
	/// \param element element to check
	/// \return true if the element is an identity, false otherwise
	/// \details The identity element or or neutral element is a special element in a group that leaves
	///   other elements unchanged when combined with it.
	/// \details IsIdentity() must be implemented in a derived class.
	virtual bool IsIdentity(const Element &element) const =0;

	/// \brief Exponentiates a base to multiple exponents
	/// \param results an array of Elements
	/// \param base the base to raise to the exponents
	/// \param exponents an array of exponents
	/// \param exponentsCount the number of exponents in the array
	/// \details SimultaneousExponentiate() raises the base to each exponent in the exponents array and stores the
	///   result at the respective position in the results array.
	/// \details SimultaneousExponentiate() must be implemented in a derived class.
	/// \pre <tt>COUNTOF(results) == exponentsCount</tt>
	/// \pre <tt>COUNTOF(exponents) == exponentsCount</tt>
	virtual void SimultaneousExponentiate(Element *results, const Element &base, const Integer *exponents, unsigned int exponentsCount) const =0;

protected:
	void ParametersChanged() {m_validationLevel = 0;}

private:
	mutable unsigned int m_validationLevel;
};

/// \brief Base implementation of Discrete Log (DL) group parameters
/// \tparam GROUP_PRECOMP group precomputation class
/// \tparam BASE_PRECOMP fixed base precomputation class
/// \tparam BASE class or type of an element
template <class GROUP_PRECOMP, class BASE_PRECOMP = DL_FixedBasePrecomputationImpl<typename GROUP_PRECOMP::Element>, class BASE = DL_GroupParameters<typename GROUP_PRECOMP::Element> >
class DL_GroupParametersImpl : public BASE
{
public:
	typedef GROUP_PRECOMP GroupPrecomputation;
	typedef typename GROUP_PRECOMP::Element Element;
	typedef BASE_PRECOMP BasePrecomputation;

	virtual ~DL_GroupParametersImpl() {}

	/// \brief Retrieves the group precomputation
	/// \return a const reference to the group precomputation
	const DL_GroupPrecomputation<Element> & GetGroupPrecomputation() const {return m_groupPrecomputation;}

	/// \brief Retrieves the group precomputation
	/// \return a const reference to the group precomputation using a fixed base
	const DL_FixedBasePrecomputation<Element> & GetBasePrecomputation() const {return m_gpc;}

	/// \brief Retrieves the group precomputation
	/// \return a non-const reference to the group precomputation using a fixed base
	DL_FixedBasePrecomputation<Element> & AccessBasePrecomputation() {return m_gpc;}

protected:
	GROUP_PRECOMP m_groupPrecomputation;
	BASE_PRECOMP m_gpc;
};

/// \brief Base class for a Discrete Log (DL) key
/// \tparam T class or type of an element
/// \details The element is usually an Integer, \ref ECP "ECP::Point" or \ref EC2N "EC2N::Point"
template <class T>
class CRYPTOPP_NO_VTABLE DL_Key
{
public:
	virtual ~DL_Key() {}

	/// \brief Retrieves abstract group parameters
	/// \return a const reference to the group parameters
	virtual const DL_GroupParameters<T> & GetAbstractGroupParameters() const =0;
	/// \brief Retrieves abstract group parameters
	/// \return a non-const reference to the group parameters
	virtual DL_GroupParameters<T> & AccessAbstractGroupParameters() =0;
};

/// \brief Interface for Discrete Log (DL) public keys
template <class T>
class CRYPTOPP_NO_VTABLE DL_PublicKey : public DL_Key<T>
{
	typedef DL_PublicKey<T> ThisClass;

public:
	typedef T Element;

	virtual ~DL_PublicKey();

	/// \brief Get a named value
	/// \param name the name of the object or value to retrieve
	/// \param valueType reference to a variable that receives the value
	/// \param pValue void pointer to a variable that receives the value
	/// \returns true if the value was retrieved, false otherwise
	/// \details GetVoidValue() retrieves the value of name if it exists.
	/// \note GetVoidValue() is an internal function and should be implemented
	///   by derived classes. Users should use one of the other functions instead.
	/// \sa GetValue(), GetValueWithDefault(), GetIntValue(), GetIntValueWithDefault(),
	///   GetRequiredParameter() and GetRequiredIntParameter()
	bool GetVoidValue(const char *name, const std::type_info &valueType, void *pValue) const
	{
		return GetValueHelper(this, name, valueType, pValue, &this->GetAbstractGroupParameters())
				CRYPTOPP_GET_FUNCTION_ENTRY(PublicElement);
	}

	/// \brief Initialize or reinitialize this key
	/// \param source NameValuePairs to assign
	void AssignFrom(const NameValuePairs &source);

	/// \brief Retrieves the public element
	/// \returns the public element
	virtual const Element & GetPublicElement() const {return GetPublicPrecomputation().GetBase(this->GetAbstractGroupParameters().GetGroupPrecomputation());}

	/// \brief Sets the public element
	/// \param y the public element
	virtual void SetPublicElement(const Element &y) {AccessPublicPrecomputation().SetBase(this->GetAbstractGroupParameters().GetGroupPrecomputation(), y);}

	/// \brief Exponentiates this element
	/// \param exponent the exponent to raise the base
	/// \returns the public element raised to the exponent
	virtual Element ExponentiatePublicElement(const Integer &exponent) const
	{
		const DL_GroupParameters<T> &params = this->GetAbstractGroupParameters();
		return GetPublicPrecomputation().Exponentiate(params.GetGroupPrecomputation(), exponent);
	}

	/// \brief Exponentiates an element
	/// \param baseExp the first exponent
	/// \param publicExp the second exponent
	/// \returns the public element raised to the exponent
	/// \details CascadeExponentiateBaseAndPublicElement raises the public element to
	///   the base element and precomputation.
	virtual Element CascadeExponentiateBaseAndPublicElement(const Integer &baseExp, const Integer &publicExp) const
	{
		const DL_GroupParameters<T> &params = this->GetAbstractGroupParameters();
		return params.GetBasePrecomputation().CascadeExponentiate(params.GetGroupPrecomputation(), baseExp, GetPublicPrecomputation(), publicExp);
	}

	/// \brief Accesses the public precomputation
	/// \details GetPublicPrecomputation returns a const reference, while
	///   AccessPublicPrecomputation returns a non-const reference. Must be
	///   overridden in derived classes.
	virtual const DL_FixedBasePrecomputation<T> & GetPublicPrecomputation() const =0;

	/// \brief Accesses the public precomputation
	/// \details GetPublicPrecomputation returns a const reference, while
	///   AccessPublicPrecomputation returns a non-const reference. Must be
	///   overridden in derived classes.
	virtual DL_FixedBasePrecomputation<T> & AccessPublicPrecomputation() =0;
};

// Out-of-line dtor due to AIX and GCC, http://github.com/weidai11/cryptopp/issues/499
template<class T>
DL_PublicKey<T>::~DL_PublicKey() {}

/// \brief Interface for Discrete Log (DL) private keys
template <class T>
class CRYPTOPP_NO_VTABLE DL_PrivateKey : public DL_Key<T>
{
	typedef DL_PrivateKey<T> ThisClass;

public:
	typedef T Element;

	virtual ~DL_PrivateKey();

	/// \brief Initializes a public key from this key
	/// \param pub reference to a public key
	void MakePublicKey(DL_PublicKey<T> &pub) const
	{
		pub.AccessAbstractGroupParameters().AssignFrom(this->GetAbstractGroupParameters());
		pub.SetPublicElement(this->GetAbstractGroupParameters().ExponentiateBase(GetPrivateExponent()));
	}

	/// \brief Get a named value
	/// \param name the name of the object or value to retrieve
	/// \param valueType reference to a variable that receives the value
	/// \param pValue void pointer to a variable that receives the value
	/// \returns true if the value was retrieved, false otherwise
	/// \details GetVoidValue() retrieves the value of name if it exists.
	/// \note GetVoidValue() is an internal function and should be implemented
	///   by derived classes. Users should use one of the other functions instead.
	/// \sa GetValue(), GetValueWithDefault(), GetIntValue(), GetIntValueWithDefault(),
	///   GetRequiredParameter() and GetRequiredIntParameter()
	bool GetVoidValue(const char *name, const std::type_info &valueType, void *pValue) const
	{
		return GetValueHelper(this, name, valueType, pValue, &this->GetAbstractGroupParameters())
				CRYPTOPP_GET_FUNCTION_ENTRY(PrivateExponent);
	}

	/// \brief Initialize or reinitialize this key
	/// \param source NameValuePairs to assign
	void AssignFrom(const NameValuePairs &source)
	{
		this->AccessAbstractGroupParameters().AssignFrom(source);
		AssignFromHelper(this, source)
			CRYPTOPP_SET_FUNCTION_ENTRY(PrivateExponent);
	}

	/// \brief Retrieves the private exponent
	/// \returns the private exponent
	/// \details Must be overridden in derived classes.
	virtual const Integer & GetPrivateExponent() const =0;
	/// \brief Sets the private exponent
	/// \param x the private exponent
	/// \details Must be overridden in derived classes.
	virtual void SetPrivateExponent(const Integer &x) =0;
};

// Out-of-line dtor due to AIX and GCC, http://github.com/weidai11/cryptopp/issues/499
template<class T>
DL_PrivateKey<T>::~DL_PrivateKey() {}

template <class T>
void DL_PublicKey<T>::AssignFrom(const NameValuePairs &source)
{
	DL_PrivateKey<T> *pPrivateKey = NULLPTR;
	if (source.GetThisPointer(pPrivateKey))
		pPrivateKey->MakePublicKey(*this);
	else
	{
		this->AccessAbstractGroupParameters().AssignFrom(source);
		AssignFromHelper(this, source)
			CRYPTOPP_SET_FUNCTION_ENTRY(PublicElement);
	}
}

class OID;

/// \brief Discrete Log (DL) key base implementation
/// \tparam PK Key class
/// \tparam GP GroupParameters class
/// \tparam O OID class
template <class PK, class GP, class O = OID>
class DL_KeyImpl : public PK
{
public:
	typedef GP GroupParameters;

	virtual ~DL_KeyImpl() {}

	O GetAlgorithmID() const {return GetGroupParameters().GetAlgorithmID();}
	bool BERDecodeAlgorithmParameters(BufferedTransformation &bt)
		{AccessGroupParameters().BERDecode(bt); return true;}
	bool DEREncodeAlgorithmParameters(BufferedTransformation &bt) const
		{GetGroupParameters().DEREncode(bt); return true;}

	const GP & GetGroupParameters() const {return m_groupParameters;}
	GP & AccessGroupParameters() {return m_groupParameters;}

private:
	GP m_groupParameters;
};

class X509PublicKey;
class PKCS8PrivateKey;

/// \brief Discrete Log (DL) private key base implementation
/// \tparam GP GroupParameters class
template <class GP>
class DL_PrivateKeyImpl : public DL_PrivateKey<typename GP::Element>, public DL_KeyImpl<PKCS8PrivateKey, GP>
{
public:
	typedef typename GP::Element Element;

	virtual ~DL_PrivateKeyImpl() {}

	// GeneratableCryptoMaterial
	bool Validate(RandomNumberGenerator &rng, unsigned int level) const
	{
		CRYPTOPP_ASSERT(GetAbstractGroupParameters().Validate(rng, level));
		bool pass = GetAbstractGroupParameters().Validate(rng, level);

		const Integer &q = GetAbstractGroupParameters().GetSubgroupOrder();
		const Integer &x = GetPrivateExponent();

		CRYPTOPP_ASSERT(x.IsPositive());
		CRYPTOPP_ASSERT(x < q);
		pass = pass && x.IsPositive() && x < q;

		if (level >= 1)
		{
			CRYPTOPP_ASSERT(Integer::Gcd(x, q) == Integer::One());
			pass = pass && Integer::Gcd(x, q) == Integer::One();
		}
		return pass;
	}

	bool GetVoidValue(const char *name, const std::type_info &valueType, void *pValue) const
	{
		return GetValueHelper<DL_PrivateKey<Element> >(this, name, valueType, pValue).Assignable();
	}

	void AssignFrom(const NameValuePairs &source)
	{
		AssignFromHelper<DL_PrivateKey<Element> >(this, source);
	}

	void GenerateRandom(RandomNumberGenerator &rng, const NameValuePairs &params)
	{
		if (!params.GetThisObject(this->AccessGroupParameters()))
			this->AccessGroupParameters().GenerateRandom(rng, params);
		Integer x(rng, Integer::One(), GetAbstractGroupParameters().GetMaxExponent());
		SetPrivateExponent(x);
	}

	bool SupportsPrecomputation() const {return true;}

	void Precompute(unsigned int precomputationStorage=16)
		{AccessAbstractGroupParameters().Precompute(precomputationStorage);}

	void LoadPrecomputation(BufferedTransformation &storedPrecomputation)
		{AccessAbstractGroupParameters().LoadPrecomputation(storedPrecomputation);}

	void SavePrecomputation(BufferedTransformation &storedPrecomputation) const
		{GetAbstractGroupParameters().SavePrecomputation(storedPrecomputation);}

	// DL_Key
	const DL_GroupParameters<Element> & GetAbstractGroupParameters() const {return this->GetGroupParameters();}
	DL_GroupParameters<Element> & AccessAbstractGroupParameters() {return this->AccessGroupParameters();}

	// DL_PrivateKey
	const Integer & GetPrivateExponent() const {return m_x;}
	void SetPrivateExponent(const Integer &x) {m_x = x;}

	// PKCS8PrivateKey
	void BERDecodePrivateKey(BufferedTransformation &bt, bool, size_t)
		{m_x.BERDecode(bt);}
	void DEREncodePrivateKey(BufferedTransformation &bt) const
		{m_x.DEREncode(bt);}

private:
	Integer m_x;
};

template <class BASE, class SIGNATURE_SCHEME>
class DL_PrivateKey_WithSignaturePairwiseConsistencyTest : public BASE
{
public:
	virtual ~DL_PrivateKey_WithSignaturePairwiseConsistencyTest() {}

	void GenerateRandom(RandomNumberGenerator &rng, const NameValuePairs &params)
	{
		BASE::GenerateRandom(rng, params);

		if (FIPS_140_2_ComplianceEnabled())
		{
			typename SIGNATURE_SCHEME::Signer signer(*this);
			typename SIGNATURE_SCHEME::Verifier verifier(signer);
			SignaturePairwiseConsistencyTest_FIPS_140_Only(signer, verifier);
		}
	}
};

/// \brief Discrete Log (DL) public key base implementation
/// \tparam GP GroupParameters class
template <class GP>
class DL_PublicKeyImpl : public DL_PublicKey<typename GP::Element>, public DL_KeyImpl<X509PublicKey, GP>
{
public:
	typedef typename GP::Element Element;

	virtual ~DL_PublicKeyImpl();

	// CryptoMaterial
	bool Validate(RandomNumberGenerator &rng, unsigned int level) const
	{
		CRYPTOPP_ASSERT(GetAbstractGroupParameters().Validate(rng, level));
		bool pass = GetAbstractGroupParameters().Validate(rng, level);
		CRYPTOPP_ASSERT(GetAbstractGroupParameters().ValidateElement(level, this->GetPublicElement(), &GetPublicPrecomputation()));
		pass = pass && GetAbstractGroupParameters().ValidateElement(level, this->GetPublicElement(), &GetPublicPrecomputation());
		return pass;
	}

	bool GetVoidValue(const char *name, const std::type_info &valueType, void *pValue) const
	{
		return GetValueHelper<DL_PublicKey<Element> >(this, name, valueType, pValue).Assignable();
	}

	void AssignFrom(const NameValuePairs &source)
	{
		AssignFromHelper<DL_PublicKey<Element> >(this, source);
	}

	bool SupportsPrecomputation() const {return true;}

	void Precompute(unsigned int precomputationStorage=16)
	{
		AccessAbstractGroupParameters().Precompute(precomputationStorage);
		AccessPublicPrecomputation().Precompute(GetAbstractGroupParameters().GetGroupPrecomputation(), GetAbstractGroupParameters().GetSubgroupOrder().BitCount(), precomputationStorage);
	}

	void LoadPrecomputation(BufferedTransformation &storedPrecomputation)
	{
		AccessAbstractGroupParameters().LoadPrecomputation(storedPrecomputation);
		AccessPublicPrecomputation().Load(GetAbstractGroupParameters().GetGroupPrecomputation(), storedPrecomputation);
	}

	void SavePrecomputation(BufferedTransformation &storedPrecomputation) const
	{
		GetAbstractGroupParameters().SavePrecomputation(storedPrecomputation);
		GetPublicPrecomputation().Save(GetAbstractGroupParameters().GetGroupPrecomputation(), storedPrecomputation);
	}

	// DL_Key
	const DL_GroupParameters<Element> & GetAbstractGroupParameters() const {return this->GetGroupParameters();}
	DL_GroupParameters<Element> & AccessAbstractGroupParameters() {return this->AccessGroupParameters();}

	// DL_PublicKey
	const DL_FixedBasePrecomputation<Element> & GetPublicPrecomputation() const {return m_ypc;}
	DL_FixedBasePrecomputation<Element> & AccessPublicPrecomputation() {return m_ypc;}

	// non-inherited
	bool operator==(const DL_PublicKeyImpl<GP> &rhs) const
		{return this->GetGroupParameters() == rhs.GetGroupParameters() && this->GetPublicElement() == rhs.GetPublicElement();}

private:
	typename GP::BasePrecomputation m_ypc;
};

// Out-of-line dtor due to AIX and GCC, http://github.com/weidai11/cryptopp/issues/499
template<class GP>
DL_PublicKeyImpl<GP>::~DL_PublicKeyImpl() {}

/// \brief Interface for Elgamal-like signature algorithms
/// \tparam T Field element
template <class T>
class CRYPTOPP_NO_VTABLE DL_ElgamalLikeSignatureAlgorithm
{
public:
	virtual ~DL_ElgamalLikeSignatureAlgorithm() {}

	/// \brief Sign a message using a private key
	/// \param params GroupParameters
	/// \param privateKey private key
	/// \param k signing exponent
	/// \param e encoded message
	/// \param r r part of signature
	/// \param s s part of signature
	virtual void Sign(const DL_GroupParameters<T> &params, const Integer &privateKey, const Integer &k, const Integer &e, Integer &r, Integer &s) const =0;

	/// \brief Verify a message using a public key
	/// \param params GroupParameters
	/// \param publicKey public key
	/// \param e encoded message
	/// \param r r part of signature
	/// \param s s part of signature
	virtual bool Verify(const DL_GroupParameters<T> &params, const DL_PublicKey<T> &publicKey, const Integer &e, const Integer &r, const Integer &s) const =0;

	/// \brief Recover a Presignature
	/// \param params GroupParameters
	/// \param publicKey public key
	/// \param r r part of signature
	/// \param s s part of signature
	virtual Integer RecoverPresignature(const DL_GroupParameters<T> &params, const DL_PublicKey<T> &publicKey, const Integer &r, const Integer &s) const
	{
		CRYPTOPP_UNUSED(params); CRYPTOPP_UNUSED(publicKey); CRYPTOPP_UNUSED(r); CRYPTOPP_UNUSED(s);
		throw NotImplemented("DL_ElgamalLikeSignatureAlgorithm: this signature scheme does not support message recovery");
		MAYBE_RETURN(Integer::Zero());
	}

	/// \brief Retrieve R length
	/// \param params GroupParameters
	virtual size_t RLen(const DL_GroupParameters<T> &params) const
		{return params.GetSubgroupOrder().ByteCount();}

	/// \brief Retrieve S length
	/// \param params GroupParameters
	virtual size_t SLen(const DL_GroupParameters<T> &params) const
		{return params.GetSubgroupOrder().ByteCount();}

	/// \brief Signature scheme flag
	/// \returns true if the signature scheme is deterministic, false otherwise
	/// \details IsDeterministic() is provided for DL signers. It is used by RFC 6979 signature schemes.
	virtual bool IsDeterministic() const
		{return false;}
};

/// \brief Interface for deterministic signers
/// \details RFC 6979 signers which generate k based on the encoded message and private key
class CRYPTOPP_NO_VTABLE DeterministicSignatureAlgorithm
{
public:
	virtual ~DeterministicSignatureAlgorithm() {}

	/// \brief Generate k
	/// \param x private key
	/// \param q subgroup generator
	/// \param e encoded message
	virtual Integer GenerateRandom(const Integer &x, const Integer &q, const Integer &e) const =0;
};

/// \brief Interface for DL key agreement algorithms
/// \tparam T Field element
template <class T>
class CRYPTOPP_NO_VTABLE DL_KeyAgreementAlgorithm
{
public:
	typedef T Element;

	virtual ~DL_KeyAgreementAlgorithm() {}

	virtual Element AgreeWithEphemeralPrivateKey(const DL_GroupParameters<Element> &params, const DL_FixedBasePrecomputation<Element> &publicPrecomputation, const Integer &privateExponent) const =0;
	virtual Element AgreeWithStaticPrivateKey(const DL_GroupParameters<Element> &params, const Element &publicElement, bool validateOtherPublicKey, const Integer &privateExponent) const =0;
};

/// \brief Interface for key derivation algorithms used in DL cryptosystems
/// \tparam T Field element
template <class T>
class CRYPTOPP_NO_VTABLE DL_KeyDerivationAlgorithm
{
public:
	virtual ~DL_KeyDerivationAlgorithm() {}

	virtual bool ParameterSupported(const char *name) const
		{CRYPTOPP_UNUSED(name); return false;}
	virtual void Derive(const DL_GroupParameters<T> &groupParams, byte *derivedKey, size_t derivedLength, const T &agreedElement, const T &ephemeralPublicKey, const NameValuePairs &derivationParams) const =0;
};

/// \brief Interface for symmetric encryption algorithms used in DL cryptosystems
class CRYPTOPP_NO_VTABLE DL_SymmetricEncryptionAlgorithm
{
public:
	virtual ~DL_SymmetricEncryptionAlgorithm() {}

	virtual bool ParameterSupported(const char *name) const
		{CRYPTOPP_UNUSED(name); return false;}
	virtual size_t GetSymmetricKeyLength(size_t plaintextLength) const =0;
	virtual size_t GetSymmetricCiphertextLength(size_t plaintextLength) const =0;
	virtual size_t GetMaxSymmetricPlaintextLength(size_t ciphertextLength) const =0;
	virtual void SymmetricEncrypt(RandomNumberGenerator &rng, const byte *key, const byte *plaintext, size_t plaintextLength, byte *ciphertext, const NameValuePairs &parameters) const =0;
	virtual DecodingResult SymmetricDecrypt(const byte *key, const byte *ciphertext, size_t ciphertextLength, byte *plaintext, const NameValuePairs &parameters) const =0;
};

/// \brief Discrete Log (DL) base interface
/// \tparam KI public or private key interface
template <class KI>
class CRYPTOPP_NO_VTABLE DL_Base
{
protected:
	typedef KI KeyInterface;
	typedef typename KI::Element Element;

	virtual ~DL_Base() {}

	const DL_GroupParameters<Element> & GetAbstractGroupParameters() const {return GetKeyInterface().GetAbstractGroupParameters();}
	DL_GroupParameters<Element> & AccessAbstractGroupParameters() {return AccessKeyInterface().AccessAbstractGroupParameters();}

	virtual KeyInterface & AccessKeyInterface() =0;
	virtual const KeyInterface & GetKeyInterface() const =0;
};

/// \brief Discrete Log (DL) signature scheme base implementation
/// \tparam INTFACE PK_Signer or PK_Verifier derived class
/// \tparam KEY_INTFACE DL_Base key base used in the scheme
/// \details DL_SignatureSchemeBase provides common functions for signers and verifiers.
///   DL_Base<DL_PrivateKey> is used for signers, and DL_Base<DL_PublicKey> is used for verifiers.
template <class INTFACE, class KEY_INTFACE>
class CRYPTOPP_NO_VTABLE DL_SignatureSchemeBase : public INTFACE, public DL_Base<KEY_INTFACE>
{
public:
	virtual ~DL_SignatureSchemeBase() {}

	/// \brief Provides the signature length
	/// \returns signature length, in bytes
	/// \details SignatureLength returns the size required for <tt>r+s</tt>.
	size_t SignatureLength() const
	{
		return GetSignatureAlgorithm().RLen(this->GetAbstractGroupParameters())
			+ GetSignatureAlgorithm().SLen(this->GetAbstractGroupParameters());
	}

	/// \brief Provides the maximum recoverable length
	/// \returns maximum recoverable length, in bytes
	size_t MaxRecoverableLength() const
		{return GetMessageEncodingInterface().MaxRecoverableLength(0, GetHashIdentifier().second, GetDigestSize());}

	/// \brief Provides the maximum recoverable length
	/// \param signatureLength the size fo the signature
	/// \returns maximum recoverable length based on signature length, in bytes
	/// \details this function is not implemented and always returns 0.
	size_t MaxRecoverableLengthFromSignatureLength(size_t signatureLength) const
		{CRYPTOPP_UNUSED(signatureLength); CRYPTOPP_ASSERT(false); return 0;}	// TODO

	/// \brief Determines if the scheme is probabilistic
	/// \returns true if the scheme is probabilistic, false otherwise
	bool IsProbabilistic() const
		{return true;}

	/// \brief Determines if the scheme has non-recoverable part
	/// \returns true if the message encoding has a non-recoverable part, false otherwise.
	bool AllowNonrecoverablePart() const
		{return GetMessageEncodingInterface().AllowNonrecoverablePart();}

	/// \brief Determines if the scheme allows recoverable part first
	/// \returns true if the message encoding allows the recoverable part, false otherwise.
	bool RecoverablePartFirst() const
		{return GetMessageEncodingInterface().RecoverablePartFirst();}

protected:
	size_t MessageRepresentativeLength() const {return BitsToBytes(MessageRepresentativeBitLength());}
	size_t MessageRepresentativeBitLength() const {return this->GetAbstractGroupParameters().GetSubgroupOrder().BitCount();}

	// true if the scheme conforms to RFC 6979
	virtual bool IsDeterministic() const {return false;}

	virtual const DL_ElgamalLikeSignatureAlgorithm<typename KEY_INTFACE::Element> & GetSignatureAlgorithm() const =0;
	virtual const PK_SignatureMessageEncodingMethod & GetMessageEncodingInterface() const =0;
	virtual HashIdentifier GetHashIdentifier() const =0;
	virtual size_t GetDigestSize() const =0;
};

/// \brief Discrete Log (DL) signature scheme signer base implementation
/// \tparam T Field element
template <class T>
class CRYPTOPP_NO_VTABLE DL_SignerBase : public DL_SignatureSchemeBase<PK_Signer, DL_PrivateKey<T> >
{
public:
	virtual ~DL_SignerBase() {}

	/// \brief Testing interface
	/// \param k Integer
	/// \param e Integer
	/// \param r Integer
	/// \param s Integer
	void RawSign(const Integer &k, const Integer &e, Integer &r, Integer &s) const
	{
		const DL_ElgamalLikeSignatureAlgorithm<T> &alg = this->GetSignatureAlgorithm();
		const DL_GroupParameters<T> &params = this->GetAbstractGroupParameters();
		const DL_PrivateKey<T> &key = this->GetKeyInterface();

		r = params.ConvertElementToInteger(params.ExponentiateBase(k));
		alg.Sign(params, key.GetPrivateExponent(), k, e, r, s);
	}

	void InputRecoverableMessage(PK_MessageAccumulator &messageAccumulator, const byte *recoverableMessage, size_t recoverableMessageLength) const
	{
		PK_MessageAccumulatorBase &ma = static_cast<PK_MessageAccumulatorBase &>(messageAccumulator);
		ma.m_recoverableMessage.Assign(recoverableMessage, recoverableMessageLength);
		this->GetMessageEncodingInterface().ProcessRecoverableMessage(ma.AccessHash(),
			recoverableMessage, recoverableMessageLength,
			ma.m_presignature, ma.m_presignature.size(),
			ma.m_semisignature);
	}

	size_t SignAndRestart(RandomNumberGenerator &rng, PK_MessageAccumulator &messageAccumulator, byte *signature, bool restart) const
	{
		this->GetMaterial().DoQuickSanityCheck();

		PK_MessageAccumulatorBase &ma = static_cast<PK_MessageAccumulatorBase &>(messageAccumulator);
		const DL_ElgamalLikeSignatureAlgorithm<T> &alg = this->GetSignatureAlgorithm();
		const DL_GroupParameters<T> &params = this->GetAbstractGroupParameters();
		const DL_PrivateKey<T> &key = this->GetKeyInterface();

		SecByteBlock representative(this->MessageRepresentativeLength());
		this->GetMessageEncodingInterface().ComputeMessageRepresentative(
			rng,
			ma.m_recoverableMessage, ma.m_recoverableMessage.size(),
			ma.AccessHash(), this->GetHashIdentifier(), ma.m_empty,
			representative, this->MessageRepresentativeBitLength());
		ma.m_empty = true;
		Integer e(representative, representative.size());

		// hash message digest into random number k to prevent reusing the same k on
		// different messages after virtual machine rollback
		if (rng.CanIncorporateEntropy())
			rng.IncorporateEntropy(representative, representative.size());

		Integer k, ks;
		const Integer& q = params.GetSubgroupOrder();
		if (alg.IsDeterministic())
		{
			const Integer& x = key.GetPrivateExponent();
			const DeterministicSignatureAlgorithm& det = dynamic_cast<const DeterministicSignatureAlgorithm&>(alg);
			k = det.GenerateRandom(x, q, e);
		}
		else
		{
			k.Randomize(rng, 1, params.GetSubgroupOrder()-1);
		}

		// Due to timing attack on nonce length by Jancar
		// https://github.com/weidai11/cryptopp/issues/869
		ks = k + q;
		if (ks.BitCount() == q.BitCount()) {
			ks += q;
		}

		Integer r, s;
		r = params.ConvertElementToInteger(params.ExponentiateBase(ks));
		alg.Sign(params, key.GetPrivateExponent(), k, e, r, s);

		/*
		Integer r, s;
		if (this->MaxRecoverableLength() > 0)
			r.Decode(ma.m_semisignature, ma.m_semisignature.size());
		else
			r.Decode(ma.m_presignature, ma.m_presignature.size());
		alg.Sign(params, key.GetPrivateExponent(), ma.m_k, e, r, s);
		*/

		const size_t rLen = alg.RLen(params);
		r.Encode(signature, rLen);
		s.Encode(signature+rLen, alg.SLen(params));

		if (restart)
			RestartMessageAccumulator(rng, ma);

		return this->SignatureLength();
	}

protected:
	void RestartMessageAccumulator(RandomNumberGenerator &rng, PK_MessageAccumulatorBase &ma) const
	{
		// k needs to be generated before hashing for signature schemes with recovery
		// but to defend against VM rollbacks we need to generate k after hashing.
		// so this code is commented out, since no DL-based signature scheme with recovery
		// has been implemented in Crypto++ anyway
		/*
		const DL_ElgamalLikeSignatureAlgorithm<T> &alg = this->GetSignatureAlgorithm();
		const DL_GroupParameters<T> &params = this->GetAbstractGroupParameters();
		ma.m_k.Randomize(rng, 1, params.GetSubgroupOrder()-1);
		ma.m_presignature.New(params.GetEncodedElementSize(false));
		params.ConvertElementToInteger(params.ExponentiateBase(ma.m_k)).Encode(ma.m_presignature, ma.m_presignature.size());
		*/
		CRYPTOPP_UNUSED(rng); CRYPTOPP_UNUSED(ma);
	}
};

/// \brief Discret Log (DL) Verifier base class
/// \tparam T Field element
template <class T>
class CRYPTOPP_NO_VTABLE DL_VerifierBase : public DL_SignatureSchemeBase<PK_Verifier, DL_PublicKey<T> >
{
public:
	virtual ~DL_VerifierBase() {}

	void InputSignature(PK_MessageAccumulator &messageAccumulator, const byte *signature, size_t signatureLength) const
	{
		CRYPTOPP_UNUSED(signature); CRYPTOPP_UNUSED(signatureLength);
		PK_MessageAccumulatorBase &ma = static_cast<PK_MessageAccumulatorBase &>(messageAccumulator);
		const DL_ElgamalLikeSignatureAlgorithm<T> &alg = this->GetSignatureAlgorithm();
		const DL_GroupParameters<T> &params = this->GetAbstractGroupParameters();

		const size_t rLen = alg.RLen(params);
		ma.m_semisignature.Assign(signature, rLen);
		ma.m_s.Decode(signature+rLen, alg.SLen(params));

		this->GetMessageEncodingInterface().ProcessSemisignature(ma.AccessHash(), ma.m_semisignature, ma.m_semisignature.size());
	}

	bool VerifyAndRestart(PK_MessageAccumulator &messageAccumulator) const
	{
		this->GetMaterial().DoQuickSanityCheck();

		PK_MessageAccumulatorBase &ma = static_cast<PK_MessageAccumulatorBase &>(messageAccumulator);
		const DL_ElgamalLikeSignatureAlgorithm<T> &alg = this->GetSignatureAlgorithm();
		const DL_GroupParameters<T> &params = this->GetAbstractGroupParameters();
		const DL_PublicKey<T> &key = this->GetKeyInterface();

		SecByteBlock representative(this->MessageRepresentativeLength());
		this->GetMessageEncodingInterface().ComputeMessageRepresentative(NullRNG(), ma.m_recoverableMessage, ma.m_recoverableMessage.size(),
			ma.AccessHash(), this->GetHashIdentifier(), ma.m_empty,
			representative, this->MessageRepresentativeBitLength());
		ma.m_empty = true;
		Integer e(representative, representative.size());

		Integer r(ma.m_semisignature, ma.m_semisignature.size());
		return alg.Verify(params, key, e, r, ma.m_s);
	}

	DecodingResult RecoverAndRestart(byte *recoveredMessage, PK_MessageAccumulator &messageAccumulator) const
	{
		this->GetMaterial().DoQuickSanityCheck();

		PK_MessageAccumulatorBase &ma = static_cast<PK_MessageAccumulatorBase &>(messageAccumulator);
		const DL_ElgamalLikeSignatureAlgorithm<T> &alg = this->GetSignatureAlgorithm();
		const DL_GroupParameters<T> &params = this->GetAbstractGroupParameters();
		const DL_PublicKey<T> &key = this->GetKeyInterface();

		SecByteBlock representative(this->MessageRepresentativeLength());
		this->GetMessageEncodingInterface().ComputeMessageRepresentative(
			NullRNG(),
			ma.m_recoverableMessage, ma.m_recoverableMessage.size(),
			ma.AccessHash(), this->GetHashIdentifier(), ma.m_empty,
			representative, this->MessageRepresentativeBitLength());
		ma.m_empty = true;
		Integer e(representative, representative.size());

		ma.m_presignature.New(params.GetEncodedElementSize(false));
		Integer r(ma.m_semisignature, ma.m_semisignature.size());
		alg.RecoverPresignature(params, key, r, ma.m_s).Encode(ma.m_presignature, ma.m_presignature.size());

		return this->GetMessageEncodingInterface().RecoverMessageFromSemisignature(
			ma.AccessHash(), this->GetHashIdentifier(),
			ma.m_presignature, ma.m_presignature.size(),
			ma.m_semisignature, ma.m_semisignature.size(),
			recoveredMessage);
	}
};

/// \brief Discrete Log (DL) cryptosystem base implementation
/// \tparam PK field element type
/// \tparam KI public or private key interface
template <class PK, class KI>
class CRYPTOPP_NO_VTABLE DL_CryptoSystemBase : public PK, public DL_Base<KI>
{
public:
	typedef typename DL_Base<KI>::Element Element;

	virtual ~DL_CryptoSystemBase() {}

	size_t MaxPlaintextLength(size_t ciphertextLength) const
	{
		unsigned int minLen = this->GetAbstractGroupParameters().GetEncodedElementSize(true);
		return ciphertextLength < minLen ? 0 : GetSymmetricEncryptionAlgorithm().GetMaxSymmetricPlaintextLength(ciphertextLength - minLen);
	}

	size_t CiphertextLength(size_t plaintextLength) const
	{
		size_t len = GetSymmetricEncryptionAlgorithm().GetSymmetricCiphertextLength(plaintextLength);
		return len == 0 ? 0 : this->GetAbstractGroupParameters().GetEncodedElementSize(true) + len;
	}

	bool ParameterSupported(const char *name) const
		{return GetKeyDerivationAlgorithm().ParameterSupported(name) || GetSymmetricEncryptionAlgorithm().ParameterSupported(name);}

protected:
	virtual const DL_KeyAgreementAlgorithm<Element> & GetKeyAgreementAlgorithm() const =0;
	virtual const DL_KeyDerivationAlgorithm<Element> & GetKeyDerivationAlgorithm() const =0;
	virtual const DL_SymmetricEncryptionAlgorithm & GetSymmetricEncryptionAlgorithm() const =0;
};

/// \brief Discrete Log (DL) decryptor base implementation
/// \tparam T Field element
template <class T>
class CRYPTOPP_NO_VTABLE DL_DecryptorBase : public DL_CryptoSystemBase<PK_Decryptor, DL_PrivateKey<T> >
{
public:
	typedef T Element;

	virtual ~DL_DecryptorBase() {}

	DecodingResult Decrypt(RandomNumberGenerator &rng, const byte *ciphertext, size_t ciphertextLength, byte *plaintext, const NameValuePairs &parameters = g_nullNameValuePairs) const
	{
		try
		{
			CRYPTOPP_UNUSED(rng);
			const DL_KeyAgreementAlgorithm<T> &agreeAlg = this->GetKeyAgreementAlgorithm();
			const DL_KeyDerivationAlgorithm<T> &derivAlg = this->GetKeyDerivationAlgorithm();
			const DL_SymmetricEncryptionAlgorithm &encAlg = this->GetSymmetricEncryptionAlgorithm();
			const DL_GroupParameters<T> &params = this->GetAbstractGroupParameters();
			const DL_PrivateKey<T> &key = this->GetKeyInterface();

			Element q = params.DecodeElement(ciphertext, true);
			size_t elementSize = params.GetEncodedElementSize(true);
			ciphertext += elementSize;
			ciphertextLength -= elementSize;

			Element z = agreeAlg.AgreeWithStaticPrivateKey(params, q, true, key.GetPrivateExponent());

			SecByteBlock derivedKey(encAlg.GetSymmetricKeyLength(encAlg.GetMaxSymmetricPlaintextLength(ciphertextLength)));
			derivAlg.Derive(params, derivedKey, derivedKey.size(), z, q, parameters);

			return encAlg.SymmetricDecrypt(derivedKey, ciphertext, ciphertextLength, plaintext, parameters);
		}
		catch (DL_BadElement &)
		{
			return DecodingResult();
		}
	}
};

/// \brief Discrete Log (DL) encryptor base implementation
/// \tparam T Field element
template <class T>
class CRYPTOPP_NO_VTABLE DL_EncryptorBase : public DL_CryptoSystemBase<PK_Encryptor, DL_PublicKey<T> >
{
public:
	typedef T Element;

	virtual ~DL_EncryptorBase() {}

	void Encrypt(RandomNumberGenerator &rng, const byte *plaintext, size_t plaintextLength, byte *ciphertext, const NameValuePairs &parameters = g_nullNameValuePairs) const
	{
		const DL_KeyAgreementAlgorithm<T> &agreeAlg = this->GetKeyAgreementAlgorithm();
		const DL_KeyDerivationAlgorithm<T> &derivAlg = this->GetKeyDerivationAlgorithm();
		const DL_SymmetricEncryptionAlgorithm &encAlg = this->GetSymmetricEncryptionAlgorithm();
		const DL_GroupParameters<T> &params = this->GetAbstractGroupParameters();
		const DL_PublicKey<T> &key = this->GetKeyInterface();

		Integer x(rng, Integer::One(), params.GetMaxExponent());
		Element q = params.ExponentiateBase(x);
		params.EncodeElement(true, q, ciphertext);
		unsigned int elementSize = params.GetEncodedElementSize(true);
		ciphertext += elementSize;

		Element z = agreeAlg.AgreeWithEphemeralPrivateKey(params, key.GetPublicPrecomputation(), x);

		SecByteBlock derivedKey(encAlg.GetSymmetricKeyLength(plaintextLength));
		derivAlg.Derive(params, derivedKey, derivedKey.size(), z, q, parameters);

		encAlg.SymmetricEncrypt(rng, derivedKey, plaintext, plaintextLength, ciphertext, parameters);
	}
};

/// \brief Discrete Log (DL) scheme options
/// \tparam T1 algorithm information
/// \tparam T2 group parameters for the scheme
template <class T1, class T2>
struct DL_SchemeOptionsBase
{
	typedef T1 AlgorithmInfo;
	typedef T2 GroupParameters;
	typedef typename GroupParameters::Element Element;
};

/// \brief Discrete Log (DL) key options
/// \tparam T1 algorithm information
/// \tparam T2 keys used in the scheme
template <class T1, class T2>
struct DL_KeyedSchemeOptions : public DL_SchemeOptionsBase<T1, typename T2::PublicKey::GroupParameters>
{
	typedef T2 Keys;
	typedef typename Keys::PrivateKey PrivateKey;
	typedef typename Keys::PublicKey PublicKey;
};

/// \brief Discrete Log (DL) signature scheme options
/// \tparam T1 algorithm information
/// \tparam T2 keys used in the scheme
/// \tparam T3 signature algorithm
/// \tparam T4 message encoding method
/// \tparam T5 hash function
template <class T1, class T2, class T3, class T4, class T5>
struct DL_SignatureSchemeOptions : public DL_KeyedSchemeOptions<T1, T2>
{
	typedef T3 SignatureAlgorithm;
	typedef T4 MessageEncodingMethod;
	typedef T5 HashFunction;
};

/// \brief Discrete Log (DL) crypto scheme options
/// \tparam T1 algorithm information
/// \tparam T2 keys used in the scheme
/// \tparam T3 key agreement algorithm
/// \tparam T4 key derivation algorithm
/// \tparam T5 symmetric encryption algorithm
template <class T1, class T2, class T3, class T4, class T5>
struct DL_CryptoSchemeOptions : public DL_KeyedSchemeOptions<T1, T2>
{
	typedef T3 KeyAgreementAlgorithm;
	typedef T4 KeyDerivationAlgorithm;
	typedef T5 SymmetricEncryptionAlgorithm;
};

/// \brief Discrete Log (DL) base object implementation
/// \tparam BASE TODO
/// \tparam SCHEME_OPTIONS options for the scheme
/// \tparam KEY key used in the scheme
template <class BASE, class SCHEME_OPTIONS, class KEY>
class CRYPTOPP_NO_VTABLE DL_ObjectImplBase : public AlgorithmImpl<BASE, typename SCHEME_OPTIONS::AlgorithmInfo>
{
public:
	typedef SCHEME_OPTIONS SchemeOptions;
	typedef typename KEY::Element Element;

	virtual ~DL_ObjectImplBase() {}

	PrivateKey & AccessPrivateKey() {return m_key;}
	PublicKey & AccessPublicKey() {return m_key;}

	// KeyAccessor
	const KEY & GetKey() const {return m_key;}
	KEY & AccessKey() {return m_key;}

protected:
	typename BASE::KeyInterface & AccessKeyInterface() {return m_key;}
	const typename BASE::KeyInterface & GetKeyInterface() const {return m_key;}

	// for signature scheme
	HashIdentifier GetHashIdentifier() const
	{
		typedef typename SchemeOptions::MessageEncodingMethod::HashIdentifierLookup HashLookup;
		return HashLookup::template HashIdentifierLookup2<typename SchemeOptions::HashFunction>::Lookup();
	}
	size_t GetDigestSize() const
	{
		typedef typename SchemeOptions::HashFunction H;
		return H::DIGESTSIZE;
	}

private:
	KEY m_key;
};

/// \brief Discrete Log (DL) object implementation
/// \tparam BASE TODO
/// \tparam SCHEME_OPTIONS options for the scheme
/// \tparam KEY key used in the scheme
template <class BASE, class SCHEME_OPTIONS, class KEY>
class CRYPTOPP_NO_VTABLE DL_ObjectImpl : public DL_ObjectImplBase<BASE, SCHEME_OPTIONS, KEY>
{
public:
	typedef typename KEY::Element Element;

	virtual ~DL_ObjectImpl() {}

protected:
	const DL_ElgamalLikeSignatureAlgorithm<Element> & GetSignatureAlgorithm() const
		{return Singleton<typename SCHEME_OPTIONS::SignatureAlgorithm>().Ref();}
	const DL_KeyAgreementAlgorithm<Element> & GetKeyAgreementAlgorithm() const
		{return Singleton<typename SCHEME_OPTIONS::KeyAgreementAlgorithm>().Ref();}
	const DL_KeyDerivationAlgorithm<Element> & GetKeyDerivationAlgorithm() const
		{return Singleton<typename SCHEME_OPTIONS::KeyDerivationAlgorithm>().Ref();}
	const DL_SymmetricEncryptionAlgorithm & GetSymmetricEncryptionAlgorithm() const
		{return Singleton<typename SCHEME_OPTIONS::SymmetricEncryptionAlgorithm>().Ref();}
	HashIdentifier GetHashIdentifier() const
		{return HashIdentifier();}
	const PK_SignatureMessageEncodingMethod & GetMessageEncodingInterface() const
		{return Singleton<typename SCHEME_OPTIONS::MessageEncodingMethod>().Ref();}
};

/// \brief Discrete Log (DL) signer implementation
/// \tparam SCHEME_OPTIONS options for the scheme
template <class SCHEME_OPTIONS>
class DL_SignerImpl : public DL_ObjectImpl<DL_SignerBase<typename SCHEME_OPTIONS::Element>, SCHEME_OPTIONS, typename SCHEME_OPTIONS::PrivateKey>
{
public:
	PK_MessageAccumulator * NewSignatureAccumulator(RandomNumberGenerator &rng) const
	{
		member_ptr<PK_MessageAccumulatorBase> p(new PK_MessageAccumulatorImpl<typename SCHEME_OPTIONS::HashFunction>);
		this->RestartMessageAccumulator(rng, *p);
		return p.release();
	}
};

/// \brief Discrete Log (DL) verifier implementation
/// \tparam SCHEME_OPTIONS options for the scheme
template <class SCHEME_OPTIONS>
class DL_VerifierImpl : public DL_ObjectImpl<DL_VerifierBase<typename SCHEME_OPTIONS::Element>, SCHEME_OPTIONS, typename SCHEME_OPTIONS::PublicKey>
{
public:
	PK_MessageAccumulator * NewVerificationAccumulator() const
	{
		return new PK_MessageAccumulatorImpl<typename SCHEME_OPTIONS::HashFunction>;
	}
};

/// \brief Discrete Log (DL) encryptor implementation
/// \tparam SCHEME_OPTIONS options for the scheme
template <class SCHEME_OPTIONS>
class DL_EncryptorImpl : public DL_ObjectImpl<DL_EncryptorBase<typename SCHEME_OPTIONS::Element>, SCHEME_OPTIONS, typename SCHEME_OPTIONS::PublicKey>
{
};

/// \brief Discrete Log (DL) decryptor implementation
/// \tparam SCHEME_OPTIONS options for the scheme
template <class SCHEME_OPTIONS>
class DL_DecryptorImpl : public DL_ObjectImpl<DL_DecryptorBase<typename SCHEME_OPTIONS::Element>, SCHEME_OPTIONS, typename SCHEME_OPTIONS::PrivateKey>
{
};

// ********************************************************

/// \brief Discrete Log (DL) simple key agreement base implementation
/// \tparam T class or type
template <class T>
class CRYPTOPP_NO_VTABLE DL_SimpleKeyAgreementDomainBase : public SimpleKeyAgreementDomain
{
public:
	typedef T Element;

	virtual ~DL_SimpleKeyAgreementDomainBase() {}

	CryptoParameters & AccessCryptoParameters() {return AccessAbstractGroupParameters();}
	unsigned int AgreedValueLength() const {return GetAbstractGroupParameters().GetEncodedElementSize(false);}
	unsigned int PrivateKeyLength() const {return GetAbstractGroupParameters().GetSubgroupOrder().ByteCount();}
	unsigned int PublicKeyLength() const {return GetAbstractGroupParameters().GetEncodedElementSize(true);}

	void GeneratePrivateKey(RandomNumberGenerator &rng, byte *privateKey) const
	{
		Integer x(rng, Integer::One(), GetAbstractGroupParameters().GetMaxExponent());
		x.Encode(privateKey, PrivateKeyLength());
	}

	void GeneratePublicKey(RandomNumberGenerator &rng, const byte *privateKey, byte *publicKey) const
	{
		CRYPTOPP_UNUSED(rng);
		const DL_GroupParameters<T> &params = GetAbstractGroupParameters();
		Integer x(privateKey, PrivateKeyLength());
		Element y = params.ExponentiateBase(x);
		params.EncodeElement(true, y, publicKey);
	}

	bool Agree(byte *agreedValue, const byte *privateKey, const byte *otherPublicKey, bool validateOtherPublicKey=true) const
	{
		try
		{
			const DL_GroupParameters<T> &params = GetAbstractGroupParameters();
			Integer x(privateKey, PrivateKeyLength());
			Element w = params.DecodeElement(otherPublicKey, validateOtherPublicKey);

			Element z = GetKeyAgreementAlgorithm().AgreeWithStaticPrivateKey(
				GetAbstractGroupParameters(), w, validateOtherPublicKey, x);
			params.EncodeElement(false, z, agreedValue);
		}
		catch (DL_BadElement &)
		{
			return false;
		}
		return true;
	}

	/// \brief Retrieves a reference to the group generator
	/// \returns const reference to the group generator
	const Element &GetGenerator() const {return GetAbstractGroupParameters().GetSubgroupGenerator();}

protected:
	virtual const DL_KeyAgreementAlgorithm<Element> & GetKeyAgreementAlgorithm() const =0;
	virtual DL_GroupParameters<Element> & AccessAbstractGroupParameters() =0;
	const DL_GroupParameters<Element> & GetAbstractGroupParameters() const {return const_cast<DL_SimpleKeyAgreementDomainBase<Element> *>(this)->AccessAbstractGroupParameters();}
};

/// \brief Methods for avoiding "Small-Subgroup" attacks on Diffie-Hellman Key Agreement
/// \details Additional methods exist and include public key validation and choice of prime p.
/// \sa <A HREF="http://tools.ietf.org/html/rfc2785">Methods for Avoiding the "Small-Subgroup" Attacks on the
///   Diffie-Hellman Key Agreement Method for S/MIME</A>
enum CofactorMultiplicationOption {
	/// \brief No cofactor multiplication applied
	NO_COFACTOR_MULTIPLICTION,
	/// \brief Cofactor multiplication compatible with ordinary Diffie-Hellman
	/// \details Modifies the computation of ZZ by including j (the cofactor) in the computations and is
	///   compatible with ordinary Diffie-Hellman.
	COMPATIBLE_COFACTOR_MULTIPLICTION,
	/// \brief Cofactor multiplication incompatible with ordinary Diffie-Hellman
	/// \details Modifies the computation of ZZ by including j (the cofactor) in the computations but is
	///   not compatible with ordinary Diffie-Hellman.
	INCOMPATIBLE_COFACTOR_MULTIPLICTION};

typedef EnumToType<CofactorMultiplicationOption, NO_COFACTOR_MULTIPLICTION> NoCofactorMultiplication;
typedef EnumToType<CofactorMultiplicationOption, COMPATIBLE_COFACTOR_MULTIPLICTION> CompatibleCofactorMultiplication;
typedef EnumToType<CofactorMultiplicationOption, INCOMPATIBLE_COFACTOR_MULTIPLICTION> IncompatibleCofactorMultiplication;

/// \brief Diffie-Hellman key agreement algorithm
template <class ELEMENT, class COFACTOR_OPTION>
class DL_KeyAgreementAlgorithm_DH : public DL_KeyAgreementAlgorithm<ELEMENT>
{
public:
	typedef ELEMENT Element;

	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName()
		{return COFACTOR_OPTION::ToEnum() == INCOMPATIBLE_COFACTOR_MULTIPLICTION ? "DHC" : "DH";}

	virtual ~DL_KeyAgreementAlgorithm_DH() {}

	Element AgreeWithEphemeralPrivateKey(const DL_GroupParameters<Element> &params, const DL_FixedBasePrecomputation<Element> &publicPrecomputation, const Integer &privateExponent) const
	{
		return publicPrecomputation.Exponentiate(params.GetGroupPrecomputation(),
			COFACTOR_OPTION::ToEnum() == INCOMPATIBLE_COFACTOR_MULTIPLICTION ? privateExponent*params.GetCofactor() : privateExponent);
	}

	Element AgreeWithStaticPrivateKey(const DL_GroupParameters<Element> &params, const Element &publicElement, bool validateOtherPublicKey, const Integer &privateExponent) const
	{
		if (COFACTOR_OPTION::ToEnum() == COMPATIBLE_COFACTOR_MULTIPLICTION)
		{
			const Integer &k = params.GetCofactor();
			return params.ExponentiateElement(publicElement,
				ModularArithmetic(params.GetSubgroupOrder()).Divide(privateExponent, k)*k);
		}
		else if (COFACTOR_OPTION::ToEnum() == INCOMPATIBLE_COFACTOR_MULTIPLICTION)
			return params.ExponentiateElement(publicElement, privateExponent*params.GetCofactor());
		else
		{
			CRYPTOPP_ASSERT(COFACTOR_OPTION::ToEnum() == NO_COFACTOR_MULTIPLICTION);

			if (!validateOtherPublicKey)
				return params.ExponentiateElement(publicElement, privateExponent);

			if (params.FastSubgroupCheckAvailable())
			{
				if (!params.ValidateElement(2, publicElement, NULLPTR))
					throw DL_BadElement();
				return params.ExponentiateElement(publicElement, privateExponent);
			}
			else
			{
				const Integer e[2] = {params.GetSubgroupOrder(), privateExponent};
				Element r[2];
				params.SimultaneousExponentiate(r, publicElement, e, 2);
				if (!params.IsIdentity(r[0]))
					throw DL_BadElement();
				return r[1];
			}
		}
	}
};

// ********************************************************

/// \brief Template implementing constructors for public key algorithm classes
template <class BASE>
class CRYPTOPP_NO_VTABLE PK_FinalTemplate : public BASE
{
public:
	PK_FinalTemplate() {}

	PK_FinalTemplate(const CryptoMaterial &key)
		{this->AccessKey().AssignFrom(key);}

	PK_FinalTemplate(BufferedTransformation &bt)
		{this->AccessKey().BERDecode(bt);}

	PK_FinalTemplate(const AsymmetricAlgorithm &algorithm)
		{this->AccessKey().AssignFrom(algorithm.GetMaterial());}

	PK_FinalTemplate(const Integer &v1)
		{this->AccessKey().Initialize(v1);}

	template <class T1, class T2>
	PK_FinalTemplate(const T1 &v1, const T2 &v2)
		{this->AccessKey().Initialize(v1, v2);}

	template <class T1, class T2, class T3>
	PK_FinalTemplate(const T1 &v1, const T2 &v2, const T3 &v3)
		{this->AccessKey().Initialize(v1, v2, v3);}

	template <class T1, class T2, class T3, class T4>
	PK_FinalTemplate(const T1 &v1, const T2 &v2, const T3 &v3, const T4 &v4)
		{this->AccessKey().Initialize(v1, v2, v3, v4);}

	template <class T1, class T2, class T3, class T4, class T5>
	PK_FinalTemplate(const T1 &v1, const T2 &v2, const T3 &v3, const T4 &v4, const T5 &v5)
		{this->AccessKey().Initialize(v1, v2, v3, v4, v5);}

	template <class T1, class T2, class T3, class T4, class T5, class T6>
	PK_FinalTemplate(const T1 &v1, const T2 &v2, const T3 &v3, const T4 &v4, const T5 &v5, const T6 &v6)
		{this->AccessKey().Initialize(v1, v2, v3, v4, v5, v6);}

	template <class T1, class T2, class T3, class T4, class T5, class T6, class T7>
	PK_FinalTemplate(const T1 &v1, const T2 &v2, const T3 &v3, const T4 &v4, const T5 &v5, const T6 &v6, const T7 &v7)
		{this->AccessKey().Initialize(v1, v2, v3, v4, v5, v6, v7);}

	template <class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8>
	PK_FinalTemplate(const T1 &v1, const T2 &v2, const T3 &v3, const T4 &v4, const T5 &v5, const T6 &v6, const T7 &v7, const T8 &v8)
		{this->AccessKey().Initialize(v1, v2, v3, v4, v5, v6, v7, v8);}

	template <class T1, class T2>
	PK_FinalTemplate(T1 &v1, const T2 &v2)
		{this->AccessKey().Initialize(v1, v2);}

	template <class T1, class T2, class T3>
	PK_FinalTemplate(T1 &v1, const T2 &v2, const T3 &v3)
		{this->AccessKey().Initialize(v1, v2, v3);}

	template <class T1, class T2, class T3, class T4>
	PK_FinalTemplate(T1 &v1, const T2 &v2, const T3 &v3, const T4 &v4)
		{this->AccessKey().Initialize(v1, v2, v3, v4);}

	template <class T1, class T2, class T3, class T4, class T5>
	PK_FinalTemplate(T1 &v1, const T2 &v2, const T3 &v3, const T4 &v4, const T5 &v5)
		{this->AccessKey().Initialize(v1, v2, v3, v4, v5);}

	template <class T1, class T2, class T3, class T4, class T5, class T6>
	PK_FinalTemplate(T1 &v1, const T2 &v2, const T3 &v3, const T4 &v4, const T5 &v5, const T6 &v6)
		{this->AccessKey().Initialize(v1, v2, v3, v4, v5, v6);}

	template <class T1, class T2, class T3, class T4, class T5, class T6, class T7>
	PK_FinalTemplate(T1 &v1, const T2 &v2, const T3 &v3, const T4 &v4, const T5 &v5, const T6 &v6, const T7 &v7)
		{this->AccessKey().Initialize(v1, v2, v3, v4, v5, v6, v7);}

	template <class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8>
	PK_FinalTemplate(T1 &v1, const T2 &v2, const T3 &v3, const T4 &v4, const T5 &v5, const T6 &v6, const T7 &v7, const T8 &v8)
		{this->AccessKey().Initialize(v1, v2, v3, v4, v5, v6, v7, v8);}
};

/// \brief Base class for public key encryption standard classes.
/// \details These classes are used to select from variants of algorithms.
///   Not all standards apply to all algorithms.
struct EncryptionStandard {};

/// \brief Base class for public key signature standard classes.
/// \details These classes are used to select from variants of algorithms.
///   Not all standards apply to all algorithms.
struct SignatureStandard {};

/// \brief Trapdoor Function (TF) encryption scheme
/// \tparam STANDARD standard
/// \tparam KEYS keys used in the encryption scheme
/// \tparam ALG_INFO algorithm information
template <class KEYS, class STANDARD, class ALG_INFO>
class TF_ES;

template <class KEYS, class STANDARD, class ALG_INFO = TF_ES<KEYS, STANDARD, int> >
class TF_ES : public KEYS
{
	typedef typename STANDARD::EncryptionMessageEncodingMethod MessageEncodingMethod;

public:
	/// see EncryptionStandard for a list of standards
	typedef STANDARD Standard;
	typedef TF_CryptoSchemeOptions<ALG_INFO, KEYS, MessageEncodingMethod> SchemeOptions;

	static std::string CRYPTOPP_API StaticAlgorithmName() {return std::string(KEYS::StaticAlgorithmName()) + "/" + MessageEncodingMethod::StaticAlgorithmName();}

	/// implements PK_Decryptor interface
	typedef PK_FinalTemplate<TF_DecryptorImpl<SchemeOptions> > Decryptor;
	/// implements PK_Encryptor interface
	typedef PK_FinalTemplate<TF_EncryptorImpl<SchemeOptions> > Encryptor;
};

/// \brief Trapdoor Function (TF) Signature Scheme
/// \tparam STANDARD standard
/// \tparam H hash function
/// \tparam KEYS keys used in the signature scheme
/// \tparam ALG_INFO algorithm information
template <class KEYS, class STANDARD, class H, class ALG_INFO>
class TF_SS;

template <class KEYS, class STANDARD, class H, class ALG_INFO = TF_SS<KEYS, STANDARD, H, int> >
class TF_SS : public KEYS
{
public:
	/// see SignatureStandard for a list of standards
	typedef STANDARD Standard;
	typedef typename Standard::SignatureMessageEncodingMethod MessageEncodingMethod;
	typedef TF_SignatureSchemeOptions<ALG_INFO, KEYS, MessageEncodingMethod, H> SchemeOptions;

	static std::string CRYPTOPP_API StaticAlgorithmName() {return std::string(KEYS::StaticAlgorithmName()) + "/" + MessageEncodingMethod::StaticAlgorithmName() + "(" + H::StaticAlgorithmName() + ")";}

	/// implements PK_Signer interface
	typedef PK_FinalTemplate<TF_SignerImpl<SchemeOptions> > Signer;
	/// implements PK_Verifier interface
	typedef PK_FinalTemplate<TF_VerifierImpl<SchemeOptions> > Verifier;
};

/// \brief Discrete Log (DL) signature scheme
/// \tparam KEYS keys used in the signature scheme
/// \tparam SA signature algorithm
/// \tparam MEM message encoding method
/// \tparam H hash function
/// \tparam ALG_INFO algorithm information
template <class KEYS, class SA, class MEM, class H, class ALG_INFO>
class DL_SS;

template <class KEYS, class SA, class MEM, class H, class ALG_INFO = DL_SS<KEYS, SA, MEM, H, int> >
class DL_SS : public KEYS
{
	typedef DL_SignatureSchemeOptions<ALG_INFO, KEYS, SA, MEM, H> SchemeOptions;

public:
	static std::string StaticAlgorithmName() {return SA::StaticAlgorithmName() + std::string("/EMSA1(") + H::StaticAlgorithmName() + ")";}

	/// implements PK_Signer interface
	typedef PK_FinalTemplate<DL_SignerImpl<SchemeOptions> > Signer;
	/// implements PK_Verifier interface
	typedef PK_FinalTemplate<DL_VerifierImpl<SchemeOptions> > Verifier;
};

/// \brief Discrete Log (DL) encryption scheme
/// \tparam KEYS keys used in the encryption scheme
/// \tparam AA key agreement algorithm
/// \tparam DA key derivation algorithm
/// \tparam EA encryption algorithm
/// \tparam ALG_INFO algorithm information
template <class KEYS, class AA, class DA, class EA, class ALG_INFO>
class DL_ES : public KEYS
{
	typedef DL_CryptoSchemeOptions<ALG_INFO, KEYS, AA, DA, EA> SchemeOptions;

public:
	/// implements PK_Decryptor interface
	typedef PK_FinalTemplate<DL_DecryptorImpl<SchemeOptions> > Decryptor;
	/// implements PK_Encryptor interface
	typedef PK_FinalTemplate<DL_EncryptorImpl<SchemeOptions> > Encryptor;
};

NAMESPACE_END

#if CRYPTOPP_MSC_VERSION
# pragma warning(pop)
#endif

#endif
