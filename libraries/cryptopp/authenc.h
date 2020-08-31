// authenc.h - originally written and placed in the public domain by Wei Dai

/// \file
/// \brief Classes for authenticated encryption modes of operation
/// \details Authenticated encryption (AE) schemes combine confidentiality and authenticity
///   into a single mode of operation They gained traction in the early 2000's because manually
///   combining them was error prone for the typical developer. Around that time, the desire to
///   authenticate but not ecrypt additional data (AAD) was also identified. When both features
///   are available from a scheme, the system is referred to as an AEAD scheme.
/// \details Crypto++ provides four authenticated encryption modes of operation - CCM, EAX, GCM
///   and OCB mode. All modes derive from AuthenticatedSymmetricCipherBase() and the
///   motivation for the API, like calling AAD a &quot;header&quot;, can be found in Bellare,
///   Rogaway and Wagner's <A HREF="http://web.cs.ucdavis.edu/~rogaway/papers/eax.pdf">The EAX
///   Mode of Operation</A>. The EAX paper suggested a basic API to help standardize AEAD
///   schemes in software and promote adoption of the modes.
/// \sa <A HREF="http://www.cryptopp.com/wiki/Authenticated_Encryption">Authenticated
///   Encryption</A> on the Crypto++ wiki.
/// \since Crypto++ 5.6.0

#ifndef CRYPTOPP_AUTHENC_H
#define CRYPTOPP_AUTHENC_H

#include "cryptlib.h"
#include "secblock.h"

NAMESPACE_BEGIN(CryptoPP)

/// \brief Base class for authenticated encryption modes of operation
/// \details AuthenticatedSymmetricCipherBase() serves as a base implementation for one direction
///   (encryption or decryption) of a stream cipher or block cipher mode with authentication.
/// \details Crypto++ provides four authenticated encryption modes of operation - CCM, EAX, GCM
///   and OCB mode. All modes derive from AuthenticatedSymmetricCipherBase() and the
///   motivation for the API, like calling AAD a &quot;header&quot;, can be found in Bellare,
///   Rogaway and Wagner's <A HREF="http://web.cs.ucdavis.edu/~rogaway/papers/eax.pdf">The EAX
///   Mode of Operation</A>. The EAX paper suggested a basic API to help standardize AEAD
///   schemes in software and promote adoption of the modes.
/// \sa <A HREF="http://www.cryptopp.com/wiki/Authenticated_Encryption">Authenticated
///   Encryption</A> on the Crypto++ wiki.
/// \since Crypto++ 5.6.0
class CRYPTOPP_DLL CRYPTOPP_NO_VTABLE AuthenticatedSymmetricCipherBase : public AuthenticatedSymmetricCipher
{
public:
	AuthenticatedSymmetricCipherBase() : m_totalHeaderLength(0), m_totalMessageLength(0),
		m_totalFooterLength(0), m_bufferedDataLength(0), m_state(State_Start) {}

	// StreamTransformation interface
	bool IsRandomAccess() const {return false;}
	bool IsSelfInverting() const {return true;}

	void SetKey(const byte *userKey, size_t keylength, const NameValuePairs &params);
	void Restart() {if (m_state > State_KeySet) m_state = State_KeySet;}
	void Resynchronize(const byte *iv, int length=-1);
	void Update(const byte *input, size_t length);
	void ProcessData(byte *outString, const byte *inString, size_t length);
	void TruncatedFinal(byte *mac, size_t macSize);

protected:
	void UncheckedSetKey(const byte * key, unsigned int length,const CryptoPP::NameValuePairs &params)
		{CRYPTOPP_UNUSED(key), CRYPTOPP_UNUSED(length), CRYPTOPP_UNUSED(params); CRYPTOPP_ASSERT(false);}

	void AuthenticateData(const byte *data, size_t len);
	const SymmetricCipher & GetSymmetricCipher() const
		{return const_cast<AuthenticatedSymmetricCipherBase *>(this)->AccessSymmetricCipher();}

	virtual SymmetricCipher & AccessSymmetricCipher() =0;
	virtual bool AuthenticationIsOnPlaintext() const =0;
	virtual unsigned int AuthenticationBlockSize() const =0;
	virtual void SetKeyWithoutResync(const byte *userKey, size_t keylength, const NameValuePairs &params) =0;
	virtual void Resync(const byte *iv, size_t len) =0;
	virtual size_t AuthenticateBlocks(const byte *data, size_t len) =0;
	virtual void AuthenticateLastHeaderBlock() =0;
	virtual void AuthenticateLastConfidentialBlock() {}
	virtual void AuthenticateLastFooterBlock(byte *mac, size_t macSize) =0;

	// State_AuthUntransformed: authentication is applied to plain text (Authenticate-then-Encrypt)
	// State_AuthTransformed: authentication is applied to cipher text (Encrypt-then-Authenticate)
	enum State {State_Start, State_KeySet, State_IVSet, State_AuthUntransformed, State_AuthTransformed, State_AuthFooter};

	AlignedSecByteBlock m_buffer;
	lword m_totalHeaderLength, m_totalMessageLength, m_totalFooterLength;
	unsigned int m_bufferedDataLength;
	State m_state;
};

NAMESPACE_END

#endif
