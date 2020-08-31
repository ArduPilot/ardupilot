// randpool.h - originally written and placed in the public domain by Wei Dai
//              OldRandPool added by JW in August, 2017.

/// \file randpool.h
/// \brief Class file for Randomness Pool
/// \details RandomPool can be used to generate cryptographic quality pseudorandom bytes
///  after seeding the pool with IncorporateEntropy(). Internally, the generator uses
///  AES-256 to produce the stream. Entropy is stirred in using SHA-256.
/// \details RandomPool used to follow the design of randpool in PGP 2.6.x. At version 5.5
///  RandomPool was redesigned to reduce the risk of reusing random numbers after state
///  rollback (which may occur when running in a virtual machine like VMware or a hosted
///  environment).
/// \details If you need the pre-Crypto++ 5.5 generator then use OldRandomPool class. You
///  should migrate away from OldRandomPool at the earliest opportunity. Use RandomPool
///  or AutoSeededRandomPool instead.
/// \since Crypto++ 4.0 (PGP 2.6.x style), Crypto++ 5.5 (AES-256 based)

#ifndef CRYPTOPP_RANDPOOL_H
#define CRYPTOPP_RANDPOOL_H

#include "cryptlib.h"
#include "filters.h"
#include "secblock.h"
#include "smartptr.h"
#include "aes.h"

NAMESPACE_BEGIN(CryptoPP)

/// \brief Randomness Pool based on AES-256
/// \details RandomPool can be used to generate cryptographic quality pseudorandom bytes
///  after seeding the pool with IncorporateEntropy(). Internally, the generator uses
///  AES-256 to produce the stream. Entropy is stirred in using SHA-256.
/// \details RandomPool used to follow the design of randpool in PGP 2.6.x. At version 5.5
///  RandomPool was redesigned to reduce the risk of reusing random numbers after state
///  rollback, which may occur when running in a virtual machine like VMware or a hosted
///  environment.
/// \details You should reseed the generator after a fork() to avoid multiple generators
///  with the same internal state.
/// \details If you need the pre-Crypto++ 5.5 generator then use OldRandomPool class. You
///  should migrate away from OldRandomPool at the earliest opportunity.
/// \sa OldRandomPool
/// \since Crypto++ 4.0 (PGP 2.6.x style), Crypto++ 5.5 (AES-256 based)
class CRYPTOPP_DLL RandomPool : public RandomNumberGenerator, public NotCopyable
{
public:
	/// \brief Construct a RandomPool
	RandomPool();

	bool CanIncorporateEntropy() const {return true;}
	void IncorporateEntropy(const byte *input, size_t length);
	void GenerateIntoBufferedTransformation(BufferedTransformation &target, const std::string &channel, lword size);

private:
	FixedSizeAlignedSecBlock<byte, 16, true> m_seed;
	FixedSizeAlignedSecBlock<byte, 32> m_key;
	member_ptr<BlockCipher> m_pCipher;
	bool m_keySet;
};

/// \brief Randomness Pool based on PGP 2.6.x with MDC
/// \details If you need the pre-Crypto++ 5.5 generator then use OldRandomPool class. The
///  OldRandomPool also provides the modern nterface, including <tt>CanIncorporateEntropy</tt>,
///  <tt>IncorporateEntropy</tt> and <tt>GenerateIntoBufferedTransformation</tt>.
/// \details You should reseed the generator after a fork() to avoid multiple generators
///  with the same internal state.
/// \details You should migrate away from OldRandomPool at the earliest opportunity. Use a
///  modern random number generator or key derivation function, like AutoSeededRandomPool or
///  HKDF.
/// \warning This class uses an old style PGP 2.6.x with MDC. The generator risks reusing
///  random numbers after state rollback. You should migrate away from OldRandomPool at
///  the earliest opportunity.
/// \sa RandomPool, AutoSeededRandomPool, HKDF, P1363_KDF2, PKCS12_PBKDF, PKCS5_PBKDF2_HMAC
/// \since Crypto++ 6.0 (PGP 2.6.x style)
class CRYPTOPP_DLL OldRandomPool : public RandomNumberGenerator
{
public:
	/// \brief Construct an OldRandomPool
	/// \param poolSize internal pool size of the generator
	/// \details poolSize must be greater than 16
	OldRandomPool(unsigned int poolSize=384);

	// RandomNumberGenerator interface (Crypto++ 5.5 and above)
	bool CanIncorporateEntropy() const {return true;}
	void IncorporateEntropy(const byte *input, size_t length);
	void GenerateIntoBufferedTransformation(BufferedTransformation &target, const std::string &channel, lword size);

	byte GenerateByte();
	void GenerateBlock(byte *output, size_t size);

protected:
	void Stir();

private:
	SecByteBlock pool, key;
	size_t addPos, getPos;
};

NAMESPACE_END

#endif
