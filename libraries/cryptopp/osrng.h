// osrng.h - originally written and placed in the public domain by Wei Dai

/// \file osrng.h
/// \brief Classes for access to the operating system's random number generators

#ifndef CRYPTOPP_OSRNG_H
#define CRYPTOPP_OSRNG_H

#include "config.h"

#if !defined(NO_OS_DEPENDENCE) && defined(OS_RNG_AVAILABLE)

#include "cryptlib.h"
#include "randpool.h"
#include "smartptr.h"
#include "fips140.h"
#include "hkdf.h"
#include "rng.h"
#include "aes.h"
#include "sha.h"

NAMESPACE_BEGIN(CryptoPP)

/// \brief Exception thrown when an operating system error is encountered
class CRYPTOPP_DLL OS_RNG_Err : public Exception
{
public:
	/// \brief Constructs an OS_RNG_Err
	/// \param operation the operation or API call when the error occurs
	OS_RNG_Err(const std::string &operation);
};

#ifdef NONBLOCKING_RNG_AVAILABLE

#ifdef CRYPTOPP_WIN32_AVAILABLE
/// \brief Wrapper for Microsoft crypto service provider
/// \sa \def USE_MS_CRYPTOAPI, \def USE_MS_CNGAPI
class CRYPTOPP_DLL MicrosoftCryptoProvider
{
public:
	/// \brief Construct a MicrosoftCryptoProvider
	MicrosoftCryptoProvider();
	~MicrosoftCryptoProvider();

// type HCRYPTPROV and BCRYPT_ALG_HANDLE, avoid #include <windows.h>
#if defined(USE_MS_CRYPTOAPI)
# if defined(__CYGWIN__) && defined(__x86_64__)
	typedef unsigned long long ProviderHandle;
# elif defined(WIN64) || defined(_WIN64)
	typedef unsigned __int64 ProviderHandle;
# else
	typedef unsigned long ProviderHandle;
# endif
#elif defined(USE_MS_CNGAPI)
	typedef void *PVOID;
	typedef PVOID ProviderHandle;
#endif // USE_MS_CRYPTOAPI or USE_MS_CNGAPI

	/// \brief Retrieves the provider handle
	/// \returns CryptoAPI provider handle
	/// \details If USE_MS_CRYPTOAPI is in effect, then CryptAcquireContext()
	///  acquires then handle and CryptReleaseContext() releases the handle
	///  upon destruction. If USE_MS_CNGAPI is in effect, then
	///  BCryptOpenAlgorithmProvider() acquires then handle and
	///  BCryptCloseAlgorithmProvider() releases the handle upon destruction.
	ProviderHandle GetProviderHandle() const {return m_hProvider;}

private:
	ProviderHandle m_hProvider;
};

#if defined(_MSC_VER) && defined(USE_MS_CRYPTOAPI)
# pragma comment(lib, "advapi32.lib")
#endif

#if defined(_MSC_VER) && defined(USE_MS_CNGAPI)
# pragma comment(lib, "bcrypt.lib")
#endif

#endif // CRYPTOPP_WIN32_AVAILABLE

/// \brief Wrapper class for /dev/random and /dev/srandom
/// \details Encapsulates CryptoAPI's CryptGenRandom() or CryptoNG's BCryptGenRandom()
///  on Windows, or /dev/urandom on Unix and compatibles.
class CRYPTOPP_DLL NonblockingRng : public RandomNumberGenerator
{
public:
	CRYPTOPP_STATIC_CONSTEXPR const char* StaticAlgorithmName() { return "NonblockingRng"; }

	~NonblockingRng();

	/// \brief Construct a NonblockingRng
	NonblockingRng();

	/// \brief Generate random array of bytes
	/// \param output the byte buffer
	/// \param size the length of the buffer, in bytes
	/// \details GenerateIntoBufferedTransformation() calls are routed to GenerateBlock().
	void GenerateBlock(byte *output, size_t size);

protected:
#ifdef CRYPTOPP_WIN32_AVAILABLE
	MicrosoftCryptoProvider m_Provider;
#else
	int m_fd;
#endif
};

#endif

#if defined(BLOCKING_RNG_AVAILABLE) || defined(CRYPTOPP_DOXYGEN_PROCESSING)

/// \brief Wrapper class for /dev/random and /dev/srandom
/// \details Encapsulates /dev/random on Linux, OS X and Unix; and /dev/srandom on the BSDs.
/// \note On Linux the /dev/random interface is effectively deprecated. According to the
///  Kernel Crypto developers, /dev/urandom or getrandom(2) should be used instead. Also
///  see <A HREF="https://lkml.org/lkml/2017/7/20/993">[RFC PATCH v12 3/4] Linux Random
///  Number Generator</A> on the kernel-crypto mailing list.
class CRYPTOPP_DLL BlockingRng : public RandomNumberGenerator
{
public:
	CRYPTOPP_STATIC_CONSTEXPR const char* StaticAlgorithmName() { return "BlockingRng"; }

	~BlockingRng();

	/// \brief Construct a BlockingRng
	BlockingRng();

	/// \brief Generate random array of bytes
	/// \param output the byte buffer
	/// \param size the length of the buffer, in bytes
	/// \details GenerateIntoBufferedTransformation() calls are routed to GenerateBlock().
	void GenerateBlock(byte *output, size_t size);

protected:
	int m_fd;
};

#endif

/// OS_GenerateRandomBlock
/// \brief Generate random array of bytes
/// \param blocking specifies whther a bobcking or non-blocking generator should be used
/// \param output the byte buffer
/// \param size the length of the buffer, in bytes
/// \details OS_GenerateRandomBlock() uses the underlying operating system's
///  random number generator. On Windows, CryptGenRandom() is called using NonblockingRng.
/// \details On Unix and compatibles, /dev/urandom is called if blocking is false using
///  NonblockingRng. If blocking is true, then either /dev/randomd or /dev/srandom is used
///  by way of BlockingRng, if available.
CRYPTOPP_DLL void CRYPTOPP_API OS_GenerateRandomBlock(bool blocking, byte *output, size_t size);

/// \brief Automatically Seeded Randomness Pool
/// \details This class seeds itself using an operating system provided RNG.
///  AutoSeededRandomPool was suggested by Leonard Janke.
/// \details You should reseed the generator after a fork() to avoid multiple generators
///  with the same internal state.
class CRYPTOPP_DLL AutoSeededRandomPool : public RandomPool
{
public:
	CRYPTOPP_STATIC_CONSTEXPR const char* StaticAlgorithmName() { return "AutoSeededRandomPool"; }

	~AutoSeededRandomPool() {}

	/// \brief Construct an AutoSeededRandomPool
	/// \param blocking controls seeding with BlockingRng or NonblockingRng
	/// \param seedSize the size of the seed, in bytes
	/// \details Use blocking to choose seeding with BlockingRng or NonblockingRng.
	///  The parameter is ignored if only one of these is available.
	explicit AutoSeededRandomPool(bool blocking = false, unsigned int seedSize = 32)
		{Reseed(blocking, seedSize);}

	/// \brief Reseed an AutoSeededRandomPool
	/// \param blocking controls seeding with BlockingRng or NonblockingRng
	/// \param seedSize the size of the seed, in bytes
	void Reseed(bool blocking = false, unsigned int seedSize = 32);
};

/// \tparam BLOCK_CIPHER a block cipher
/// \brief Automatically Seeded X9.17 RNG
/// \details AutoSeededX917RNG is from ANSI X9.17 Appendix C, seeded using an OS provided RNG.
///  If 3-key TripleDES (DES_EDE3) is used, then its a X9.17 conforming generator. If AES is
///  used, then its a X9.31 conforming generator.
/// \details Though ANSI X9 prescribes 3-key TripleDES, the template parameter BLOCK_CIPHER
///  can be any BlockTransformation derived class.
/// \details You should reseed the generator after a fork() to avoid multiple generators
///  with the same internal state.
/// \sa X917RNG, DefaultAutoSeededRNG
template <class BLOCK_CIPHER>
class AutoSeededX917RNG : public RandomNumberGenerator, public NotCopyable
{
public:
	static std::string StaticAlgorithmName() {
		return std::string("AutoSeededX917RNG(") + BLOCK_CIPHER::StaticAlgorithmName() + std::string(")");
	}

	~AutoSeededX917RNG() {}

	/// \brief Construct an AutoSeededX917RNG
	/// \param blocking controls seeding with BlockingRng or NonblockingRng
	/// \param autoSeed controls auto seeding of the generator
	/// \details Use blocking to choose seeding with BlockingRng or NonblockingRng.
	///  The parameter is ignored if only one of these is available.
	/// \sa X917RNG
	explicit AutoSeededX917RNG(bool blocking = false, bool autoSeed = true)
		{if (autoSeed) Reseed(blocking);}

	/// \brief Reseed an AutoSeededX917RNG
	/// \param blocking controls seeding with BlockingRng or NonblockingRng
	/// \param additionalEntropy additional entropy to add to the generator
	/// \param length the size of the additional entropy, in bytes
	/// \details Internally, the generator uses SHA256 to extract the entropy from
	///  from the seed and then stretch the material for the block cipher's key
	///  and initialization vector.
	void Reseed(bool blocking = false, const byte *additionalEntropy = NULLPTR, size_t length = 0);

	/// \brief Deterministically reseed an AutoSeededX917RNG for testing
	/// \param key the key to use for the deterministic reseeding
	/// \param keylength the size of the key, in bytes
	/// \param seed the seed to use for the deterministic reseeding
	/// \param timeVector a time vector to use for deterministic reseeding
	/// \details This is a testing interface for testing purposes, and should \a NOT
	///  be used in production.
	void Reseed(const byte *key, size_t keylength, const byte *seed, const byte *timeVector);

	bool CanIncorporateEntropy() const {return true;}
	void IncorporateEntropy(const byte *input, size_t length) {Reseed(false, input, length);}
	void GenerateIntoBufferedTransformation(BufferedTransformation &target, const std::string &channel, lword length)
		{m_rng->GenerateIntoBufferedTransformation(target, channel, length);}

	std::string AlgorithmProvider() const;

private:
	member_ptr<RandomNumberGenerator> m_rng;
};

template <class BLOCK_CIPHER>
void AutoSeededX917RNG<BLOCK_CIPHER>::Reseed(const byte *key, size_t keylength, const byte *seed, const byte *timeVector)
{
	m_rng.reset(new X917RNG(new typename BLOCK_CIPHER::Encryption(key, keylength), seed, timeVector));
}

template <class BLOCK_CIPHER>
void AutoSeededX917RNG<BLOCK_CIPHER>::Reseed(bool blocking, const byte *input, size_t length)
{
	enum {BlockSize=BLOCK_CIPHER::BLOCKSIZE};
	enum {KeyLength=BLOCK_CIPHER::DEFAULT_KEYLENGTH};
	enum {SeedSize=BlockSize + KeyLength};

	SecByteBlock seed(SeedSize), temp(SeedSize);
	const byte label[] = "X9.17 key generation";
	const byte *key=NULLPTR;

	do
	{
		OS_GenerateRandomBlock(blocking, temp, temp.size());

		HKDF<SHA256> hkdf;
		hkdf.DeriveKey(
			seed, seed.size(),  // derived secret
			temp, temp.size(),  // instance secret
			input, length,      // user secret
			label, 20           // unique label
		);

		key = seed + BlockSize;
	}	// check that seed and key don't have same value
	while (memcmp(key, seed, STDMIN((size_t)BlockSize, (size_t)KeyLength)) == 0);

	Reseed(key, KeyLength, seed, NULLPTR);
}

template <class BLOCK_CIPHER>
std::string AutoSeededX917RNG<BLOCK_CIPHER>::AlgorithmProvider() const
{
	// Hack for now... We need to instantiate one
	typename BLOCK_CIPHER::Encryption bc;
	return bc.AlgorithmProvider();
}

CRYPTOPP_DLL_TEMPLATE_CLASS AutoSeededX917RNG<AES>;

#if defined(CRYPTOPP_DOXYGEN_PROCESSING)
/// \brief A typedef providing a default generator
/// \details DefaultAutoSeededRNG is a typedef of either AutoSeededX917RNG<AES> or AutoSeededRandomPool.
///  If CRYPTOPP_ENABLE_COMPLIANCE_WITH_FIPS_140_2 is defined, then DefaultAutoSeededRNG is
///  AutoSeededX917RNG<AES>. Otherwise, DefaultAutoSeededRNG is AutoSeededRandomPool.
/// \details You should reseed the generator after a fork() to avoid multiple generators
///  with the same internal state.
class DefaultAutoSeededRNG {}
#else
// AutoSeededX917RNG<AES> in FIPS mode, otherwise it's AutoSeededRandomPool
#if CRYPTOPP_ENABLE_COMPLIANCE_WITH_FIPS_140_2
typedef AutoSeededX917RNG<AES> DefaultAutoSeededRNG;
#else
typedef AutoSeededRandomPool DefaultAutoSeededRNG;
#endif
#endif // CRYPTOPP_DOXYGEN_PROCESSING

NAMESPACE_END

#endif

#endif
