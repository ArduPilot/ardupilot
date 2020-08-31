// sha.h - originally written and placed in the public domain by Wei Dai

/// \file sha.h
/// \brief Classes for SHA-1 and SHA-2 family of message digests
/// \since SHA1 since Crypto++ 1.0, SHA2 since Crypto++ 4.0, ARMv8 SHA since
///   Crypto++ 6.0, Intel SHA since Crypto++ 6.0, Power8 SHA since Crypto++ 6.1

#ifndef CRYPTOPP_SHA_H
#define CRYPTOPP_SHA_H

#include "config.h"
#include "iterhash.h"

// Clang 3.3 integrated assembler crash on Linux. Clang 3.4 due to compiler
// error with .intel_syntax, http://llvm.org/bugs/show_bug.cgi?id=24232
#if CRYPTOPP_BOOL_X32 || defined(CRYPTOPP_DISABLE_MIXED_ASM)
# define CRYPTOPP_DISABLE_SHA_ASM 1
#endif

NAMESPACE_BEGIN(CryptoPP)

/// \brief SHA-1 message digest
/// \sa <a href="http://www.weidai.com/scan-mirror/md.html#SHA-1">SHA-1</a>
/// \since SHA1 since Crypto++ 1.0, SHA2 since Crypto++ 4.0, ARMv8 SHA since
///   Crypto++ 6.0, Intel SHA since Crypto++ 6.0
class CRYPTOPP_DLL SHA1 : public IteratedHashWithStaticTransform<word32, BigEndian, 64, 20, SHA1>
{
public:
	/// \brief Initialize state array
	/// \param state the state of the hash
	/// \details InitState sets a state array to SHA1 initial values
	/// \details Hashes which derive from IteratedHashWithStaticTransform provide static
	///   member functions InitState and Transform. External classes, like SEAL and MDC,
	///   can initialize state with a user provided key and operate the hash on the data
	///   with the user supplied state.
	/// \note On Intel platforms the state array must be 16-byte aligned for SSE2.
	static void CRYPTOPP_API InitState(HashWordType *state);
	/// \brief Operate the hash
	/// \param digest the state of the hash
	/// \param data the data to be digested
	/// \details Transform operates the hash on <tt>data</tt>. When the call is invoked
	///   <tt>digest</tt> holds initial state. Upon return <tt>digest</tt> holds the hash
	///   or updated state.
	/// \details Hashes which derive from IteratedHashWithStaticTransform provide static
	///   member functions InitState and Transform. External classes, like SEAL and MDC,
	///   can initialize state with a user provided key and operate the hash on the data
	///   with the user supplied state.
	/// \note On Intel platforms the state array and data must be 16-byte aligned for SSE2.
	static void CRYPTOPP_API Transform(HashWordType *digest, const HashWordType *data);
	/// \brief The algorithm name
	/// \returns C-style string "SHA-1"
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "SHA-1";}
	// Algorithm class
	std::string AlgorithmProvider() const;

protected:
	size_t HashMultipleBlocks(const HashWordType *input, size_t length);
};

/// \brief SHA-256 message digest
/// \sa <a href="http://www.weidai.com/scan-mirror/md.html#SHA-256">SHA-256</a>
/// \since SHA2 since Crypto++ 4.0, ARMv8 SHA since Crypto++ 6.0,
///   Intel SHA since Crypto++ 6.0, Power8 SHA since Crypto++ 6.1
class CRYPTOPP_DLL SHA256 : public IteratedHashWithStaticTransform<word32, BigEndian, 64, 32, SHA256, 32, true>
{
public:
	/// \brief Initialize state array
	/// \param state the state of the hash
	/// \details InitState sets a state array to SHA256 initial values
	/// \details Hashes which derive from IteratedHashWithStaticTransform provide static
	///   member functions InitState and Transform. External classes, like SEAL and MDC,
	///   can initialize state with a user provided key and operate the hash on the data
	///   with the user supplied state.
	/// \note On Intel platforms the state array must be 16-byte aligned for SSE2.
	static void CRYPTOPP_API InitState(HashWordType *state);
	/// \brief Operate the hash
	/// \param digest the state of the hash
	/// \param data the data to be digested
	/// \details Transform operates the hash on <tt>data</tt>. When the call is invoked
	///   <tt>digest</tt> holds initial state. Upon return <tt>digest</tt> holds the hash
	///   or updated state.
	/// \details Hashes which derive from IteratedHashWithStaticTransform provide static
	///   member functions InitState and Transform. External classes, like SEAL and MDC,
	///   can initialize state with a user provided key and operate the hash on the data
	///   with the user supplied state.
	/// \note On Intel platforms the state array and data must be 16-byte aligned for SSE2.
	static void CRYPTOPP_API Transform(HashWordType *digest, const HashWordType *data);
	/// \brief The algorithm name
	/// \returns C-style string "SHA-256"
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "SHA-256";}

	// Algorithm class
	std::string AlgorithmProvider() const;

protected:
	size_t HashMultipleBlocks(const HashWordType *input, size_t length);
};

/// \brief SHA-224 message digest
/// \sa <a href="http://www.weidai.com/scan-mirror/md.html#SHA-224">SHA-224</a>
/// \since SHA2 since Crypto++ 4.0, ARMv8 SHA since Crypto++ 6.0,
///   Intel SHA since Crypto++ 6.0, Power8 SHA since Crypto++ 6.1
class CRYPTOPP_DLL SHA224 : public IteratedHashWithStaticTransform<word32, BigEndian, 64, 32, SHA224, 28, true>
{
public:
	/// \brief Initialize state array
	/// \param state the state of the hash
	/// \details InitState sets a state array to SHA224 initial values
	/// \details Hashes which derive from IteratedHashWithStaticTransform provide static
	///   member functions InitState and Transform. External classes, like SEAL and MDC,
	///   can initialize state with a user provided key and operate the hash on the data
	///   with the user supplied state.
	/// \note On Intel platforms the state array must be 16-byte aligned for SSE2.
	static void CRYPTOPP_API InitState(HashWordType *state);
	/// \brief Operate the hash
	/// \param digest the state of the hash
	/// \param data the data to be digested
	/// \details Transform operates the hash on <tt>data</tt>. When the call is invoked
	///   <tt>digest</tt> holds initial state. Upon return <tt>digest</tt> holds the hash
	///   or updated state.
	/// \details Hashes which derive from IteratedHashWithStaticTransform provide static
	///   member functions InitState and Transform. External classes, like SEAL and MDC,
	///   can initialize state with a user provided key and operate the hash on the data
	///   with the user supplied state.
	/// \note On Intel platforms the state array and data must be 16-byte aligned for SSE2.
	static void CRYPTOPP_API Transform(HashWordType *digest, const HashWordType *data) {SHA256::Transform(digest, data);}
	/// \brief The algorithm name
	/// \returns C-style string "SHA-224"
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "SHA-224";}

	// Algorithm class
	std::string AlgorithmProvider() const;

protected:
	size_t HashMultipleBlocks(const HashWordType *input, size_t length);
};

/// \brief SHA-512 message digest
/// \sa <a href="http://www.weidai.com/scan-mirror/md.html#SHA-512">SHA-512</a>
/// \since SHA2 since Crypto++ 4.0, Power8 SHA since Crypto++ 6.1
class CRYPTOPP_DLL SHA512 : public IteratedHashWithStaticTransform<word64, BigEndian, 128, 64, SHA512, 64, true>
{
public:
	/// \brief Initialize state array
	/// \param state the state of the hash
	/// \details InitState sets a state array to SHA512 initial values
	/// \details Hashes which derive from IteratedHashWithStaticTransform provide static
	///   member functions InitState and Transform. External classes, like SEAL and MDC,
	///   can initialize state with a user provided key and operate the hash on the data
	///   with the user supplied state.
	/// \note On Intel platforms the state array must be 16-byte aligned for SSE2.
	static void CRYPTOPP_API InitState(HashWordType *state);
	/// \brief Operate the hash
	/// \param digest the state of the hash
	/// \param data the data to be digested
	/// \details Transform operates the hash on <tt>data</tt>. When the call is invoked
	///   <tt>digest</tt> holds initial state. Upon return <tt>digest</tt> holds the hash
	///   or updated state.
	/// \details Hashes which derive from IteratedHashWithStaticTransform provide static
	///   member functions InitState and Transform. External classes, like SEAL and MDC,
	///   can initialize state with a user provided key and operate the hash on the data
	///   with the user supplied state.
	/// \note On Intel platforms the state array and data must be 16-byte aligned for SSE2.
	static void CRYPTOPP_API Transform(HashWordType *digest, const HashWordType *data);
	/// \brief The algorithm name
	/// \returns C-style string "SHA-512"
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "SHA-512";}

	// Algorithm class
	std::string AlgorithmProvider() const;
};

/// \brief SHA-384 message digest
/// \sa <a href="http://www.weidai.com/scan-mirror/md.html#SHA-384">SHA-384</a>
/// \since SHA2 since Crypto++ 4.0, Power8 SHA since Crypto++ 6.1
class CRYPTOPP_DLL SHA384 : public IteratedHashWithStaticTransform<word64, BigEndian, 128, 64, SHA384, 48, true>
{
public:
	/// \brief Initialize state array
	/// \param state the state of the hash
	/// \details InitState sets a state array to SHA384 initial values
	/// \details Hashes which derive from IteratedHashWithStaticTransform provide static
	///   member functions InitState and Transform. External classes, like SEAL and MDC,
	///   can initialize state with a user provided key and operate the hash on the data
	///   with the user supplied state.
	/// \note On Intel platforms the state array must be 16-byte aligned for SSE2.
	static void CRYPTOPP_API InitState(HashWordType *state);
	/// \brief Operate the hash
	/// \param digest the state of the hash
	/// \param data the data to be digested
	/// \details Transform operates the hash on <tt>data</tt>. When the call is invoked
	///   <tt>digest</tt> holds initial state. Upon return <tt>digest</tt> holds the hash
	///   or updated state.
	/// \details Hashes which derive from IteratedHashWithStaticTransform provide static
	///   member functions InitState and Transform. External classes, like SEAL and MDC,
	///   can initialize state with a user provided key and operate the hash on the data
	///   with the user supplied state.
	/// \note On Intel platforms the state array and data must be 16-byte aligned for SSE2.
	static void CRYPTOPP_API Transform(HashWordType *digest, const HashWordType *data) {SHA512::Transform(digest, data);}
	/// \brief The algorithm name
	/// \returns C-style string "SHA-384"
	CRYPTOPP_STATIC_CONSTEXPR const char* CRYPTOPP_API StaticAlgorithmName() {return "SHA-384";}

	// Algorithm class
	std::string AlgorithmProvider() const;
};

NAMESPACE_END

#endif
