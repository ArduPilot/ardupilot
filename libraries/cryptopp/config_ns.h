// config_ns.h - written and placed in public domain by Jeffrey Walton
//               the bits that make up this source file are from the
//               library's monolithic config.h.

/// \file config_ns.h
/// \brief Library configuration file
/// \details <tt>config.h</tt> was split into components in May 2019 to better
///  integrate with Autoconf and its feature tests. The splitting occured so
///  users could continue to include <tt>config.h</tt> while allowing Autoconf
///  to write new <tt>config_asm.h</tt> and new <tt>config_cxx.h</tt> using
///  its feature tests.
/// \sa <A HREF="https://github.com/weidai11/cryptopp/issues/835">Issue 835</A>
/// \since Crypto++ 8.3

#ifndef CRYPTOPP_CONFIG_NAMESPACE_H
#define CRYPTOPP_CONFIG_NAMESPACE_H

// namespace support is now required
#ifdef NO_NAMESPACE
# error namespace support is now required
#endif

#ifdef CRYPTOPP_DOXYGEN_PROCESSING

/// \namespace CryptoPP
/// \brief Crypto++ library namespace
/// \details Nearly all classes are located in the CryptoPP namespace. Within
/// the namespace, there are two additional namespaces.
///   <ul>
///     <li>Name - namespace for names used with NameValuePairs and documented
///         in argnames.h
///     <li>NaCl - namespace for NaCl test functions like crypto_box,
///         crypto_box_open, crypto_sign, and crypto_sign_open
///     <li>Donna - namespace for curve25519 library operations. The name was
///         selected due to use of Langley and Moon's curve25519-donna.
///     <li>Test - namespace for testing and benchmarks classes
///     <li>Weak - namespace for weak and wounded algorithms, like ARC4, MD5
///         and Pananma
///   </ul>
namespace CryptoPP { }

// Bring in the symbols found in the weak namespace; and fold Weak1 into Weak
#define CRYPTOPP_ENABLE_NAMESPACE_WEAK 1
#define Weak1 Weak
// Avoid putting "CryptoPP::" in front of everything in Doxygen output
#define CryptoPP
#define NAMESPACE_BEGIN(x)
#define NAMESPACE_END
// Get Doxygen to generate better documentation for these typedefs
#define DOCUMENTED_TYPEDEF(x, y) class y : public x {}
// Make "protected" "private" so the functions and members are not documented
#define protected private

#else
// Not Doxygen
#define NAMESPACE_BEGIN(x) namespace x {
#define NAMESPACE_END }
#define DOCUMENTED_TYPEDEF(x, y) typedef x y

#endif  // CRYPTOPP_DOXYGEN_PROCESSING

#define ANONYMOUS_NAMESPACE_BEGIN namespace {
#define ANONYMOUS_NAMESPACE_END }
#define USING_NAMESPACE(x) using namespace x;
#define DOCUMENTED_NAMESPACE_BEGIN(x) namespace x {
#define DOCUMENTED_NAMESPACE_END }

#endif  // CRYPTOPP_CONFIG_NAMESPACE_H
