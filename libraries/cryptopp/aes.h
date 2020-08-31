// aes.h - originally written and placed in the public domain by Wei Dai

/// \file
/// \brief Class file for the AES cipher (Rijndael)
/// \details AES is a typdef for Rijndael classes. All key sizes are supported.
///   The library only provides Rijndael with 128-bit blocks, and not 192-bit or 256-bit blocks
/// \since Rijndael since Crypto++ 3.1, Intel AES-NI since Crypto++ 5.6.1, ARMv8 AES since Crypto++ 6.0,
///   Power8 AES since Crypto++ 6.0

#ifndef CRYPTOPP_AES_H
#define CRYPTOPP_AES_H

#include "rijndael.h"

NAMESPACE_BEGIN(CryptoPP)

/// \brief AES block cipher (Rijndael)
/// \details AES is a typdef for Rijndael classes. All key sizes are supported.
///   The library only provides Rijndael with 128-bit blocks, and not 192-bit or 256-bit blocks
/// \sa <a href="http://www.cryptolounge.org/wiki/AES">AES</a> winner, announced on 10/2/2000
/// \since Rijndael since Crypto++ 3.1, Intel AES-NI since Crypto++ 5.6.1, ARMv8 AES since Crypto++ 6.0,
///   Power8 AES since Crypto++ 6.0
DOCUMENTED_TYPEDEF(Rijndael, AES);

typedef RijndaelEncryption AESEncryption;
typedef RijndaelDecryption AESDecryption;

NAMESPACE_END

#endif
