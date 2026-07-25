/*
        smdevp.h

        gSOAP interface for (signed) message digest

gSOAP XML Web services tools
Copyright (C) 2000-2015, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under one of the following licenses:
GPL or the gSOAP public license.
--------------------------------------------------------------------------------
gSOAP public license.

The contents of this file are subject to the gSOAP Public License Version 1.3
(the "License"); you may not use this file except in compliance with the
License. You may obtain a copy of the License at
http://www.cs.fsu.edu/~engelen/soaplicense.html
Software distributed under the License is distributed on an "AS IS" basis,
WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
for the specific language governing rights and limitations under the License.

The Initial Developer of the Original Code is Robert A. van Engelen.
Copyright (C) 2000-2015, Robert van Engelen, Genivia, Inc., All Rights Reserved.
--------------------------------------------------------------------------------
GPL license.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

Author contact information:
engelen@genivia.com / engelen@acm.org

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#ifndef SMDEVP_H
#define SMDEVP_H

#include "stdsoap2.h"

#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/ssl.h>
#include <openssl/err.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Expose EVP_PKEY in a portable representation */
#define SOAP_SMD_KEY_TYPE       EVP_PKEY

/** Expose EVP_MAX_MD_SIZE in a portable representation */
#define SOAP_SMD_MAX_SIZE       EVP_MAX_MD_SIZE

/** MD5 digest size in octets */
#define SOAP_SMD_MD5_SIZE       (16)
/** SHA1 digest size in octets */
#define SOAP_SMD_SHA1_SIZE      (20)
/** SHA224 digest size in octets */
#define SOAP_SMD_SHA224_SIZE    (28)
/** SHA256 digest size in octets */
#define SOAP_SMD_SHA256_SIZE    (32)
/** SHA384 digest size in octets */
#define SOAP_SMD_SHA384_SIZE    (48)
/** SHA512 digest size in octets */
#define SOAP_SMD_SHA512_SIZE    (64)

/******************************************************************************\
 *
 *      Supported algorithms
 *
\******************************************************************************/

#define SOAP_SMD_NONE   (0x0000)
#define SOAP_SMD_HASH   (0x000F)
#define SOAP_SMD_ALGO   (0x0030)
#define SOAP_SMD_MASK   (0x01FF)

/** MD5 HASH */
#define SOAP_SMD_MD5    (0x00)
/** SHA1 HASH */
#define SOAP_SMD_SHA1   (0x01)
/** SHA224 HASH */
#define SOAP_SMD_SHA224 (0x02)
/** SHA256 HASH */
#define SOAP_SMD_SHA256 (0x03)
/** SHA384 HASH */
#define SOAP_SMD_SHA384 (0x04)
/** SHA512 HASH */
#define SOAP_SMD_SHA512 (0x05)

/** HMAC ALGO */
#define SOAP_SMD_HMAC   (0x00)
/** Digest ALGO */
#define SOAP_SMD_DGST   (0x10)
/** Sign ALGO */
#define SOAP_SMD_SIGN   (0x20)
/** Verify ALGO */
#define SOAP_SMD_VRFY   (0x30)

/** DSA Sign/Verify ALGO */
#define SOAP_SMD_DSA    (0x40)

/** ECDSA Sign/Verify ALGO */
#define SOAP_SMD_ECDSA  (0x80)

/** Additional flag: msg sends will pass through digest/signature algorithm */
#define SOAP_SMD_PASSTHRU        (0x100)

/** HMAC-MD5 shared key signature algorithm */
#define SOAP_SMD_HMAC_MD5        (SOAP_SMD_HMAC | SOAP_SMD_MD5)
/** HMAC-SHA1 shared key signature algorithm */
#define SOAP_SMD_HMAC_SHA1       (SOAP_SMD_HMAC | SOAP_SMD_SHA1)
/** HMAC-SHA224 shared key signature algorithm */
#define SOAP_SMD_HMAC_SHA224     (SOAP_SMD_HMAC | SOAP_SMD_SHA224)
/** HMAC-SHA256 shared key signature algorithm */
#define SOAP_SMD_HMAC_SHA256     (SOAP_SMD_HMAC | SOAP_SMD_SHA256)
/** HMAC-SHA384 shared key signature algorithm */
#define SOAP_SMD_HMAC_SHA384     (SOAP_SMD_HMAC | SOAP_SMD_SHA384)
/** HMAC-SHA512 shared key signature algorithm */
#define SOAP_SMD_HMAC_SHA512     (SOAP_SMD_HMAC | SOAP_SMD_SHA512)

/** DGST-MD5 digest algorithm */
#define SOAP_SMD_DGST_MD5        (SOAP_SMD_DGST | SOAP_SMD_MD5)
/** DGST-SHA1 digest algorithm */
#define SOAP_SMD_DGST_SHA1       (SOAP_SMD_DGST | SOAP_SMD_SHA1)
/** DGST-SHA224 digest algorithm */
#define SOAP_SMD_DGST_SHA224     (SOAP_SMD_DGST | SOAP_SMD_SHA224)
/** DGST-SHA256 digest algorithm */
#define SOAP_SMD_DGST_SHA256     (SOAP_SMD_DGST | SOAP_SMD_SHA256)
/** DGST-SHA384 digest algorithm */
#define SOAP_SMD_DGST_SHA384     (SOAP_SMD_DGST | SOAP_SMD_SHA384)
/** DGST-SHA512 digest algorithm */
#define SOAP_SMD_DGST_SHA512     (SOAP_SMD_DGST | SOAP_SMD_SHA512)

/** RSA-MD5 secret key signature algorithm */
#define SOAP_SMD_SIGN_RSA_MD5    (SOAP_SMD_SIGN | SOAP_SMD_MD5)
/** RSA-SHA1 secret key signature algorithm */
#define SOAP_SMD_SIGN_RSA_SHA1   (SOAP_SMD_SIGN | SOAP_SMD_SHA1)
/** RSA-SHA224 secret key signature algorithm */
#define SOAP_SMD_SIGN_RSA_SHA224 (SOAP_SMD_SIGN | SOAP_SMD_SHA224)
/** RSA-SHA256 secret key signature algorithm */
#define SOAP_SMD_SIGN_RSA_SHA256 (SOAP_SMD_SIGN | SOAP_SMD_SHA256)
/** RSA-SHA384 secret key signature algorithm */
#define SOAP_SMD_SIGN_RSA_SHA384 (SOAP_SMD_SIGN | SOAP_SMD_SHA384)
/** RSA-SHA512 secret key signature algorithm */
#define SOAP_SMD_SIGN_RSA_SHA512 (SOAP_SMD_SIGN | SOAP_SMD_SHA512)

/** DSA-MD5 secret key signature algorithm */
#define SOAP_SMD_SIGN_DSA_MD5    (SOAP_SMD_SIGN | SOAP_SMD_DSA | SOAP_SMD_MD5)
/** DSA-SHA1 secret key signature algorithm */
#define SOAP_SMD_SIGN_DSA_SHA1   (SOAP_SMD_SIGN | SOAP_SMD_DSA | SOAP_SMD_SHA1)
/** DSA-SHA224 secret key signature algorithm */
#define SOAP_SMD_SIGN_DSA_SHA224 (SOAP_SMD_SIGN | SOAP_SMD_DSA | SOAP_SMD_SHA224)
/** DSA-SHA256 secret key signature algorithm */
#define SOAP_SMD_SIGN_DSA_SHA256 (SOAP_SMD_SIGN | SOAP_SMD_DSA | SOAP_SMD_SHA256)
/** DSA-SHA384 secret key signature algorithm */
#define SOAP_SMD_SIGN_DSA_SHA384 (SOAP_SMD_SIGN | SOAP_SMD_DSA | SOAP_SMD_SHA384)
/** DSA-SHA512 secret key signature algorithm */
#define SOAP_SMD_SIGN_DSA_SHA512 (SOAP_SMD_SIGN | SOAP_SMD_DSA | SOAP_SMD_SHA512)

/** ECDSA-MD5 secret key signature algorithm */
#define SOAP_SMD_SIGN_ECDSA_MD5    (SOAP_SMD_SIGN | SOAP_SMD_ECDSA | SOAP_SMD_MD5)
/** ECDSA-SHA1 secret key signature algorithm */
#define SOAP_SMD_SIGN_ECDSA_SHA1   (SOAP_SMD_SIGN | SOAP_SMD_ECDSA | SOAP_SMD_SHA1)
/** ECDSA-SHA224 secret key signature algorithm */
#define SOAP_SMD_SIGN_ECDSA_SHA224 (SOAP_SMD_SIGN | SOAP_SMD_ECDSA | SOAP_SMD_SHA224)
/** ECDSA-SHA256 secret key signature algorithm */
#define SOAP_SMD_SIGN_ECDSA_SHA256 (SOAP_SMD_SIGN | SOAP_SMD_ECDSA | SOAP_SMD_SHA256)
/** ECDSA-SHA384 secret key signature algorithm */
#define SOAP_SMD_SIGN_ECDSA_SHA384 (SOAP_SMD_SIGN | SOAP_SMD_ECDSA | SOAP_SMD_SHA384)
/** ECDSA-SHA512 secret key signature algorithm */
#define SOAP_SMD_SIGN_ECDSA_SHA512 (SOAP_SMD_SIGN | SOAP_SMD_ECDSA | SOAP_SMD_SHA512)

/** RSA-MD5 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_RSA_MD5    (SOAP_SMD_VRFY | SOAP_SMD_MD5)
/** RSA-SHA1 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_RSA_SHA1   (SOAP_SMD_VRFY | SOAP_SMD_SHA1)
/** RSA-SHA224 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_RSA_SHA224 (SOAP_SMD_VRFY | SOAP_SMD_SHA224)
/** RSA-SHA256 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_RSA_SHA256 (SOAP_SMD_VRFY | SOAP_SMD_SHA256)
/** RSA-SHA384 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_RSA_SHA384 (SOAP_SMD_VRFY | SOAP_SMD_SHA384)
/** RSA-SHA512 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_RSA_SHA512 (SOAP_SMD_VRFY | SOAP_SMD_SHA512)

/** DSA-MD5 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_DSA_MD5    (SOAP_SMD_VRFY | SOAP_SMD_DSA | SOAP_SMD_MD5)
/** DSA-SHA1 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_DSA_SHA1   (SOAP_SMD_VRFY | SOAP_SMD_DSA | SOAP_SMD_SHA1)
/** DSA-SHA224 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_DSA_SHA224 (SOAP_SMD_VRFY | SOAP_SMD_DSA | SOAP_SMD_SHA224)
/** DSA-SHA256 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_DSA_SHA256 (SOAP_SMD_VRFY | SOAP_SMD_DSA | SOAP_SMD_SHA256)
/** DSA-SHA384 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_DSA_SHA384 (SOAP_SMD_VRFY | SOAP_SMD_DSA | SOAP_SMD_SHA384)
/** DSA-SHA512 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_DSA_SHA512 (SOAP_SMD_VRFY | SOAP_SMD_DSA | SOAP_SMD_SHA512)

/** ECDSA-MD5 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_ECDSA_MD5    (SOAP_SMD_VRFY | SOAP_SMD_ECDSA | SOAP_SMD_MD5)
/** ECDSA-SHA1 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_ECDSA_SHA1   (SOAP_SMD_VRFY | SOAP_SMD_ECDSA | SOAP_SMD_SHA1)
/** ECDSA-SHA224 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_ECDSA_SHA224 (SOAP_SMD_VRFY | SOAP_SMD_ECDSA | SOAP_SMD_SHA224)
/** ECDSA-SHA256 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_ECDSA_SHA256 (SOAP_SMD_VRFY | SOAP_SMD_ECDSA | SOAP_SMD_SHA256)
/** ECDSA-SHA384 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_ECDSA_SHA384 (SOAP_SMD_VRFY | SOAP_SMD_ECDSA | SOAP_SMD_SHA384)
/** ECDSA-SHA512 secret key signature verification algorithm */
#define SOAP_SMD_VRFY_ECDSA_SHA512 (SOAP_SMD_VRFY | SOAP_SMD_ECDSA | SOAP_SMD_SHA512)

/**
@struct soap_smd_data
@brief The smdevp engine context data, which is hooked up to soap->data[0]
*/
struct soap_smd_data {
  int alg;              /**< The digest or signature algorithm used */
  void *ctx;            /**< EVP_MD_CTX or HMAC_CTX */
  const void *key;      /**< EVP_PKEY */
  int (*fsend)(struct soap*, const char*, size_t);
  size_t (*frecv)(struct soap*, char*, size_t);
  soap_mode mode;       /**< to preserve soap->mode value */
};

/******************************************************************************\
 *
 *      soap_smd API functions
 *
\******************************************************************************/

SOAP_FMAC1 size_t SOAP_FMAC2 soap_smd_size(int alg, const void *key);

SOAP_FMAC1 int SOAP_FMAC2 soap_smd_begin(struct soap *soap, int alg, const void *key, int keylen);
SOAP_FMAC1 int SOAP_FMAC2 soap_smd_end(struct soap *soap, char *buf, int *len);

SOAP_FMAC1 int SOAP_FMAC2 soap_smd_init(struct soap *soap, struct soap_smd_data *data, int alg, const void *key, int keylen);
SOAP_FMAC1 int SOAP_FMAC2 soap_smd_update(struct soap *soap, struct soap_smd_data *data, const char *buf, size_t len);
SOAP_FMAC1 int SOAP_FMAC2 soap_smd_final(struct soap *soap, struct soap_smd_data *data, char *buf, int *len);
SOAP_FMAC1 void SOAP_FMAC2 soap_smd_cleanup(struct soap *soap, struct soap_smd_data *data);

#ifdef __cplusplus
}
#endif

#endif
