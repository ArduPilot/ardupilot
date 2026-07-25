/*
        smdevp.c

        gSOAP interface for (signed) message digest

gSOAP XML Web services tools
Copyright (C) 2000-2019, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2019, Robert van Engelen, Genivia, Inc., All Rights Reserved.
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

/**

@page smdevp The smdevp signed message digest engine

The gSOAP smdevp engine computes signed/unsigned message digests over any type
of data using the EVP interface of OpenSSL. It currently supports MD5,
SHA1/224/256/384/512, HMAC_SHA1/224/256/384/512, DSA_SHA1/224256/384/512, and
RSA_SHA1/224/256/384/512.

A digest or signature algorithm is selected with one the following:

- @ref SOAP_SMD_HMAC_MD5        to compute HMAC-MD5 message authentication code
- @ref SOAP_SMD_HMAC_SHA1       to compute HMAC-SHA1 message authentication code
- @ref SOAP_SMD_HMAC_SHA224     to compute HMAC-SHA224 message authentication code
- @ref SOAP_SMD_HMAC_SHA256     to compute HMAC-SHA256 message authentication code
- @ref SOAP_SMD_HMAC_SHA384     to compute HMAC-SHA384 message authentication code
- @ref SOAP_SMD_HMAC_SHA512     to compute HMAC-SHA512 message authentication code

- @ref SOAP_SMD_DGST_MD5        to compute MD5 128-bit digests
- @ref SOAP_SMD_DGST_SHA1       to compute SHA1 160-bit digests
- @ref SOAP_SMD_DGST_SHA224     to compute SHA224 224-bit digests
- @ref SOAP_SMD_DGST_SHA256     to compute SHA256 256-bit digests
- @ref SOAP_SMD_DGST_SHA384     to compute SHA384 384-bit digests
- @ref SOAP_SMD_DGST_SHA512     to compute SHA512 512-bit digests

- @ref SOAP_SMD_SIGN_DSA_SHA1   to compute DSA-SHA1 signatures
- @ref SOAP_SMD_SIGN_DSA_SHA224 to compute DSA-SHA224 signatures
- @ref SOAP_SMD_SIGN_DSA_SHA256 to compute DSA-SHA256 signatures
- @ref SOAP_SMD_SIGN_DSA_SHA384 to compute DSA-SHA384 signatures
- @ref SOAP_SMD_SIGN_DSA_SHA512 to compute DSA-SHA512 signatures

- @ref SOAP_SMD_SIGN_RSA_SHA1   to compute RSA-SHA1 signatures
- @ref SOAP_SMD_SIGN_RSA_SHA224 to compute RSA-SHA224 signatures
- @ref SOAP_SMD_SIGN_RSA_SHA256 to compute RSA-SHA256 signatures
- @ref SOAP_SMD_SIGN_RSA_SHA384 to compute RSA-SHA384 signatures
- @ref SOAP_SMD_SIGN_RSA_SHA512 to compute RSA-SHA512 signatures

- @ref SOAP_SMD_VRFY_DSA_SHA1   to verify DSA-SHA1 signatures
- @ref SOAP_SMD_VRFY_DSA_SHA224 to verify DSA-SHA224 signatures
- @ref SOAP_SMD_VRFY_DSA_SHA256 to verify DSA-SHA256 signatures
- @ref SOAP_SMD_VRFY_DSA_SHA384 to verify DSA-SHA384 signatures
- @ref SOAP_SMD_VRFY_DSA_SHA512 to verify DSA-SHA512 signatures

- @ref SOAP_SMD_VRFY_RSA_SHA1   to verify RSA-SHA1 signatures
- @ref SOAP_SMD_VRFY_RSA_SHA224 to verify RSA-SHA224 signatures
- @ref SOAP_SMD_VRFY_RSA_SHA256 to verify RSA-SHA256 signatures
- @ref SOAP_SMD_VRFY_RSA_SHA384 to verify RSA-SHA384 signatures
- @ref SOAP_SMD_VRFY_RSA_SHA512 to verify RSA-SHA512 signatures

Algorithm options:

- @ref SOAP_SMD_PASSTHRU        to pass XML through the smdevp engine

The smdevp engine wraps the EVP API with three new functions:

- @ref soap_smd_init    to initialize the engine
- @ref soap_smd_update  to update the state with a message part
- @ref soap_smd_final   to compute the digest, signature, or verify a signature
                        and to deallocate the engine
- @ref soap_smd_cleanup to deallocate the engine

A higher-level interface for computing (signed) message digests over
messages produced by the gSOAP engine is defined by two new functions:

- @ref soap_smd_begin   to start a digest or signature computation/verification
- @ref soap_smd_end     to finalize the digest or signature and clean up

Compile all source codes with -DWITH_OPENSSL and link with ssl and crypto
libraries.

Here is an example to sign an XML serialized C++ object using an RSA private
key applied to the SHA digest of the serialized object:

@code
    #include "smdevp.h"
    ns__Object object;
    int alg = SOAP_SMD_SIGN_RSA_SHA1;
    FILE *fd = fopen("key.pem", "r");
    EVP_PKEY *key = PEM_read_PrivateKey(fd, NULL, NULL, "password");
    char *sig = (char*)soap_malloc(soap, soap_smd_size(alg, key));
    int siglen;
    fclose(fd);
    if (soap_smd_begin(soap, alg, key, 0)
     || soap_out_ns__Object(soap, "ns:Object", 0, &object, NULL))
    {
      soap_smd_end(soap, NULL, NULL); // clean up
      soap_print_fault(soap, stderr);
    }
    else if (soap_smd_end(soap, sig, &siglen))
      soap_print_fault(soap, stderr);
    else
      ... // sig contains RSA-SHA1 signature of length siglen 
    EVP_PKEY_free(key);
@endcode

Compile the gSOAP sources and your code with -DWITH_OPENSSL and link with
OpenSSL libraries.

There is no XML output generated by this example, as the object is simply
serialized to the smdevp engine. To actually pass the XML object through the
smdevp engine and output it to a stream or file simultaneously, use the
SOAP_SMD_PASSTHRU flag with the algorithm selection as follows:

@code
    ns__Object object;
    int alg = SOAP_SMD_SIGN_RSA_SHA1;
    FILE *fd = fopen("key.pem", "r");
    EVP_PKEY *key = PEM_read_PrivateKey(fd, NULL, NULL, "password");
    char *sig = (char*)soap_malloc(soap, soap_smd_size(alg, key));
    int siglen;
    fclose(fd);
    soap->sendfd = open("object.xml", O_CREAT | O_WRONLY, 0600); // a file to save object to
    if (soap_smd_begin(soap, alg | SOAP_SMD_PASSTHRU, key, 0)
     || soap_begin_send(soap)
     || soap_out_ns__Object(soap, "ns:Object", 0, &object, NULL) // save to "object.xml"
     || soap_end_send(soap))
    {
      soap_smd_end(soap, NULL, NULL); // clean up
      soap_print_fault(soap, stderr);
    }
    else if (soap_smd_end(soap, sig, &siglen))
      soap_print_fault(soap, stderr);
    else
      ... // sig contains RSA-SHA1 signature of length siglen 
    close(soap->sendfd);
    EVP_PKEY_free(key);
@endcode

Note that we used soap_begin_send and soap_end_send to emit the XML to a
stream. Each type also has a reader (e.g. soap_read_ns__Object) and writer
(e.g. soap_write_ns__Object) that can be used instead as these include
soap_begin_recv/soap_end_recv and soap_begin_send/soap_end_send call sequences.

To verify the signature of an object read from a stream or file, we pass it
through the smdevp engine as follows:

@code
    char *sig = ...;
    int siglen = ...;
    ns__Object object;
    int alg = SOAP_SMD_VRFY_RSA_SHA1;
    FILE *fd = fopen("key.pem", "r");
    EVP_PKEY *key;
    if (...) // key file contains public key?
      key = PEM_read_PUBKEY(fd, NULL, NULL, NULL);
    else // key file contains certificate
    {
      X509 *cert = PEM_read_X509(fd, NULL, NULL, NULL);
      key = X509_get_pubkey(cert);
      X509_free(cert);
    }
    fclose(fd);
    soap->recvfd = open("object.xml", O_RDONLY);
    if (soap_smd_begin(soap, alg, key, 0)
     || soap_begin_recv(soap)
     || soap_in_ns__Object(soap, "ns:Object", &object, NULL) == NULL
     || soap_end_recv(soap))
    {
      soap_smd_end(soap, NULL, NULL); // clean up
      soap_print_fault(soap, stderr);
    }
    else if (soap_smd_end(soap, sig, &siglen))
      soap_print_fault(soap, stderr);
    else
      ... // sig verified, i.e. signed object was not changed
    close(soap->recvfd);
    EVP_PKEY_free(key);
@endcode

To verify the signature of an object stored in memory, we use the RSA public
key and re-run the octet stream (by re-serialization in this example) through
the smdevp engine using the SOAP_SMD_VRFY_RSA_SHA1 algorithm. Note that a PEM
file may contain both the (encrypted) private and public keys.

@code
    char *sig = ...;
    int siglen = ...;
    ns__Object object;
    int alg = SOAP_SMD_VRFY_RSA_SHA1;
    FILE *fd = fopen("key.pem", "r");
    EVP_PKEY *key;
    if (...) // key file contains public key?
      key = PEM_read_PUBKEY(fd, NULL, NULL, NULL);
    else // key file contains certificate
    {
      X509 *cert = PEM_read_X509(fd, NULL, NULL, NULL);
      key = X509_get_pubkey(cert);
      X509_free(cert);
    }
    fclose(fd);
    if (soap_smd_begin(soap, alg, key, 0)
     || soap_out_ns__Object(soap, "ns:Object", 0, &object, NULL))
    {
      soap_smd_end(soap, NULL, NULL); // clean up
      soap_print_fault(soap, stderr);
    }
    else if (soap_smd_end(soap, sig, &siglen))
      soap_print_fault(soap, stderr);
    else
      ... // sig verified, i.e. signed object was not changed
    EVP_PKEY_free(key);
@endcode

The HMAC algorithm uses a shared secret key (hence both the sender and receiver
must keep it secret) to sign and verify a message:

@code
    ns__Object object;
    int alg = SOAP_SMD_HMAC_SHA1;
    static char key[16] =
    { 0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88,
      0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00 };
    char *sig = (char*)soap_malloc(soap, soap_smd_size(alg, NULL));
    int siglen;
    if (soap_smd_begin(soap, alg, key, sizeof(key))
     || soap_out_ns__Object(soap, "ns:Object", 0, &object, NULL))
    {
      soap_smd_end(soap, NULL, NULL); // clean up
      soap_print_fault(soap, stderr);
    }
    else if (soap_smd_end(soap, sig, &siglen))
      soap_print_fault(soap, stderr);
    else
      ... // sig holds the signature
@endcode

HMAC signature verification proceeds by recomputing the signature value for
comparison.

A digest is a hash value of an octet stream computed using the MD5 or SHA
algorithms:

@code
    ns__Object object;
    int alg = SOAP_SMD_DGST_SHA1;
    char *digest = (char*)soap_malloc(soap, soap_smd_size(alg, NULL));
    int digestlen;
    if (soap_smd_begin(soap, alg, NULL, 0)
     || soap_out_ns__Object(soap, "ns:Object", 0, &object, NULL))
    {
      soap_smd_end(soap, NULL, NULL); // clean up
      soap_print_fault(soap, stderr);
    }
    else if (soap_smd_end(soap, digest, &digestlen))
      soap_print_fault(soap, stderr);
    else
      ... // digest holds hash value of serialized object
@endcode

Note that indentation (SOAP_XML_INDENT) and exc-c14n canonicalization
(SOAP_XML_CANONICAL) affects the XML serialization format and, therefore,
the digest or signature produced.

*/

#include "smdevp.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************\
 *
 *      Static protos
 *
\******************************************************************************/

static int soap_smd_send(struct soap *soap, const char *buf, size_t len);
static size_t soap_smd_recv(struct soap *soap, char *buf, size_t len);
static int soap_smd_check(struct soap *soap, struct soap_smd_data *data, int err, const char *msg);

/******************************************************************************\
 *
 *      soap_smd API functions
 *
\******************************************************************************/

/**
@fn size_t soap_smd_size(int alg, const void *key)
@brief Returns the number of octets needed to store the digest or signature returned by soap_smd_end.
@param[in] alg is the digest or signature algorithm to be used
@param[in] key is a pointer to an EVP_PKEY object for RSA/DSA signatures or NULL for digests and HMAC
@return size_t number of octets that is needed to hold digest or signature
@see soap_smd_end

The values returned for digests are SOAP_SMD_MD5_SIZE, SOAP_SMD_SHA1_SIZE, SOAP_SMD_SHA256_SIZE, SOAP_SMD_SHA512_SIZE.
*/
SOAP_FMAC1
size_t
SOAP_FMAC2
soap_smd_size(int alg, const void *key)
{
  switch ((alg & SOAP_SMD_ALGO))
  {
    case SOAP_SMD_SIGN:
    case SOAP_SMD_VRFY:
      /* OpenSSL EVP_PKEY_size returns size of signatures given a key */
      return (size_t)EVP_PKEY_size((EVP_PKEY*)key);
    case SOAP_SMD_HMAC:
    case SOAP_SMD_DGST:
      switch ((alg & SOAP_SMD_HASH))
      {
        case SOAP_SMD_MD5:
          return SOAP_SMD_MD5_SIZE;
        case SOAP_SMD_SHA1:
          return SOAP_SMD_SHA1_SIZE;
        case SOAP_SMD_SHA224:
          return SOAP_SMD_SHA224_SIZE;
        case SOAP_SMD_SHA256:
          return SOAP_SMD_SHA256_SIZE;
        case SOAP_SMD_SHA384:
          return SOAP_SMD_SHA384_SIZE;
        case SOAP_SMD_SHA512:
          return SOAP_SMD_SHA512_SIZE;
        default:
          break;
      }
    default:
      break;
  }
  return 0;
}

/******************************************************************************/

/**
@fn int soap_smd_begin(struct soap *soap, int alg, const void *key, int keylen)
@brief Initiates a digest or signature computation.
@param soap context
@param[in] alg is the digest or signature (sign/verification) algorithm used
@param[in] key is a HMAC key or pointer to EVP_PKEY object or NULL for digests
@param[in] keylen is the length of the HMAC key or 0
@return SOAP_OK, SOAP_EOM, or SOAP_SSL_ERROR
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_smd_begin(struct soap *soap, int alg, const void *key, int keylen)
{
  struct soap_smd_data *data;
  data = (struct soap_smd_data*)SOAP_MALLOC(soap, sizeof(struct soap_smd_data));
  if (!data)
    return soap->error = SOAP_EOM;
  /* save and set the engine's 'data' field to pass data to the callbacks */
  soap->data[0] = (void*)data;
  /* save and override the send and recv callbacks */
  data->fsend = soap->fsend;
  data->frecv = soap->frecv;
  soap->fsend = soap_smd_send;
  soap->frecv = soap_smd_recv;
  /* save the mode flag */
  data->mode = soap->mode;
  /* clear the IO flags and DOM flag */
  soap->mode &= ~(SOAP_IO | SOAP_IO_LENGTH | SOAP_ENC_ZLIB | SOAP_XML_DOM);
  /* clear the XML attribute store */
  soap_clr_attr(soap);
  /* load the local XML namespaces store */
  soap_set_local_namespaces(soap);
  if ((soap->mode & SOAP_XML_CANONICAL))
    soap->ns = 0; /* for in c14n, we must have all xmlns bindings available */
  else if (!(alg & SOAP_SMD_PASSTHRU))
    soap->ns = 2; /* we don't want leading whitespace in serialized XML */
  /* init the soap_smd engine */
  return soap_smd_init(soap, data, alg, key, keylen);
}

/******************************************************************************/

/**
@fn int soap_smd_end(struct soap *soap, char *buf, int *len)
@brief Completes a digest or signature computation. Also deallocates temporary storage allocated by soap_smd_begin(), so MUST be called after soap_smd_begin().
@param soap context
@param[in] buf contains signature for verification (when using a SOAP_SMD_VRFY algorithm) or NULL for cleanup
@param[out] buf is populated with the digest or signature with maximum length soap_smd_size(alg, key)
@param[in] len points to length of signature to verify (when using a SOAP_SMD_VRFY algorithm) or NULL for cleanup
@param[out] len points to length of stored digest or signature (when not NULL)
@return SOAP_OK, SOAP_USER_ERROR, or SOAP_SSL_ERROR
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_smd_end(struct soap *soap, char *buf, int *len)
{
  struct soap_smd_data *data;
  int err;
  data = (struct soap_smd_data*)soap->data[0];
  if (!data)
  {
    if (soap->error)
      return soap->error;
    return soap->error = SOAP_USER_ERROR;
  }
  /* finalize the digest/signature computation and store data in buf[len] */
  /* for signature verification, buf[len] contain the signature */
  err = soap_smd_final(soap, data, buf, len);
  /* restore the callbacks */
  soap->fsend = data->fsend;
  soap->frecv = data->frecv;
  /* restore the mode flag */
  soap->mode = data->mode;
  /* free data */
  SOAP_FREE(soap, data);
  soap->data[0] = NULL;
  /* return SOAP_OK or error */
  return err;
}

/******************************************************************************/

/**
@fn int soap_smd_init(struct soap *soap, struct soap_smd_data *data, int alg, const void *key, int keylen)
@brief Initiates a (signed) digest computation.
@param soap context
@param[in,out] data smdevp engine context
@param[in] alg is algorithm to use
@param[in] key is key to use or NULL for digests
@param[in] keylen is length of HMAC key (when provided)
@return SOAP_OK or SOAP_SSL_ERROR
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_smd_init(struct soap *soap, struct soap_smd_data *data, int alg, const void *key, int keylen)
{
  int ok = 1;
  const EVP_MD *type;
  soap_ssl_init();
  /* the algorithm to use */
  data->alg = alg;
  /* the key to use */
  data->key = key;
  /* allocate and init the OpenSSL HMAC or EVP_MD context */
  if ((alg & SOAP_SMD_ALGO) == SOAP_SMD_HMAC)
  {
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
    data->ctx = (void*)SOAP_MALLOC(soap, sizeof(HMAC_CTX));
    if (data->ctx)
      HMAC_CTX_init((HMAC_CTX*)data->ctx);
#else
    data->ctx = (void*)HMAC_CTX_new();
#endif
  }
  else
  {
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
    data->ctx = (void*)SOAP_MALLOC(soap, sizeof(EVP_MD_CTX));
    if (data->ctx)
      EVP_MD_CTX_init((EVP_MD_CTX*)data->ctx);
#else
    data->ctx = (void*)EVP_MD_CTX_new();
#endif
  }
  if (!data->ctx)
    return soap_set_receiver_error(soap, "soap_smd_init() failed", "No context", SOAP_SSL_ERROR);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "-- SMD Init alg=%x (%p) --\n", alg, data->ctx));
  /* init the digest or signature computations */
  switch ((alg & SOAP_SMD_HASH))
  {
    case SOAP_SMD_MD5:
      type = EVP_md5();
      break;
    case SOAP_SMD_SHA1:
      type = EVP_sha1();
      break;
#if (OPENSSL_VERSION_NUMBER >= 0x0090800fL)
    case SOAP_SMD_SHA224:
      type = EVP_sha224();
      break;
    case SOAP_SMD_SHA256:
      type = EVP_sha256();
      break;
    case SOAP_SMD_SHA384:
      type = EVP_sha384();
      break;
    case SOAP_SMD_SHA512:
      type = EVP_sha512();
      break;
#endif
    default:
      return soap_smd_check(soap, data, 0, "soap_smd_init() failed: cannot load digest");
  }
  switch ((alg & SOAP_SMD_ALGO))
  {
    case SOAP_SMD_HMAC:
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
      HMAC_Init((HMAC_CTX*)data->ctx, key, keylen, type);
#else
      HMAC_Init_ex((HMAC_CTX*)data->ctx, key, keylen, type, NULL);
#endif
      break;
    case SOAP_SMD_DGST:
      EVP_DigestInit((EVP_MD_CTX*)data->ctx, type);
      break;
    case SOAP_SMD_SIGN:
      ok = EVP_SignInit((EVP_MD_CTX*)data->ctx, type);
      break;
    case SOAP_SMD_VRFY:
      ok = EVP_VerifyInit((EVP_MD_CTX*)data->ctx, type);
      break;
    default:
      return soap_set_receiver_error(soap, "Unsupported digest algorithm", NULL, SOAP_SSL_ERROR);
  }
  /* check and return */
  return soap_smd_check(soap, data, ok, "soap_smd_init() failed");
}

/******************************************************************************/

/**
@fn int soap_smd_update(struct soap *soap, struct soap_smd_data *data, const char *buf, size_t len)
@brief Updates (signed) digest computation with message part.
@param soap context
@param[in,out] data smdevp engine context
@param[in] buf contains message part
@param[in] len of message part
@return SOAP_OK or SOAP_SSL_ERROR
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_smd_update(struct soap *soap, struct soap_smd_data *data, const char *buf, size_t len)
{
  int ok = 1;
  if (!data->ctx)
    return soap_set_receiver_error(soap, "soap_smd_update() failed", "No context", SOAP_SSL_ERROR);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "-- SMD Update alg=%x n=%lu (%p) --\n", data->alg, (unsigned long)len, data->ctx));
  switch ((data->alg & SOAP_SMD_ALGO))
  {
    case SOAP_SMD_HMAC:
      HMAC_Update((HMAC_CTX*)data->ctx, (const unsigned char*)buf, len);
      break;
    case SOAP_SMD_DGST:
      EVP_DigestUpdate((EVP_MD_CTX*)data->ctx, (const void*)buf, (unsigned int)len);
      break;
    case SOAP_SMD_SIGN:
      ok = EVP_SignUpdate((EVP_MD_CTX*)data->ctx, (const void*)buf, (unsigned int)len);
      break;
    case SOAP_SMD_VRFY:
      ok = EVP_VerifyUpdate((EVP_MD_CTX*)data->ctx, (const void*)buf, (unsigned int)len);
      break;
  }
  DBGMSG(TEST, buf, len);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--"));
  /* check and return */
  return soap_smd_check(soap, data, ok, "soap_smd_update() failed");
}

/******************************************************************************/

/**
@fn int soap_smd_final(struct soap *soap, struct soap_smd_data *data, char *buf, int *len)
@brief Finalizes (signed) digest computation, delete context and returns digest or signature.
@param soap context
@param[in,out] data smdevp engine context
@param[in] buf contains signature for verification (SOAP_SMD_VRFY algorithms)
@param[out] buf is populated with the digest or signature
@param[in] len points to length of signature to verify (SOAP_SMD_VRFY algorithms)
@param[out] len points to length of stored digest or signature (pass NULL if you are not interested in this value)
@return SOAP_OK or SOAP_SSL_ERROR
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_smd_final(struct soap *soap, struct soap_smd_data *data, char *buf, int *len)
{
  unsigned int n = 0;
  int ok = 1;
  if (!data->ctx)
    return soap_set_receiver_error(soap, "soap_smd_final() failed", "No context", SOAP_SSL_ERROR);
  if (buf)
  {
    /* finalize the digest or signature computation */
    switch ((data->alg & SOAP_SMD_ALGO))
    {
      case SOAP_SMD_HMAC:
        HMAC_Final((HMAC_CTX*)data->ctx, (unsigned char*)buf, &n);
        break;
      case SOAP_SMD_DGST:
        EVP_DigestFinal_ex((EVP_MD_CTX*)data->ctx, (unsigned char*)buf, &n);
        break;
      case SOAP_SMD_SIGN:
        ok = EVP_SignFinal((EVP_MD_CTX*)data->ctx, (unsigned char*)buf, &n, (EVP_PKEY*)data->key);
        break;
      case SOAP_SMD_VRFY:
        if (len)
        {
          n = (unsigned int)*len;
          ok = EVP_VerifyFinal((EVP_MD_CTX*)data->ctx, (unsigned char*)buf, n, (EVP_PKEY*)data->key);
        }
        else
          ok = 0;
        break;
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "-- SMD Final alg=%x (%p) %d bytes--\n", data->alg, data->ctx, n));
    DBGHEX(TEST, buf, n);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--"));
    /* pass back length of digest or signature produced */
    if (len)
      *len = (int)n;
  }
  /* cleanup */
  soap_smd_cleanup(soap, data);
  /* check and return */
  return soap_smd_check(soap, data, ok, "soap_smd_final() failed");
}

/**
@fn void soap_smd_cleanup(struct soap *soap, struct soap_smd_data *data)
@brief Clear (signed) digest computation and delete context
@param soap context
@param[in,out] data smdevp engine context
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_smd_cleanup(struct soap *soap, struct soap_smd_data *data)
{
  (void)soap;
  if (data->ctx)
  {
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
    if ((data->alg & SOAP_SMD_ALGO) == SOAP_SMD_HMAC)
      HMAC_CTX_cleanup((HMAC_CTX*)data->ctx);
    else
      EVP_MD_CTX_cleanup((EVP_MD_CTX*)data->ctx);
    SOAP_FREE(soap, data->ctx);
#else
    if ((data->alg & SOAP_SMD_ALGO) == SOAP_SMD_HMAC)
      HMAC_CTX_free((HMAC_CTX*)data->ctx);
    else
      EVP_MD_CTX_free((EVP_MD_CTX*)data->ctx);
#endif
    data->ctx = NULL;
  }
}

/******************************************************************************\
 *
 *      Static local functions
 *
\******************************************************************************/

/**
@fn int soap_smd_check(struct soap *soap, struct soap_smd_data *data, int ok, const char *msg)
@brief Check result of init/update/final smdevp engine operations.
@param soap context
@param[in,out] data smdevp engine context
@param[in] ok EVP error value
@param[in] msg error message
@return SOAP_OK or SOAP_SSL_ERROR
*/
static int
soap_smd_check(struct soap *soap, struct soap_smd_data *data, int ok, const char *msg)
{
  if (ok <= 0)
  {
    unsigned long r;
    while ((r = ERR_get_error()))
    {
      ERR_error_string_n(r, soap->msgbuf, sizeof(soap->msgbuf));
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "-- SMD Error (%d) %s: %s\n", ok, msg, soap->msgbuf));
    }
    if (data->ctx)
    {
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
      if ((data->alg & SOAP_SMD_ALGO) == SOAP_SMD_HMAC)
        HMAC_CTX_cleanup((HMAC_CTX*)data->ctx);
      else
        EVP_MD_CTX_cleanup((EVP_MD_CTX*)data->ctx);
      SOAP_FREE(soap, data->ctx);
#else
      if ((data->alg & SOAP_SMD_ALGO) == SOAP_SMD_HMAC)
        HMAC_CTX_free((HMAC_CTX*)data->ctx);
      else
        EVP_MD_CTX_free((EVP_MD_CTX*)data->ctx);
#endif
      data->ctx = NULL;
    }
    return soap_set_receiver_error(soap, msg, soap->msgbuf, SOAP_SSL_ERROR);
  }
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      Callbacks registered by plugin
 *
\******************************************************************************/

/**
@fn int soap_smd_send(struct soap *soap, const char *buf, size_t len)
@brief Callback to intercept messages for digest or signature computation/verification.
@param soap context
@param[in] buf message
@param[in] len message length
@return SOAP_OK or SOAP_SSL_ERROR
*/
static int
soap_smd_send(struct soap *soap, const char *buf, size_t len)
{
  int err;
  if (((struct soap_smd_data*)soap->data[0])->alg & SOAP_SMD_PASSTHRU)
  {
    err = ((struct soap_smd_data*)soap->data[0])->fsend(soap, buf, len);
    if (err)
      return err;
  }
  return soap_smd_update(soap, (struct soap_smd_data*)soap->data[0], buf, len);
}

/******************************************************************************/

/**
@fn size_t soap_smd_recv(struct soap *soap, char *buf, size_t len)
@brief Callback to intercept messages for digest or signature computation/verification.
@param soap context
@param[in] buf buffer
@param[in] len max buffer length
@return message size in buffer or 0 on error.
*/
static size_t
soap_smd_recv(struct soap *soap, char *buf, size_t len)
{
  size_t ret = ((struct soap_smd_data*)soap->data[0])->frecv(soap, buf, len);
  if (ret && soap_smd_update(soap, (struct soap_smd_data*)soap->data[0], buf, ret))
    return 0;
  return ret;
}

/******************************************************************************/

#ifdef __cplusplus
}
#endif
