/*
        mecevp.c

        gSOAP interface for streaming message encryption and decryption

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

/**

@page mecevp The mecevp streaming message encryption and decryption engine

The gSOAP mecevp engine encrypts and decrypts messages using the EVP interface
of OpenSSL. It supports envelope encryption/decryption with public and private
RSA keys and symmetric encryption with shared secret keys. Streaming and
buffered message encryption modes are supported.

An encryption and decryption algorithm and mode is selected with one of the
following:

- @ref SOAP_MEC_ENV_ENC_DES_CBC         envelope encryption with triple DES EDE CBC
- @ref SOAP_MEC_ENV_ENC_AES256_CBC      envelope encryption with AES256 CBC
- @ref SOAP_MEC_ENV_ENC_AES256_GCM      envelope authenticated encryption with AES256 GCM
- @ref SOAP_MEC_ENC_DES_CBC             symmetric encryption with triple DES EDE CBC
- @ref SOAP_MEC_ENC_AES256_CBC          symmetric encryption with AES256 CBC
- @ref SOAP_MEC_ENC_AES256_GCM          symmetric authenticated encryption with AES256 GCM
- @ref SOAP_MEC_ENV_DEC_DES_CBC         envelope decryption with triple DES EDE CBC
- @ref SOAP_MEC_ENV_DEC_AES256_CBC      envelope decryption with AES256 CBC
- @ref SOAP_MEC_ENV_DEC_AES256_GCM      envelope authenticated decryption with AES256 GCM
- @ref SOAP_MEC_DEC_DES_CBC             symmetric decryption with triple DES EDE CBC
- @ref SOAP_MEC_DEC_AES256_CBC          symmetric decryption with AES256 CBC
- @ref SOAP_MEC_DEC_AES256_GCM          symmetric authenticated decryption with AES256 GCM

where, in the above, AES256 can be replaced with AES128 ot AES192.

Algorithm options:

- @ref SOAP_MEC_STORE           buffer all output in memory
- @ref SOAP_MEC_OAEP            use OAEP padding for CBC

The mecevp engine wraps the EVP API with four new functions:

- @ref soap_mec_init    to initialize the engine
- @ref soap_mec_update  to encrypt/decrypt a message part
- @ref soap_mec_final   to finalize encryption/decryption, does not deallocate
                        tne engine and buffers
- @ref soap_mec_cleanup to deallocate the engine and buffers

All cipher data is written and read in base64 format.

A higher-level interface for message encryption/decryption in parts (such as
individual XML elements) is defined by two new functions:

- @ref soap_mec_begin   to begin a streaming sequence of encryptions/decryptions
- @ref soap_mec_start   to start encryption/decryption of a message part
- @ref soap_mec_stop    to stop encryption/decryption of a message part
- @ref soap_mec_end     to end the sequence and deallocate the engine buffers

Compile all source codes with -DWITH_OPENSSL and link with ssl and crypto
libraries.

Here is an example to encrypt a message while streaming it to the output. The
example uses the public key of the recipient/reader of the message. The
recipient/reader uses its private key to decrypt. Envelope encryption is used
with SOAP_MEC_ENV_ENC_DES_CBC, which means an ephemeral secret key is generated
and encrypted with the public key. This encrypted secret key should be
communicated to the recipient/reader with the message to decrypt:

@code
    #include "mecevp.h"
    soap_mec_data mec;
    ns__Object object;
    int alg = SOAP_MEC_ENV_ENC_DES_CBC;
    FILE *fd = fopen("key.pem", "r");
    EVP_PKEY *pubk;
    unsigned char *key;
    int keylen;
    if (...) // key file contains public key?
      pubk = PEM_read_PUBKEY(fd, NULL, NULL, NULL);
    else // key file contains certificate
    {
      X509 *cert = PEM_read_X509(fd, NULL, NULL, NULL);
      pubk = X509_get_pubkey(cert);
      X509_free(cert);
    }
    fclose(fd);
    key = soap_malloc(soap, soap_mec_size(alg, pubk));
    if (soap_begin_send(soap)
     || soap_mec_begin(soap, &mec, alg, pubk, key, &keylen)
     || soap_mec_start(soap, NULL)
     || soap_out_ns__Object(soap, "ns:Object", 0, &object, NULL)
     || soap_mec_stop(soap)
     || soap_mec_end(soap, &mec)
     || soap_end_send(soap))
    {
      soap_mec_cleanup(soap, &mec); // clean up when error
      soap_print_fault(soap, stderr);
    }
    EVP_PKEY_free(pubk);
@endcode

The example given above sends the output to stdout. To save the output in a
string use the following in C:

@code
    char *str = NULL; // string to be set to the encrypted output
    soap->os = &str;  // for C code only 
    if (soap_begin_send(soap)
      ...
    soap->os = NULL;
    ... // use str, which is set to the encrypted output
    soap_end(soap); // auto-deletes str
@endcode

With C++ you should use a string stream:

@code
    std::stringstream ss;
    soap->os = &ss;
    if (soap_begin_send(soap)
      ...
    soap->os = NULL;
    ... // use ss.str()
    soap_destroy(soap); // cleanup
    soap_end(soap);     // cleanup
@endcode

The decryption by the recipient/reader requires the ephemeral encrypted secret
key generated by soap_mec_begin by the sender (as set above) to decrypt the
message using envelope decryption with SOAP_MEC_ENV_DEC_DES_CBC.

@code
    #include "mecevp.h"
    soap_mec_data mec;
    ns__Object object;
    int alg = SOAP_MEC_ENV_DEC_DES_CBC;
    FILE *fd = fopen("key.pem", "r");
    EVP_PKEY *privk = PEM_read_PrivateKey(fd, NULL, NULL, "password");
    unsigned char *key;
    int keylen;
    fclose(fd);
    key = ... // value set as above by sender
    keylen = ... // value set as above by sender
    if (soap_begin_recv(soap)
     || soap_mec_begin(soap, &mec, alg, privk, key, &keylen)
     || soap_mec_start(soap)
     || soap_in_ns__Object(soap, "ns:Object", &object, NULL) == NULL
     || soap_mec_stop(soap)
     || soap_mec_end(soap, &mec)
     || soap_end_recv(soap))
    {
      soap_mec_cleanup(soap, &mec); // clean up when error
      soap_print_fault(soap, stderr);
    }
    EVP_PKEY_free(privk);
@endcode 

The example given above reads the input from stdin. To read input from a string
use the following in C:

@code
    char *str; // string with encrupted input
    soap->is = str;
    if (soap_begin_recv(soap)
      ...
    soap->is = NULL;
    soap_end(soap); // cleanup
@endcode

With C++ you should use a string stream:

@code
    std::stringstream ss;
    ss.str(...); // string with encrypted input
    soap->is = &ss;
    if (soap_begin_recv(soap)
      ...
    soap->is = NULL;
    soap_destroy(soap); // cleanup
    soap_end(soap);     // cleanup
@endcode

Note that the encrypted secret key can be sent in the clear or stored openly,
since only the recipient/reader will be able to decode it (with its private
key) and use it for message decryption.

Symmetric encryption and decryption can be used if both parties can safely
share a secret symmetric key that no other party has access to. We use
SOAP_MEC_ENC_DES_CBC for encryption and SOAP_MEC_DEC_DES_CBC for decryption
using a 160-bit triple DES key. You can also use AES128, AES192, AES256
ciphers.

Here is an example to encrypt a message using a shared secret key while
streaming it to the output.

@code
    #include "mecevp.h"
    soap_mec_data mec;
    ns__Object object;
    int alg = SOAP_MEC_ENC_DES_CBC;
    unsigned char key[20] = { ... }; // shared secret triple DES key
    int keylen = 20;
    if (soap_begin_send(soap)
     || soap_mec_begin(soap, &mec, alg, NULL, key, &keylen)
     || soap_mec_start(soap, NULL)
     || soap_out_ns__Object(soap, "ns:Object", 0, &object, NULL)
     || soap_mec_stop(soap)
     || soap_mec_end(soap, &mec)
     || soap_end_send(soap))
    {
      soap_mec_cleanup(soap, &mec); // clean up when error
      soap_print_fault(soap, stderr);
    }
@endcode

The decryption by the recipient/reader requires the same shared secret key to
decrypt the message using envelope decryption with SOAP_MEC_DEC_DES_CBC. This
key is secret and unencrypted, so it should never be shared with any other
party besides the sender/writer and recipient/reader.

@code
    #include "mecevp.h"
    soap_mec_data mec;
    ns__Object object;
    int alg = SOAP_MEC_DEC_DES_CBC;
    unsigned char key[20] = { ... }; // shared secret triple DES key
    int keylen = 20;
    if (soap_begin_recv(soap)
     || soap_mec_begin(soap, &mec, alg, NULL, key, &keylen)
     || soap_mec_start(soap)
     || soap_in_ns__Object(soap, "ns:Object", &object, NULL) == NULL
     || soap_mec_stop(soap)
     || soap_mec_end(soap, &mec)
     || soap_end_recv(soap))
    {
      soap_mec_cleanup(soap, &mec); // clean up when error
      soap_print_fault(soap, stderr);
    }
@endcode

@note
The mecevp engine uses callbacks of the gSOAP engine that were introduced in
version 2.8.1. Earlier gSOAP version releases are not compatible with the
mecevp plugin and engine.

*/

#include "mecevp.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************\
 *
 *      Static protos
 *
\******************************************************************************/

static int soap_mec_upd(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n, int final);
static int soap_mec_upd_enc(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n, int final);
static int soap_mec_upd_dec(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n, int final);

static int soap_mec_check(struct soap *soap, struct soap_mec_data *data, int err, const char *msg);

static void soap_mec_put_base64(struct soap *soap, struct soap_mec_data *data, const unsigned char *s, int n);
static void soap_mec_end_base64(struct soap *soap, struct soap_mec_data *data);
static int soap_mec_get_base64(struct soap *soap, struct soap_mec_data *data, char *t, size_t *l, const char *s, size_t n, const char **r, size_t *k);

static int soap_mec_filtersend(struct soap *soap, const char **s, size_t *n);
static int soap_mec_filterrecv(struct soap *soap, char *buf, size_t *len, size_t maxlen);

/******************************************************************************\
 *
 *      soap_mec API functions
 *
\******************************************************************************/

/**
@fn int soap_mec_init(struct soap *soap, struct soap_mec_data *data, int alg, SOAP_MEC_KEY_TYPE *pkey, unsigned char *key, int *keylen)
@brief Initialize mecevp engine state and create context for encryption/decryption algorithm using a private/public key or symmetric secret
key.
@param soap context
@param[in,out] data mecevp engine context
@param[in] alg encryption/decryption algorithm
@param[in] pkey public/private key or NULL
@param[in,out] key secret key or encrypted ephemeral secret key set with envelope encryption, or NULL
@param[in,out] keylen secret key length
@return SOAP_OK or SOAP_SSL_ERROR
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_mec_init(struct soap *soap, struct soap_mec_data *data, int alg, SOAP_MEC_KEY_TYPE *pkey, unsigned char *key, int *keylen)
{
  int ok = 1;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "soap_mec_init()\n"));
  soap_ssl_init();
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
  data->ctx = (EVP_CIPHER_CTX*)SOAP_MALLOC(soap, sizeof(EVP_CIPHER_CTX));
  if (data->ctx)
    EVP_CIPHER_CTX_init(data->ctx);
#else
  data->ctx = EVP_CIPHER_CTX_new();
#endif
  if (!data->ctx)
    return soap->error = SOAP_EOM;
  data->alg = alg;
  data->state = SOAP_MEC_STATE_NONE;
  data->restidx = 0;
  data->taglen = 0;
  switch (alg & SOAP_MEC_ALGO)
  {
    case SOAP_MEC_DES_CBC:
      data->type = EVP_des_ede3_cbc();
      break;
    case SOAP_MEC_AES128_CBC:
      data->type = EVP_aes_128_cbc();
      break;
    case SOAP_MEC_AES192_CBC:
      data->type = EVP_aes_192_cbc();
      break;
    case SOAP_MEC_AES256_CBC:
      data->type = EVP_aes_256_cbc();
      break;
    case SOAP_MEC_AES512_CBC:
      data->type = NULL; /* N/A */
      break;
#if (OPENSSL_VERSION_NUMBER >= 0x10002000L)
    case SOAP_MEC_AES128_GCM:
      data->type = EVP_aes_128_gcm();
      break;
    case SOAP_MEC_AES192_GCM:
      data->type = EVP_aes_192_gcm();
      break;
    case SOAP_MEC_AES256_GCM:
      data->type = EVP_aes_256_gcm();
      break;
    case SOAP_MEC_AES512_GCM:
      data->type = NULL; /* N/A */
      break;
#endif
    default:
      data->type = NULL;
  }
  if (alg & SOAP_MEC_ENC)
  {
    if (!data->type)
      return soap_mec_check(soap, data, 0, "soap_mec_init() failed: cannot load cipher");
    ok = EVP_EncryptInit_ex(data->ctx, data->type, NULL, NULL, NULL);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_EncryptInit ok=%d\n", ok));
  }
  if (alg & SOAP_MEC_GCM)
    EVP_CIPHER_CTX_set_padding(data->ctx, 0);
  else
  {
    if (alg & SOAP_MEC_OAEP)
      EVP_CIPHER_CTX_set_padding(data->ctx, RSA_PKCS1_OAEP_PADDING);
    else
      EVP_CIPHER_CTX_set_padding(data->ctx, RSA_PKCS1_PADDING);
  }
  switch (alg & SOAP_MEC_MASK & ~SOAP_MEC_ALGO)
  {
    case SOAP_MEC_ENV_ENC:
#if (OPENSSL_VERSION_NUMBER >= 0x0090800fL)
      ok = EVP_CIPHER_CTX_rand_key(data->ctx, data->ekey);
#elif defined(EVP_CIPH_RAND_KEY)
      if (data->ctx->cipher->flags & EVP_CIPH_RAND_KEY)
        ok = EVP_CIPHER_CTX_ctrl(data->ctx, EVP_CTRL_RAND_KEY, 0, data->ekey);
      else
        ok = RAND_bytes(data->ekey, data->ctx->key_len);
#else
      ok = RAND_bytes(data->ekey, data->ctx->key_len);
#endif
      /* generate ephemeral secret key */
#if (OPENSSL_VERSION_NUMBER >= 0x10000000L)
      *keylen = EVP_PKEY_encrypt_old(key, data->ekey, EVP_CIPHER_CTX_key_length(data->ctx), pkey);
#else
      *keylen = EVP_PKEY_encrypt(key, data->ekey, EVP_CIPHER_CTX_key_length(data->ctx), pkey);
#endif
      key = data->ekey;
      /* fall through to next arm */
    case SOAP_MEC_ENC:
      data->bufidx = 0;
      data->buflen = 1024; /* > iv in base64 must fit */
      data->buf = (char*)SOAP_MALLOC(soap, data->buflen);
      data->key = key;
      break;
    case SOAP_MEC_ENV_DEC:
    case SOAP_MEC_DEC:
      data->pkey = pkey;
      data->key = key;
      data->keylen = *keylen;
      break;
    default:
      return soap_set_receiver_error(soap, "Unsupported encryption algorithm", NULL, SOAP_SSL_ERROR);
  }
  return soap_mec_check(soap, data, ok, "soap_mec_init() failed");
}

/******************************************************************************/

/**
@fn int soap_mec_update(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n)
@brief Update mecevp engine state: encrypts plain text (or raw data) or decrypts cipher data in base64 format.
@param soap context
@param[in,out] data mecevp engine context
@param[in,out] s input data to convert, afterwards points to converted data (original content is unchanged)
@param[in,out] n size of input, afterwards size of output
@return SOAP_OK or SOAP_SSL_ERROR
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_mec_update(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n)
{
  return soap_mec_upd(soap, data, s, n, 0);
}

/******************************************************************************/

/**
@fn int soap_mec_final(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n)
@brief Ends mecevp engine state: encrypt/decrypt remainder from buffers.
@param soap context
@param[in,out] data mecevp engine context
@param[out] s afterwards points to converted remaining data in streaming mode, or entire converted data in buffer mode (SOAP_MEC_STORE option)
@param[out] n afterwards size of remaining data
@return SOAP_OK or SOAP_SSL_ERROR
*/
int
soap_mec_final(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "soap_mec_final()\n"));
  *n = 0;
  if (!data->ctx)
    return SOAP_OK;
  if (soap_mec_upd(soap, data, s, n, 1))
    return soap->error;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn void soap_mec_cleanup(struct soap *soap, struct soap_mec_data *data)
@brief Clean up mecevp engine and deallocate cipher context and buffers.
@param soap context
@param[in,out] data mecevp engine context
@return SOAP_OK or SOAP_SSL_ERROR
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_mec_cleanup(struct soap *soap, struct soap_mec_data *data)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "soap_mec_cleanup()\n"));
  data->alg = SOAP_MEC_NONE;
  data->state = SOAP_MEC_STATE_NONE;
  data->type = NULL;
  if (data->ctx)
  {
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
    EVP_CIPHER_CTX_cleanup(data->ctx);
    SOAP_FREE(soap, data->ctx);
#else
    EVP_CIPHER_CTX_free(data->ctx);
#endif
    data->ctx = NULL;
  }
  if (data->buf)
  {
    SOAP_FREE(soap, data->buf);
    data->buf = NULL;
    data->buflen = 0;
  }
  if (data->rest)
  {
    SOAP_FREE(soap, data->rest);
    data->rest = NULL;
    data->restlen = 0;
  }
}

/******************************************************************************/

/**
@fn int soap_mec_begin(struct soap *soap, struct soap_mec_data *data, int alg, SOAP_MEC_KEY_TYPE *pkey, unsigned char *key, int *keylen)
@brief Initialize the mecevp engine data and begin encryption or decryption message sequence using a private/public key or symmetric secret key.
@param soap context
@param[in,out] data mecevp engine context
@param[in] alg encryption/decryption algorithm
@param[in] pkey public/private key or NULL
@param[in,out] key secret key or encrypted ephemeral secret key set with envelope encryption, or NULL
@param[in,out] keylen secret key length
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_mec_begin(struct soap *soap, struct soap_mec_data *data, int alg, SOAP_MEC_KEY_TYPE *pkey, unsigned char *key, int *keylen)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "MEC Begin alg=0x%x\n", alg));
  /* save and set the engine's 'data' field to pass data to the callbacks */
  soap->data[1] = (void*)data;
  data->ctx = NULL;
  data->type = NULL;
  data->pkey = NULL;
  data->key = NULL;
  data->buf = NULL;
  data->bufidx = 0;
  data->buflen = 0;
  data->rest = NULL;
  data->restidx = 0;
  data->restlen = 0;
  data->taglen = 0;
  /* save the mode flag */
  data->mode = soap->mode;
  if (alg & SOAP_MEC_ENC)
  {
    /* clear the IO flags and DOM flag */
    soap->mode &= ~(SOAP_IO | SOAP_IO_LENGTH | SOAP_ENC_ZLIB | SOAP_XML_DOM);
    /* clear the XML attribute store */
    soap_clr_attr(soap);
    /* load the local XML namespaces store */
    soap_set_local_namespaces(soap);
    if (soap->mode & SOAP_XML_CANONICAL)
      soap->ns = 0; /* for in c14n, we must have all xmlns bindings available */
  }
  else if (soap->ffilterrecv != soap_mec_filterrecv)
  {
    /* save and override the callbacks */
    data->ffilterrecv = soap->ffilterrecv;
    soap->ffilterrecv = soap_mec_filterrecv;
  }
  /* init the soap_mec engine */
  return soap_mec_init(soap, data, alg, pkey, key, keylen);
}

/******************************************************************************/

/**
@fn int soap_mec_start_alg(struct soap *soap, int alg, const unsigned char *key)
@brief Start encryption or decryption of current message. If key is non-NULL, use the symmetric key with alg. Use soap_mec_start only after soap_mec_begin. The soap_mec_start should be followed by a soap_mec_stop call.
@param soap context
@param[in] alg algorithm
@param[in] key secret DES/AES key or NULL for private key
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_mec_start_alg(struct soap *soap, int alg, const unsigned char *key)
{
  struct soap_mec_data *data;
  int ok = 1;
  data = (struct soap_mec_data*)soap->data[1];
  if (!data)
    return soap->error = SOAP_USER_ERROR;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "MEC Start alg=0x%x\n", data->alg));
  if (key)
    data->key = key;
  if (alg != SOAP_MEC_NONE)
    data->alg = alg;
  if (data->alg & SOAP_MEC_ENC)
  {
    unsigned char iv[EVP_MAX_IV_LENGTH];
    int ivlen;
    if (!data->type)
      return soap_mec_check(soap, data, 0, "soap_mec_start_alg() failed: no cipher");
    /* save and override the callbacks */
    data->ffiltersend = soap->ffiltersend;
    soap->ffiltersend = soap_mec_filtersend;
    data->bufidx = 0;
    data->i = 0;
    data->m = 0;
    ivlen = EVP_CIPHER_iv_length(data->type);
    if (ivlen)
    {
#if (OPENSSL_VERSION_NUMBER >= 0x10100000L)
      RAND_bytes(iv, ivlen);
#else
      RAND_pseudo_bytes(iv, ivlen);
#endif
      soap_mec_put_base64(soap, data, (unsigned char*)iv, ivlen);
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "IV = "));
    DBGHEX(TEST, iv, ivlen);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n"));
    ok = EVP_EncryptInit_ex(data->ctx, NULL, NULL, data->key, iv);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_EncryptInit ok=%d\n", ok));
  }
  else
  {
    size_t len;
    /* algorithm */
    switch (data->alg & SOAP_MEC_ALGO)
    {
      case SOAP_MEC_DES_CBC:
        data->type = EVP_des_ede3_cbc();
        break;
      case SOAP_MEC_AES128_CBC:
        data->type = EVP_aes_128_cbc();
        break;
      case SOAP_MEC_AES192_CBC:
        data->type = EVP_aes_192_cbc();
        break;
      case SOAP_MEC_AES256_CBC:
        data->type = EVP_aes_256_cbc();
        break;
      case SOAP_MEC_AES512_CBC:
        data->type = NULL; /* N/A */
        break;
#if (OPENSSL_VERSION_NUMBER >= 0x10002000L)
      case SOAP_MEC_AES128_GCM:
        data->type = EVP_aes_128_gcm();
        break;
      case SOAP_MEC_AES192_GCM:
        data->type = EVP_aes_192_gcm();
        break;
      case SOAP_MEC_AES256_GCM:
        data->type = EVP_aes_256_gcm();
        break;
      case SOAP_MEC_AES512_GCM:
        data->type = NULL; /* N/A */
        break;
#endif
      default:
        data->type = NULL;
    }
    if (!data->type)
      return soap_mec_check(soap, data, 0, "soap_mec_start_alg() failed: cannot load cipher");
    len = 2 * sizeof(soap->buf) + EVP_CIPHER_block_size(data->type);
    if (!data->buf || data->buflen < len)
    {
      if (data->buf)
        SOAP_FREE(soap, data->buf);
      data->buflen = len;
      data->buf = (char*)SOAP_MALLOC(soap, data->buflen);
    }
    data->bufidx = soap->buflen - soap->bufidx;
    /* copy buf[bufidx..buflen-1] to data buf */
    (void)soap_memcpy((void*)data->buf, data->buflen, (const void*)(soap->buf + soap->bufidx), data->bufidx);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Alloc buf=%lu, copy %lu message bytes\n", (unsigned long)data->buflen, (unsigned long)data->bufidx));
    /* trigger ffilterrecv() */
    soap->bufidx = soap->buflen;
    /* INIT state */
    data->i = 0;
    data->m = 0;
    data->state = SOAP_MEC_STATE_INIT;
  }
  return soap_mec_check(soap, data, ok, "soap_mec_start() failed");
}

/******************************************************************************/

/**
@fn int soap_mec_start(struct soap *soap, const unsigned char *key)
@brief Start encryption or decryption of current message. If key is non-NULL, use the symmetric key with alg. Use soap_mec_start only after soap_mec_begin. The soap_mec_start should be followed by a soap_mec_stop call.
@param soap context
@param[in] key secret DES/AES key or NULL
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_mec_start(struct soap *soap, const unsigned char *key)
{
  return soap_mec_start_alg(soap, SOAP_MEC_NONE, key);
}

/******************************************************************************/

/**
@fn int soap_mec_stop(struct soap *soap)
@brief Stops encryption or decryption of current message. Use after soap_mec_start.
@param soap context
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_mec_stop(struct soap *soap)
{
  struct soap_mec_data *data;
  int err = SOAP_OK;
  data = (struct soap_mec_data*)soap->data[1];
  if (!data)
    return soap->error = SOAP_USER_ERROR;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "MEC Stop alg=0x%x\n", data->alg));
  if (data->alg & SOAP_MEC_ENC)
  {
    const char *s = NULL;
    size_t n = 0;
    err = soap_mec_final(soap, data, &s, &n);
    /* reset callbacks */
    if (soap->ffiltersend == soap_mec_filtersend)
      soap->ffiltersend = data->ffiltersend;
    /* send remaining cipher data */
    if (!err && n)
      if (soap_send_raw(soap, s, n))
        return soap->error;
  }
  return err;
}

/******************************************************************************/

/**
@fn int soap_mec_end(struct soap *soap, struct soap_mec_data *data)
@brief Ends encryption or decryption of a sequence of message parts that began
with soap_mec_begin.
@param soap context
@param[in,out] data mecevp engine context
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_mec_end(struct soap *soap, struct soap_mec_data *data)
{
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "MEC End alg=0x%x\n", data->alg));
  /* reset callbacks */
  if (soap->ffiltersend == soap_mec_filtersend)
    soap->ffiltersend = data->ffiltersend;
  if (soap->ffilterrecv == soap_mec_filterrecv)
    soap->ffilterrecv = data->ffilterrecv;
  /* restore the mode flag */
  soap->mode = data->mode;
  /* cleanup and reset mecevp engine */
  soap_mec_cleanup(soap, data);
  soap->data[1] = NULL;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn size_t soap_mec_size(int alg, SOAP_MEC_KEY_TYPE *pkey)
@brief Returns the number of octets needed to store the public/private key or
the symmetric key, depending on the algorithm.
@param[in] alg is the algorithm to be used
@param[in] pkey is a pointer to an EVP_PKEY object or NULL for symmetric keys
@return size_t number of octets that is needed to hold the key.
*/
SOAP_FMAC1
size_t
SOAP_FMAC2
soap_mec_size(int alg, SOAP_MEC_KEY_TYPE *pkey)
{
  if (alg & SOAP_MEC_ENV)
    return (size_t)EVP_PKEY_size(pkey);
  switch (alg & SOAP_MEC_ALGO & ~SOAP_MEC_GCM)
  {
    case SOAP_MEC_DES_CBC:
      return 24;
    case SOAP_MEC_AES128_CBC: /* CBC and GCM */
      return 16;
    case SOAP_MEC_AES192_CBC: /* CBC and GCM */
      return 24;
    case SOAP_MEC_AES256_CBC: /* CBC and GCM */
      return 32;
    case SOAP_MEC_AES512_CBC: /* CBC and GCM */
      return 64;
  }
  return 0;
}

/******************************************************************************\
 *
 *      Static local functions
 *
\******************************************************************************/

/**
@fn int soap_mec_upd(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n, int final)
@brief Update encryption/decryption state depending on the current algorithm
@param soap context
@param[in,out] data mecevp engine context
@param[in,out] s input data to convert, afterwards points to converted data (original content is unchanged)
@param[in,out] n size of input, afterwards size of output
@param[in] final flag to indicate no more input, output is flushed to s
@return SOAP_OK or SOAP_SSL_ERROR
*/
static int
soap_mec_upd(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n, int final)
{
  if (!data || !data->ctx)
    return soap->error = SOAP_USER_ERROR;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "-- MEC Update alg=0x%x n=%lu final=%d (%p) --\n", data->alg, (unsigned long)*n, final, data->ctx));
  DBGMSG(TEST, *s, *n);                                     
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--\n"));
  if (data->alg & SOAP_MEC_ENC)
  {
    if (soap_mec_upd_enc(soap, data, s, n, final))
      return soap->error;
  }
  else
  {
    if (soap_mec_upd_dec(soap, data, s, n, final))
      return soap->error;
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--\n"));
  DBGMSG(TEST, *s, *n);                                     
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--\n"));
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_mec_upd_enc(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n, int final)
@brief Update encryption state with input plain text (or raw) data and output
in base64 format.
@param soap context
@param[in,out] data mecevp engine context
@param[in,out] s input plain text, afterwards points to output cipher data
@param[in,out] n size of input text, afterwards size of cipher data
@param[in] final flag to indicate no more input, output is flushed to s
@return SOAP_OK or SOAP_SSL_ERROR
*/
static int
soap_mec_upd_enc(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n, int final)
{
  size_t k;
  int m;
  int ok = 0;
  if (!data->type)
    return soap_mec_check(soap, data, 0, "soap_mec_upd_enc() failed");
  /* cipher size */
  k = *n + EVP_CIPHER_block_size(data->type);
  /* scale by base64 size + in-use part + 8 margin */
  m = data->bufidx + 8 + (k + 2) / 3 * 4 + 1;
#if (OPENSSL_VERSION_NUMBER >= 0x10002000L)
  if (final && (data->alg & SOAP_MEC_GCM))
    m += sizeof(data->tag);
#endif
  /* fits in buf after bufidx? */
  if (m > (int)data->buflen)
  {
    char *t = data->buf;
    data->buflen = (size_t)m; /* + slack? */
    data->buf = (char*)SOAP_MALLOC(soap, m);
    if (t)
    {
      (void)soap_memcpy((void*)data->buf, (size_t)m, (const void*)t, data->bufidx); /* copy in-use part */
      SOAP_FREE(soap, t);
    }
  }
  if (!final)
  {
    /* envelope encryption or with shared key? */
    if (data->alg & SOAP_MEC_ENV)
    {
      ok = EVP_SealUpdate(data->ctx, (unsigned char*)data->buf + data->buflen - k, &m, (unsigned char*)*s, (int)*n);
      DBGHEX(TEST, (unsigned char*)(data->buf + data->buflen - k), m);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--\n"));
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_SealUpdate ok=%d\n", ok));
    }
    else
    {
      ok = EVP_EncryptUpdate(data->ctx, (unsigned char*)data->buf + data->buflen - k, &m, (unsigned char*)*s, (int)*n);
      DBGHEX(TEST, (unsigned char*)(data->buf + data->buflen - k), m);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--\n"));
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_EncryptUpdate ok=%d\n", ok));
    }
    /* convert to base64 */
    soap_mec_put_base64(soap, data, (unsigned char*)(data->buf + data->buflen - k), m);
    *s = data->buf;
    *n = data->bufidx;
    if (!(data->alg & SOAP_MEC_STORE))
      data->bufidx = 0;
  }
  else
  {
    /* envelope encryption or with shared key? */
    if (data->alg & SOAP_MEC_ENV)
    {
      ok = EVP_SealFinal(data->ctx, (unsigned char*)data->buf + data->buflen - k, &m);
      DBGHEX(TEST, (unsigned char*)(data->buf + data->buflen - k), m);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--\n"));
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_SealFinal ok=%d\n", ok));
    }
    else
    {
      ok = EVP_EncryptFinal(data->ctx, (unsigned char*)data->buf + data->buflen - k, &m);
      DBGHEX(TEST, (unsigned char*)(data->buf + data->buflen - k), m);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--\n"));
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_EncryptFinal ok=%d\n", ok));
    }
    /* convert to base64 */
    soap_mec_put_base64(soap, data, (unsigned char*)(data->buf + data->buflen - k), m);
#if (OPENSSL_VERSION_NUMBER >= 0x10002000L)
    if (data->alg & SOAP_MEC_GCM)
    {
      /* add GCM tag in base64 */
      EVP_CIPHER_CTX_ctrl(data->ctx, EVP_CTRL_GCM_GET_TAG, sizeof(data->tag), data->tag);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Get GCM tag = "));
      DBGHEX(TEST, data->tag, sizeof(data->tag));
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n"));
      soap_mec_put_base64(soap, data, (unsigned char*)(data->tag), sizeof(data->tag));
    }
#endif
    soap_mec_end_base64(soap, data);
    *s = data->buf;
    *n = data->bufidx;
    if (!(data->alg & SOAP_MEC_STORE))
      data->bufidx = 0;
  }
  if (m > (int)k)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Assertion m<=k failed k=%lu m=%lu\n", (unsigned long)k, (unsigned long)m));
    return soap->error = SOAP_USER_ERROR;
  }
  return soap_mec_check(soap, data, ok, "soap_mec_upd_enc() failed");
}

/******************************************************************************/

/**
@fn int soap_mec_upd_dec(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n, int final)
@brief Update decryption state with input cipher data in base64 format and output in plain text (or raw) format
@param soap context
@param[in,out] data mecevp engine context
@param[in,out] s input cipher data, afterwards points to output plain text
@param[in,out] n size of input cipher data, afterwards size of plain text
@param[in] final flag to indicate no more input, output is flushed to s
@return SOAP_OK or SOAP_SSL_ERROR
*/
static int
soap_mec_upd_dec(struct soap *soap, struct soap_mec_data *data, const char **s, size_t *n, int final)
{
  const char *r = NULL;
  size_t k = 0, l = 0, m = 0;
  int len = 0;
  int ok = 1;
  int rest = 0;
  enum SOAP_MEC_STATE state = data->state;
  if (!data->type)
    return soap_mec_check(soap, data, 0, "soap_mec_upd_dec() failed");
  if (final && state == SOAP_MEC_STATE_DECRYPT)
    data->state = SOAP_MEC_STATE_FINAL;
  /* if flushing the buf, no base64-decode or decryption to do */
  if (state == SOAP_MEC_STATE_FLUSH || state == SOAP_MEC_STATE_NONE)
  {
    /* old + new fit in buf? */
    if (data->bufidx + *n > data->buflen)
    {
      char *t = data->buf;
      do
        data->buflen += sizeof(soap->buf);
      while (data->buflen < data->bufidx + *n);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Enlarging buffer n=%lu\n", (unsigned long)data->buflen));
      data->buf = (char*)SOAP_MALLOC(soap, data->buflen);
      if (t)
      {
        (void)soap_memcpy((void*)data->buf, data->buflen, (const void*)t, data->bufidx); /* copy old */
        SOAP_FREE(soap, t);
      }
    }
    /* concat old + new */
    (void)soap_memcpy((void*)(data->buf + data->bufidx), data->buflen - data->bufidx, (const void*)*s, *n);
    *s = data->buf;
    *n += data->bufidx;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Flush state n=%lu\n", (unsigned long)*n));
    /* release old + new for next round (assuming consumer fetches all) */
    if (!(data->alg & SOAP_MEC_STORE))
      data->bufidx = 0;
    return SOAP_OK;
  }
  if (state == SOAP_MEC_STATE_INIT)
  {
    /* at init, base64 is in data->buf[bufidx] copied from buf[] */
    data->i = 0;
    data->m = 0;
    k = (data->bufidx + *n + 3) / 4 * 3; /* decoded size from old + new */
    data->taglen = 0; /* no GCM tag (yet) */
  }
  else
    k = (*n + 3) / 4 * 3; /* base64-decoded size */
  m = k + EVP_CIPHER_block_size(data->type); /* decrypted data size */
  /* decrypted + base64-decoded + GCM tag all fit in current buf? */
  if (data->buflen < data->bufidx + m + k + data->taglen)
  {
    /* no, need to enlarge */
    char *t = data->buf;
    do
      data->buflen += sizeof(soap->buf);
    while (data->buflen < data->bufidx + m + k + data->taglen);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Enlarging buffer n=%lu\n", (unsigned long)data->buflen));
    data->buf = (char*)SOAP_MALLOC(soap, data->buflen);
    if (t)
    {
      (void)soap_memcpy((void*)data->buf, data->buflen, (const void*)t, data->bufidx); /* copy old part */
      SOAP_FREE(soap, t);
    }
  }
  /* base64 decode */
  if (state == SOAP_MEC_STATE_INIT)
  {
    /* base64 is in data buf[0..bufidx-1] and *s */
    if (soap_mec_get_base64(soap, data, data->buf + data->buflen - k, &m, data->buf, data->bufidx, &r, &l))
      return soap->error;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Base64 stage 1 s=%p n=%lu m=%lu r=%p l=%lu\n", *s, (unsigned long)data->bufidx, (unsigned long)m, r, (unsigned long)l));
    /* position 'r' is at a spot that gets overwritten, copy to rest */
    if (r)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Base64 stage 2 rest=%lu bytes\n", (unsigned long)*n));
      rest = *n;
    }
    else
    {
      size_t j;
      /* base64-decode *s */
      if (soap_mec_get_base64(soap, data, data->buf + data->buflen - k + m, &j, *s, *n, &r, &l))
        return soap->error;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Base64 stage 3 s=%p n=%lu m=%lu r=%p l=%lu\n", *s, (unsigned long)*n, (unsigned long)m, r, (unsigned long)l));
      m += j;
    }
    data->bufidx = 0;
  }
  else if (state != SOAP_MEC_STATE_FINAL && state != SOAP_MEC_STATE_FLUSH)
  {
    /* base64-decode *s */
    if (soap_mec_get_base64(soap, data, data->buf + data->buflen - k, &m, *s, *n, &r, &l))
      return soap->error;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Base64 stage 4 n=%lu m=%lu l=%lu\n", (unsigned long)*n, (unsigned long)m, (unsigned long)l));
  }
  if (r)
  {
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Rest = %lu + %lu bytes\n", (unsigned long)l, (unsigned long)rest));
    if (data->restlen < l + rest)
    {
      if (data->rest)
        SOAP_FREE(soap, data->rest);
      data->restlen = l + rest;
      data->rest = (char*)SOAP_MALLOC(soap, data->restlen);
    }
    data->restidx = l + rest;
    (void)soap_memcpy((void*)data->rest, data->restlen, (const void*)r, l);
    (void)soap_memcpy((void*)(data->rest + l), data->restlen - l, (const void*)*s, rest);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "--\n"));
    DBGMSG(TEST, data->rest, data->restidx);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--\n"));
  }
  /* debug */
  DBGHEX(TEST, (unsigned char*)(data->buf + data->buflen - k), m);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--\n"));
  /* decryption of data buf[buflen-k] */
  switch (data->state)
  {
    case SOAP_MEC_STATE_INIT:
      /* move to next state */
      state = SOAP_MEC_STATE_IV;
    case SOAP_MEC_STATE_IV:
      /* get the IV data from buf[buflen-k] */
      (void)soap_memmove((void*)(data->buf + data->bufidx), data->buflen - data->bufidx, (const void*)(data->buf + data->buflen - k), m);
      /* add to IV */
      data->bufidx += m;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Getting IV new size=%lu\n", data->bufidx));
      /* got all IV data? */
      if (data->bufidx >= (size_t)EVP_CIPHER_iv_length(data->type))
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Got IV = "));
        DBGHEX(TEST, (unsigned char*)data->buf, EVP_CIPHER_iv_length(data->type));
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\nInitializing alg=0x%x\n", data->alg));
        switch (data->alg & SOAP_MEC_MASK & ~SOAP_MEC_ALGO)
        {
          case SOAP_MEC_ENV_DEC:
            ok = EVP_OpenInit(data->ctx, data->type, (unsigned char*)data->key, data->keylen, (unsigned char*)data->buf, (EVP_PKEY*)data->pkey);
            EVP_CIPHER_CTX_set_padding(data->ctx, 0); /* disable padding check when decrypting for interop */
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_OpenInit ok=%d\n", ok));
            break;
          case SOAP_MEC_DEC:
            ok = EVP_DecryptInit_ex(data->ctx, data->type, NULL, data->key, (unsigned char*)data->buf);
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_DecryptInit_ex ok=%d\n", ok));
            break;
        }
        if (ok)
        {
          /* shift rest of data to cipher section */
          m = k = data->bufidx - EVP_CIPHER_iv_length(data->type);
          (void)soap_memmove((void*)(data->buf + data->buflen - k), m, (const void*)(data->buf + EVP_CIPHER_iv_length(data->type)), m);
#if (OPENSSL_VERSION_NUMBER >= 0x10002000L)
          if (data->alg & SOAP_MEC_GCM)
          {
            /* rotate the last 128 bits through tag[] buffer */
            if (m < sizeof(data->tag))
            {
              (void)soap_memcpy((void*)data->tag, sizeof(data->tag), (const void*)(data->buf + data->buflen - k), m);
              data->taglen = m;
              m = 0;
            }
            else
            {
              m -= sizeof(data->tag);
              (void)soap_memcpy((void*)data->tag, sizeof(data->tag), (const void*)(data->buf + data->buflen - k + m), sizeof(data->tag));
              data->taglen = sizeof(data->tag);
            }
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Init GCM tag with %lu bytes\n", (unsigned long)data->taglen));
          }
#endif
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Decrypt %lu bytes\n", (unsigned long)m));
          /* decrypt to buf */
          len = 0;
          switch (data->alg & SOAP_MEC_MASK & ~SOAP_MEC_ALGO)
          {
            case SOAP_MEC_ENV_DEC:
              ok = EVP_OpenUpdate(data->ctx, (unsigned char*)data->buf, &len, (unsigned char*)(data->buf + data->buflen - k), m);
              DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_OpenUpdate len=%d ok=%d\n", len, ok));
              break;
            case SOAP_MEC_DEC:
              ok = EVP_DecryptUpdate(data->ctx, (unsigned char*)data->buf, &len, (unsigned char*)(data->buf + data->buflen - k), m);
              DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_DecryptUpdate len=%d ok=%d\n", len, ok));
              break;
          }
          *s = data->buf;
          *n = (size_t)len;
        }
        if (!(data->alg & SOAP_MEC_STORE))
          data->bufidx = 0; /* next decoded goes to start of buf */
        else
          data->bufidx = *n;
        /* next state */
        state = SOAP_MEC_STATE_DECRYPT;
      }
      else
      {
        /* nothing to return yet, need more data */
        *n = 0;
      }
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "IV/decrypt state n=%lu\n", (unsigned long)*n));
      break;
    case SOAP_MEC_STATE_DECRYPT:
#if (OPENSSL_VERSION_NUMBER >= 0x10002000L)
      if (data->alg & SOAP_MEC_GCM)
      {
        /* rotate the last 128 bits through tag[] buffer */
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Rotate GCM tag with %lu bytes\n", (unsigned long)data->taglen));
        k += data->taglen;
        m += data->taglen;
        (void)soap_memcpy((void*)(data->buf + data->buflen - k), k, (const void*)data->tag, data->taglen);
        if (m < sizeof(data->tag))
        {
          (void)soap_memcpy((void*)data->tag, sizeof(data->tag), (const void*)(data->buf + data->buflen - k), m);
          data->taglen = m;
          m = 0;
        }
        else
        {
          m -= sizeof(data->tag);
          (void)soap_memcpy((void*)data->tag, sizeof(data->tag), (const void*)(data->buf + data->buflen - k + m), sizeof(data->tag));
          data->taglen = sizeof(data->tag);
        }
      }
#endif
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Decrypt %lu bytes\n", (unsigned long)m));
      len = 0;
      switch (data->alg & SOAP_MEC_MASK & ~SOAP_MEC_ALGO)
      {
        case SOAP_MEC_ENV_DEC:
          ok = EVP_OpenUpdate(data->ctx, (unsigned char*)(data->buf + data->bufidx), &len, (unsigned char*)(data->buf + data->buflen - k), m);
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_OpenUpdate len=%d ok=%d\n", len, ok));
          break;
        case SOAP_MEC_DEC:
          ok = EVP_DecryptUpdate(data->ctx, (unsigned char*)(data->buf + data->bufidx), &len, (unsigned char*)data->buf + data->buflen - k, m);
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_DecryptUpdate len=%d ok=%d\n", len, ok));
          break;
      }
      *s = data->buf;
      *n = data->bufidx + (size_t)len;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Decrypt state n=%lu\n", (unsigned long)*n));
      if (!(data->alg & SOAP_MEC_STORE))
        data->bufidx = 0; /* next decoded goes to start of buf */
      break;
    case SOAP_MEC_STATE_FINAL:
    {
      /* we know there is enough space to flush *s and *n through the buf */
      const char *t = *s;
#if (OPENSSL_VERSION_NUMBER >= 0x10002000L)
      if (data->alg & SOAP_MEC_GCM)
      {
        /* use the tag[] buffer */
        EVP_CIPHER_CTX_ctrl(data->ctx, EVP_CTRL_GCM_SET_TAG, sizeof(data->tag), data->tag);
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Set GCM tag = "));
        DBGHEX(TEST, data->tag, sizeof(data->tag));
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n"));
      }
#endif
      k = *n;
      len = 0;
      switch (data->alg & SOAP_MEC_MASK & ~SOAP_MEC_ALGO)
      {
        case SOAP_MEC_ENV_DEC:
          ok = EVP_OpenFinal(data->ctx, (unsigned char*)(data->buf + data->bufidx), &len);
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_OpenFinal ok=%d\n", ok));
          break;
        case SOAP_MEC_DEC:
          ok = EVP_DecryptFinal(data->ctx, (unsigned char*)(data->buf + data->bufidx), &len);
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EVP_DecryptFinal ok=%d\n", ok));
          break;
      }
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Append %d bytes from decrypted\n", len));
      *s = data->buf;
      *n = data->bufidx + (size_t)len;
      if (data->restidx)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Append %lu bytes from rest\n", data->restidx));
        (void)soap_memcpy((void*)(data->buf + *n), data->buflen - *n, (const void*)data->rest, data->restidx);
        *n += data->restidx;
      }
      if (k)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Append %lu bytes from input\n", k));
        (void)soap_memmove((void*)(data->buf + *n), data->buflen - *n, (const void*)t, k);
        *n += k;
      }
      if (!(data->alg & SOAP_MEC_STORE))
        data->bufidx = 0; /* next decoded goes to start of buf */
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Final len=%lu\n", (unsigned long)*n));
      state = SOAP_MEC_STATE_FLUSH; /* flush data buf, if needed */
      break;
    }
    default:
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Error in decryption state\n"));
      return soap->error = SOAP_SSL_ERROR;
  }
  if (r)
  {
    if (state == SOAP_MEC_STATE_DECRYPT)
    {
      state = SOAP_MEC_STATE_FINAL;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Append rest of stream %lu (%lu <= %lu)\n", data->restidx, *n, data->buflen));
    }
  }
  data->state = state;
  return soap_mec_check(soap, data, ok, "soap_mec_upd_dec() failed");
}

/******************************************************************************/

/**
@fn int soap_mec_check(struct soap *soap, struct soap_mec_data *data, int ok, const char *msg)
@brief Check result of init/update/final mecevp engine operations.
@param soap context
@param[in,out] data mecevp engine context
@param[in] ok EVP error value
@param[in] msg error message
@return SOAP_OK or SOAP_SSL_ERROR
*/
static int
soap_mec_check(struct soap *soap, struct soap_mec_data *data, int ok, const char *msg)
{
  if (ok <= 0)
  {
    unsigned long r;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "-- MEC Error (%d)", ok));
    while ((r = ERR_get_error()))
    {
      ERR_error_string_n(r, soap->msgbuf, sizeof(soap->msgbuf));
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, " %s: \"%s\";", msg, soap->msgbuf));
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n"));
    /* cleanup and reset mecevp engine */
    soap_mec_cleanup(soap, data);
    return soap_set_receiver_error(soap, msg, soap->msgbuf, SOAP_SSL_ERROR);
  }
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn void soap_mec_put_base64(struct soap *soap, struct soap_mec_data *data, const unsigned char *s, int n)
@brief Write base64 formatted data stored in s of length n to internal buffer
@param soap context
@param[in,out] data mecevp engine context
@param[in] s data to convert
@param[in] n length of data to convert
*/
static void
soap_mec_put_base64(struct soap *soap, struct soap_mec_data *data, const unsigned char *s, int n)
{
  char *t;
  int i;
  unsigned long m;
  (void)soap;
  if (!s || !n)
    return;
  t = data->buf + data->bufidx;
  i = data->i;
  m = data->m;
  while (n--)
  {
    m = (m << 8) | *s++;
    if (i++ == 2)
    {
      for (i = 4; i > 0; m >>= 6)
        t[--i] = soap_base64o[m & 0x3F];
      t += 4;
      data->bufidx += 4;
    }
  }
  data->i = i;
  data->m = m;
}

/******************************************************************************/

/**
@fn void soap_mec_end_base64(struct soap *soap, struct soap_mec_data *data)
@brief End writing base64 formatted data to internal buffer
@param soap context
@param[in,out] data mecevp engine context
*/
static void
soap_mec_end_base64(struct soap *soap, struct soap_mec_data *data)
{
  (void)soap;
  if (data->i)
  {
    char *t;
    int i;
    unsigned long m;
    t = data->buf + data->bufidx;
    i = data->i;
    m = data->m;
    for (; i < 3; i++)
      m <<= 8;
    for (i++; i > 0; m >>= 6)
      t[--i] = soap_base64o[m & 0x3F];
    for (i = 3; i > data->i; i--)
      t[i] = '=';
    data->bufidx += 4;
  }
  data->i = 0;
  data->m = 0;
}

/******************************************************************************/

/**
@fn int soap_mec_get_base64(struct soap *soap, struct soap_mec_data *data, char *t, size_t *l, const char *s, size_t n, const char **r, size_t *k)
@brief Convert base64-formatted data from s[0..n-1] into raw data in t[0..l-1]
where l is the max size and set equal or lower if data fits in t. If data does
not fit r points to remainder in s[0..n-1] of size k.
@param soap context
@param[in,out] data mecevp engine context
@param[in] t raw data (converted from base64)
@param[in,out] l max size of t[], afterwards actual size of data written to t[]
@param[in] s data in base64 format
@param[in] n size of base64 data
@param[out] r if data does not fit in t[], points to s[] remainder to convert
@param[out] k if data does not fit in t[], size of remainder in r[]
@return SOAP_OK or SOAP_SSL_ERROR
*/
static int
soap_mec_get_base64(struct soap *soap, struct soap_mec_data *data, char *t, size_t *l, const char *s, size_t n, const char **r, size_t *k)
{
  int i;
  unsigned long m;
  size_t j;
  int c;
  i = data->i;
  m = data->m;
  j = 0;
  for (;;)
  {
    do
    {
      if (!n--)
      {
        *l = j;
        data->i = i;
        data->m = m;
        *r = NULL;
        *k = 0;
        return SOAP_OK;
      }
      c = *s++;
      if (c == '=' || c == '<')
      {
        switch (i)
        {
          case 2:
            *t++ = (char)((m >> 4) & 0xFF);
            j++;
            break;
          case 3:
            *t++ = (char)((m >> 10) & 0xFF);
            *t++ = (char)((m >> 2) & 0xFF);
            j += 2;
        }
        if (c == '<')
        {
          s--;
          n++;
        }
        else if (n && *s == '=')
        {
          s++;
          n--;
        }
        *l = j;
        *k = n;
        *r = s;
        return SOAP_OK;
      }
      if (c >= '+' && c <= '+' + 79)
      {
        int b = soap_base64i[c - '+'];
        if (b >= 64)
          return soap->error = SOAP_SSL_ERROR;
        m = (m << 6) + b;
        i++;
      }
      else if (c < 0 || c > 32)
        return soap->error = SOAP_SSL_ERROR;
    } while (i < 4);
    *t++ = (char)((m >> 16) & 0xFF);
    *t++ = (char)((m >> 8) & 0xFF);
    *t++ = (char)(m & 0xFF);
    j += 3;
    i = 0;
    m = 0;
  }
}

/******************************************************************************\
 *
 *      Callbacks registered by plugin
 *
\******************************************************************************/

/**
@fn int soap_mec_filtersend(struct soap *soap, const char **s, size_t *n)
@brief Callback to modify outbound messages by encrypting through the engine.
@param soap context
@param[in,out] s plain text message, afterwards set to encrypted message
@param[in,out] n plain text message size, afterwards set to encrypted message size
@return SOAP_OK or SOAP_SSL_ERROR
*/
static int
soap_mec_filtersend(struct soap *soap, const char **s, size_t *n)
{
  struct soap_mec_data *data = (struct soap_mec_data*)soap->data[1];
  if (!data)
     return SOAP_OK;
  /* encrypt to base64 */
  return soap_mec_upd(soap, data, s, n, 0);
}

/******************************************************************************/

/**
@fn int soap_mec_filterrecv(struct soap *soap, char *buf, size_t *len, size_t maxlen)
@brief Callback to modify inbound messages by decrypting through the engine.
@param soap context
@param[in,out] buf encrypted message, afterwards contains decrypted content
@param[in,out] len encrypted message size, afterwards set to decrypted content size
@param[in] maxlen max length of allocated buf size to contain decrypted content
@return SOAP_OK or SOAP_SSL_ERROR
*/
static int
soap_mec_filterrecv(struct soap *soap, char *buf, size_t *len, size_t maxlen)
{
  struct soap_mec_data *data = (struct soap_mec_data*)soap->data[1];
  const char *s = buf;
  if (!data || (data->alg & SOAP_MEC_MASK) == SOAP_MEC_NONE || (data->alg & SOAP_MEC_ENC))
    return SOAP_OK;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Filter recv len in=%lu maxlen=%lu\n", (unsigned long)*len, (unsigned long)maxlen));
  /* convert s[len] to new s with len (new s = data->buf) */
  if (soap_mec_upd(soap, data, &s, len, 0))
    return soap->error;
  /* need more data? */
  if (*len == 0)
    return SOAP_OK;
  /* does the result fit in buf[maxlen]? */
  if (*len <= maxlen)
  {
    (void)soap_memcpy((void*)buf, maxlen, (const void*)s, *len); /* yes: copy data to buf[] */
    data->bufidx = 0;
  }
  else
  {
    (void)soap_memcpy((void*)buf, maxlen, (const void*)s, maxlen); /* no: copy first part to buf[maxlen] */
    (void)soap_memmove((void*)data->buf, data->buflen, (const void*)(s + maxlen), *len - maxlen); /* shift rest to the left */
    data->bufidx = *len - maxlen; /* keep rest of the data in s (data->buf) */
    *len = maxlen;
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Filter recv len out=%lu\n", (unsigned long)*len));
  return SOAP_OK;
}

/******************************************************************************/

#ifdef __cplusplus
}
#endif
