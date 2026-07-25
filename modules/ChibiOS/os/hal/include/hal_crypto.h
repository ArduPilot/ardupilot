/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_crypto.h
 * @brief   Cryptographic Driver macros and structures.
 *
 * @addtogroup CRYPTO
 * @{
 */

#ifndef HAL_CRYPTO_H
#define HAL_CRYPTO_H

#if (HAL_USE_CRY == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Enables the SW fall-back of the cryptographic driver.
 * @details When enabled, this option, activates a fall-back software
 *          implementation for algorithms not supported by the underlying
 *          hardware.
 * @note    Fall-back implementations may not be present for all algorithms.
 */
#if !defined(HAL_CRY_USE_FALLBACK) || defined(__DOXYGEN__)
#define HAL_CRY_USE_FALLBACK                FALSE
#endif

/**
 * @brief   Makes the driver forcibly use the fall-back implementations.
 * @note    If enabled then the LLD driver is not included at all.
 */
#if !defined(HAL_CRY_ENFORCE_FALLBACK) || defined(__DOXYGEN__)
#define HAL_CRY_ENFORCE_FALLBACK            FALSE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if HAL_CRY_ENFORCE_FALLBACK == TRUE
#undef HAL_CRY_USE_FALLBACK
#define HAL_CRY_USE_FALLBACK                TRUE
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Size, in bits, of a crypto field or message.
 * @note    It is assumed, for simplicity, that this type is equivalent to
 *          a @p size_t.
 */
typedef size_t bitsize_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  CRY_UNINIT = 0,                           /**< Not initialized.           */
  CRY_STOP = 1,                             /**< Stopped.                   */
  CRY_READY = 2                             /**< Ready.                     */
} crystate_t;

/**
 * @brief   Driver error codes.
 */
typedef enum {
  CRY_NOERROR = 0,                          /**< No error.                  */
  CRY_ERR_INV_ALGO = 1,                     /**< Invalid cypher/mode.       */
  CRY_ERR_INV_KEY_SIZE = 2,                 /**< Invalid key size.          */
  CRY_ERR_INV_KEY_TYPE = 3,                 /**< Invalid key type.          */
  CRY_ERR_INV_KEY_ID = 4,                   /**< Invalid key identifier.    */
  CRY_ERR_AUTH_FAILED = 5,                  /**< Failed authentication.     */
  CRY_ERR_OP_FAILURE = 6                    /**< Failed operation.          */
} cryerror_t;

/**
 * @brief   Type of an algorithm identifier.
 * @note    It is only used to determine the key required for operations.
 */
typedef enum {
  cry_algo_none = 0,
  cry_algo_aes,                             /**< AES 128, 192, 256 bits.    */
  cry_algo_des,                             /**< DES 56, TDES 112, 168 bits.*/
  cry_algo_hmac                             /**< HMAC variable size.        */
} cryalgorithm_t;

#if HAL_CRY_ENFORCE_FALLBACK == FALSE
/* Use the defined low level driver.*/
#include "hal_crypto_lld.h"

#if !defined(CRY_LLD_SUPPORTS_AES) ||                                       \
    !defined(CRY_LLD_SUPPORTS_AES_ECB) ||                                   \
    !defined(CRY_LLD_SUPPORTS_AES_CBC) ||                                   \
    !defined(CRY_LLD_SUPPORTS_AES_CFB) ||                                   \
    !defined(CRY_LLD_SUPPORTS_AES_CTR) ||                                   \
    !defined(CRY_LLD_SUPPORTS_AES_GCM) ||                                   \
    !defined(CRY_LLD_SUPPORTS_DES) ||                                       \
    !defined(CRY_LLD_SUPPORTS_DES_ECB) ||                                   \
    !defined(CRY_LLD_SUPPORTS_DES_CBC) ||                                   \
    !defined(CRY_LLD_SUPPORTS_SHA1) ||                                      \
    !defined(CRY_LLD_SUPPORTS_SHA256) ||                                    \
    !defined(CRY_LLD_SUPPORTS_SHA512) ||                                    \
    !defined(CRY_LLD_SUPPORTS_HMAC_SHA256) ||                               \
    !defined(CRY_LLD_SUPPORTS_HMAC_SHA512)
#error "CRYPTO LLD does not export the required switches"
#endif

#else /* HAL_CRY_ENFORCE_FALLBACK == TRUE */
/* No LLD at all, using the standalone mode.*/

#define CRY_LLD_SUPPORTS_AES                FALSE
#define CRY_LLD_SUPPORTS_AES_ECB            FALSE
#define CRY_LLD_SUPPORTS_AES_CBC            FALSE
#define CRY_LLD_SUPPORTS_AES_CFB            FALSE
#define CRY_LLD_SUPPORTS_AES_CTR            FALSE
#define CRY_LLD_SUPPORTS_AES_GCM            FALSE
#define CRY_LLD_SUPPORTS_DES                FALSE
#define CRY_LLD_SUPPORTS_DES_ECB            FALSE
#define CRY_LLD_SUPPORTS_DES_CBC            FALSE
#define CRY_LLD_SUPPORTS_SHA1               FALSE
#define CRY_LLD_SUPPORTS_SHA256             FALSE
#define CRY_LLD_SUPPORTS_SHA512             FALSE
#define CRY_LLD_SUPPORTS_HMAC_SHA256        FALSE
#define CRY_LLD_SUPPORTS_HMAC_SHA512        FALSE

typedef uint_fast8_t crykey_t;

typedef struct CRYDriver CRYDriver;

typedef struct {
  uint32_t                  dummy;
} CRYConfig;

struct CRYDriver {
  crystate_t                state;
  const CRYConfig           *config;
};
#endif /* HAL_CRY_ENFORCE_FALLBACK == TRUE */

/* The fallback header is included only if required by settings.*/
#if HAL_CRY_USE_FALLBACK == TRUE
#include "hal_crypto_fallback.h"
#endif

#if (HAL_CRY_USE_FALLBACK == FALSE) && (CRY_LLD_SUPPORTS_SHA1 == FALSE)
/* Stub @p SHA1Context structure type declaration. It is not provided by
   the LLD and the fallback is not enabled.*/
typedef struct {
  uint32_t dummy;
} SHA1Context;
#endif

#if (HAL_CRY_USE_FALLBACK == FALSE) && (CRY_LLD_SUPPORTS_SHA256 == FALSE)
/* Stub @p SHA256Context structure type declaration. It is not provided by
   the LLD and the fallback is not enabled.*/
typedef struct {
  uint32_t dummy;
} SHA256Context;
#endif

#if (HAL_CRY_USE_FALLBACK == FALSE) && (CRY_LLD_SUPPORTS_SHA512 == FALSE)
/* Stub @p SHA512Context structure type declaration. It is not provided by
   the LLD and the fallback is not enabled.*/
typedef struct {
  uint32_t dummy;
} SHA512Context;
#endif

#if (HAL_CRY_USE_FALLBACK == FALSE) && (CRY_LLD_SUPPORTS_HMAC_SHA256 == FALSE)
/* Stub @p HMACSHA256Context structure type declaration. It is not provided by
   the LLD and the fallback is not enabled.*/
typedef struct {
  uint32_t dummy;
} HMACSHA256Context;
#endif

#if (HAL_CRY_USE_FALLBACK == FALSE) && (CRY_LLD_SUPPORTS_HMAC_SHA512 == FALSE)
/* Stub @p HMACSHA512Context structure type declaration. It is not provided by
   the LLD and the fallback is not enabled.*/
typedef struct {
  uint32_t dummy;
} HMACSHA512Context;
#endif

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Low level driver helper macros
 * @{
 */
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void cryInit(void);
  void cryObjectInit(CRYDriver *cryp);
  msg_t cryStart(CRYDriver *cryp, const CRYConfig *config);
  void cryStop(CRYDriver *cryp);
  cryerror_t cryLoadAESTransientKey(CRYDriver *cryp,
                                    size_t size,
                                    const uint8_t *keyp);
  cryerror_t cryEncryptAES(CRYDriver *cryp,
                               crykey_t key_id,
                               const uint8_t *in,
                               uint8_t *out);
  cryerror_t cryDecryptAES(CRYDriver *cryp,
                           crykey_t key_id,
                           const uint8_t *in,
                           uint8_t *out);
  cryerror_t cryEncryptAES_ECB(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out);
  cryerror_t cryDecryptAES_ECB(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out);
  cryerror_t cryEncryptAES_CBC(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out,
                               const uint8_t *iv);
  cryerror_t cryDecryptAES_CBC(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out,
                               const uint8_t *iv);
  cryerror_t cryEncryptAES_CFB(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out,
                               const uint8_t *iv);
  cryerror_t cryDecryptAES_CFB(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out,
                               const uint8_t *iv);
  cryerror_t cryEncryptAES_CTR(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out,
                               const uint8_t *iv);
  cryerror_t cryDecryptAES_CTR(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out,
                               const uint8_t *iv);
  cryerror_t cryEncryptAES_GCM(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t auth_size,
                               const uint8_t *auth_in,
                               size_t text_size,
                               const uint8_t *text_in,
                               uint8_t *text_out,
                               const uint8_t *iv,
                               size_t tag_size,
                               uint8_t *tag_out);
  cryerror_t cryDecryptAES_GCM(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t auth_size,
                               const uint8_t *auth_in,
                               size_t text_size,
                               const uint8_t *text_in,
                               uint8_t *text_out,
                               const uint8_t *iv,
                               size_t tag_size,
                               const uint8_t *tag_in);
  cryerror_t cryLoadDESTransientKey(CRYDriver *cryp,
                                    size_t size,
                                    const uint8_t *keyp);
  cryerror_t cryEncryptDES(CRYDriver *cryp,
                           crykey_t key_id,
                           const uint8_t *in,
                           uint8_t *out);
  cryerror_t cryDecryptDES(CRYDriver *cryp,
                           crykey_t key_id,
                           const uint8_t *in,
                           uint8_t *out);
  cryerror_t cryEncryptDES_ECB(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out);
  cryerror_t cryDecryptDES_ECB(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out);
  cryerror_t cryEncryptDES_CBC(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out,
                               const uint8_t *iv);
  cryerror_t cryDecryptDES_CBC(CRYDriver *cryp,
                               crykey_t key_id,
                               size_t size,
                               const uint8_t *in,
                               uint8_t *out,
                               const uint8_t *iv);
  cryerror_t crySHA1Init(CRYDriver *cryp, SHA1Context *sha1ctxp);
  cryerror_t crySHA1Update(CRYDriver *cryp, SHA1Context *sha1ctxp,
                           size_t size, const uint8_t *in);
  cryerror_t crySHA1Final(CRYDriver *cryp, SHA1Context *sha1ctxp,
                          uint8_t *out);
  cryerror_t crySHA256Init(CRYDriver *cryp, SHA256Context *sha256ctxp);
  cryerror_t crySHA256Update(CRYDriver *cryp, SHA256Context *sha256ctxp,
                             size_t size, const uint8_t *in);
  cryerror_t crySHA256Final(CRYDriver *cryp, SHA256Context *sha256ctxp,
                            uint8_t *out);
  cryerror_t crySHA512Init(CRYDriver *cryp, SHA512Context *sha512ctxp);
  cryerror_t crySHA512Update(CRYDriver *cryp, SHA512Context *sha512ctxp,
                             size_t size, const uint8_t *in);
  cryerror_t crySHA512Final(CRYDriver *cryp, SHA512Context *sha512ctxp,
                            uint8_t *out);
  cryerror_t cryLoadHMACTransientKey(CRYDriver *cryp,
                                     size_t size,
                                     const uint8_t *keyp);
  cryerror_t cryHMACSHA256Init(CRYDriver *cryp,
                               HMACSHA256Context *hmacsha256ctxp);
  cryerror_t cryHMACSHA256Update(CRYDriver *cryp,
                                 HMACSHA256Context *hmacsha256ctxp,
                                 size_t size,
                                 const uint8_t *in);
  cryerror_t cryHMACSHA256Final(CRYDriver *cryp,
                                HMACSHA256Context *hmacsha256ctxp,
                                uint8_t *out);
  cryerror_t cryHMACSHA512Init(CRYDriver *cryp,
                               HMACSHA512Context *hmacsha512ctxp);
  cryerror_t cryHMACSHA512Update(CRYDriver *cryp,
                                 HMACSHA512Context *hmacsha512ctxp,
                                 size_t size,
                                 const uint8_t *in);
  cryerror_t cryHMACSHA512Final(CRYDriver *cryp,
                                HMACSHA512Context *hmacsha512ctxp,
                                uint8_t *out);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_CRY == TRUE */

#endif /* HAL_CRYPTO_H */

/** @} */
