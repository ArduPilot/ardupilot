/*
    ChibiOS - Copyright (C) 2016..2018 Theodore Ateba

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
 * @file    hal_crypto_lld.h
 * @brief   AVR cryptographic subsystem low level driver header.
 *
 * @addtogroup CRYPTO
 * @{
 */

#ifndef HAL_CRYPTO_LLD_H
#define HAL_CRYPTO_LLD_H

#if (HAL_USE_CRY == TRUE) || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/**
 * @name    Driver capability switches
 * @{
 */
#define CRY_LLD_SUPPORTS_AES                TRUE
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
#define CRY_LLD_SUPPORTS_TRNG               FALSE
/** @} */

/**
 * @brief   Size of one block data, always 128-bits (16 bytes).
 */
#define AES_BLOCK_SIZE  16

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @name    AVR configuration options
 * @{
 */

/**
 * @brief   CRY1 driver enable switch.
 * @details If set to @p TRUE the support for CRY1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_CRY_USE_CRY1) || defined(__DOXYGEN__)
#define AVR_CRY_USE_CRY1                  FALSE
#endif
/** @} */

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief   CRY key identifier type.
 */
typedef uint16_t crykey_t;

/**
 * @brief   Type of a structure representing an CRY driver.
 */
typedef struct CRYDriver CRYDriver;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  bool                      autof;  /* Auto start feature.  */
  bool                      xorf;   /* XOR feature.         */
} CRYConfig;

/**
 * @brief   Structure representing an CRY driver.
 */
struct CRYDriver {
  /**
   * @brief   Driver state.
   */
  crystate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const CRYConfig           *config;
  /**
   * @brief   Algorithm type of transient key.
   */
  cryalgorithm_t            key0_type;
  /**
   * @brief   Size of transient key.
   */
  size_t                    key0_size;
#if (HAL_CRY_USE_FALLBACK == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Key buffer for the fall-back implementation.
   */
  uint8_t                   key0_buffer[HAL_CRY_MAX_KEY_SIZE];
#endif
#if defined(CRY_DRIVER_EXT_FIELDS)
  CRY_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
};

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#if (AVR_CRY_USE_CRY1 == TRUE) && !defined(__DOXYGEN__)
extern CRYDriver CRYD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void cry_lld_init(void);
  void cry_lld_start(CRYDriver *cryp);
  void cry_lld_stop(CRYDriver *cryp);
#if (CRY_LLD_SUPPORTS_AES == TRUE) ||                                       \
    (CRY_LLD_SUPPORTS_AES_ECB == TRUE) ||                                   \
    (CRY_LLD_SUPPORTS_AES_CBC == TRUE) ||                                   \
    (CRY_LLD_SUPPORTS_AES_CFB == TRUE) ||                                   \
    (CRY_LLD_SUPPORTS_AES_CTR == TRUE) ||                                   \
    (CRY_LLD_SUPPORTS_AES_GCM == TRUE) ||                                   \
    defined(__DOXYGEN__)
  cryerror_t cry_lld_aes_loadkey(CRYDriver *cryp,
                                 size_t size,
                                 const uint8_t *keyp);
#endif
#if (CRY_LLD_SUPPORTS_AES == TRUE) || defined(__DOXYGEN__)
  cryerror_t cry_lld_encrypt_AES(CRYDriver *cryp,
                                 crykey_t key_id,
                                 const uint8_t *in,
                                 uint8_t *outi);
  cryerror_t cry_lld_decrypt_AES(CRYDriver *cryp,
                                 crykey_t key_id,
                                 const uint8_t *in,
                                 uint8_t *out);
#endif

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_CRY == TRUE */

#endif /* HAL_CRYPTO_LLD_H */

/** @} */
