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
 * @file    hal_crypto.c
 * @brief   Cryptographic Driver code.
 *
 * @addtogroup CRYPTO
 * @{
 */

#include "hal.h"

#if (HAL_USE_CRY == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Cryptographic Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void cryInit(void) {

#if HAL_CRY_ENFORCE_FALLBACK == FALSE
  cry_lld_init();
#endif
}

/**
 * @brief   Initializes the standard part of a @p CRYDriver structure.
 *
 * @param[out] cryp             pointer to the @p CRYDriver object
 *
 * @init
 */
void cryObjectInit(CRYDriver *cryp) {

  cryp->state    = CRY_STOP;
  cryp->config   = NULL;
#if defined(CRY_DRIVER_EXT_INIT_HOOK)
  CRY_DRIVER_EXT_INIT_HOOK(cryp);
#endif
}

/**
 * @brief   Configures and activates the cryptographic peripheral.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] config            pointer to the @p CRYConfig object. Depending
 *                              on the implementation the value can be @p NULL.
 * @return                      The operation status.
 *
 * @api
 */
msg_t cryStart(CRYDriver *cryp, const CRYConfig *config) {
  msg_t msg;

  osalDbgCheck(cryp != NULL);

  osalSysLock();
  osalDbgAssert((cryp->state == CRY_STOP) || (cryp->state == CRY_READY),
                "invalid state");
  cryp->config = config;
#if HAL_CRY_ENFORCE_FALLBACK == FALSE
#if defined(CRY_LLD_ENHANCED_API)
  msg = cry_lld_start(cryp);
  if (msg == HAL_RET_SUCCESS) {
    cryp->state = CRY_READY;
  }
  else {
    cryp->state = CRY_STOP;
  }
#else
  cry_lld_start(cryp);
  cryp->state = CRY_READY;
  msg = HAL_RET_SUCCESS;
#endif
#else
  cryp->state = CRY_READY;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the cryptographic peripheral.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 *
 * @api
 */
void cryStop(CRYDriver *cryp) {

  osalDbgCheck(cryp != NULL);

  osalSysLock();

  osalDbgAssert((cryp->state == CRY_STOP) || (cryp->state == CRY_READY),
                "invalid state");

#if HAL_CRY_ENFORCE_FALLBACK == FALSE
  cry_lld_stop(cryp);
#endif
  cryp->config = NULL;
  cryp->state  = CRY_STOP;

  osalSysUnlock();
}

/**
 * @brief   Initializes the AES transient key.
 * @note    It is the underlying implementation to decide which key sizes are
 *          allowable.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] size              key size in bytes
 * @param[in] keyp              pointer to the key data
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the algorithm is unsupported.
 * @retval CRY_ERR_INV_KEY_SIZE if the specified key size is invalid for
 *                              the specified algorithm.
 *
 * @api
 */
cryerror_t cryLoadAESTransientKey(CRYDriver *cryp,
                                  size_t size,
                                  const uint8_t *keyp) {

  osalDbgCheck((cryp != NULL) &&  (keyp != NULL));

#if CRY_LLD_SUPPORTS_AES == TRUE
  return cry_lld_aes_loadkey(cryp, size, keyp);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_aes_loadkey(cryp, size, keyp);
#else
  (void)cryp;
  (void)size;
  (void)keyp;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Encryption of a single block using AES.
 * @note    The implementation of this function must guarantee that it can
 *          be called from any context.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] in                buffer containing the input plaintext
 * @param[out] out              buffer for the output ciphertext
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @special
 */
cryerror_t cryEncryptAES(CRYDriver *cryp,
                         crykey_t key_id,
                         const uint8_t *in,
                         uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES == TRUE
  return cry_lld_encrypt_AES(cryp, key_id, in, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_encrypt_AES(cryp, key_id, in, out);
#else
  (void)cryp;
  (void)key_id;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Decryption of a single block using AES.
 * @note    The implementation of this function must guarantee that it can
 *          be called from any context.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] in                buffer containing the input ciphertext
 * @param[out] out              buffer for the output plaintext
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @special
 */
cryerror_t cryDecryptAES(CRYDriver *cryp,
                         crykey_t key_id,
                         const uint8_t *in,
                         uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES == TRUE
  return cry_lld_decrypt_AES(cryp, key_id, in, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_decrypt_AES(cryp, key_id, in, out);
#else
  (void)cryp;
  (void)key_id;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Encryption operation using AES-ECB.
 * @note    The function operates on data buffers whose length is a multiple
 *          of an AES block, this means that padding must be done by the
 *          caller.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers, this number must be a
 *                              multiple of 16
 * @param[in] in                buffer containing the input plaintext
 * @param[out] out              buffer for the output ciphertext
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryEncryptAES_ECB(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t size,
                             const uint8_t *in,
                             uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               ((size & (size_t)15) == (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES_ECB == TRUE
  return cry_lld_encrypt_AES_ECB(cryp, key_id, size, in, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_encrypt_AES_ECB(cryp, key_id, size, in, out);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Decryption operation using AES-ECB.
 * @note    The function operates on data buffers whose length is a multiple
 *          of an AES block, this means that padding must be done by the
 *          caller.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers, this number must be a
 *                              multiple of 16
 * @param[in] in                buffer containing the input ciphertext
 * @param[out] out              buffer for the output plaintext
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryDecryptAES_ECB(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t size,
                             const uint8_t *in,
                             uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               ((size & (size_t)15) == (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES_ECB == TRUE
  return cry_lld_decrypt_AES_ECB(cryp, key_id, size, in, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_decrypt_AES_ECB(cryp, key_id, size, in, out);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Encryption operation using AES-CBC.
 * @note    The function operates on data buffers whose length is a multiple
 *          of an AES block, this means that padding must be done by the
 *          caller.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers, this number must be a
 *                              multiple of 16
 * @param[in] in                buffer containing the input plaintext
 * @param[out] out              buffer for the output ciphertext
 * @param[in] iv                128 bits input vector
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryEncryptAES_CBC(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t size,
                             const uint8_t *in,
                             uint8_t *out,
                             const uint8_t *iv) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               (iv != NULL) && ((size & (size_t)15) == (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES_CBC == TRUE
  return cry_lld_encrypt_AES_CBC(cryp, key_id, size, in, out, iv);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_encrypt_AES_CBC(cryp, key_id, size, in, out, iv);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Decryption operation using AES-CBC.
 * @note    The function operates on data buffers whose length is a multiple
 *          of an AES block, this means that padding must be done by the
 *          caller.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers, this number must be a
 *                              multiple of 16
 * @param[in] in                buffer containing the input ciphertext
 * @param[out] out              buffer for the output plaintext
 * @param[in] iv                128 bits input vector
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryDecryptAES_CBC(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t size,
                             const uint8_t *in,
                             uint8_t *out,
                             const uint8_t *iv) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               (iv != NULL) && ((size & (size_t)15) == (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES_CBC == TRUE
  return cry_lld_decrypt_AES_CBC(cryp, key_id, size, in, out, iv);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_decrypt_AES_CBC(cryp, key_id, size, in, out, iv);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Encryption operation using AES-CFB.
 * @note    This is a stream cipher, there are no size restrictions.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers
 * @param[in] in                buffer containing the input plaintext
 * @param[out] out              buffer for the output ciphertext
 * @param[in] iv                128 bits input vector
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryEncryptAES_CFB(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t size,
                             const uint8_t *in,
                             uint8_t *out,
                             const uint8_t *iv) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               (iv != NULL) && (size > (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES_CFB == TRUE
  return cry_lld_encrypt_AES_CFB(cryp, key_id, size, in, out, iv);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_encrypt_AES_CFB(cryp, key_id, size, in, out, iv);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Decryption operation using AES-CFB.
 * @note    This is a stream cipher, there are no size restrictions.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers
 * @param[in] in                buffer containing the input ciphertext
 * @param[out] out              buffer for the output plaintext
 * @param[in] iv                128 bits input vector
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryDecryptAES_CFB(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t size,
                             const uint8_t *in,
                             uint8_t *out,
                             const uint8_t *iv) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               (iv != NULL) && (size > (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES_CFB == TRUE
  return cry_lld_decrypt_AES_CFB(cryp, key_id, size, in, out, iv);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_decrypt_AES_CFB(cryp, key_id, size, in, out, iv);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Encryption operation using AES-CTR.
 * @note    This is a stream cipher, there are no size restrictions.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers
 * @param[in] in                buffer containing the input plaintext
 * @param[out] out              buffer for the output ciphertext
 * @param[in] iv                128 bits input vector + counter, it contains
 *                              a 96 bits IV and a 32 bits counter
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryEncryptAES_CTR(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t size,
                             const uint8_t *in,
                             uint8_t *out,
                             const uint8_t *iv) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               (iv != NULL) && (size > (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES_CTR == TRUE
  return cry_lld_encrypt_AES_CTR(cryp, key_id, size, in, out, iv);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_encrypt_AES_CTR(cryp, key_id, size, in, out, iv);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Decryption operation using AES-CTR.
 * @note    This is a stream cipher, there are no size restrictions.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers
 * @param[in] in                buffer containing the input ciphertext
 * @param[out] out              buffer for the output plaintext
 * @param[in] iv                128 bits input vector + counter, it contains
 *                              a 96 bits IV and a 32 bits counter
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryDecryptAES_CTR(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t size,
                             const uint8_t *in,
                             uint8_t *out,
                             const uint8_t *iv) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               (iv != NULL) && (size > (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES_CTR == TRUE
  return cry_lld_decrypt_AES_CTR(cryp, key_id, size, in, out, iv);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_decrypt_AES_CTR(cryp, key_id, size, in, out, iv);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Encryption operation using AES-GCM.
 * @note    This is a stream cipher, there are no size restrictions.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] auth_size         size of the data buffer to be authenticated
 * @param[in] auth_in           buffer containing the data to be authenticated
 * @param[in] text_size         size of the text buffer
 * @param[in] text_in           buffer containing the input plaintext
 * @param[out] text_out         buffer for the output ciphertext
 * @param[in] iv                128 bits input vector
 * @param[in] tag_size          size of the authentication tag, this number
 *                              must be between 1 and 16
 * @param[out] tag_out          buffer for the generated authentication tag
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryEncryptAES_GCM(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t auth_size,
                             const uint8_t *auth_in,
                             size_t text_size,
                             const uint8_t *text_in,
                             uint8_t *text_out,
                             const uint8_t *iv,
                             size_t tag_size,
                             uint8_t *tag_out) {

  osalDbgCheck((cryp != NULL) && (auth_in != NULL) &&
               (text_size > (size_t)0) &&
               (text_in != NULL) && (text_out != NULL) && (iv != NULL) &&
               (tag_size >= (size_t)1) && (tag_size <= (size_t)16) &&
               (tag_out != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES_GCM == TRUE
  return cry_lld_encrypt_AES_GCM(cryp, key_id, auth_size, auth_in,
                                 text_size, text_in, text_out, iv,
                                 tag_size, tag_out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_encrypt_AES_GCM(cryp, key_id, auth_size, auth_in,
                                      text_size, text_in, text_out, iv,
                                      tag_size, tag_out);
#else
  (void)cryp;
  (void)key_id;
  (void)auth_size;
  (void)auth_in;
  (void)text_size;
  (void)text_in;
  (void)text_out;
  (void)iv;
  (void)tag_size;
  (void)tag_out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Decryption operation using AES-GCM.
 * @note    This is a stream cipher, there are no size restrictions.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] auth_size         size of the data buffer to be authenticated
 * @param[in] auth_in           buffer containing the data to be authenticated
 * @param[in] text_size         size of the text buffer
 * @param[in] text_in           buffer containing the input plaintext
 * @param[out] text_out         buffer for the output ciphertext
 * @param[in] iv                128 bits input vector
 * @param[in] tag_size          size of the authentication tag, this number
 *                              must be between 1 and 16
 * @param[in] tag_in            buffer for the received authentication tag
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_AUTH_FAILED  authentication failed
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryDecryptAES_GCM(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t auth_size,
                             const uint8_t *auth_in,
                             size_t text_size,
                             const uint8_t *text_in,
                             uint8_t *text_out,
                             const uint8_t *iv,
                             size_t tag_size,
                             const uint8_t *tag_in) {

  osalDbgCheck((cryp != NULL) && (auth_in != NULL) &&
               (text_size > (size_t)0) &&
               (text_in != NULL) && (text_out != NULL) && (iv != NULL) &&
               (tag_size >= (size_t)1) && (tag_size <= (size_t)16) &&
               (tag_in != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_AES_GCM == TRUE
  return cry_lld_decrypt_AES_GCM(cryp, key_id, auth_size, auth_in,
                                 text_size, text_in, text_out, iv,
                                 tag_size, tag_in);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_decrypt_AES_GCM(cryp, key_id, auth_size, auth_in,
                                      text_size, text_in, text_out, iv,
                                      tag_size, tag_in);
#else
  (void)cryp;
  (void)key_id;
  (void)auth_size;
  (void)auth_in;
  (void)text_size;
  (void)text_in;
  (void)text_out;
  (void)iv;
  (void)tag_size;
  (void)tag_in;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Initializes the DES transient key.
 * @note    It is the underlying implementation to decide which key sizes are
 *          allowable.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] size              key size in bytes
 * @param[in] keyp              pointer to the key data
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the algorithm is unsupported.
 * @retval CRY_ERR_INV_KEY_SIZE if the specified key size is invalid for
 *                              the specified algorithm.
 *
 * @api
 */
cryerror_t cryLoadDESTransientKey(CRYDriver *cryp,
                                  size_t size,
                                  const uint8_t *keyp) {

  osalDbgCheck((cryp != NULL) &&  (keyp != NULL));

#if CRY_LLD_SUPPORTS_DES == TRUE
  return cry_lld_des_loadkey(cryp, size, keyp);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_des_loadkey(cryp, size, keyp);
#else
  (void)cryp;
  (void)size;
  (void)keyp;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Encryption of a single block using (T)DES.
 * @note    The implementation of this function must guarantee that it can
 *          be called from any context.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] in                buffer containing the input plaintext
 * @param[out] out              buffer for the output ciphertext
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @special
 */
cryerror_t cryEncryptDES(CRYDriver *cryp,
                         crykey_t key_id,
                         const uint8_t *in,
                         uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_DES == TRUE
  return cry_lld_encrypt_DES(cryp, key_id, in, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_encrypt_DES(cryp, key_id, in, out);
#else
  (void)cryp;
  (void)key_id;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Decryption of a single block using (T)DES.
 * @note    The implementation of this function must guarantee that it can
 *          be called from any context.
 *
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] in                buffer containing the input ciphertext
 * @param[out] out              buffer for the output plaintext
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @special
 */
cryerror_t cryDecryptDES(CRYDriver *cryp,
                         crykey_t key_id,
                         const uint8_t *in,
                         uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_DES == TRUE
  return cry_lld_decrypt_DES(cryp, key_id, in, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_decrypt_DES(cryp, key_id, in, out);
#else
  (void)cryp;
  (void)key_id;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Encryption operation using (T)DES-ECB.
 * @note    The function operates on data buffers whose length is a multiple
 *          of an DES block, this means that padding must be done by the
 *          caller.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers, this number must be a
 *                              multiple of 8
 * @param[in] in                buffer containing the input plaintext
 * @param[out] out              buffer for the output ciphertext
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryEncryptDES_ECB(CRYDriver *cryp,
                              crykey_t key_id,
                              size_t size,
                              const uint8_t *in,
                              uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               ((size & (size_t)7) == (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_DES_ECB == TRUE
  return cry_lld_encrypt_DES_ECB(cryp, key_id, size, in, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_encrypt_DES_ECB(cryp, key_id, size, in, out);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Decryption operation using (T)DES-ECB.
 * @note    The function operates on data buffers whose length is a multiple
 *          of an DES block, this means that padding must be done by the
 *          caller.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers, this number must be a
 *                              multiple of 8
 * @param[in] in                buffer containing the input ciphertext
 * @param[out] out              buffer for the output plaintext
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryDecryptDES_ECB(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t size,
                             const uint8_t *in,
                             uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               ((size & (size_t)7) == (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_DES_ECB == TRUE
  return cry_lld_decrypt_DES_ECB(cryp, key_id, size, in, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_decrypt_DES_ECB(cryp, key_id, size, in, out);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Encryption operation using (T)DES-CBC.
 * @note    The function operates on data buffers whose length is a multiple
 *          of an DES block, this means that padding must be done by the
 *          caller.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers, this number must be a
 *                              multiple of 8
 * @param[in] in                buffer containing the input plaintext
 * @param[out] out              buffer for the output ciphertext
 * @param[in] iv                64 bits input vector
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryEncryptDES_CBC(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t size,
                             const uint8_t *in,
                             uint8_t *out,
                             const uint8_t *iv) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               (iv != NULL) && ((size & (size_t)7) == (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_DES_CBC == TRUE
  return cry_lld_encrypt_DES_CBC(cryp, key_id, size, in, out, iv);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_encrypt_DES_CBC(cryp, key_id, size, in, out, iv);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Decryption operation using (T)DES-CBC.
 * @note    The function operates on data buffers whose length is a multiple
 *          of an DES block, this means that padding must be done by the
 *          caller.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] key_id            the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in] size              size of both buffers, this number must be a
 *                              multiple of 8
 * @param[in] in                buffer containing the input ciphertext
 * @param[out] out              buffer for the output plaintext
 * @param[in] iv                64 bits input vector
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryDecryptDES_CBC(CRYDriver *cryp,
                             crykey_t key_id,
                             size_t size,
                             const uint8_t *in,
                             uint8_t *out,
                             const uint8_t *iv) {

  osalDbgCheck((cryp != NULL) && (in != NULL) && (out != NULL) &&
               (iv != NULL) && ((size & (size_t)7) == (size_t)0));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_DES_CBC == TRUE
  return cry_lld_decrypt_DES_CBC(cryp, key_id, size, in, out, iv);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_decrypt_DES_CBC(cryp, key_id, size, in, out, iv);
#else
  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash initialization using SHA1.
 * @note    Use of this algorithm is not recommended because proven weak.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[out] sha1ctxp         pointer to a SHA1 context to be initialized
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t crySHA1Init(CRYDriver *cryp, SHA1Context *sha1ctxp) {

  osalDbgCheck((cryp != NULL) && (sha1ctxp != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_SHA1 == TRUE
  return cry_lld_SHA1_init(cryp, sha1ctxp);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_SHA1_init(cryp, sha1ctxp);
#else
  (void)cryp;
  (void)sha1ctxp;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash update using SHA1.
 * @note    Use of this algorithm is not recommended because proven weak.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] sha1ctxp          pointer to a SHA1 context
 * @param[in] size              size of input buffer
 * @param[in] in                buffer containing the input text
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t crySHA1Update(CRYDriver *cryp, SHA1Context *sha1ctxp,
                         size_t size, const uint8_t *in) {

  osalDbgCheck((cryp != NULL) && (sha1ctxp != NULL) && (in != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_SHA1 == TRUE
  return cry_lld_SHA1_update(cryp, sha1ctxp, size, in);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_SHA1_update(cryp, sha1ctxp, size, in);
#else
  (void)cryp;
  (void)sha1ctxp;
  (void)size;
  (void)in;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash finalization using SHA1.
 * @note    Use of this algorithm is not recommended because proven weak.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] sha1ctxp          pointer to a SHA1 context
 * @param[out] out              160 bits output buffer
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t crySHA1Final(CRYDriver *cryp, SHA1Context *sha1ctxp, uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (sha1ctxp != NULL) && (out != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_SHA1 == TRUE
  return cry_lld_SHA1_final(cryp, sha1ctxp, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_SHA1_final(cryp, sha1ctxp, out);
#else
  (void)cryp;
  (void)sha1ctxp;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash initialization using SHA256.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[out] sha256ctxp       pointer to a SHA256 context to be initialized
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t crySHA256Init(CRYDriver *cryp, SHA256Context *sha256ctxp) {

  osalDbgCheck((cryp != NULL) && (sha256ctxp != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_SHA256 == TRUE
  return cry_lld_SHA256_init(cryp, sha256ctxp);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_SHA256_init(cryp, sha256ctxp);
#else
  (void)cryp;
  (void)sha256ctxp;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash update using SHA256.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] sha256ctxp        pointer to a SHA256 context
 * @param[in] size              size of input buffer
 * @param[in] in                buffer containing the input text
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t crySHA256Update(CRYDriver *cryp, SHA256Context *sha256ctxp,
                           size_t size, const uint8_t *in) {

  osalDbgCheck((cryp != NULL) && (sha256ctxp != NULL) && (in != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_SHA256 == TRUE
  return cry_lld_SHA256_update(cryp, sha256ctxp, size, in);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_SHA256_update(cryp, sha256ctxp, size, in);
#else
  (void)cryp;
  (void)sha256ctxp;
  (void)size;
  (void)in;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash finalization using SHA256.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] sha256ctxp        pointer to a SHA256 context
 * @param[out] out              256 bits output buffer
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t crySHA256Final(CRYDriver *cryp, SHA256Context *sha256ctxp,
                          uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (sha256ctxp != NULL) && (out != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_SHA256 == TRUE
  return cry_lld_SHA256_final(cryp, sha256ctxp, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_SHA256_final(cryp, sha256ctxp, out);
#else
  (void)cryp;
  (void)sha256ctxp;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash initialization using SHA512.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[out] sha512ctxp       pointer to a SHA512 context to be initialized
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t crySHA512Init(CRYDriver *cryp, SHA512Context *sha512ctxp) {

  osalDbgCheck((cryp != NULL) && (sha512ctxp != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_SHA512 == TRUE
  return cry_lld_SHA512_init(cryp, sha512ctxp);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_SHA512_init(cryp, sha512ctxp);
#else
  (void)cryp;
  (void)sha512ctxp;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash update using SHA512.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] sha512ctxp        pointer to a SHA512 context
 * @param[in] size              size of input buffer
 * @param[in] in                buffer containing the input text
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t crySHA512Update(CRYDriver *cryp, SHA512Context *sha512ctxp,
                           size_t size, const uint8_t *in) {

  osalDbgCheck((cryp != NULL) && (sha512ctxp != NULL) && (in != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_SHA512 == TRUE
  return cry_lld_SHA512_update(cryp, sha512ctxp, size, in);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_SHA512_update(cryp, sha512ctxp, size, in);
#else
  (void)cryp;
  (void)sha512ctxp;
  (void)size;
  (void)in;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash finalization using SHA512.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] sha512ctxp        pointer to a SHA512 context
 * @param[out] out              512 bits output buffer
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t crySHA512Final(CRYDriver *cryp, SHA512Context *sha512ctxp,
                          uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (sha512ctxp != NULL) && (out != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_SHA512 == TRUE
  return cry_lld_SHA512_final(cryp, sha512ctxp, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_SHA512_final(cryp, sha512ctxp, out);
#else
  (void)cryp;
  (void)sha512ctxp;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Initializes the HMAC transient key.
 * @note    It is the underlying implementation to decide which key sizes are
 *          allowable.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] size              key size in bytes
 * @param[in] keyp              pointer to the key data
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the algorithm is unsupported.
 * @retval CRY_ERR_INV_KEY_SIZE if the specified key size is invalid for
 *                              the specified algorithm.
 *
 * @api
 */
cryerror_t cryLoadHMACTransientKey(CRYDriver *cryp,
                                   size_t size,
                                   const uint8_t *keyp) {

  osalDbgCheck((cryp != NULL) &&  (keyp != NULL));

#if (CRY_LLD_SUPPORTS_HMAC_SHA256 == TRUE) ||                               \
    (CRY_LLD_SUPPORTS_HMAC_SHA512 == TRUE)
  return cry_lld_hmac_loadkey(cryp, size, keyp);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_hmac_loadkey(cryp, size, keyp);
#else
  (void)cryp;
  (void)size;
  (void)keyp;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash initialization using HMAC_SHA256.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[out] hmacsha256ctxp   pointer to a HMAC_SHA256 context to be
 *                              initialized
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryHMACSHA256Init(CRYDriver *cryp,
                             HMACSHA256Context *hmacsha256ctxp) {

  osalDbgCheck((cryp != NULL) && (hmacsha256ctxp != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_HMAC_SHA256 == TRUE
  return cry_lld_HMACSHA256_init(cryp, hmacsha256ctxp);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_HMACSHA256_init(cryp, hmacsha256ctxp);
#else
  (void)cryp;
  (void)hmacsha256ctxp;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash update using HMAC.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] hmacsha256ctxp    pointer to a HMAC_SHA256 context
 * @param[in] size              size of input buffer
 * @param[in] in                buffer containing the input text
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryHMACSHA256Update(CRYDriver *cryp,
                               HMACSHA256Context *hmacsha256ctxp,
                               size_t size,
                               const uint8_t *in) {

  osalDbgCheck((cryp != NULL) && (hmacsha256ctxp != NULL) && (in != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_HMAC_SHA256 == TRUE
  return cry_lld_HMACSHA256_update(cryp, hmacsha256ctxp, size, in);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_HMACSHA256_update(cryp, hmacsha256ctxp, size, in);
#else
  (void)cryp;
  (void)hmacsha256ctxp;
  (void)size;
  (void)in;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash finalization using HMAC.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] hmacsha256ctxp    pointer to a HMAC_SHA256 context
 * @param[out] out              256 bits output buffer
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryHMACSHA256Final(CRYDriver *cryp,
                              HMACSHA256Context *hmacsha256ctxp,
                              uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (hmacsha256ctxp != NULL) && (out != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_HMAC_SHA256 == TRUE
  return cry_lld_HMACSHA256_final(cryp, hmacsha256ctxp, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_HMACSHA256_final(cryp, hmacsha256ctxp, out);
#else
  (void)cryp;
  (void)hmacsha256ctxp;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash initialization using HMAC_SHA512.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[out] hmacsha512ctxp   pointer to a HMAC_SHA512 context to be
 *                              initialized
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryHMACSHA512Init(CRYDriver *cryp,
                             HMACSHA512Context *hmacsha512ctxp) {

  osalDbgCheck((cryp != NULL) && (hmacsha512ctxp != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_HMAC_SHA512 == TRUE
  return cry_lld_HMACSHA512_init(cryp, hmacsha512ctxp);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_HMACSHA512_init(cryp, hmacsha512ctxp);
#else
  (void)cryp;
  (void)hmacsha512ctxp;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash update using HMAC.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] hmacsha512ctxp    pointer to a HMAC_SHA512 context
 * @param[in] size              size of input buffer
 * @param[in] in                buffer containing the input text
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryHMACSHA512Update(CRYDriver *cryp,
                               HMACSHA512Context *hmacsha512ctxp,
                               size_t size,
                               const uint8_t *in) {

  osalDbgCheck((cryp != NULL) && (hmacsha512ctxp != NULL) && (in != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_HMAC_SHA512 == TRUE
  return cry_lld_HMACSHA512_update(cryp, hmacsha512ctxp, size, in);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_HMACSHA512_update(cryp, hmacsha512ctxp, size, in);
#else
  (void)cryp;
  (void)hmacsha512ctxp;
  (void)size;
  (void)in;

  return CRY_ERR_INV_ALGO;
#endif
}

/**
 * @brief   Hash finalization using HMAC.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] hmacsha512ctxp    pointer to a HMAC_SHA512 context
 * @param[out] out              512 bits output buffer
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @api
 */
cryerror_t cryHMACSHA512Final(CRYDriver *cryp,
                              HMACSHA512Context *hmacsha512ctxp,
                              uint8_t *out) {

  osalDbgCheck((cryp != NULL) && (hmacsha512ctxp != NULL) && (out != NULL));

  osalDbgAssert(cryp->state == CRY_READY, "not ready");

#if CRY_LLD_SUPPORTS_HMAC_SHA512 == TRUE
  return cry_lld_HMACSHA512_final(cryp, hmacsha512ctxp, out);
#elif HAL_CRY_USE_FALLBACK == TRUE
  return cry_fallback_HMACSHA512_final(cryp, hmacsha512ctxp, out);
#else
  (void)cryp;
  (void)hmacsha512ctxp;
  (void)out;

  return CRY_ERR_INV_ALGO;
#endif
}

#endif /* HAL_USE_CRY == TRUE */

/** @} */
