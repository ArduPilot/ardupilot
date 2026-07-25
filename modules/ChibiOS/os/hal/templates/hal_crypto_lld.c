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
 * @file    hal_crypto_lld.c
 * @brief   PLATFORM cryptographic subsystem low level driver source.
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

/** @brief CRY1 driver identifier.*/
#if PLATFORM_CRY_USE_CRY1 || defined(__DOXYGEN__)
CRYDriver CRYD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level crypto driver initialization.
 *
 * @notapi
 */
void cry_lld_init(void) {

}

/**
 * @brief   Configures and activates the crypto peripheral.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 *
 * @notapi
 */
void cry_lld_start(CRYDriver *cryp) {

  if (cryp->state == CRY_STOP) {

  }
}

/**
 * @brief   Deactivates the crypto peripheral.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 *
 * @notapi
 */
void cry_lld_stop(CRYDriver *cryp) {

  if (cryp->state == CRY_READY) {

  }
}

#if (CRY_LLD_SUPPORTS_AES == TRUE) || defined(__DOXYGEN__)
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
 * @notapi
 */
cryerror_t cry_lld_aes_loadkey(CRYDriver *cryp,
                               size_t size,
                               const uint8_t *keyp) {

  (void)cryp;
  (void)size;
  (void)keyp;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_encrypt_AES(CRYDriver *cryp,
                               crykey_t key_id,
                               const uint8_t *in,
                               uint8_t *out) {

  (void)cryp;
  (void)key_id;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_decrypt_AES(CRYDriver *cryp,
                               crykey_t key_id,
                               const uint8_t *in,
                               uint8_t *out) {

  (void)cryp;
  (void)key_id;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_AES_ECB == TRUE) || defined(__DOXYGEN__)
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
 * @notapi
 */
cryerror_t cry_lld_encrypt_AES_ECB(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_decrypt_AES_ECB(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_AES_CBC == TRUE) || defined(__DOXYGEN__)
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
 * @param[in] iv                128 bits initial vector
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
 * @notapi
 */
cryerror_t cry_lld_encrypt_AES_CBC(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out,
                                   const uint8_t *iv) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
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
 * @param[in] in                buffer containing the input plaintext
 * @param[out] out              buffer for the output ciphertext
 * @param[in] iv                128 bits initial vector
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
 * @notapi
 */
cryerror_t cry_lld_decrypt_AES_CBC(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out,
                                   const uint8_t *iv) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_AES_CFB == TRUE) || defined(__DOXYGEN__)
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
 * @param[in] iv                128 bits initial vector
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
 * @notapi
 */
cryerror_t cry_lld_encrypt_AES_CFB(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out,
                                   const uint8_t *iv) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
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
 * @param[in] in                buffer containing the input plaintext
 * @param[out] out              buffer for the output ciphertext
 * @param[in] iv                128 bits initial vector
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
 * @notapi
 */
cryerror_t cry_lld_decrypt_AES_CFB(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out,
                                   const uint8_t *iv) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_AES_CTR == TRUE) || defined(__DOXYGEN__)
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
 * @param[in] iv                128 bits initial vector + counter, it contains
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
 * @notapi
 */
cryerror_t cry_lld_encrypt_AES_CTR(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out,
                                   const uint8_t *iv) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
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
 * @param[in] iv                128 bits initial vector + counter, it contains
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
 * @notapi
 */
cryerror_t cry_lld_decrypt_AES_CTR(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out,
                                   const uint8_t *iv) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_AES_GCM == TRUE) || defined(__DOXYGEN__)
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
 * @notapi
 */
cryerror_t cry_lld_encrypt_AES_GCM(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t auth_size,
                                   const uint8_t *auth_in,
                                   size_t text_size,
                                   const uint8_t *text_in,
                                   uint8_t *text_out,
                                   const uint8_t *iv,
                                   size_t tag_size,
                                   uint8_t *tag_out) {

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
 * @notapi
 */
cryerror_t cry_lld_decrypt_AES_GCM(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t auth_size,
                                   const uint8_t *auth_in,
                                   size_t text_size,
                                   const uint8_t *text_in,
                                   uint8_t *text_out,
                                   const uint8_t *iv,
                                   size_t tag_size,
                                   const uint8_t *tag_in) {

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
}
#endif

#if (CRY_LLD_SUPPORTS_DES == TRUE) || defined(__DOXYGEN__)
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
 * @notapi
 */
cryerror_t cry_lld_des_loadkey(CRYDriver *cryp,
                               size_t size,
                               const uint8_t *keyp) {

  (void)cryp;
  (void)size;
  (void)keyp;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_encrypt_DES(CRYDriver *cryp,
                               crykey_t key_id,
                               const uint8_t *in,
                               uint8_t *out) {

  (void)cryp;
  (void)key_id;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_decrypt_DES(CRYDriver *cryp,
                               crykey_t key_id,
                               const uint8_t *in,
                               uint8_t *out) {

  (void)cryp;
  (void)key_id;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_DES_ECB == TRUE) || defined(__DOXYGEN__)
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
 * @param[in] size              size of the plaintext buffer, this number must
 *                              be a multiple of 8
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
 * @notapi
 */
cryerror_t cry_lld_encrypt_DES_ECB(CRYDriver *cryp,
                                  crykey_t key_id,
                                  size_t size,
                                  const uint8_t *in,
                                  uint8_t *out) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
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
 * @param[in] size              size of the plaintext buffer, this number must
 *                              be a multiple of 8
 * @param[in] in                buffer containing the input ciphertext
 * @param[out] out              buffer for the output plaintext
 * @return              T       he operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @notapi
 */
cryerror_t cry_lld_decrypt_DES_ECB(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_DES_CBC == TRUE) || defined(__DOXYGEN__)
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
 * @param[in] size              size of the plaintext buffer, this number must
 *                              be a multiple of 8
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
 * @notapi
 */
cryerror_t cry_lld_encrypt_DES_CBC(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out,
                                   const uint8_t *iv) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
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
 * @param[in] size              size of the plaintext buffer, this number must
 *                              be a multiple of 8
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
 * @notapi
 */
cryerror_t cry_lld_decrypt_DES_CBC(CRYDriver *cryp,
                                   crykey_t key_id,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out,
                                   const uint8_t *iv) {

  (void)cryp;
  (void)key_id;
  (void)size;
  (void)in;
  (void)out;
  (void)iv;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_SHA1 == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Hash initialization using SHA1.
 * @note    Use of this algorithm is not recommended because proven weak.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[out] sha1ctxp         pointer to a SHA1 context to be initialized
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @notapi
 */
cryerror_t cry_lld_SHA1_init(CRYDriver *cryp, SHA1Context *sha1ctxp) {

  (void)cryp;
  (void)sha1ctxp;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_SHA1_update(CRYDriver *cryp, SHA1Context *sha1ctxp,
                               size_t size, const uint8_t *in) {

  (void)cryp;
  (void)sha1ctxp;
  (void)size;
  (void)in;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_SHA1_final(CRYDriver *cryp, SHA1Context *sha1ctxp,
                              uint8_t *out) {

  (void)cryp;
  (void)sha1ctxp;
  (void)out;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_SHA256 == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Hash initialization using SHA256.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[out] sha256ctxp       pointer to a SHA256 context to be initialized
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @notapi
 */
cryerror_t cry_lld_SHA256_init(CRYDriver *cryp, SHA256Context *sha256ctxp) {

  (void)cryp;
  (void)sha256ctxp;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_SHA256_update(CRYDriver *cryp, SHA256Context *sha256ctxp,
                                 size_t size, const uint8_t *in) {

  (void)cryp;
  (void)sha256ctxp;
  (void)size;
  (void)in;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_SHA256_final(CRYDriver *cryp, SHA256Context *sha256ctxp,
                                uint8_t *out) {

  (void)cryp;
  (void)sha256ctxp;
  (void)out;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_SHA512 == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Hash initialization using SHA512.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[out] sha512ctxp       pointer to a SHA512 context to be initialized
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 *
 * @notapi
 */
cryerror_t cry_lld_SHA512_init(CRYDriver *cryp, SHA512Context *sha512ctxp) {

  (void)cryp;
  (void)sha512ctxp;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_SHA512_update(CRYDriver *cryp, SHA512Context *sha512ctxp,
                                 size_t size, const uint8_t *in) {

  (void)cryp;
  (void)sha512ctxp;
  (void)size;
  (void)in;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_SHA512_final(CRYDriver *cryp, SHA512Context *sha512ctxp,
                                uint8_t *out) {

  (void)cryp;
  (void)sha512ctxp;
  (void)out;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_HMAC_SHA256 == TRUE) || defined(__DOXYGEN__)
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
 * @notapi
 */
cryerror_t cry_lld_hmac_loadkey(CRYDriver *cryp,
                                size_t size,
                                const uint8_t *keyp) {

  (void)cryp;
  (void)size;
  (void)keyp;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_HMACSHA256_init(CRYDriver *cryp,
                                   HMACSHA256Context *hmacsha256ctxp) {

  (void)cryp;
  (void)hmacsha256ctxp;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_HMACSHA256_update(CRYDriver *cryp,
                                     HMACSHA256Context *hmacsha256ctxp,
                                     size_t size,
                                     const uint8_t *in) {

  (void)cryp;
  (void)hmacsha256ctxp;
  (void)size;
  (void)in;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_HMACSHA256_final(CRYDriver *cryp,
                                    HMACSHA256Context *hmacsha256ctxp,
                                    uint8_t *out) {

  (void)cryp;
  (void)hmacsha256ctxp;
  (void)out;

  return CRY_ERR_INV_ALGO;
}
#endif

#if (CRY_LLD_SUPPORTS_HMAC_SHA512 == TRUE) || defined(__DOXYGEN__)
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
 * @notapi
 */
cryerror_t cry_lld_HMACSHA512_init(CRYDriver *cryp,
                                   HMACSHA512Context *hmacsha512ctxp) {

  (void)cryp;
  (void)hmacsha512ctxp;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_HMACSHA512_update(CRYDriver *cryp,
                                     HMACSHA512Context *hmacsha512ctxp,
                                     size_t size,
                                     const uint8_t *in) {

  (void)cryp;
  (void)hmacsha512ctxp;
  (void)size;
  (void)in;

  return CRY_ERR_INV_ALGO;
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
 * @notapi
 */
cryerror_t cry_lld_HMACSHA512_final(CRYDriver *cryp,
                                    HMACSHA512Context *hmacsha512ctxp,
                                    uint8_t *out) {

  (void)cryp;
  (void)hmacsha512ctxp;
  (void)out;

  return CRY_ERR_INV_ALGO;
}
#endif

#endif /* HAL_USE_CRY == TRUE */

/** @} */
