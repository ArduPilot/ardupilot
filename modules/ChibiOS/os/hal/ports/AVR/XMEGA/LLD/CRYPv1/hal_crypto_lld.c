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
 * @file    hal_crypto_lld.c
 * @brief   AVR cryptographic subsystem low level driver source.
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
#if AVR_CRY_USE_CRY1 || defined(__DOXYGEN__)
CRYDriver CRYD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Enabable the AES auto start feature.
 */
static void aes_lld_enable_auto_start(void) {
  AES.CTRL |= (1 << AES_AUTO_bp);
}

/**
 * @brief   Disable the AES auto start feature.
 */
static void aes_lld_disable_auto_start(void) {
  AES.CTRL &= ~(1 << AES_AUTO_bp);
}

/**
 * @brief   Software reset of the AES crypto module.
 */
void aes_lld_reset(void) {
  AES.CTRL |= (1 << AES_RESET_bp);
}

/**
 * @brief   Set the AES module to Encrypt data.
 */
static void aes_lld_set_mode_encrypt(void) {
  AES.CTRL &= ~(1 << AES_DECRYPT_bp);
}

/**
 * @brief   Set the AES module to decrypt data.
 */
static void aes_lld_set_mode_decrypt(void) {
  AES.CTRL |= (1 << AES_DECRYPT_bp);
}

/**
 * @brief   Enable the XOR feature in the AES module.
 */
static void aes_lld_enable_xor(void) {
  AES.CTRL |= (1 << AES_XOR_bp);
}

/**
 * @brief   Enable the XOR feature in the AES module.
 */
static void aes_lld_disable_xor(void) {
  AES.CTRL &= ~(1 << AES_XOR_bp);
}

/**
 * @brief   Start the Encryption/Decryption procedure.
 */
static void aes_lld_start(void) {
  AES.CTRL |= (1 << AES_START_bp);
}

/**
 * @brief   Stop the Encryption/Decryption procedure.
 */
static void aes_lld_stop(void) {
  AES.CTRL &= ~(1 << AES_START_bp);
}

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

#if AVR_CRY_USE_CRY1 || defined(__DOXYGEN__)
  aes_lld_reset();  /* Reset the AES module.  */
  aes_lld_stop();   /* Stop the AES module.   */
  CRYD1.config = NULL;
  CRYD1.state = CRY_STOP;
#endif
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
    if (cryp->config->xorf) {
      aes_lld_enable_xor();
    }
    else {
      aes_lld_disable_xor();
    }

    if (cryp->config->autof) {
      aes_lld_enable_auto_start();
    }
    else {
      aes_lld_disable_auto_start();
    }
  }
  aes_lld_reset();
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
    aes_lld_stop();   /* Stop the AES module. */
  }
}

#if (CRY_LLD_SUPPORTS_AES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Initializes the AES transient key.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] size              key size in bytes
 * @param[in] keyp              pointer to the key data
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_KEY_SIZE if the specified key size is invalid for the specified algorithm.
 *
 * @notapi
 */
cryerror_t cry_lld_aes_loadkey(CRYDriver        *cryp,
                           size_t           size,
                           const uint8_t    *keyp) {

  uint8_t i;

  (void)cryp;

  if (size != AES_BLOCK_SIZE) {
    return CRY_ERR_INV_KEY_SIZE; /* Invalid size error code. */
  }

  /* Load the Key into the AES key memory. */
  for (i = 0; i < AES_BLOCK_SIZE; i++) {
    AES.KEY = keyp[i];
  }

  return CRY_NOERROR;
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
 * @param[in]   src             source buffer containing the input plaintext
 * @param[out]  dest            destination buffer for the output cyphertext
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 *
 * @notapi
 */
cryerror_t cry_lld_encrypt_AES(CRYDriver      *cryp,
                               crykey_t       key_id,
                               const uint8_t  *src,
                               uint8_t        *dest) {
  uint8_t i;

  (void)cryp;
  (void)key_id;

  /* Load the Data into the AES state memory. */
  for (i = 0; i < AES_BLOCK_SIZE; i++) {
    AES.STATE = src[i];
  }

  /* Set the AES encryption mode. */
  aes_lld_set_mode_encrypt();

  /* Start the AES. */
  aes_lld_start();

  /* Wait the Encryption to finish or an error to occurs. */
  do {
  }
  while ((AES.STATUS & (AES_SRIF_bm|AES_ERROR_bm)) == 0);

  /* Check error. */
  if ((AES.STATUS & AES_ERROR_bm) == 0) {
    /* Store the result of the encryption. */
    for (i = 0; i < AES_BLOCK_SIZE; i++) {
      dest[i] = AES.STATE;
    }
  }
  else {
    return CRY_ERR_OP_FAILURE;
  }

  return CRY_NOERROR;
}

/**
 * @brief   Decryption of a single block using AES.
 * @note    The implementation of this function must guarantee that it can
 *          be called from any context.
 *
 * @param[in]   cryp            pointer to the @p CRYDriver object
 * @param[in]   key_id          the key to be used for the operation, zero is
 *                              the transient key, other values are keys stored
 *                              in an unspecified way
 * @param[in]   src             source buffer containing the input cyphertext
 * @param[out]  dest            destination buffer for the output plaintext
 * @return                      the operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_INV_ALGO     if the operation is unsupported on this
 *                              device instance.
 * @retval CRY_ERR_INV_KEY_TYPE the selected key is invalid for this operation.
 * @retval CRY_ERR_INV_KEY_ID   if the specified key identifier is invalid
 *                              or refers to an empty key slot.
 *
 * @notapi
 */
cryerror_t cry_lld_decrypt_AES(CRYDriver      *cryp,
                               crykey_t       key_id,
                               const uint8_t  *src,
                               uint8_t        *dest) {

  uint8_t i;

  (void)cryp;
  (void)key_id;

  /* Load data into AES state memory. */
  for (i = 0; i < AES_BLOCK_SIZE; i++) {
    AES.STATE =  src[i];
  }

  /* Set the AES decryption mode. */
  aes_lld_set_mode_decrypt();

  /* Start the AES. */
  aes_lld_start();

  /* Wait the Encryption to finish or an error to occurs. */
  do {
  }
  while ((AES.STATUS & (AES_SRIF_bm|AES_ERROR_bm)) == 0);

  /* Check if not error. */
  if ((AES.STATUS & AES_ERROR_bm) == 0) {
    /* Store the result. */
    for (i = 0; i < AES_BLOCK_SIZE; i++) {
      dest[i] = AES.STATE;
    }
  }
  else {
    return CRY_ERR_OP_FAILURE;
  }

  return CRY_NOERROR;
}
#endif

#endif /* HAL_USE_CRY == TRUE */

/** @} */
