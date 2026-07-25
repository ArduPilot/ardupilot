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
 * @file    CRYPv1/hal_crypto_lld.c
 * @brief   STM32 cryptographic subsystem low level driver source.
 *
 * @addtogroup CRYPTO
 * @{
 */

#include <string.h>

#include "hal.h"

#if (HAL_USE_CRY == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define CRYP1_IN_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_CRY_CRYP1_IN_DMA_STREAM,                       \
                       STM32_CRYP1_IN_DMA_CHN)

#define CRYP1_OUT_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_CRY_CRYP1_OUT_DMA_STREAM,                      \
                       STM32_CRYP1_OUT_DMA_CHN)

#define HASH1_DMA_CHANNEL                                                   \
  STM32_DMA_GETCHANNEL(STM32_CRY_HASH1_DMA_STREAM,                          \
                       STM32_HASH1_DMA_CHN)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief CRY1 driver identifier.*/
#if (STM32_CRY_ENABLED1 == TRUE) || defined(__DOXYGEN__)
CRYDriver CRYD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if (STM32_CRY_USE_CRYP1 == TRUE) || defined (__DOXYGEN__)
/**
 * @brief   Setting AES key for encryption.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] algomode          algorithm mode field of CR register
 */
static inline void cryp_set_key_encrypt(CRYDriver *cryp, uint32_t algomode) {
  uint32_t cr;

  /* Loading key data.*/
  CRYP->K0LR = cryp->cryp_k[0];
  CRYP->K0RR = cryp->cryp_k[1];
  CRYP->K1LR = cryp->cryp_k[2];
  CRYP->K1RR = cryp->cryp_k[3];
  CRYP->K2LR = cryp->cryp_k[4];
  CRYP->K2RR = cryp->cryp_k[5];
  CRYP->K3LR = cryp->cryp_k[6];
  CRYP->K3RR = cryp->cryp_k[7];

  /* Setting up then starting operation.*/
  cr  = CRYP->CR;
  cr &= ~(CRYP_CR_KEYSIZE_Msk | CRYP_CR_ALGOMODE_Msk | CRYP_CR_ALGODIR_Msk);
  cr |= cryp->cryp_ksize | algomode | CRYP_CR_CRYPEN;
  CRYP->CR = cr;

  cryp->cryp_ktype = cryp_key_aes_encrypt;
}

/**
 * @brief   Setting AES key for decryption.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] algomode          algorithm field of CR register
 */
static inline void cryp_set_key_decrypt(CRYDriver *cryp, uint32_t algomode) {
  uint32_t cr;

  /* Loading key data then doing transformation for decrypt.*/
  cryp_set_key_encrypt(cryp, CRYP_CR_ALGOMODE_AES_KEY);
  while ((CRYP->CR & CRYP_CR_CRYPEN) != 0U) {
  }

  /* Setting up then starting operation.*/
  cr  = CRYP->CR;
  cr &= ~(CRYP_CR_KEYSIZE_Msk | CRYP_CR_ALGOMODE_Msk | CRYP_CR_ALGODIR_Msk);
  cr |= cryp->cryp_ksize | algomode | CRYP_CR_ALGODIR | CRYP_CR_CRYPEN;
  CRYP->CR = cr;

  cryp->cryp_ktype = cryp_key_aes_decrypt;
}

/**
 * @brief   Setting IV.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] iv                128 bits initial vector
 */
static inline void cryp_set_iv(CRYDriver *cryp, const uint8_t *iv) {

  (void)cryp;

  CRYP->IV0LR = __REV(__UNALIGNED_UINT32_READ(&iv[0]));
  CRYP->IV0RR = __REV(__UNALIGNED_UINT32_READ(&iv[4]));
  CRYP->IV1LR = __REV(__UNALIGNED_UINT32_READ(&iv[8]));
  CRYP->IV1RR = __REV(__UNALIGNED_UINT32_READ(&iv[12]));
}

/**
 * @brief   Performs a CRYP operation using DMA.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] size              size of both buffers, this number must be a
 *                              multiple of 16
 * @param[in] in                input buffer
 * @param[out] out              output buffer
 * @return                      The operation status.
 * @retval CRY_NOERROR          if the operation succeeded.
 * @retval CRY_ERR_OP_FAILURE   if the operation failed, implementation
 *                              dependent.
 */
static cryerror_t cryp_do_transfer(CRYDriver *cryp,
                                   size_t size,
                                   const uint8_t *in,
                                   uint8_t *out) {
  uint32_t szw;

  szw = (uint32_t)(size / sizeof (uint32_t));
#if STM32_CRY_CRYP_SIZE_THRESHOLD > 1
  if (size >= STM32_CRY_CRYP_SIZE_THRESHOLD)
#endif
#if STM32_CRY_CRYP_SIZE_THRESHOLD != 0
  {
    /* DMA limitation.*/
    osalDbgCheck(size < 0x10000U * 4U);

    osalSysLock();

    /* Preparing DMAs.*/
    dmaStreamSetTransactionSize(cryp->cryp_dma_in,  szw);
    dmaStreamSetTransactionSize(cryp->cryp_dma_out, szw);
    dmaStreamSetMemory0(cryp->cryp_dma_in,  in);
    dmaStreamSetMemory0(cryp->cryp_dma_out, out);
    dmaStreamEnable(cryp->cryp_dma_in);
    dmaStreamEnable(cryp->cryp_dma_out);

    (void) osalThreadSuspendS(&cryp->cryp_tr);

    osalSysUnlock();
  }
#endif
#if STM32_CRY_CRYP_SIZE_THRESHOLD > 1
  else
#endif
#if STM32_CRY_CRYP_SIZE_THRESHOLD != 1
  {
    uint32_t nr, nw;

    nr = 0U;
    nw = 0U;
    while (nw < szw) {

      if ((CRYP->SR & CRYP_SR_OFNE) != 0U) {
        __UNALIGNED_UINT32_WRITE(out, CRYP->DOUT);
        out += 4;
        nw++;
        continue;   /* Priority to output FIFO.*/
      }

      if ((nr < szw) && ((CRYP->SR & CRYP_SR_IFNF) != 0U)) {
        CRYP->DIN = __UNALIGNED_UINT32_READ(in);
        in += 4;
        nr++;
      }
    }
  }
#endif

  /* Disabling unit.*/
  CRYP->CR &= ~CRYP_CR_CRYPEN;

  return CRY_NOERROR;
}

/**
 * @brief   CRYP-IN DMA ISR.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] flags             pre-shifted content of the ISR register
 */
static void cry_lld_serve_cryp_in_interrupt(CRYDriver *cryp, uint32_t flags) {

  (void)cryp;

  /* DMA errors handling.*/
#if defined(STM32_CRY_CRYP_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0U) {
    STM32_CRY_CRYP_DMA_ERROR_HOOK(cryp);
  }
#endif
}

/**
 * @brief   CRYP-OUT DMA ISR.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] flags             pre-shifted content of the ISR register
 */
static void cry_lld_serve_cryp_out_interrupt(CRYDriver *cryp, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(STM32_CRY_CRYP_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0U) {
    STM32_CRY_CRYP_DMA_ERROR_HOOK(cryp);
  }
#endif

  /* End buffer interrupt.*/
  if ((flags & STM32_DMA_ISR_TCIF) != 0U) {

    /* Clearing flags of the other stream too.*/
    dmaStreamClearInterrupt(cryp->cryp_dma_in);

    /* Resuming waiting thread.*/
    osalSysLockFromISR();
    osalThreadResumeI(&cryp->cryp_tr, MSG_OK);
    osalSysUnlockFromISR();
  }
}
#endif

#if (STM32_CRY_USE_HASH1 == TRUE) || defined (__DOXYGEN__)
#if (STM32_CRY_HASH_SIZE_THRESHOLD != 0) || defined (__DOXYGEN__)
/**
 * @brief   HASH DMA ISR.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] flags             pre-shifted content of the ISR register
 */
static void cry_lld_serve_hash_interrupt(CRYDriver *cryp, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(STM32_HASH_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0U) {
    STM32_CRY_HASH_DMA_ERROR_HOOK(cryp);
  }
#endif

  /* End buffer interrupt.*/
  if ((flags & STM32_DMA_ISR_TCIF) != 0U) {

    /* Resuming waiting thread.*/
    osalSysLockFromISR();
    osalThreadResumeI(&cryp->hash_tr, MSG_OK);
    osalSysUnlockFromISR();
  }
}
#endif
#endif

/**
 * @brief   Pushes a series of words into the hash engine.
 *
 * @param[in] cryp              pointer to the @p CRYDriver object
 * @param[in] n                 the number of words to be pushed
 * @param[in] p                 pointer to the words buffer
 */
static void cry_lld_hash_push(CRYDriver *cryp, uint32_t n, const uint32_t *p) {

  (void)cryp; /* Not touched in some cases, needs this.*/

  /* Data is processed in 32kB blocks because DMA size limitations.*/
  while (n > 0U) {
    uint32_t chunk = n > 0x8000U ? 0x8000U : n;
    n -= chunk;

#if STM32_CRY_HASH_SIZE_THRESHOLD > 1
    if (chunk >= STM32_CRY_HASH_SIZE_THRESHOLD)
#endif
#if STM32_CRY_HASH_SIZE_THRESHOLD != 0
    {
      /* Setting up transfer.*/
      dmaStreamSetTransactionSize(cryp->hash_dma, chunk);
      dmaStreamSetPeripheral(cryp->hash_dma, p);
      p += chunk;

      osalSysLock();

      /* Enabling DMA channel then HASH engine.*/
      dmaStreamEnable(cryp->hash_dma);

      /* Waiting for DMA operation completion.*/
      osalThreadSuspendS(&cryp->hash_tr);

      osalSysUnlock();
    }
#endif
#if STM32_CRY_HASH_SIZE_THRESHOLD > 1
    else
#endif
#if STM32_CRY_HASH_SIZE_THRESHOLD != 1
    {
      /* Small chunk, just pushing data without touching DMA.*/
      do {
        HASH->DIN = *p++;
        chunk--;
      } while (chunk > 0U);
    }
#endif
  }

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

#if STM32_CRY_ENABLED1
  cryObjectInit(&CRYD1);

#if STM32_CRY_USE_CRYP1
#if STM32_CRY_CRYP_SIZE_THRESHOLD != 0
  CRYD1.cryp_tr      = NULL;
  CRYD1.cryp_dma_in  = NULL;
  CRYD1.cryp_dma_out = NULL;
#endif
#endif

#if STM32_CRY_USE_HASH1
#if STM32_CRY_HASH_SIZE_THRESHOLD != 0
  CRYD1.hash_tr      = NULL;
  CRYD1.hash_dma     = NULL;
#endif /* STM32_CRY_HASH_SIZE_THRESHOLD != 0 */
#endif /* STM32_CRY_USE_HASH1 */

#endif /* STM32_CRY_ENABLED1 */
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

#if STM32_CRY_ENABLED1
    if (&CRYD1 == cryp) {
#if STM32_CRY_USE_CRYP1 == TRUE
#if STM32_CRY_CRYP_SIZE_THRESHOLD != 0
      /* Allocating DMA channels.*/
      cryp->cryp_dma_in  = dmaStreamAllocI(STM32_CRY_CRYP1_IN_DMA_STREAM,
                                           STM32_CRY_CRYP1_IRQ_PRIORITY,
                                           (stm32_dmaisr_t)cry_lld_serve_cryp_in_interrupt,
                                           (void *)cryp);
      osalDbgAssert(cryp->cryp_dma_in != NULL, "unable to allocate stream");
      cryp->cryp_dma_out = dmaStreamAllocI(STM32_CRY_CRYP1_OUT_DMA_STREAM,
                                           STM32_CRY_CRYP1_IRQ_PRIORITY,
                                           (stm32_dmaisr_t)cry_lld_serve_cryp_out_interrupt,
                                           (void *)cryp);
      osalDbgAssert(cryp->cryp_dma_out != NULL, "unable to allocate stream");

      /* Preparing the DMA channels.*/
      dmaStreamSetMode(cryp->cryp_dma_in,
                       STM32_DMA_CR_CHSEL(CRYP1_IN_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_CRY_CRYP1_IN_DMA_PRIORITY) |
                       STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_M2P |
                       STM32_DMA_CR_MSIZE_WORD | STM32_DMA_CR_PSIZE_WORD |
                       STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE);
      dmaStreamSetMode(cryp->cryp_dma_out,
                       STM32_DMA_CR_CHSEL(CRYP1_OUT_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_CRY_CRYP1_OUT_DMA_PRIORITY) |
                       STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_P2M |
                       STM32_DMA_CR_MSIZE_WORD | STM32_DMA_CR_PSIZE_WORD |
                       STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE |
                       STM32_DMA_CR_TCIE);
      dmaStreamSetPeripheral(cryp->cryp_dma_in,  &CRYP->DIN);
      dmaStreamSetPeripheral(cryp->cryp_dma_out, &CRYP->DOUT);
      dmaStreamSetFIFO(cryp->cryp_dma_in,  STM32_DMA_FCR_DMDIS);
      dmaStreamSetFIFO(cryp->cryp_dma_out, STM32_DMA_FCR_DMDIS);
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(cryp->cryp_dma_in, STM32_DMAMUX1_CRYP_IN);
      dmaSetRequestSource(cryp->cryp_dma_out, STM32_DMAMUX1_CRYP_OUT);
#endif
#endif /* STM32_CRY_CRYP_SIZE_THRESHOLD != 0 */
      rccEnableCRYP(true);
#endif /* STM32_CRY_USE_CRYP1 == TRUE */

#if STM32_CRY_USE_HASH1 == TRUE
#if STM32_CRY_HASH_SIZE_THRESHOLD != 0
      cryp->hash_dma = dmaStreamAllocI(STM32_CRY_HASH1_DMA_STREAM,
                                       STM32_CRY_HASH1_IRQ_PRIORITY,
                                       (stm32_dmaisr_t)cry_lld_serve_hash_interrupt,
                                       (void *)cryp);
      osalDbgAssert(cryp->hash_dma != NULL, "unable to allocate stream");

      /* Preparing the DMA channel.*/
      dmaStreamSetMode(cryp->hash_dma,
                       STM32_DMA_CR_CHSEL(HASH1_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_CRY_HASH1_DMA_PRIORITY) |
                       STM32_DMA_CR_PINC | STM32_DMA_CR_DIR_M2M |
                       STM32_DMA_CR_MSIZE_WORD | STM32_DMA_CR_PSIZE_WORD |
                       STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE |
                       STM32_DMA_CR_TCIE);
      dmaStreamSetMemory0(cryp->hash_dma, &HASH->DIN);
      dmaStreamSetFIFO(cryp->hash_dma, STM32_DMA_FCR_DMDIS);
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(cryp->hash_dma, STM32_DMAMUX1_HASH_IN);
#endif
#endif /* STM32_CRY_HASH_SIZE_THRESHOLD != 0 */
      rccEnableHASH(true);
#endif /* STM32_CRY_USE_HASH1 == TRUE */
    }
#endif
  }

  /* Resetting trasient key data.*/
  cryp->cryp_ktype = cryp_key_none;
  cryp->cryp_ksize = 0U;
  cryp->cryp_k[0]  = 0U;
  cryp->cryp_k[1]  = 0U;
  cryp->cryp_k[2]  = 0U;
  cryp->cryp_k[3]  = 0U;
  cryp->cryp_k[4]  = 0U;
  cryp->cryp_k[5]  = 0U;
  cryp->cryp_k[6]  = 0U;
  cryp->cryp_k[7]  = 0U;

#if STM32_CRY_USE_CRYP1
  /* CRYP setup.*/
  CRYP->CR    = CRYP_CR_DATATYPE_1;
  CRYP->DMACR = CRYP_DMACR_DIEN | CRYP_DMACR_DOEN;
#endif

#if STM32_CRY_USE_HASH1
  /* HASH setup and enable.*/
#endif
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

    /* Resetting CRYP.*/
    CRYP->CR    = 0U;
    CRYP->DMACR = 0U;

#if STM32_CRY_ENABLED1
    if (&CRYD1 == cryp) {
#if STM32_CRY_USE_CRYP1
#if STM32_CRY_CRYP_SIZE_THRESHOLD != 0
      dmaStreamFreeI(cryp->cryp_dma_in);
      dmaStreamFreeI(cryp->cryp_dma_out);
      cryp->cryp_dma_in = NULL;
      cryp->cryp_dma_out = NULL;
#endif
      rccDisableCRYP();
#endif

#if STM32_CRY_USE_HASH1
#if STM32_CRY_HASH_SIZE_THRESHOLD != 0
      dmaStreamFreeI(cryp->hash_dma);
      cryp->hash_dma = NULL;
#endif
      rccDisableHASH();
#endif
    }
#endif
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

  /* Fetching key data.*/
  if (size == (size_t)32) {
    cryp->cryp_ksize = CRYP_CR_KEYSIZE_1;
    cryp->cryp_k[0] = __REV(__UNALIGNED_UINT32_READ(&keyp[0]));
    cryp->cryp_k[1] = __REV(__UNALIGNED_UINT32_READ(&keyp[4]));
    cryp->cryp_k[2] = __REV(__UNALIGNED_UINT32_READ(&keyp[8]));
    cryp->cryp_k[3] = __REV(__UNALIGNED_UINT32_READ(&keyp[12]));
    cryp->cryp_k[4] = __REV(__UNALIGNED_UINT32_READ(&keyp[16]));
    cryp->cryp_k[5] = __REV(__UNALIGNED_UINT32_READ(&keyp[20]));
    cryp->cryp_k[6] = __REV(__UNALIGNED_UINT32_READ(&keyp[24]));
    cryp->cryp_k[7] = __REV(__UNALIGNED_UINT32_READ(&keyp[28]));
  }
  else if (size == (size_t)24) {
    cryp->cryp_ksize = CRYP_CR_KEYSIZE_0;
    cryp->cryp_k[0] = 0U;
    cryp->cryp_k[1] = 0U;
    cryp->cryp_k[2] = __REV(__UNALIGNED_UINT32_READ(&keyp[8]));
    cryp->cryp_k[3] = __REV(__UNALIGNED_UINT32_READ(&keyp[12]));
    cryp->cryp_k[4] = __REV(__UNALIGNED_UINT32_READ(&keyp[16]));
    cryp->cryp_k[5] = __REV(__UNALIGNED_UINT32_READ(&keyp[20]));
    cryp->cryp_k[6] = __REV(__UNALIGNED_UINT32_READ(&keyp[24]));
    cryp->cryp_k[7] = __REV(__UNALIGNED_UINT32_READ(&keyp[28]));
  }
  else if (size == (size_t)16) {
    cryp->cryp_ksize = 0U;
    cryp->cryp_k[0] = 0U;
    cryp->cryp_k[1] = 0U;
    cryp->cryp_k[2] = 0U;
    cryp->cryp_k[3] = 0U;
    cryp->cryp_k[4] = __REV(__UNALIGNED_UINT32_READ(&keyp[16]));
    cryp->cryp_k[5] = __REV(__UNALIGNED_UINT32_READ(&keyp[20]));
    cryp->cryp_k[6] = __REV(__UNALIGNED_UINT32_READ(&keyp[24]));
    cryp->cryp_k[7] = __REV(__UNALIGNED_UINT32_READ(&keyp[28]));
  }
  else {
    return CRY_ERR_INV_KEY_SIZE;
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
  unsigned i;

  /* Only key zero is supported.*/
  if (key_id != 0U) {
    return CRY_ERR_INV_KEY_ID;
  }

  /* Setting the stored key.*/
  if (cryp->cryp_ktype != cryp_key_aes_encrypt) {
    cryp_set_key_encrypt(cryp, CRYP_CR_ALGOMODE_AES_ECB);
  }

  /* Pushing the AES block in the FIFO, it is assumed to be empty.*/
  CRYP->DIN = __UNALIGNED_UINT32_READ(&in[0]);
  CRYP->DIN = __UNALIGNED_UINT32_READ(&in[4]);
  CRYP->DIN = __UNALIGNED_UINT32_READ(&in[8]);
  CRYP->DIN = __UNALIGNED_UINT32_READ(&in[12]);

  /* Reading the result.*/
  for (i = 0U; i < 4; i++, out += 4) {
    while ((CRYP->SR & CRYP_SR_OFNE) == 0U) {
    }
    __UNALIGNED_UINT32_WRITE(out, CRYP->DOUT);
  }

  /* Disabling unit.*/
  CRYP->CR &= ~CRYP_CR_CRYPEN;

  return CRY_NOERROR;
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
  unsigned i;

  /* Only key zero is supported.*/
  if (key_id != 0U) {
    return CRY_ERR_INV_KEY_ID;
  }

  /* Setting the stored key.*/
  if (cryp->cryp_ktype != cryp_key_aes_decrypt) {
    cryp_set_key_decrypt(cryp, CRYP_CR_ALGOMODE_AES_ECB);
  }

  /* Pushing the AES block in the FIFO, it is assumed to be empty.*/
  CRYP->DIN = __UNALIGNED_UINT32_READ(&in[0]);
  CRYP->DIN = __UNALIGNED_UINT32_READ(&in[4]);
  CRYP->DIN = __UNALIGNED_UINT32_READ(&in[8]);
  CRYP->DIN = __UNALIGNED_UINT32_READ(&in[12]);

  /* Reading the result.*/
  for (i = 0U; i < 4; i++, out += 4) {
    while ((CRYP->SR & CRYP_SR_OFNE) == 0U) {
    }
    __UNALIGNED_UINT32_WRITE(out, CRYP->DOUT);
  }

  /* Disabling unit.*/
  CRYP->CR &= ~CRYP_CR_CRYPEN;

  return CRY_NOERROR;
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

  /* Only key zero is supported.*/
  if (key_id != 0U) {
    return CRY_ERR_INV_KEY_ID;
  }

  /* Setting the stored key.*/
  if (cryp->cryp_ktype != cryp_key_aes_encrypt) {
    cryp_set_key_encrypt(cryp, CRYP_CR_ALGOMODE_AES_ECB);
  }

  return cryp_do_transfer(cryp, size, in, out);
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

  /* Only key zero is supported.*/
  if (key_id != 0U) {
    return CRY_ERR_INV_KEY_ID;
  }

  /* Setting the stored key.*/
  if (cryp->cryp_ktype != cryp_key_aes_decrypt) {
    cryp_set_key_decrypt(cryp, CRYP_CR_ALGOMODE_AES_ECB);
  }

  return cryp_do_transfer(cryp, size, in, out);
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

  /* Only key zero is supported.*/
  if (key_id != 0U) {
    return CRY_ERR_INV_KEY_ID;
  }

  /* Setting the stored key and IV.*/
  cryp_set_iv(cryp, iv);
  if (cryp->cryp_ktype != cryp_key_aes_encrypt) {
    cryp_set_key_encrypt(cryp, CRYP_CR_ALGOMODE_AES_CBC);
  }

  return cryp_do_transfer(cryp, size, in, out);
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

  /* Only key zero is supported.*/
  if (key_id != 0U) {
    return CRY_ERR_INV_KEY_ID;
  }

  /* Setting the stored key and IV.*/
  cryp_set_iv(cryp, iv);
  if (cryp->cryp_ktype != cryp_key_aes_decrypt) {
    cryp_set_key_decrypt(cryp, CRYP_CR_ALGOMODE_AES_CBC);
  }

  return cryp_do_transfer(cryp, size, in, out);
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

  return CRY_NOERROR;
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

  /* Initializing context structure.*/
  sha256ctxp->last_data = 0U;
  sha256ctxp->last_size = 0U;

  /* Initializing operation.*/
  HASH->CR = /* HASH_CR_MDMAT |*/ HASH_CR_ALGO_1 | HASH_CR_ALGO_0 |
             HASH_CR_DATATYPE_1 | HASH_CR_INIT;

  return CRY_NOERROR;
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
  const uint32_t *wp = (const uint32_t *)(const void *)in;

  /* This HW is unable to hash blocks that are not a multiple of 4 bytes
     except for the last block in the stream which is handled in the
     "final" function.*/
  if (sha256ctxp->last_size != 0U) {
    return CRY_ERR_OP_FAILURE;
  }

  /* Any unaligned data is deferred to the "final" function.*/
  sha256ctxp->last_size = 8U * (size % sizeof (uint32_t));
  if (sha256ctxp->last_size > 0U) {
    sha256ctxp->last_data = wp[size / sizeof (uint32_t)];
  }

  /* Pushing data.*/
  cry_lld_hash_push(cryp, (uint32_t)(size / sizeof (uint32_t)), wp);

  return CRY_NOERROR;
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
  uint32_t digest[8];

  (void)cryp;

  if (sha256ctxp->last_size > 0U) {
    HASH->DIN = sha256ctxp->last_data;
  }

  /* Triggering final calculation and wait for result.*/
  HASH->SR  = 0U;
  HASH->STR = sha256ctxp->last_size;
  HASH->STR = sha256ctxp->last_size | HASH_STR_DCAL;
  while ((HASH->SR & HASH_SR_DCIS) == 0U) {
  }

  /* Reading digest.*/
  digest[0] = HASH_DIGEST->HR[0];
  digest[1] = HASH_DIGEST->HR[1];
  digest[2] = HASH_DIGEST->HR[2];
  digest[3] = HASH_DIGEST->HR[3];
  digest[4] = HASH_DIGEST->HR[4];
  digest[5] = HASH_DIGEST->HR[5];
  digest[6] = HASH_DIGEST->HR[6];
  digest[7] = HASH_DIGEST->HR[7];
  memcpy((void *)out, (const void *)digest, sizeof digest);

  return CRY_NOERROR;
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

  return CRY_NOERROR;
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
