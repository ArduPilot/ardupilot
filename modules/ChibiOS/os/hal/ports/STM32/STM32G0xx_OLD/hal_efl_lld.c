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
 * @file    hal_efl_lld.c
 * @brief   STM32G07/8nxx Embedded Flash subsystem low level driver source.
 *
 * @addtogroup HAL_EFL
 * @{
 */

#include <string.h>

#include "hal.h"

#if (HAL_USE_EFL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define STM32_FLASH_LINE_SIZE               8U
#define STM32_FLASH_LINE_MASK               (STM32_FLASH_LINE_SIZE - 1U)

#define FLASH_PDKEY1                        0x04152637U
#define FLASH_PDKEY2                        0xFAFBFCFDU

#define FLASH_KEY1                          0x45670123U
#define FLASH_KEY2                          0xCDEF89ABU

#define FLASH_OPTKEY1                       0x08192A3BU
#define FLASH_OPTKEY2                       0x4C5D6E7FU

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   EFL1 driver identifier.
 */
EFlashDriver EFLD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/* The descriptor for 64K devices. */
static const flash_descriptor_t efl_lld_size1[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Bank 1. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED |
                            FLASH_ATTR_ECC_CAPABLE   |
                            FLASH_ATTR_ECC_ZERO_LINE_CAPABLE,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH_SECTORS_TOTAL_64K,
       .sectors           = NULL,
       .sectors_size      = STM32_FLASH_SECTOR_SIZE_64K,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH_SIZE_64K * STM32_FLASH_SIZE_SCALE
      }
};

/* The descriptor for 128K devices. */
static const flash_descriptor_t efl_lld_size2[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Bank 1. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED |
                            FLASH_ATTR_ECC_CAPABLE   |
                            FLASH_ATTR_ECC_ZERO_LINE_CAPABLE,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH_SECTORS_TOTAL_128K,
       .sectors           = NULL,
       .sectors_size      = STM32_FLASH_SECTOR_SIZE_128K,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH_SIZE_128K * STM32_FLASH_SIZE_SCALE
      }
};

/* Table describing possible flash sizes and descriptors for this device. */
static const efl_lld_size_t efl_lld_flash_sizes[] = {
      {
       .desc = efl_lld_size1
      },
      {
       .desc = efl_lld_size2
      }
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void stm32_flash_lock(EFlashDriver *eflp) {

  eflp->flash->CR |= FLASH_CR_LOCK;
}

static inline void stm32_flash_unlock(EFlashDriver *eflp) {

  eflp->flash->KEYR |= FLASH_KEY1;
  eflp->flash->KEYR |= FLASH_KEY2;
}

static inline void stm32_flash_enable_pgm(EFlashDriver *eflp) {

  eflp->flash->CR |= FLASH_CR_PG;
}

static inline void stm32_flash_disable_pgm(EFlashDriver *eflp) {

  eflp->flash->CR &= ~FLASH_CR_PG;
}

static inline void stm32_flash_clear_status(EFlashDriver *eflp) {

  eflp->flash->SR = 0x0000FFFFU;
}

static inline void stm32_flash_wait_busy(EFlashDriver *eflp) {

  /* Wait for busy bit clear.*/
  while ((eflp->flash->SR & FLASH_SR_BSY1) != 0U) {
  }
}

static inline size_t stm32_flash_get_size(void) {
  return *(uint16_t*)((uint32_t) STM32_FLASH_SIZE_REGISTER) * STM32_FLASH_SIZE_SCALE;
}

static inline bool stm32_flash_dual_bank(EFlashDriver *eflp) {

#if STM32_FLASH_NUMBER_OF_BANKS > 1
#error "Device settings incorrectly configured - single bank mode only supported"
#endif
  (void)eflp;
  return false;
}

static inline flash_error_t stm32_flash_check_errors(EFlashDriver *eflp) {
  uint32_t sr = eflp->flash->SR;

  /* Clearing error conditions.*/
  eflp->flash->SR = sr & 0x0000FFFFU;

  /* Some errors are only caught by assertion.*/
  osalDbgAssert((sr & (FLASH_SR_FASTERR |
                       FLASH_SR_MISERR |
                       FLASH_SR_SIZERR)) == 0U, "unexpected flash error");

  /* Decoding relevant errors.*/
  if ((sr & FLASH_SR_WRPERR) != 0U) {
    return FLASH_ERROR_HW_FAILURE;
  }

  if ((sr & (FLASH_SR_PGAERR | FLASH_SR_PROGERR | FLASH_SR_OPERR)) != 0U) {
    return eflp->state == FLASH_PGM ? FLASH_ERROR_PROGRAM : FLASH_ERROR_ERASE;
  }

  return FLASH_NO_ERROR;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level Embedded Flash driver initialization.
 *
 * @notapi
 */
void efl_lld_init(void) {

  /* Driver initialization.*/
  eflObjectInit(&EFLD1);
  EFLD1.flash = FLASH;
  /* Find the size of the flash and set descriptor reference. */
  uint8_t i;
  for (i = 0; i < (sizeof(efl_lld_flash_sizes) / sizeof(efl_lld_size_t)); i++) {
      if (efl_lld_flash_sizes[i].desc->size == stm32_flash_get_size()) {
        EFLD1.descriptor = efl_lld_flash_sizes[i].desc;
      if (stm32_flash_dual_bank(&EFLD1)) {
        /* Point to the dual bank descriptor. */
        EFLD1.descriptor++;
      }
      return;
    }
  }
  osalDbgAssert(false, "invalid flash configuration");
}

/**
 * @brief   Configures and activates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_start(EFlashDriver *eflp) {
  stm32_flash_unlock(eflp);
  FLASH->CR = 0x00000000U;
}

/**
 * @brief   Deactivates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_stop(EFlashDriver *eflp) {

  stm32_flash_lock(eflp);
}

/**
 * @brief   Gets the flash descriptor structure.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @return                          A flash device descriptor.
 * @retval                          Pointer to single bank if DBM not enabled.
 * @retval                          Pointer to bank1 if DBM enabled.
 *
 * @notapi
 */
const flash_descriptor_t *efl_lld_get_descriptor(void *instance) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  return devp->descriptor;
}

/**
 * @brief   Read operation.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] offset                offset within full flash address space
 * @param[in] n                     number of bytes to be read
 * @param[out] rp                   pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_READ         if the read operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_read(void *instance, flash_offset_t offset,
                           size_t n, uint8_t *rp) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck((instance != NULL) && (rp != NULL) && (n > 0U));

  const flash_descriptor_t *bank = efl_lld_get_descriptor(instance);
  osalDbgCheck((size_t)offset + n <= (size_t)bank->size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No reading while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  /* Clearing error status bits.*/
  stm32_flash_clear_status(devp);

  /* Actual read implementation.*/
  memcpy((void *)rp, (const void *)efl_lld_get_descriptor(instance)->address
                                   + offset, n);

  /* Checking for errors after reading.*/
  if ((devp->flash->SR & FLASH_SR_RDERR) != 0U) {
    err = FLASH_ERROR_READ;
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;

}

/**
 * @brief   Program operation.
 * @note    The device supports ECC, it is only possible to write erased
 *          pages once except when writing all zeroes.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] offset                offset within full flash address space
 * @param[in] n                     number of bytes to be programmed
 * @param[in] pp                    pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_PROGRAM      if the program operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_program(void *instance, flash_offset_t offset,
                              size_t n, const uint8_t *pp) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  const flash_descriptor_t *bank = efl_lld_get_descriptor(instance);
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck((instance != NULL) && (pp != NULL) && (n > 0U));
  osalDbgCheck((size_t)offset + n <= (size_t)bank->size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No programming while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_PGM state while the operation is performed.*/
  devp->state = FLASH_PGM;

  /* Clearing error status bits.*/
  stm32_flash_clear_status(devp);

  /* Enabling PGM mode in the controller.*/
  stm32_flash_enable_pgm(devp);

  /* Actual program implementation.*/
  while (n > 0U) {
    volatile uint32_t *address;

    union {
      uint32_t  w[STM32_FLASH_LINE_SIZE / sizeof (uint32_t)];
      uint8_t   b[STM32_FLASH_LINE_SIZE / sizeof (uint8_t)];
    } line;

    /* Unwritten bytes are initialized to all ones.*/
    line.w[0] = 0xFFFFFFFFU;
    line.w[1] = 0xFFFFFFFFU;

    /* Programming address aligned to flash lines.*/
    address = (volatile uint32_t *)(bank->address +
                                    (offset & ~STM32_FLASH_LINE_MASK));

    /* Copying data inside the prepared line.*/
    do {
      line.b[offset & STM32_FLASH_LINE_MASK] = *pp;
      offset++;
      n--;
      pp++;
    }
    while ((n > 0U) & ((offset & STM32_FLASH_LINE_MASK) != 0U));

    /* Programming line.*/
    address[0] = line.w[0];
    address[1] = line.w[1];
    stm32_flash_wait_busy(devp);
    err = stm32_flash_check_errors(devp);
    if (err != FLASH_NO_ERROR) {
      break;
    }
  }

  /* Disabling PGM mode in the controller.*/
  stm32_flash_disable_pgm(devp);

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

/**
 * @brief   Starts a whole-device erase operation.
 * @note    This function only erases bank 2 if it is present. Bank 1 is not
 *          allowed since it is normally where the primary program is located.
 *          Pages on bank 1 can be individually erased.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_all(void *instance) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  osalDbgCheck(instance != NULL);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

#if defined(FLASH_CR_MER2)
  /* If dual bank is active then mass erase bank2. */
  if (stm32_flash_dual_bank(devp)) {

    /* FLASH_ERASE state while the operation is performed.*/
    devp->state = FLASH_ERASE;

    /* Clearing error status bits.*/
    stm32_flash_clear_status(devp);

    devp->flash->CR |= FLASH_CR_MER2;
    devp->flash->CR |= FLASH_CR_STRT;
    return FLASH_NO_ERROR;
  }
#endif

  /* Mass erase not allowed. */
  return FLASH_ERROR_UNIMPLEMENTED;
}

/**
 * @brief   Starts an sector erase operation.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be erased
 *                                  this is an index within the total sectors
 *                                  in a flash bank
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_sector(void *instance,
                                         flash_sector_t sector) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  const flash_descriptor_t *bank = efl_lld_get_descriptor(instance);
  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < bank->sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_PGM state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  /* Clearing error status bits.*/
  stm32_flash_clear_status(devp);

  /* Enable page erase.*/
  devp->flash->CR |= FLASH_CR_PER;

#if defined(FLASH_CR_BKER)
  /* If dual bank is active then setup relevant bank. */
  if (stm32_flash_dual_bank(devp)) {
    if (sector < (bank->sectors_count / 2)) {
      /* First bank.*/
      devp->flash->CR &= ~FLASH_CR_BKER;
    }
    else {
      /* Second bank. Adjust sector index. */
      sector -= (bank->sectors_count / 2);
      devp->flash->CR |= FLASH_CR_BKER;
    }
  }
#endif

  /* Mask off the page selection bits.*/
  devp->flash->CR &= ~FLASH_CR_PNB;

  /* Set the page selection bits.*/
  devp->flash->CR |= sector << FLASH_CR_PNB_Pos;

  /* Start the erase.*/
  devp->flash->CR |= FLASH_CR_STRT;

  return FLASH_NO_ERROR;
}

/**
 * @brief   Queries the driver for erase operation progress.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[out] msec                 recommended time, in milliseconds, that
 *                                  should be spent before calling this
 *                                  function again, can be @p NULL
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_ERASE        if the erase operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @api
 */
flash_error_t efl_lld_query_erase(void *instance, uint32_t *msec) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err;

  /* If there is an erase in progress then the device must be checked.*/
  if (devp->state == FLASH_ERASE) {

    /* Checking for operation in progress.*/
    if ((devp->flash->SR & FLASH_SR_BSY1) == 0U) {

      /* Disabling the various erase control bits.*/
      devp->flash->CR &= ~(FLASH_CR_MER1 |
#if defined(FLASH_CR_MER2)
                           FLASH_CR_MER2 |
#endif
                           FLASH_CR_PER);

      /* No operation in progress, checking for errors.*/
      err = stm32_flash_check_errors(devp);

      /* Back to ready state.*/
      devp->state = FLASH_READY;
    }
    else {
      /* Recommended time before polling again. This is a simplified
         implementation.*/
      if (msec != NULL) {
        *msec = (uint32_t)STM32_FLASH_WAIT_TIME_MS;
      }

      err = FLASH_BUSY_ERASING;
    }
  }
  else {
    err = FLASH_NO_ERROR;
  }

  return err;
}

/**
 * @brief   Returns the erase state of a sector.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be verified
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if the sector is erased.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_VERIFY       if the verify operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_verify_erase(void *instance, flash_sector_t sector) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  uint32_t *address;
  const flash_descriptor_t *bank = efl_lld_get_descriptor(instance);
  flash_error_t err = FLASH_NO_ERROR;
  unsigned i;

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < bank->sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No verifying while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* Address of the sector in the selected bank.*/
  address = (uint32_t *)(bank->address +
                        flashGetSectorOffset(getBaseFlash(devp), sector));

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  /* Scanning the sector space.*/
  uint32_t sector_size = flashGetSectorSize(getBaseFlash(devp), sector);
  for (i = 0U; i < sector_size / sizeof(uint32_t); i++) {
    if (*address != 0xFFFFFFFFU) {
      err = FLASH_ERROR_VERIFY;
      break;
    }
    address++;
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

#endif /* HAL_USE_EFL == TRUE */

/** @} */
