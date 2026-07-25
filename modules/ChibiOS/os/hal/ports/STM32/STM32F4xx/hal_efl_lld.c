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
 * @brief   STM32F4xx Embedded Flash subsystem low level driver source.
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

#define STM32_FLASH_LINE_SIZE               (1 << STM32_FLASH_PSIZE)
#define STM32_FLASH_LINE_MASK               (STM32_FLASH_LINE_SIZE - 1U)

#define FLASH_PDKEY1                        0x04152637U
#define FLASH_PDKEY2                        0xFAFBFCFDU

#define FLASH_KEY1                          0x45670123U
#define FLASH_KEY2                          0xCDEF89ABU

#define FLASH_OPTKEY1                       0x08192A3BU
#define FLASH_OPTKEY2                       0x4C5D6E7FU

#if !defined(FLASH_SR_OPERR)
#define FLASH_SR_OPERR                      FLASH_SR_SOP
#endif

#if !defined(STM32_FLASH_DUAL_BANK_PERMANENT)
#define STM32_FLASH_DUAL_BANK_PERMANENT     FALSE
#endif

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

#if defined(STM32F413xx) || defined(STM32F412xx) ||                         \
    defined(STM32F40_41xxx) || defined(__DOXYGEN__)

/* Sector table for 1.5M device. */
static const flash_sector_descriptor_t efl_lld_sect1[STM32_FLASH1_SECTORS_TOTAL] = {
  {         0,                        16384},   /* Sector  0. */
  { 1 * 16384,                        16384},   /* Sector  1. */
  { 2 * 16384,                        16384},   /* Sector  2. */
  { 3 * 16384,                        16384},   /* Sector  3. */
  { 4 * 16384,                        65536},   /* Sector  4. */
  { 4 * 16384 + 65536,               131072},   /* Sector  5. */
  { 4 * 16384 + 65536 +  1 * 131072, 131072},   /* Sector  6. */
  { 4 * 16384 + 65536 +  2 * 131072, 131072},   /* Sector  7. */
  { 4 * 16384 + 65536 +  3 * 131072, 131072},   /* Sector  8. */
  { 4 * 16384 + 65536 +  4 * 131072, 131072},   /* Sector  9. */
  { 4 * 16384 + 65536 +  5 * 131072, 131072},   /* Sector 10. */
  { 4 * 16384 + 65536 +  6 * 131072, 131072},   /* Sector 11. */
  { 4 * 16384 + 65536 +  7 * 131072, 131072},   /* Sector 12. */
  { 4 * 16384 + 65536 +  8 * 131072, 131072},   /* Sector 13. */
  { 4 * 16384 + 65536 +  9 * 131072, 131072},   /* Sector 14. */
  { 4 * 16384 + 65536 + 10 * 131072, 131072}    /* Sector 15. */
};

/* Sector table for 1M device. */
static const flash_sector_descriptor_t efl_lld_sect2[STM32_FLASH2_SECTORS_TOTAL] = {
  {         0,                        16384},   /* Sector  0. */
  { 1 * 16384,                        16384},   /* Sector  1. */
  { 2 * 16384,                        16384},   /* Sector  2. */
  { 3 * 16384,                        16384},   /* Sector  3. */
  { 4 * 16384,                        65536},   /* Sector  4. */
  { 4 * 16384 + 65536,               131072},   /* Sector  5. */
  { 4 * 16384 + 65536 +  1 * 131072, 131072},   /* Sector  6. */
  { 4 * 16384 + 65536 +  2 * 131072, 131072},   /* Sector  7. */
  { 4 * 16384 + 65536 +  3 * 131072, 131072},   /* Sector  8. */
  { 4 * 16384 + 65536 +  4 * 131072, 131072},   /* Sector  9. */
  { 4 * 16384 + 65536 +  5 * 131072, 131072},   /* Sector 10. */
  { 4 * 16384 + 65536 +  6 * 131072, 131072}    /* Sector 11. */
};

/* Sector table for 512K device. */
static const flash_sector_descriptor_t efl_lld_sect3[STM32_FLASH3_SECTORS_TOTAL] = {
  {         0,                        16384},   /* Sector  0. */
  { 1 * 16384,                        16384},   /* Sector  1. */
  { 2 * 16384,                        16384},   /* Sector  2. */
  { 3 * 16384,                        16384},   /* Sector  3. */
  { 4 * 16384,                        65536},   /* Sector  4. */
  { 4 * 16384 + 65536,               131072},   /* Sector  5. */
  { 4 * 16384 + 65536 +  1 * 131072, 131072},   /* Sector  6. */
  { 4 * 16384 + 65536 +  2 * 131072, 131072},   /* Sector  7. */
};

/* The descriptors for 1.5M device. */
static const flash_descriptor_t efl_lld_size1[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Single bank organisation. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH1_SECTORS_TOTAL,
       .sectors           = efl_lld_sect1,
       .sectors_size      = 0,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH1_SIZE * STM32_FLASH_SIZE_SCALE
      }
};

/* The descriptors for 1M device. */
static const flash_descriptor_t efl_lld_size2[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Single bank organisation. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH2_SECTORS_TOTAL,
       .sectors           = efl_lld_sect2,
       .sectors_size      = 0,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH2_SIZE * STM32_FLASH_SIZE_SCALE
      }
};

/* The descriptors for 512K device. */
static const flash_descriptor_t efl_lld_size3[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Single bank organisation. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH3_SECTORS_TOTAL,
       .sectors           = efl_lld_sect3,
       .sectors_size      = 0,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH3_SIZE * STM32_FLASH_SIZE_SCALE
      }
};

/* Table describing possible flash sizes and descriptors for this device. */
static const efl_lld_size_t efl_lld_flash_sizes[] = {
      {
       .desc = efl_lld_size1
      },
      {
       .desc = efl_lld_size2
      },
      {
       .desc = efl_lld_size3
      }
};
#elif defined(STM32F401xx) || defined(STM32F411xx) ||                         \
    defined(__DOXYGEN__)

/* Sector table for 128k device. */
static const flash_sector_descriptor_t efl_lld_sect1[STM32_FLASH1_SECTORS_TOTAL] = {
  {         0,                        16384},   /* Sector  0. */
  { 1 * 16384,                        16384},   /* Sector  1. */
  { 2 * 16384,                        16384},   /* Sector  2. */
  { 3 * 16384,                        16384},   /* Sector  3. */
  { 4 * 16384,                        65536},   /* Sector  4. */
};

/* Sector table for 256k device. */
static const flash_sector_descriptor_t efl_lld_sect2[STM32_FLASH2_SECTORS_TOTAL] = {
  {         0,                        16384},   /* Sector  0. */
  { 1 * 16384,                        16384},   /* Sector  1. */
  { 2 * 16384,                        16384},   /* Sector  2. */
  { 3 * 16384,                        16384},   /* Sector  3. */
  { 4 * 16384,                        65536},   /* Sector  4. */
  { 4 * 16384 + 65536,               131072},   /* Sector  5. */
};

/* Sector table for 384k device. */
static const flash_sector_descriptor_t efl_lld_sect3[STM32_FLASH3_SECTORS_TOTAL] = {
  {         0,                        16384},   /* Sector  0. */
  { 1 * 16384,                        16384},   /* Sector  1. */
  { 2 * 16384,                        16384},   /* Sector  2. */
  { 3 * 16384,                        16384},   /* Sector  3. */
  { 4 * 16384,                        65536},   /* Sector  4. */
  { 4 * 16384 + 65536,               131072},   /* Sector  5. */
  { 4 * 16384 + 65536 +  1 * 131072, 131072},   /* Sector  6. */
};

/* Sector table for 512k device. */
static const flash_sector_descriptor_t efl_lld_sect4[STM32_FLASH4_SECTORS_TOTAL] = {
  {         0,                        16384},   /* Sector  0. */
  { 1 * 16384,                        16384},   /* Sector  1. */
  { 2 * 16384,                        16384},   /* Sector  2. */
  { 3 * 16384,                        16384},   /* Sector  3. */
  { 4 * 16384,                        65536},   /* Sector  4. */
  { 4 * 16384 + 65536,               131072},   /* Sector  5. */
  { 4 * 16384 + 65536 +  1 * 131072, 131072},   /* Sector  6. */
  { 4 * 16384 + 65536 +  2 * 131072, 131072},   /* Sector  7. */
};

/* The descriptors for 128k device. */
static const flash_descriptor_t efl_lld_size1[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Single bank organisation. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH1_SECTORS_TOTAL,
       .sectors           = efl_lld_sect1,
       .sectors_size      = 0,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH1_SIZE * STM32_FLASH_SIZE_SCALE
      }
};

/* The descriptors for 256k device. */
static const flash_descriptor_t efl_lld_size2[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Single bank organisation. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH2_SECTORS_TOTAL,
       .sectors           = efl_lld_sect2,
       .sectors_size      = 0,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH2_SIZE * STM32_FLASH_SIZE_SCALE
      }
};

/* The descriptors for 384k device. */
static const flash_descriptor_t efl_lld_size3[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Single bank organisation. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH3_SECTORS_TOTAL,
       .sectors           = efl_lld_sect3,
       .sectors_size      = 0,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH3_SIZE * STM32_FLASH_SIZE_SCALE
      }
};

/* The descriptors for 512k device. */
static const flash_descriptor_t efl_lld_size4[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Single bank organisation. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH4_SECTORS_TOTAL,
       .sectors           = efl_lld_sect4,
       .sectors_size      = 0,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH4_SIZE * STM32_FLASH_SIZE_SCALE
      }
};

/* Table describing possible flash sizes and descriptors for this device. */
static const efl_lld_size_t efl_lld_flash_sizes[] = {
      {
       .desc = efl_lld_size1
      },
      {
       .desc = efl_lld_size2
      },
      {
       .desc = efl_lld_size3
      },
      {
       .desc = efl_lld_size4
      }
};
#elif defined(STM32F429_439xx) || defined(STM32F427_437xx) || \
      defined(__DOXYGEN__)

/* Sector table for 1M device in SBM. */
static const flash_sector_descriptor_t efl_lld_sect_1m_sbm[STM32_FLASH_SECTORS_TOTAL_1M_SBM] = {
  {         0,                            16384},    /* Sector  0. */
  { 1 * 16384,                            16384},    /* Sector  1. */
  { 2 * 16384,                            16384},    /* Sector  2. */
  { 3 * 16384,                            16384},    /* Sector  3. */
  { 4 * 16384,                            65536},    /* Sector  4. */
  { 4 * 16384 + 1 * 65536,               131072},    /* Sector  5. */
  { 4 * 16384 + 1 * 65536 +  1 * 131072, 131072},    /* Sector  6. */
  { 4 * 16384 + 1 * 65536 +  2 * 131072, 131072},    /* Sector  7. */
  { 4 * 16384 + 1 * 65536 +  3 * 131072, 131072},    /* Sector  8. */
  { 4 * 16384 + 1 * 65536 +  4 * 131072, 131072},    /* Sector  9. */
  { 4 * 16384 + 1 * 65536 +  5 * 131072, 131072},    /* Sector 10. */
  { 4 * 16384 + 1 * 65536 +  6 * 131072, 131072}     /* Sector 11. */
};

/* Sector table for 1M device in DBM. */
static const flash_sector_descriptor_t efl_lld_sect_1m_dbm[STM32_FLASH_SECTORS_TOTAL_1M_DBM] = {
  {         0,                            16384},    /* Sector  0.  */
  { 1 * 16384,                            16384},    /* Sector  1.  */
  { 2 * 16384,                            16384},    /* Sector  2.  */
  { 3 * 16384,                            16384},    /* Sector  3.  */
  { 4 * 16384,                            65536},    /* Sector  4.  */
  { 4 * 16384 + 1 * 65536,               131072},    /* Sector  5.  */
  { 4 * 16384 + 1 * 65536 +  1 * 131072, 131072},    /* Sector  6.  */
  { 4 * 16384 + 1 * 65536 +  2 * 131072, 131072},    /* Sector  7.  */
  { 4 * 16384 + 1 * 65536 +  3 * 131072,      0},    /* Invalid.    */
  { 4 * 16384 + 1 * 65536 +  3 * 131072,      0},    /* Invalid.    */
  { 4 * 16384 + 1 * 65536 +  3 * 131072,      0},    /* Invalid.    */
  { 4 * 16384 + 1 * 65536 +  3 * 131072,      0},    /* Invalid.    */
  { 4 * 16384 + 1 * 65536 +  3 * 131072,  16384},    /* Sector  12. */
  { 5 * 16384 + 1 * 65536 +  3 * 131072,  16384},    /* Sector  13. */
  { 6 * 16384 + 1 * 65536 +  3 * 131072,  16384},    /* Sector  14. */
  { 7 * 16384 + 1 * 65536 +  3 * 131072,  16384},    /* Sector  15. */
  { 8 * 16384 + 1 * 65536 +  3 * 131072,  65536},    /* Sector  16. */
  { 8 * 16384 + 2 * 65536 +  3 * 131072, 131072},    /* Sector  17. */
  { 8 * 16384 + 2 * 65536 +  4 * 131072, 131072},    /* Sector  18. */
  { 8 * 16384 + 2 * 65536 +  5 * 131072, 131072}     /* Sector  19. */
};

/* Sector table for 2M device banks. */
static const flash_sector_descriptor_t efl_lld_sect_2m[STM32_FLASH_SECTORS_TOTAL_2M] = {
  {         0,                            16384},    /* Sector  0. */
  { 1 * 16384,                            16384},    /* Sector  1. */
  { 2 * 16384,                            16384},    /* Sector  2. */
  { 3 * 16384,                            16384},    /* Sector  3. */
  { 4 * 16384,                            65536},    /* Sector  4. */
  { 4 * 16384 + 1 * 65536,               131072},    /* Sector  5. */
  { 4 * 16384 + 1 * 65536 +  1 * 131072, 131072},    /* Sector  6. */
  { 4 * 16384 + 1 * 65536 +  2 * 131072, 131072},    /* Sector  7. */
  { 4 * 16384 + 1 * 65536 +  3 * 131072, 131072},    /* Sector  8. */
  { 4 * 16384 + 1 * 65536 +  4 * 131072, 131072},    /* Sector  9. */
  { 4 * 16384 + 1 * 65536 +  5 * 131072, 131072},    /* Sector 10. */
  { 4 * 16384 + 1 * 65536 +  6 * 131072, 131072},    /* Sector 11. */
  { 4 * 16384 + 1 * 65536 +  7 * 131072,  16384},    /* Sector 12. */
  { 5 * 16384 + 1 * 65536 +  7 * 131072,  16384},    /* Sector 13. */
  { 6 * 16384 + 1 * 65536 +  7 * 131072,  16384},    /* Sector 14. */
  { 7 * 16384 + 1 * 65536 +  7 * 131072,  16384},    /* Sector 15. */
  { 8 * 16384 + 1 * 65536 +  7 * 131072,  65536},    /* Sector 16. */
  { 8 * 16384 + 2 * 65536 +  7 * 131072, 131072},    /* Sector 17. */
  { 8 * 16384 + 2 * 65536 +  8 * 131072, 131072},    /* Sector 18. */
  { 8 * 16384 + 2 * 65536 +  9 * 131072, 131072},    /* Sector 19. */
  { 8 * 16384 + 2 * 65536 + 10 * 131072, 131072},    /* Sector 20. */
  { 8 * 16384 + 2 * 65536 + 11 * 131072, 131072},    /* Sector 21. */
  { 8 * 16384 + 2 * 65536 + 12 * 131072, 131072},    /* Sector 22. */
  { 8 * 16384 + 2 * 65536 + 13 * 131072, 131072}     /* Sector 23. */
};

/* The descriptors for 1M device. */
static const flash_descriptor_t efl_lld_size_1m[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Bank 1 (SBM) organisation. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH_SECTORS_TOTAL_1M_SBM,
       .sectors           = efl_lld_sect_1m_sbm,
       .sectors_size      = 0,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH_SIZE_1M * STM32_FLASH_SIZE_SCALE
      },
      { /* Bank 1 & 2 (DBM) organisation. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH_SECTORS_TOTAL_1M_DBM,
       .sectors           = efl_lld_sect_1m_dbm,
       .sectors_size      = 0,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH_SIZE_1M * STM32_FLASH_SIZE_SCALE
      }
};

/* The descriptors for 2M device. */
static const flash_descriptor_t efl_lld_size_2m[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Dual bank organisation. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH_SECTORS_TOTAL_2M,
       .sectors           = efl_lld_sect_2m,
       .sectors_size      = 0,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH_SIZE_2M * STM32_FLASH_SIZE_SCALE
      },
      { /* Dual bank organisation. */
       .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                            FLASH_ATTR_MEMORY_MAPPED,
       .page_size         = STM32_FLASH_LINE_SIZE,
       .sectors_count     = STM32_FLASH_SECTORS_TOTAL_2M,
       .sectors           = efl_lld_sect_2m,
       .sectors_size      = 0,
       .address           = (uint8_t *)FLASH_BASE,
       .size              = STM32_FLASH_SIZE_2M * STM32_FLASH_SIZE_SCALE
      }
};

/* Table describing possible flash sizes and descriptors for this device. */
static const efl_lld_size_t efl_lld_flash_sizes[] = {
      {
       .desc = efl_lld_size_1m
      },
      {
       .desc = efl_lld_size_2m
      }
};
#else
#error "This EFL driver does not support the selected device"
#endif

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

  /* Set parallelism. */
  eflp->flash->CR &= ~FLASH_CR_PSIZE;
  eflp->flash->CR |= STM32_FLASH_PSIZE << FLASH_CR_PSIZE_Pos;

  /* Enable programming. */
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
  while ((eflp->flash->SR & FLASH_SR_BSY) != 0U) {
  }
}

static inline size_t stm32_flash_get_size(void) {
  return *(uint16_t*)((uint32_t) STM32_FLASH_SIZE_REGISTER) * STM32_FLASH_SIZE_SCALE;
}

static inline bool stm32_flash_dual_bank(EFlashDriver *eflp) {

#if STM32_FLASH_NUMBER_OF_BANKS > 1
  return ((eflp->flash->OPTCR & FLASH_OPTCR_DB1M) != 0U || STM32_FLASH_DUAL_BANK_PERMANENT);
#endif
  (void)eflp;
  return false;
}

static inline flash_error_t stm32_flash_check_errors(EFlashDriver *eflp) {
  uint32_t sr = eflp->flash->SR;

  /* Clearing error conditions.*/
  eflp->flash->SR = sr & 0x0000FFFFU;

  /* Some errors are only caught by assertion.*/
  osalDbgAssert((sr & 0) == 0U, "unexpected flash error");

  /* Decoding relevant errors.*/
  if ((sr & FLASH_SR_WRPERR) != 0U) {
    return FLASH_ERROR_HW_FAILURE;
  }

  if ((sr & (FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_OPERR)) != 0U) {
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

#if defined(FLASH_CR_RDERR)
  /* Checking for errors after reading.*/
  if ((devp->flash->SR & FLASH_SR_RDERR) != 0U) {
    err = FLASH_ERROR_READ;
  }
#endif

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;

}

/**
 * @brief   Program operation.
 * @note    Successive write operations are possible without the need of
 *          an erase when changing bits from one to zero. Writing one requires
 *          an erase operation.
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

    /* Create an array of sufficient size to hold line(s). */
    union {
      uint32_t  w[STM32_FLASH_LINE_SIZE / sizeof(uint32_t)];
      uint16_t  h[STM32_FLASH_LINE_SIZE / sizeof(uint16_t)];
      uint8_t   b[STM32_FLASH_LINE_SIZE / sizeof(uint8_t)];
    } line;

    /* Unwritten bytes are initialized to all ones.*/
    uint8_t i;
    for (i = 0; i < bank->page_size; i++) {
      line.b[i] = 0xFF;
    }

    /* Programming address aligned to flash lines.*/
    address = (volatile uint32_t *)(bank->address +
                                    (offset & ~STM32_FLASH_LINE_MASK));

    /* Copying data inside the prepared line(s).*/
    do {
      line.b[offset & STM32_FLASH_LINE_MASK] = *pp;
      offset++;
      n--;
      pp++;
    }
    while ((n > 0U) & ((offset & STM32_FLASH_LINE_MASK) != 0U));

    /* Programming line according to parallelism.*/
    switch (STM32_FLASH_LINE_SIZE) {
    case 1:
      address[0] = line.b[0];
      break;

    case 2:
      address[0] = line.h[0];
      break;

    case 4:
      address[0] = line.w[0];
      break;

    case 8:
      address[0] = line.w[0];
      address[1] = line.w[1];
      break;

    default:
      osalDbgAssert(false, "invalid line size");
      break;
    }

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
 * @note    This function only erases the unused bank if in dual bank mode. The
 *          currently in use bank is not allowed since it is normally where the
 *          currently running program is executing from.
 *          Sectors on the in-use bank can be individually erased.
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

    /* Erase the currently unused bank, based on Flash Bank Mode */
    if ((SYSCFG->MEMRMP & SYSCFG_MEMRMP_UFB_MODE) != 0U) {
        /* Bank 2 in use, erase Bank 1 */
        devp->flash->CR |= FLASH_CR_MER;
    }
    else {
        /* Bank 1 in use, erase Bank 2 */
        devp->flash->CR |= FLASH_CR_MER2;
    }
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

  /* Enable sector erase.*/
  devp->flash->CR |= FLASH_CR_SER;

  /* Mask off the sector and parallelism selection bits.*/
  devp->flash->CR &= ~FLASH_CR_SNB;
  devp->flash->CR &= ~FLASH_CR_PSIZE;

#if defined(FLASH_CR_MER2)
  /* Adjust sector value for dual-bank devices
   * For STM32F42x_43x devices (dual-bank), FLASH_CR_SNB values jump to 0b10000
   * for sectors 12 and up.
   */
  if (sector >= 12) {
    sector -= 12;
    sector |= 0x10U;
  }
#endif

  /* Set sector and parallelism. */
  devp->flash->CR |= (sector << FLASH_CR_SNB_Pos) |
                     (STM32_FLASH_PSIZE << FLASH_CR_PSIZE_Pos);

  /* Start the erase.*/
  devp->flash->CR |= FLASH_CR_STRT;

  return FLASH_NO_ERROR;
}

/**
 * @brief   Queries the driver for erase operation progress.
 *
 * @param[in]  instance             pointer to a @p EFlashDriver instance
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
    if ((devp->flash->SR & FLASH_SR_BSY) == 0U) {

      /* Disabling the various erase control bits.*/
      devp->flash->CR &= ~(FLASH_CR_MER  |
#if defined(FLASH_CR_MER2)
                           FLASH_CR_MER2 |
#endif
                           FLASH_CR_SER);

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

  /* Address of the sector in the bank.*/
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
