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
 * @file    hal_efl_lld.h
 * @brief   STM32L4+ Embedded Flash subsystem low level driver header.
 *
 * @addtogroup HAL_EFL
 * @{
 */

#ifndef HAL_EFL_LLD_H
#define HAL_EFL_LLD_H

#if (HAL_USE_EFL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    STM32L4xx configuration options
 * @{
 */
/**
 * @brief   Suggested wait time during erase operations polling.
 */
#if !defined(STM32_FLASH_WAIT_TIME_MS) || defined(__DOXYGEN__)
#define STM32_FLASH_WAIT_TIME_MS            5
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if defined(STM32L4P5xx) || defined(STM32L4Q5xx) || defined(STM32L4R5xx) || \
    defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || \
    defined(STM32L4S7xx) || defined(STM32L4S9xx) || defined(__DOXYGEN__)

/* Flash size register. */
#define STM32_FLASH_SIZE_REGISTER           0x1FFF75E0
#define STM32_FLASH_SIZE_SCALE              1024U

/*
 * Device flash size is:
 *  1M for STM32L4+ suffix G devices
 *  2M for STM32L4+ suffix I devices.
 *
 * For 1M devices SBM is organised as 128 x 8K pages.
 * For 1M devices DBM is organised as 128 x 4K pages per bank.
 *
 * For 2M devices SBM is organised as 256 x 8K pages.
 * For 2M devices DBM is organised as 256 x 4K pages per bank.
 */

#define STM32_FLASH_SIZE_1M                 1024U
#define STM32_FLASH_SIZE_2M                 2048U
#define STM32_FLASH_SECTORS_TOTAL_1M        128
#define STM32_FLASH_SECTORS_TOTAL_2M        256

/* Single bank mode bank 1 for 1M device.*/
#define STM32_FLASH_SECTOR_SIZE_1M          ((STM32_FLASH_SIZE_1M           \
                                              * STM32_FLASH_SIZE_SCALE)     \
                                             / STM32_FLASH_SECTORS_TOTAL_1M)

/* Dual bank mode banks 1 & 2 for 1M device.*/
#define STM32_FLASH_DUAL_SECTOR_SIZE_1M     (STM32_FLASH_SECTOR_SIZE_1M / 2)

/* Single bank mode bank 1 for 2M device.*/
#define STM32_FLASH_SECTOR_SIZE_2M          ((STM32_FLASH_SIZE_2M           \
                                              * STM32_FLASH_SIZE_SCALE)     \
                                             / STM32_FLASH_SECTORS_TOTAL_2M)

/* Dual bank mode banks 1 & 2 for 2M device.*/
#define STM32_FLASH_DUAL_SECTOR_SIZE_2M     (STM32_FLASH_SECTOR_SIZE_2M / 2)

#else
#error "This EFL driver does not support the selected device"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/* A flash size declaration. */
typedef struct {
  const flash_descriptor_t* desc;
} efl_lld_size_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the embedded flash driver structure.
 */
#define efl_lld_driver_fields                                               \
  /* Flash registers.*/                                                     \
  FLASH_TypeDef             *flash;                                         \
  const flash_descriptor_t  *descriptor;

/**
 * @brief   Low level fields of the embedded flash configuration structure.
 */
#define efl_lld_config_fields                                               \
  /* Dummy configuration, it is not needed.*/                               \
  uint32_t                  dummy;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern EFlashDriver EFLD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void efl_lld_init(void);
  void efl_lld_start(EFlashDriver *eflp);
  void efl_lld_stop(EFlashDriver *eflp);
  const flash_descriptor_t *efl_lld_get_descriptor(void *instance);
  flash_error_t efl_lld_read(void *instance, flash_offset_t offset,
                             size_t n, uint8_t *rp);
  flash_error_t efl_lld_program(void *instance, flash_offset_t offset,
                                size_t n, const uint8_t *pp);
  flash_error_t efl_lld_start_erase_all(void *instance);
  flash_error_t efl_lld_start_erase_sector(void *instance,
                                           flash_sector_t sector);
  flash_error_t efl_lld_query_erase(void *instance, uint32_t *msec);
  flash_error_t efl_lld_verify_erase(void *instance, flash_sector_t sector);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EFL == TRUE */

#endif /* HAL_EFL_LLD_H */

/** @} */
