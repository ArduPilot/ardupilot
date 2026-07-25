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
 * @file    hal_serial_nor.h
 * @brief   Serial NOR driver header.
 *
 * @addtogroup HAL_SERIAL_NOR
 * @{
 */

#ifndef HAL_SERIAL_NOR_H
#define HAL_SERIAL_NOR_H

#include "hal_flash.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Bus interface modes.
 * @{
 */
#define SNOR_BUS_DRIVER_SPI                 0U
#define SNOR_BUS_DRIVER_WSPI                1U
/** @} */

/**
 * @brief   Size of the buffer for internal operations.
 */
#define SNOR_BUFFER_SIZE                    32

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Physical transport interface.
 */
#if !defined(SNOR_BUS_DRIVER) || defined(__DOXYGEN__)
#define SNOR_BUS_DRIVER                     SNOR_BUS_DRIVER_WSPI
#endif

/**
 * @brief   Shared bus switch.
 * @details If set to @p TRUE the device acquires bus ownership
 *          on each transaction.
 * @note    Requires @p SPI_USE_MUTUAL_EXCLUSION or
 *          @p WSPI_USE_MUTUAL_EXCLUSION depending on mode selected
 *          with @p SNOR_BUS_MODE.
 */
#if !defined(SNOR_SHARED_BUS) || defined(__DOXYGEN__)
#define SNOR_SHARED_BUS                     TRUE
#endif

/**
 * @brief   Exclusive access control.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(SNOR_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define SNOR_USE_MUTUAL_EXCLUSION           TRUE
#endif

/**
 * @brief   SPI 4-bytes address switch
 * @details If set to @p TRUE the device will use 4-bytes address
 *          in SPI bus, only relevant if SPI is used
 */
#if !defined(SNOR_SPI_4BYTES_ADDRESS) || defined(__DOXYGEN__)
#define SNOR_SPI_4BYTES_ADDRESS             FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_SPI) || defined(__DOXYGEN__)
#define BUSConfig SPIConfig
#define BUSDriver SPIDriver
#elif SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI
#define BUSConfig WSPIConfig
#define BUSDriver WSPIDriver
#else
#error "invalid SNOR_BUS_DRIVER setting"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a SNOR configuration structure.
 */
typedef struct {
  BUSDriver                 *busp;
  const BUSConfig           *buscfg;
} SNORConfig;

/**
 * @brief   @p SNORDriver specific methods.
 */
#define _snor_flash_methods_alone                                           \
  /* Read SFDP.*/                                                           \
  flash_error_t (*read_sfdp)(void *instance,                                \
                 flash_offset_t offset,                                     \
                 size_t n,                                                  \
                 uint8_t *rp);

/**
 * @brief   @p SNORDriver specific methods with inherited ones.
 */
#define _snor_flash_methods                                                 \
  _base_flash_methods                                                       \
  _snor_flash_methods_alone

/**
 * @extends BaseFlashVMT
 *
 * @brief   @p SNOR virtual methods table.
 */
struct SNORDriverVMT {
  _snor_flash_methods
};

typedef struct snor_nocache_buffer {
  /**
   * @brief   Temporary generic buffer.
   */
  uint8_t                       buf[SNOR_BUFFER_SIZE];
#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) || defined(__DOXYGEN__)
  /**
   * @brief   Temporary command buffer.
   */
  wspi_command_t                cmd;
#endif
} snor_nocache_buffer_t;

/**
 * @extends BaseFlash
 *
 * @brief   Type of SNOR flash class.
 */
typedef struct {
  /**
   * @brief   SNORDriver Virtual Methods Table.
   */
  const struct SNORDriverVMT    *vmt;
  _base_flash_data
  /**
   * @brief   Current configuration data.
   */
  const SNORConfig              *config;
  /**
   * @brief   Non-cacheable buffer associated to this instance.
   */
  snor_nocache_buffer_t         *nocache;
#if (SNOR_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting SNOR.
   */
  mutex_t                       mutex;
#endif /* EFL_USE_MUTUAL_EXCLUSION == TRUE */
} SNORDriver;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#if SNOR_SHARED_BUS == FALSE
#define bus_acquire(busp, config)
#define bus_release(busp)
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
#if SNOR_SHARED_BUS == TRUE
  void bus_acquire(BUSDriver *busp, const BUSConfig *config);
  void bus_release(BUSDriver *busp);
#endif
  void bus_cmd(BUSDriver *busp, uint32_t cmd);
  void bus_cmd_send(BUSDriver *busp, uint32_t cmd, size_t n, const uint8_t *p);
  void bus_cmd_receive(BUSDriver *busp,
                       uint32_t cmd,
                       size_t n,
                       uint8_t *p);
  void bus_cmd_addr(BUSDriver *busp, uint32_t cmd, flash_offset_t offset);
  void bus_cmd_addr_send(BUSDriver *busp,
                         uint32_t cmd,
                         flash_offset_t offset,
                         size_t n,
                         const uint8_t *p);
  void bus_cmd_addr_receive(BUSDriver *busp,
                            uint32_t cmd,
                            flash_offset_t offset,
                            size_t n,
                            uint8_t *p);
  void bus_cmd_dummy_receive(BUSDriver *busp,
                             uint32_t cmd,
                             uint32_t dummy,
                             size_t n,
                             uint8_t *p);
  void bus_cmd_addr_dummy_receive(BUSDriver *busp,
                                  uint32_t cmd,
                                  flash_offset_t offset,
                                  uint32_t dummy,
                                  size_t n,
                                  uint8_t *p);
  void snorObjectInit(SNORDriver *devp, snor_nocache_buffer_t *nocache);
  void snorStart(SNORDriver *devp, const SNORConfig *config);
  void snorStop(SNORDriver *devp);
#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) || defined(__DOXYGEN__)
#if (WSPI_SUPPORTS_MEMMAP == TRUE) || defined(__DOXYGEN__)
  void snorMemoryMap(SNORDriver *devp, uint8_t ** addrp);
  void snorMemoryUnmap(SNORDriver *devp);
#endif /* QSPI_SUPPORTS_MEMMAP == TRUE */
#endif /* SNOR_BUS_MODE != SNOR_BUS_MODE_SPI */
#ifdef __cplusplus
}
#endif

/* Device-specific implementations.*/
#include "hal_flash_device.h"

#endif /* HAL_SERIAL_NOR_H */

/** @} */

