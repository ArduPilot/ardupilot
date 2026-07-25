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
 * @file    hal_mmc_spi.h
 * @brief   MMC over SPI driver header.
 *
 * @addtogroup MMC_SPI
 * @{
 */

#ifndef HAL_MMC_SPI_H
#define HAL_MMC_SPI_H

#if (HAL_USE_MMC_SPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define MMC_CMD0_RETRY                  10U
#define MMC_CMD1_RETRY                  100U
#define MMC_ACMD41_RETRY                100U
#define MMC_WAIT_DATA                   10000U

/**
 * @brief   Size of the buffer to be supplied to the driver.
 * @note    The buffer is meant to be non-cacheable on platforms with
 *          data cache.
 */
#define MMC_BUFFER_SIZE                 16U

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    MMC_SPI configuration options
 * @{
 */
/**
 * @brief   Timeout before assuming a failure while waiting for card idle.
 * @note    Time is in milliseconds.
 */
#if !defined(MMC_IDLE_TIMEOUT_MS) || defined(__DOXYGEN__)
#define MMC_IDLE_TIMEOUT_MS             1000
#endif

/**
 * @brief   Mutual exclusion on the SPI bus.
 */
#if !defined(MMC_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define MMC_USE_MUTUAL_EXCLUSION        TRUE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (HAL_USE_SPI == FALSE) || (SPI_USE_WAIT == FALSE)
#error "MMC_SPI driver requires HAL_USE_SPI and SPI_USE_WAIT"
#endif

#if (MMC_USE_MUTUAL_EXCLUSION == TRUE) && (SPI_USE_MUTUAL_EXCLUSION == FALSE)
#error "MMC_USE_MUTUAL_EXCLUSION requires SPI_USE_MUTUAL_EXCLUSION"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a MMC/SD over SPI driver configuration structure.
 */
typedef struct {
  /**
   * @brief SPI driver associated to this MMC driver.
   */
  SPIDriver                             *spip;
  /**
   * @brief SPI low speed configuration used during initialization.
   */
  const SPIConfig                       *lscfg;
  /**
   * @brief SPI high speed configuration used during transfers.
   */
  const SPIConfig                       *hscfg;
} mmc_spi_config_t;

/**
 * @brief   Legacy name for compatibility.
 * @deprecated
 */
typedef mmc_spi_config_t MMCConfig;

/**
 * @brief   @p MMCDriver specific methods.
 */
#define __mmc_driver_methods                                                \
  _mmcsd_block_device_methods

/**
 * @extends MMCSDBlockDeviceVMT
 *
 * @brief   @p MMCDriver virtual methods table.
 */
struct mmc_spi_driver_vmt {
  __mmc_driver_methods
};

/**
 * @extends MMCSDBlockDevice
 *
 * @brief   Structure representing a MMC/SD over SPI driver.
 */
typedef struct {
  /**
   * @brief   Virtual Methods Table.
   */
  const struct mmc_spi_driver_vmt       *vmt;
  _mmcsd_block_device_data
  /**
   * @brief   Current configuration data.
   */
  const mmc_spi_config_t                *config;
  /**
   * @brief   Addresses use blocks instead of bytes.
   */
  bool                                  block_addresses;
  /**
   * @brief   Pointer to an un-cacheable buffer of size @p MMC_BUFFER_SIZE.
   */
  uint8_t                               *buffer;
} mmc_spi_driver_t;

/**
 * @brief   Legacy name for compatibility.
 * @deprecated
 */
typedef mmc_spi_driver_t MMCDriver;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Returns the card insertion status.
 * @note    This macro wraps a low level function named
 *          @p mmc_lld_is_card_inserted(), this function must be
 *          provided by the application because it is not part of the
 *          MMC_SPI driver.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @return              The card state.
 * @retval false        card not inserted.
 * @retval true         card inserted.
 *
 * @api
 */
#define mmcIsCardInserted(mmcp) mmc_lld_is_card_inserted(mmcp)

/**
 * @brief   Returns the write protect status.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @return              The write protect status.
 * @retval false        card not write protected.
 * @retval true         card write protected.
 *
 * @api
 */
#define mmcIsWriteProtected(mmcp) mmc_lld_is_write_protected(mmcp)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void mmcInit(void);
  void mmcObjectInit(MMCDriver *mmcp, uint8_t *buffer);
  msg_t mmcStart(MMCDriver *mmcp, const MMCConfig *config);
  void mmcStop(MMCDriver *mmcp);
  bool mmcConnect(MMCDriver *mmcp);
  bool mmcDisconnect(MMCDriver *mmcp);
  bool mmcStartSequentialRead(MMCDriver *mmcp, uint32_t startblk);
  bool mmcSequentialRead(MMCDriver *mmcp, uint8_t *buffer);
  bool mmcStopSequentialRead(MMCDriver *mmcp);
  bool mmcStartSequentialWrite(MMCDriver *mmcp, uint32_t startblk);
  bool mmcSequentialWrite(MMCDriver *mmcp, const uint8_t *buffer);
  bool mmcStopSequentialWrite(MMCDriver *mmcp);
  bool mmcSync(MMCDriver *mmcp);
  bool mmcGetInfo(MMCDriver *mmcp, BlockDeviceInfo *bdip);
  bool mmcErase(MMCDriver *mmcp, uint32_t startblk, uint32_t endblk);
  bool mmc_lld_is_card_inserted(MMCDriver *mmcp);
  bool mmc_lld_is_write_protected(MMCDriver *mmcp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_MMC_SPI == TRUE */

#endif /* HAL_MMC_SPI_H */

/** @} */
