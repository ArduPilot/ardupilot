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
 * @file    hal_sdc.h
 * @brief   SDC Driver macros and structures.
 *
 * @addtogroup SDC
 * @{
 */

#ifndef HAL_SDC_H
#define HAL_SDC_H

#if (HAL_USE_SDC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    SD card types
 * @{
 */
#define SDC_MODE_CARDTYPE_MASK              0xFU
#define SDC_MODE_CARDTYPE_SDV11             0U
#define SDC_MODE_CARDTYPE_SDV20             1U
#define SDC_MODE_CARDTYPE_MMC               2U
#define SDC_MODE_HIGH_CAPACITY              0x10U
/** @} */

/**
 * @name    SDC bus error conditions
 * @{
 */
#define SDC_NO_ERROR                        0U
#define SDC_CMD_CRC_ERROR                   1U
#define SDC_DATA_CRC_ERROR                  2U
#define SDC_DATA_TIMEOUT                    4U
#define SDC_COMMAND_TIMEOUT                 8U
#define SDC_TX_UNDERRUN                     16U
#define SDC_RX_OVERRUN                      32U
#define SDC_STARTBIT_ERROR                  64U
#define SDC_OVERFLOW_ERROR                  128U
#define SDC_UNHANDLED_ERROR                 0xFFFFFFFFU
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    SDC configuration options
 * @{
 */
/**
 * @brief   Number of initialization attempts before rejecting the card.
 * @note    Attempts are performed at 10mS intervals.
 */
#if !defined(SDC_INIT_RETRY) || defined(__DOXYGEN__)
#define SDC_INIT_RETRY                      100
#endif

/**
 * @brief   Include support for MMC cards.
 * @note    MMC support is not yet implemented so this option must be kept
 *          at @p FALSE.
 */
#if !defined(SDC_MMC_SUPPORT) || defined(__DOXYGEN__)
#define SDC_MMC_SUPPORT                     FALSE
#endif

/**
 * @brief   Delays insertions.
 * @details If enabled this options inserts delays into the MMC waiting
 *          routines releasing some extra CPU time for the threads with
 *          lower priority, this may slow down the driver a bit however.
 */
#if !defined(SDC_NICE_WAITING) || defined(__DOXYGEN__)
#define SDC_NICE_WAITING                    TRUE
#endif

/**
 * @brief   OCR initialization constant for V20 cards.
 */
#if !defined(SDC_INIT_OCR_V20) || defined(__DOXYGEN__)
#define SDC_INIT_OCR_V20                    0x50FF8000U
#endif

/**
 * @brief   OCR initialization constant for non-V20 cards.
 */
#if !defined(SDC_INIT_OCR) || defined(__DOXYGEN__)
#define SDC_INIT_OCR                        0x80100000U
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of SDIO bus mode.
 */
typedef enum {
  SDC_MODE_1BIT = 0,
  SDC_MODE_4BIT,
  SDC_MODE_8BIT
} sdcbusmode_t;

/**
 * @brief   Max supported clock.
 */
typedef enum {
  SDC_CLK_25MHz = 0,
  SDC_CLK_50MHz
} sdcbusclk_t;

#include "hal_sdc_lld.h"

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
 *          @p sdc_lld_is_card_inserted(), this function must be
 *          provided by the application because it is not part of the
 *          SDC driver.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @return              The card state.
 * @retval false        card not inserted.
 * @retval true         card inserted.
 *
 * @api
 */
#define sdcIsCardInserted(sdcp) (sdc_lld_is_card_inserted(sdcp))

/**
 * @brief   Returns the write protect status.
 * @note    This macro wraps a low level function named
 *          @p sdc_lld_is_write_protected(), this function must be
 *          provided by the application because it is not part of the
 *          SDC driver.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @return              The card state.
 * @retval false        not write protected.
 * @retval true         write protected.
 *
 * @api
 */
#define sdcIsWriteProtected(sdcp) (sdc_lld_is_write_protected(sdcp))
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void sdcInit(void);
  void sdcObjectInit(SDCDriver *sdcp);
  msg_t sdcStart(SDCDriver *sdcp, const SDCConfig *config);
  void sdcStop(SDCDriver *sdcp);
  bool sdcConnect(SDCDriver *sdcp);
  bool sdcDisconnect(SDCDriver *sdcp);
  bool sdcRead(SDCDriver *sdcp, uint32_t startblk,
               uint8_t *buf, uint32_t n);
  bool sdcWrite(SDCDriver *sdcp, uint32_t startblk,
                const uint8_t *buf, uint32_t n);
  sdcflags_t sdcGetAndClearErrors(SDCDriver *sdcp);
  bool sdcSync(SDCDriver *sdcp);
  bool sdcGetInfo(SDCDriver *sdcp, BlockDeviceInfo *bdip);
  bool sdcErase(SDCDriver *sdcp, uint32_t startblk, uint32_t endblk);
  bool _sdc_wait_for_transfer_state(SDCDriver *sdcp);
  bool _sdc_wait_for_transfer_state_nocrc(SDCDriver *sdcp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SDC == TRUE */

#endif /* HAL_SDC_H */

/** @} */
