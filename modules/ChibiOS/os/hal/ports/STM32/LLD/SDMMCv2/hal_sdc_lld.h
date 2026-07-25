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
 * @file    SDMMCv2/hal_sdc_lld.h
 * @brief   STM32 SDC subsystem low level driver header.
 *
 * @addtogroup SDC
 * @{
 */

#ifndef HAL_SDC_LLD_H
#define HAL_SDC_LLD_H

#if HAL_USE_SDC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   SDMMC1 driver enable switch.
 * @details If set to @p TRUE the support for SDMMC1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SDC_USE_SDMMC1) || defined(__DOXYGEN__)
#define STM32_SDC_USE_SDMMC1                FALSE
#endif

/**
 * @brief   SDMMC2 driver enable switch.
 * @details If set to @p TRUE the support for SDMMC2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SDC_USE_SDMMC2) || defined(__DOXYGEN__)
#define STM32_SDC_USE_SDMMC2                FALSE
#endif

/**
 * @brief   Support for unaligned transfers.
 * @note    Unaligned transfers are much slower.
 */
#if !defined(STM32_SDC_SDMMC_UNALIGNED_SUPPORT) || defined(__DOXYGEN__)
#define STM32_SDC_SDMMC_UNALIGNED_SUPPORT   TRUE
#endif

/**
 * @brief   Write timeout in milliseconds.
 */
#if !defined(STM32_SDC_SDMMC_WRITE_TIMEOUT) || defined(__DOXYGEN__)
#define STM32_SDC_SDMMC_WRITE_TIMEOUT       10000
#endif

/**
 * @brief   Read timeout in milliseconds.
 */
#if !defined(STM32_SDC_SDMMC_READ_TIMEOUT) || defined(__DOXYGEN__)
#define STM32_SDC_SDMMC_READ_TIMEOUT        10000
#endif

/**
 * @brief   Card clock activation delay in milliseconds.
 */
#if !defined(STM32_SDC_SDMMC_CLOCK_DELAY) || defined(__DOXYGEN__)
#define STM32_SDC_SDMMC_CLOCK_DELAY         10
#endif

/**
 * @brief   Card clock power saving enable.
 */
#if !defined(STM32_SDC_SDMMC_PWRSAV) || defined(__DOXYGEN__)
#define STM32_SDC_SDMMC_PWRSAV              TRUE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks.*/
#if (STM32_SDC_USE_SDMMC1 && !defined(STM32_SDMMC1_HANDLER)) ||             \
    (STM32_SDC_USE_SDMMC2 && !defined(STM32_SDMMC2_HANDLER))
#error "STM32_SDMMCx_HANDLER not defined in registry"
#endif

#if (STM32_SDC_USE_SDMMC1 && !defined(STM32_SDMMC1_NUMBER)) ||              \
    (STM32_SDC_USE_SDMMC2 && !defined(STM32_SDMMC2_NUMBER))
#error "STM32_SDMMCx_NUMBER not defined in registry"
#endif

/* Units checks.*/
#if STM32_SDC_USE_SDMMC1 && !STM32_HAS_SDMMC1
#error "SDMMC1 not present in the selected device"
#endif

#if STM32_SDC_USE_SDMMC2 && !STM32_HAS_SDMMC2
#error "SDMMC2 not present in the selected device"
#endif

#if !STM32_SDC_USE_SDMMC1 && !STM32_SDC_USE_SDMMC2
#error "SDC driver activated but no SDMMC peripheral assigned"
#endif

/* Clock related tests.*/
#if STM32_HAS_SDMMC1 && !defined(STM32_SDMMC1CLK)
#error "STM32_SDMMC1CLK not defined"
#endif

/* Clock related tests.*/
#if STM32_HAS_SDMMC2 && !defined(STM32_SDMMC2CLK)
#error "STM32_SDMMC2CLK not defined"
#endif

#if !defined(STM32_HCLK)
#error "STM32_HCLK not defined"
#endif

// No more a thing, now it is required for the AHB bus to have, at least,
// 3 times the SDMMC bandwidth, that cannot be statically checked.
//#if STM32_HAS_SDMMC1 && (STM32_SDMMC1CLK * 10 > STM32_HCLK * 7)
//#error "STM32_SDMMC1CLK must not exceed STM32_HCLK * 0.7"
//#endif

//#if STM32_HAS_SDMMC2 && (STM32_SDMMC2CLK * 10 > STM32_HCLK * 7)
//#error "STM32_SDMMC2CLK must not exceed STM32_HCLK * 0.7"
//#endif

#if !defined(STM32_SDMMC_MAXCLK)
#error "STM32_SDMMC_MAXCLK not defined in stm32_limits.h"
#endif

#if STM32_HAS_SDMMC1 && (STM32_SDMMC1CLK > STM32_SDMMC_MAXCLK)
#error "STM32_SDMMC1CLK must not exceed STM32_SDMMC_MAXCLK"
#endif

#if STM32_HAS_SDMMC2 && (STM32_SDMMC2CLK > STM32_SDMMC_MAXCLK)
#error "STM32_SDMMC2CLK must not exceed STM32_SDMMC_MAXCLK"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of card flags.
 */
typedef uint32_t sdcmode_t;

/**
 * @brief   SDC Driver condition flags type.
 */
typedef uint32_t sdcflags_t;

/**
 * @brief   Type of a structure representing an SDC driver.
 */
typedef struct SDCDriver SDCDriver;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Bus width.
   */
  sdcbusmode_t  bus_width;
  /* End of the mandatory fields.*/
  /**
   * @brief   Bus slowdown.
   * @note    This values is added to the prescaler register in order to
   *          arbitrarily reduce clock speed.
   */
  uint32_t      slowdown;
} SDCConfig;

/**
 * @brief   @p SDCDriver specific methods.
 */
#define _sdc_driver_methods                                                 \
  _mmcsd_block_device_methods

/**
 * @extends MMCSDBlockDeviceVMT
 *
 * @brief   @p SDCDriver virtual methods table.
 */
struct SDCDriverVMT {
  _sdc_driver_methods
};

/**
 * @brief   Structure representing an SDC driver.
 */
struct SDCDriver {
  /**
   * @brief   Virtual Methods Table.
   */
  const struct SDCDriverVMT *vmt;
  _mmcsd_block_device_data
  /**
   * @brief   Current configuration data.
   */
  const SDCConfig           *config;
  /**
   * @brief   Various flags regarding the mounted card.
   */
  sdcmode_t                 cardmode;
  /**
   * @brief   Errors flags.
   */
  sdcflags_t                errors;
  /**
   * @brief   Card RCA.
   */
  uint32_t                  rca;
  /**
   * @brief   Buffer of @p MMCSD_BLOCK_SIZE bytes for internal operations.
   */
  uint8_t                   *buf;
  /* End of the mandatory fields.*/
  /**
   * @brief   Thread waiting for I/O completion IRQ.
   */
  thread_reference_t        thread;
  /**
   * @brief   Pointer to the SDMMC registers block.
   * @note    Needed for debugging aid.
   */
  SDMMC_TypeDef             *sdmmc;
  /**
   * @brief   Input clock frequency.
   */
  uint32_t                  clkfreq;
  /**
   * @brief   Uncached word buffer for small transfers.
   */
  uint32_t                  *resp;

  // bouncebuffer to support DMA to all memory regions
  struct bouncebuffer_t *bouncebuffer;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_SDC_USE_SDMMC1 && !defined(__DOXYGEN__)
extern SDCDriver SDCD1;
#endif

#if STM32_SDC_USE_SDMMC2 && !defined(__DOXYGEN__)
extern SDCDriver SDCD2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sdc_lld_init(void);
  void sdc_lld_start(SDCDriver *sdcp);
  void sdc_lld_stop(SDCDriver *sdcp);
  void sdc_lld_start_clk(SDCDriver *sdcp);
  void sdc_lld_set_data_clk(SDCDriver *sdcp, sdcbusclk_t clk);
  void sdc_lld_stop_clk(SDCDriver *sdcp);
  void sdc_lld_set_bus_mode(SDCDriver *sdcp, sdcbusmode_t mode);
  void sdc_lld_send_cmd_none(SDCDriver *sdcp, uint8_t cmd, uint32_t arg);
  bool sdc_lld_send_cmd_short(SDCDriver *sdcp, uint8_t cmd, uint32_t arg,
                              uint32_t *resp);
  bool sdc_lld_send_cmd_short_crc(SDCDriver *sdcp, uint8_t cmd, uint32_t arg,
                                  uint32_t *resp);
  bool sdc_lld_send_cmd_long_crc(SDCDriver *sdcp, uint8_t cmd, uint32_t arg,
                                 uint32_t *resp);
  bool sdc_lld_read_special(SDCDriver *sdcp, uint8_t *buf, size_t bytes,
                            uint8_t cmd, uint32_t argument);
  bool sdc_lld_read(SDCDriver *sdcp, uint32_t startblk,
                    uint8_t *buf, uint32_t blocks);
  bool sdc_lld_write(SDCDriver *sdcp, uint32_t startblk,
                     const uint8_t *buf, uint32_t blocks);
  bool sdc_lld_sync(SDCDriver *sdcp);
  bool sdc_lld_is_card_inserted(SDCDriver *sdcp);
  bool sdc_lld_is_write_protected(SDCDriver *sdcp);
  void sdc_lld_serve_interrupt(SDCDriver *sdcp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SDC */

#endif /* HAL_SDC_LLD_H */

/** @} */
