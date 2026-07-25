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
 * @file    QUADSPIv2/hal_wspi_lld.h
 * @brief   STM32 WSPI subsystem low level driver header.
 *
 * @addtogroup WSPI
 * @{
 */

#ifndef HAL_WSPI_LLD_H
#define HAL_WSPI_LLD_H

#if (HAL_USE_WSPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    WSPI implementation capabilities
 * @{
 */
#define WSPI_SUPPORTS_MEMMAP                TRUE
#define WSPI_DEFAULT_CFG_MASKS              FALSE
/** @} */

/**
 * @name    Transfer options
 * @note    The low level driver has the option to override the following
 *          definitions and use its own ones. In must take care to use
 *          the same name for the same function or compatibility is not
 *          ensured.
 * @note    There are the following limitations in this implementation:
 *          - Eight lines are not supported.
 *          - DDR mode is only supported for the whole command, separate
 *            masks are defined but all define the same bit.
 *          - Only 8 bits instructions are supported.
 *          .
 * @{
 */
#define WSPI_CFG_CMD_MODE_MASK              (3U << 8)
#define WSPI_CFG_CMD_MODE_NONE              (0U << 8)
#define WSPI_CFG_CMD_MODE_ONE_LINE          (1U << 8)
#define WSPI_CFG_CMD_MODE_TWO_LINES         (2U << 8)
#define WSPI_CFG_CMD_MODE_FOUR_LINES        (3U << 8)
#define WSPI_CFG_CMD_MODE_EIGHT_LINES       0U

#define WSPI_CFG_CMD_DTR                    (1U << 31)

#define WSPI_CFG_CMD_SIZE_MASK              0U
#define WSPI_CFG_CMD_SIZE_8                 0U
#define WSPI_CFG_CMD_SIZE_16                0U          /* Fake.*/
#define WSPI_CFG_CMD_SIZE_24                0U          /* Fake.*/
#define WSPI_CFG_CMD_SIZE_32                0U          /* Fake.*/

#define WSPI_CFG_ADDR_MODE_MASK             (3U << 10)
#define WSPI_CFG_ADDR_MODE_NONE             (0U << 10)
#define WSPI_CFG_ADDR_MODE_ONE_LINE         (1U << 10)
#define WSPI_CFG_ADDR_MODE_TWO_LINES        (2U << 10)
#define WSPI_CFG_ADDR_MODE_FOUR_LINES       (3U << 10)
#define WSPI_CFG_ADDR_MODE_EIGHT_LINES      0U          /* Fake.*/

#define WSPI_CFG_ADDR_DTR                   (1U << 31)

#define WSPI_CFG_ADDR_SIZE_MASK             (3U << 12)
#define WSPI_CFG_ADDR_SIZE_8                (0U << 12)
#define WSPI_CFG_ADDR_SIZE_16               (1U << 12)
#define WSPI_CFG_ADDR_SIZE_24               (2U << 12)
#define WSPI_CFG_ADDR_SIZE_32               (3U << 12)

#define WSPI_CFG_ALT_MODE_MASK              (3U << 14)
#define WSPI_CFG_ALT_MODE_NONE              (0U << 14)
#define WSPI_CFG_ALT_MODE_ONE_LINE          (1U << 14)
#define WSPI_CFG_ALT_MODE_TWO_LINES         (2U << 14)
#define WSPI_CFG_ALT_MODE_FOUR_LINES        (3U << 14)
#define WSPI_CFG_ALT_MODE_EIGHT_LINES       0U          /* Fake.*/

#define WSPI_CFG_ALT_DTR                    (1U << 31)

#define WSPI_CFG_ALT_SIZE_MASK              (3U << 16)
#define WSPI_CFG_ALT_SIZE_8                 (0U << 16)
#define WSPI_CFG_ALT_SIZE_16                (1U << 16)
#define WSPI_CFG_ALT_SIZE_24                (2U << 16)
#define WSPI_CFG_ALT_SIZE_32                (3U << 16)

#define WSPI_CFG_DATA_MODE_MASK             (3U << 24)
#define WSPI_CFG_DATA_MODE_NONE             (0U << 24)
#define WSPI_CFG_DATA_MODE_ONE_LINE         (1U << 24)
#define WSPI_CFG_DATA_MODE_TWO_LINES        (2U << 24)
#define WSPI_CFG_DATA_MODE_FOUR_LINES       (3U << 24)
#define WSPI_CFG_DATA_MODE_EIGHT_LINES      0U          /* Fake.*/

#define WSPI_CFG_DATA_DTR                   (1U << 31)

#define WSPI_CFG_DQS_ENABLE                 0U          /* Fake.*/

#define WSPI_CFG_SIOO                       (1U << 28)

#define WSPI_CFG_ALL_DTR                    (WSPI_CFG_CMD_DTR   |           \
                                             WSPI_CFG_ADDR_DTR  |           \
                                             WSPI_CFG_ALT_DTR   |           \
                                             WSPI_CFG_DATA_DTR)
/** @} */

/**
 * @name    Helpers for CCR register.
 * @{
 */
#define QUADSPI_CCR_DUMMY_CYCLES_MASK       (0x1FU << 18)
#define QUADSPI_CCR_DUMMY_CYCLES(n)         ((n) << 18)
/** @} */

/**
 * @name    DCR register options
 * @{
 */
#define STM32_DCR_CK_MODE                   (1U << 0)
#define STM32_DCR_CSHT_MASK                 (7U << 8)
#define STM32_DCR_CSHT(n)                   ((n) << 8)
#define STM32_DCR_FSIZE_MASK                (31U << 16)
#define STM32_DCR_FSIZE(n)                  ((n) << 16)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   WSPID1 driver enable switch.
 * @details If set to @p TRUE the support for QUADSPI1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_WSPI_USE_QUADSPI1) || defined(__DOXYGEN__)
#define STM32_WSPI_USE_QUADSPI1             FALSE
#endif

/**
 * @brief   QUADSPI1 prescaler setting.
 * @note    This is the prescaler divider value 1..256. The maximum frequency
 *          varies depending on the STM32 model and operating conditions,
 *          find the details in the data sheet.
 */
#if !defined(STM32_WSPI_QUADSPI1_PRESCALER_VALUE) || defined(__DOXYGEN__)
#define STM32_WSPI_QUADSPI1_PRESCALER_VALUE 1
#endif

/**
 * @brief   QUADSPI1 CR_SSHIFT enforcing.
 */
#if !defined(STM32_WSPI_SET_CR_SSHIFT) || defined(__DOXYGEN__)
#define STM32_WSPI_SET_CR_SSHIFT            TRUE
#endif

/**
 * @brief   QUADSPI1 MDMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_WSPI_QUADSPI1_MDMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_WSPI_QUADSPI1_MDMA_PRIORITY   1
#endif

/**
 * @brief   QUADSPI MDMA error hook.
 */
#if !defined(STM32_WSPI_MDMA_ERROR_HOOK) || defined(__DOXYGEN__)
#define STM32_WSPI_MDMA_ERROR_HOOK(wspip)   osalSysHalt("MDMA failure")
#endif

/**
 * @brief   Enables a workaround for a STM32L476 QUADSPI errata.
 * @details The document DM00111498 states: "QUADSPI_BK1_IO1 is always an
 *          input when the command is sent in dual or quad SPI mode".
 *          This workaround makes commands without address or data phases
 *          to be sent as alternate bytes.
 */
#if !defined(STM32_USE_STM32_D1_WORKAROUND) || defined(__DOXYGEN__)
#define STM32_USE_STM32_D1_WORKAROUND       TRUE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(STM32_HAS_QUADSPI1)
#define STM32_HAS_QUADSPI1                  FALSE
#endif

#if STM32_WSPI_USE_QUADSPI1 && !STM32_HAS_QUADSPI1
#error "QUADSPI1 not present in the selected device"
#endif

#if !STM32_WSPI_USE_QUADSPI1
#error "WSPI driver activated but no QUADSPI peripheral assigned"
#endif

/* MDMA-related checks.*/
#if STM32_WSPI_USE_QUADSPI1 &&                                              \
    !STM32_MDMA_IS_VALID_PRIORITY(STM32_WSPI_QUADSPI1_MDMA_PRIORITY)
#error "Invalid MDMA priority assigned to QUADSPI1"
#endif

/* Checks on prescaler setting.*/
#if (STM32_WSPI_QUADSPI1_PRESCALER_VALUE < 1) ||                            \
    (STM32_WSPI_QUADSPI1_PRESCALER_VALUE > 256)
#error "STM32_WSPI_QUADSPI1_PRESCALER_VALUE not within 1..256"
#endif

/* Check on the presence of the DMA channels settings in mcuconf.h.*/
#if STM32_WSPI_USE_QUADSPI1 && !defined(STM32_WSPI_QUADSPI1_MDMA_CHANNEL)
#error "QUADSPI1 MDMA channel not defined"
#endif

/* Check on the validity of the assigned DMA channels.*/
#if STM32_WSPI_USE_QUADSPI1 &&                                              \
    !STM32_MDMA_IS_VALID_CHANNEL(STM32_WSPI_QUADSPI1_MDMA_CHANNEL)
#error "invalid MDMA channel associated to QUADSPI1"
#endif

#if !defined(STM32_MDMA_REQUIRED)
#define STM32_MDMA_REQUIRED
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the WSPI configuration structure.
 */
#define wspi_lld_config_fields                                              \
  /* DCR register initialization data.*/                                    \
  uint32_t                      dcr;

/**
 * @brief   Low level fields of the WSPI driver structure.
 */
#define wspi_lld_driver_fields                                              \
  /* Pointer to the QUADSPIx registers block.*/                             \
  QUADSPI_TypeDef               *qspi;                                      \
  /* QUADSPI MDMA channel.*/                                                \
  const stm32_mdma_channel_t    *mdma;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (STM32_WSPI_USE_QUADSPI1 == TRUE) && !defined(__DOXYGEN__)
extern WSPIDriver WSPID1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void wspi_lld_init(void);
  void wspi_lld_start(WSPIDriver *wspip);
  void wspi_lld_stop(WSPIDriver *wspip);
  void wspi_lld_command(WSPIDriver *wspip, const wspi_command_t *cmdp);
  void wspi_lld_send(WSPIDriver *wspip, const wspi_command_t *cmdp,
                     size_t n, const uint8_t *txbuf);
  void wspi_lld_receive(WSPIDriver *wspip, const wspi_command_t *cmdp,
                        size_t n, uint8_t *rxbuf);
#if WSPI_SUPPORTS_MEMMAP == TRUE
  void wspi_lld_map_flash(WSPIDriver *wspip,
                          const wspi_command_t *cmdp,
                          uint8_t **addrp);
  void wspi_lld_unmap_flash(WSPIDriver *wspip);
#endif
  void wspi_lld_serve_interrupt(WSPIDriver *wspip);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_WSPI */

#endif /* HAL_WSPI_LLD_H */

/** @} */
