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
 * @file    OCTOSPIv1/hal_wspi_lld.h
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
#define WSPI_DEFAULT_CFG_MASKS              TRUE
/** @} */

/**
 * @name    DCR1 register options
 * @{
 */
#define STM32_DCR1_CK_MODE                  (1U << 0U)
#define STM32_DCR1_FRCK_MODE                (1U << 1U)
#define STM32_DCR1_CSHT_MASK                (7U << 8U)
#define STM32_DCR1_CSHT(n)                  ((n) << 8U)
#define STM32_DCR1_DEVSIZE_MASK             (31U << 16U)
#define STM32_DCR1_DEVSIZE(n)               ((n) << 16U)
#define STM32_DCR1_MTYP_MASK                (7U << 16U)
#define STM32_DCR1_MTYP(n)                  ((n) << 24U)
/** @} */

/**
 * @name    DCR2 register options
 * @{
 */
#define STM32_DCR2_PRESCALER_MASK           (255U << 0U)
#define STM32_DCR2_PRESCALER(n)             ((n) << 0U)
#define STM32_DCR2_WRAPSIZE_MASK            (7U << 16U)
#define STM32_DCR2_WRAPSIZE(n)              ((n) << 16U)

/**
 * @name    DCR3 register options
 * @{
 */
#define STM32_DCR3_MAXTRAN_MASK             (255U << 0U)
#define STM32_DCR3_MAXTRAN(n)               ((n) << 0U)
#define STM32_DCR3_CSBOUND_MASK             (7U << 16U)
#define STM32_DCR3_CSBOUND(n)               ((n) << 16U)

/**
 * @name    DCR4 register options
 * @{
 */
#define STM32_DCR4_REFRESH_MASK             (255U << 0U)
#define STM32_DCR4_REFRESH(n)               ((n) << 0U)
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
 * @details If set to @p TRUE the support for OCTOSPI1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_WSPI_USE_OCTOSPI1) || defined(__DOXYGEN__)
#define STM32_WSPI_USE_OCTOSPI1             FALSE
#endif

/**
 * @brief   WSPID2 driver enable switch.
 * @details If set to @p TRUE the support for OCTOSPI2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_WSPI_USE_OCTOSPI2) || defined(__DOXYGEN__)
#define STM32_WSPI_USE_OCTOSPI2             FALSE
#endif

/**
 * @brief   OCTOSPI1 prescaler setting.
 * @note    This is the prescaler divider value 1..256. The maximum frequency
 *          varies depending on the STM32 model and operating conditions,
 *          find the details in the data sheet.
 */
#if !defined(STM32_WSPI_OCTOSPI1_PRESCALER_VALUE) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI1_PRESCALER_VALUE 1
#endif

/**
 * @brief   OCTOSPI2 prescaler setting.
 * @note    This is the prescaler divider value 1..256. The maximum frequency
 *          varies depending on the STM32 model and operating conditions,
 *          find the details in the data sheet.
 */
#if !defined(STM32_WSPI_OCTOSPI2_PRESCALER_VALUE) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI2_PRESCALER_VALUE 1
#endif

/**
 * @brief   OCTOSPI1 TCR_SSHIFT enforcing.
 */
#if !defined(STM32_WSPI_OCTOSPI1_SSHIFT) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI1_SSHIFT          FALSE
#endif

/**
 * @brief   OCTOSPI2 TCR_SSHIFT enforcing.
 */
#if !defined(STM32_WSPI_OCTOSPI2_SSHIFT) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI2_SSHIFT          FALSE
#endif

/**
 * @brief   OCTOSPI1 TCR_DHQC enforcing.
 */
#if !defined(STM32_WSPI_OCTOSPI1_DHQC) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI1_DHQC            FALSE
#endif

/**
 * @brief   OCTOSPI2 TCR_DHQC enforcing.
 */
#if !defined(STM32_WSPI_OCTOSPI2_DHQC) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI2_DHQC            FALSE
#endif

/**
 * @brief   OCTOSPI1 interrupt priority level setting.
 */
#if !defined(STM32_WSPI_OCTOSPI1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI1_IRQ_PRIORITY    10
#endif

/**
 * @brief   OCTOSPI2 interrupt priority level setting.
 */
#if !defined(STM32_WSPI_OCTOSPI2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI2_IRQ_PRIORITY    10
#endif

/**
 * @brief   OCTOSPI1 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_WSPI_OCTOSPI1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI1_DMA_PRIORITY    1
#endif

/**
 * @brief   OCTOSPI2 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_WSPI_OCTOSPI2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI2_DMA_PRIORITY    1
#endif

/**
 * @brief   OCTOSPI1 DMA interrupt priority level setting.
 */
#if !defined(STM32_WSPI_OCTOSPI1_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI1_DMA_IRQ_PRIORITY 10
#endif

/**
 * @brief   OCTOSPI2 DMA interrupt priority level setting.
 */
#if !defined(STM32_WSPI_OCTOSPI2_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_WSPI_OCTOSPI2_DMA_IRQ_PRIORITY 10
#endif

/**
 * @brief   OCTOSPI DMA error hook.
 */
#if !defined(STM32_WSPI_DMA_ERROR_HOOK) || defined(__DOXYGEN__)
#define STM32_WSPI_DMA_ERROR_HOOK(wspip)    osalSysHalt("DMA failure")
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(STM32_HAS_OCTOSPI1)
#define STM32_HAS_OCTOSPI1                  FALSE
#endif

#if !defined(STM32_HAS_OCTOSPI2)
#define STM32_HAS_OCTOSPI2                  FALSE
#endif

#if STM32_WSPI_USE_OCTOSPI1 && !STM32_HAS_OCTOSPI1
#error "OCTOSPI1 not present in the selected device"
#endif

#if STM32_WSPI_USE_OCTOSPI2 && !STM32_HAS_OCTOSPI2
#error "OCTOSPI2 not present in the selected device"
#endif

#if !STM32_WSPI_USE_OCTOSPI1 && !STM32_WSPI_USE_OCTOSPI2
#error "WSPI driver activated but no OCTOSPI peripheral assigned"
#endif

/* Check on OCTOSPI prescaler setting.*/
#if (STM32_WSPI_OCTOSPI1_PRESCALER_VALUE < 1) ||                            \
    (STM32_WSPI_OCTOSPI1_PRESCALER_VALUE > 256)
#error "STM32_WSPI_OCTOSPI1_PRESCALER_VALUE not within 1..256"
#endif

#if (STM32_WSPI_OCTOSPI2_PRESCALER_VALUE < 1) ||                            \
    (STM32_WSPI_OCTOSPI2_PRESCALER_VALUE > 256)
#error "STM32_WSPI_OCTOSPI2_PRESCALER_VALUE not within 1..256"
#endif

/* Check on IRQ priorities.*/
#if STM32_WSPI_USE_OCTOSPI1 &&                                              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_WSPI_OCTOSPI1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to OCTOSPI1"
#endif

#if STM32_WSPI_USE_OCTOSPI2 &&                                              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_WSPI_OCTOSPI2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to OCTOSPI2"
#endif

#if STM32_WSPI_USE_OCTOSPI1 &&                                              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_WSPI_OCTOSPI1_DMA_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to OCTOSPI1 DMA"
#endif

#if STM32_WSPI_USE_OCTOSPI2 &&                                              \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_WSPI_OCTOSPI2_DMA_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to OCTOSPI2 DMA"
#endif

/* Check on the presence of the DMA channels settings in mcuconf.h.*/
#if STM32_WSPI_USE_OCTOSPI1 && !defined(STM32_WSPI_OCTOSPI1_DMA_STREAM)
#error "OCTOSPI1 DMA stream not defined"
#endif

#if STM32_WSPI_USE_OCTOSPI2 && !defined(STM32_WSPI_OCTOSPI2_DMA_STREAM)
#error "OCTOSPI2 DMA stream not defined"
#endif

/* Check on the validity of the assigned DMA channels.*/
#if STM32_WSPI_USE_OCTOSPI1 &&                                              \
    !STM32_DMA_IS_VALID_STREAM(STM32_WSPI_OCTOSPI1_DMA_STREAM)
#error "invalid DMA stream associated to OCTOSPI1"
#endif

#if STM32_WSPI_USE_OCTOSPI2 &&                                              \
    !STM32_DMA_IS_VALID_STREAM(STM32_WSPI_OCTOSPI2_DMA_STREAM)
#error "invalid DMA stream associated to OCTOSPI2"
#endif

/* Check on DMA channels priority.*/
#if STM32_WSPI_USE_OCTOSPI1 &&                                              \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_WSPI_OCTOSPI1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to OCTOSPI1"
#endif

#if STM32_WSPI_USE_OCTOSPI2 &&                                              \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_WSPI_OCTOSPI2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to OCTOSPI2"
#endif

#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
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
  /* DCR1 register initialization data.*/                                   \
  uint32_t                  dcr1;                                           \
  /* DCR2 register initialization data. The prescaler field is internally   \
     ORed to this field, leave it to zero.*/                                \
  uint32_t                  dcr2;                                           \
  /* DCR3 register initialization data.*/                                   \
  uint32_t                  dcr3;                                           \
  /* DCR4 register initialization data.*/                                   \
  uint32_t                  dcr4;

/**
 * @brief   Low level fields of the WSPI driver structure.
 */
#define wspi_lld_driver_fields                                              \
  /* Extra bits for the TCR register.*/                                     \
  uint32_t                      extra_tcr;                                  \
  /* Pointer to the OCTOSPIx registers block.*/                             \
  OCTOSPI_TypeDef           *ospi;                                          \
  /* OCTOSPI DMA stream.*/                                                  \
  const stm32_dma_stream_t  *dma;                                           \
  /* OCTOSPI DMA mode bit mask.*/                                           \
  uint32_t                  dmamode;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (STM32_WSPI_USE_OCTOSPI1 == TRUE) && !defined(__DOXYGEN__)
extern WSPIDriver WSPID1;
#endif

#if (STM32_WSPI_USE_OCTOSPI2 == TRUE) && !defined(__DOXYGEN__)
extern WSPIDriver WSPID2;
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
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_WSPI */

#endif /* HAL_WSPI_LLD_H */

/** @} */
