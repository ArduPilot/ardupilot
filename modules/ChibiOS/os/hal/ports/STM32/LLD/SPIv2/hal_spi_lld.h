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
 * @file    SPIv2/hal_spi_lld.h
 * @brief   STM32 SPI subsystem low level driver header.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef HAL_SPI_LLD_H
#define HAL_SPI_LLD_H

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Circular mode support flag.
 */
#define SPI_SUPPORTS_CIRCULAR           TRUE

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   SPI1 driver enable switch.
 * @details If set to @p TRUE the support for SPI1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI1) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI1                  FALSE
#endif

/**
 * @brief   SPI2 driver enable switch.
 * @details If set to @p TRUE the support for SPI2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI2) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI2                  FALSE
#endif

/**
 * @brief   SPI3 driver enable switch.
 * @details If set to @p TRUE the support for SPI3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI3) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI3                  FALSE
#endif

/**
 * @brief   SPI4 driver enable switch.
 * @details If set to @p TRUE the support for SPI4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI4) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI4                  FALSE
#endif

/**
 * @brief   SPI5 driver enable switch.
 * @details If set to @p TRUE the support for SPI5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI5) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI5                  FALSE
#endif

/**
 * @brief   SPI6 driver enable switch.
 * @details If set to @p TRUE the support for SPI6 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI6) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI6                  FALSE
#endif

/**
 * @brief   SPI1 interrupt priority level setting.
 */
#if !defined(STM32_SPI_SPI1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI1_IRQ_PRIORITY         10
#endif

/**
 * @brief   SPI2 interrupt priority level setting.
 */
#if !defined(STM32_SPI_SPI2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI2_IRQ_PRIORITY         10
#endif

/**
 * @brief   SPI3 interrupt priority level setting.
 */
#if !defined(STM32_SPI_SPI3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI3_IRQ_PRIORITY         10
#endif

/**
 * @brief   SPI4 interrupt priority level setting.
 */
#if !defined(STM32_SPI_SPI4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI4_IRQ_PRIORITY         10
#endif

/**
 * @brief   SPI5 interrupt priority level setting.
 */
#if !defined(STM32_SPI_SPI5_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI5_IRQ_PRIORITY         10
#endif

/**
 * @brief   SPI6 interrupt priority level setting.
 */
#if !defined(STM32_SPI_SPI6_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI6_IRQ_PRIORITY         10
#endif

/**
 * @brief   SPI1 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_SPI_SPI1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI1_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI2 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_SPI_SPI2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI2_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI3 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_SPI_SPI3_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI3_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI4 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_SPI_SPI4_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI4_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI5 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_SPI_SPI5_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI5_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI6 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_SPI_SPI6_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI6_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI DMA error hook.
 */
#if !defined(STM32_SPI_DMA_ERROR_HOOK) || defined(__DOXYGEN__)
#define STM32_SPI_DMA_ERROR_HOOK(spip)      osalSysHalt("DMA failure")
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_SPI_USE_SPI1 && !STM32_HAS_SPI1
#error "SPI1 not present in the selected device"
#endif

#if STM32_SPI_USE_SPI2 && !STM32_HAS_SPI2
#error "SPI2 not present in the selected device"
#endif

#if STM32_SPI_USE_SPI3 && !STM32_HAS_SPI3
#error "SPI3 not present in the selected device"
#endif

#if STM32_SPI_USE_SPI4 && !STM32_HAS_SPI4
#error "SPI4 not present in the selected device"
#endif

#if STM32_SPI_USE_SPI5 && !STM32_HAS_SPI5
#error "SPI5 not present in the selected device"
#endif

#if STM32_SPI_USE_SPI6 && !STM32_HAS_SPI6
#error "SPI6 not present in the selected device"
#endif

#if !STM32_SPI_USE_SPI1 && !STM32_SPI_USE_SPI2 && !STM32_SPI_USE_SPI3 &&    \
    !STM32_SPI_USE_SPI4 && !STM32_SPI_USE_SPI5 && !STM32_SPI_USE_SPI6
#error "SPI driver activated but no SPI peripheral assigned"
#endif

#if STM32_SPI_USE_SPI1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_SPI_SPI1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPI1"
#endif

#if STM32_SPI_USE_SPI2 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_SPI_SPI2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPI2"
#endif

#if STM32_SPI_USE_SPI3 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_SPI_SPI3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPI3"
#endif

#if STM32_SPI_USE_SPI4 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_SPI_SPI4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPI4"
#endif

#if STM32_SPI_USE_SPI5 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_SPI_SPI5_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPI5"
#endif

#if STM32_SPI_USE_SPI6 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_SPI_SPI6_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPI6"
#endif

#if STM32_SPI_USE_SPI1 &&                                                   \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_SPI_SPI1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SPI1"
#endif

#if STM32_SPI_USE_SPI2 &&                                                   \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_SPI_SPI2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SPI2"
#endif

#if STM32_SPI_USE_SPI3 &&                                                   \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_SPI_SPI3_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SPI3"
#endif

#if STM32_SPI_USE_SPI4 &&                                                   \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_SPI_SPI4_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SPI4"
#endif

#if STM32_SPI_USE_SPI5 &&                                                   \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_SPI_SPI5_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SPI5"
#endif

#if STM32_SPI_USE_SPI6 &&                                                   \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_SPI_SPI6_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SPI6"
#endif

/* Check on the presence of the DMA streams settings in mcuconf.h.*/
#if STM32_SPI_USE_SPI1 && (!defined(STM32_SPI_SPI1_RX_DMA_STREAM) ||        \
                           !defined(STM32_SPI_SPI1_TX_DMA_STREAM))
#error "SPI1 DMA streams not defined"
#endif

#if STM32_SPI_USE_SPI2 && (!defined(STM32_SPI_SPI2_RX_DMA_STREAM) ||        \
                           !defined(STM32_SPI_SPI2_TX_DMA_STREAM))
#error "SPI2 DMA streams not defined"
#endif

#if STM32_SPI_USE_SPI3 && (!defined(STM32_SPI_SPI3_RX_DMA_STREAM) ||        \
                           !defined(STM32_SPI_SPI3_TX_DMA_STREAM))
#error "SPI3 DMA streams not defined"
#endif

#if STM32_SPI_USE_SPI4 && (!defined(STM32_SPI_SPI4_RX_DMA_STREAM) ||        \
                           !defined(STM32_SPI_SPI4_TX_DMA_STREAM))
#error "SPI4 DMA streams not defined"
#endif

#if STM32_SPI_USE_SPI5 && (!defined(STM32_SPI_SPI5_RX_DMA_STREAM) ||        \
                           !defined(STM32_SPI_SPI5_TX_DMA_STREAM))
#error "SPI5 DMA streams not defined"
#endif

#if STM32_SPI_USE_SPI6 && (!defined(STM32_SPI_SPI6_RX_DMA_STREAM) ||        \
                           !defined(STM32_SPI_SPI6_TX_DMA_STREAM))
#error "SPI6 DMA streams not defined"
#endif

/* Check on the validity of the assigned DMA channels.*/
#if STM32_SPI_USE_SPI1 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI1_RX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI1 RX"
#endif

#if STM32_SPI_USE_SPI1 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI1_TX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI1 TX"
#endif

#if STM32_SPI_USE_SPI2 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI2_RX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI2 RX"
#endif

#if STM32_SPI_USE_SPI2 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI2_TX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI2 TX"
#endif

#if STM32_SPI_USE_SPI3 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI3_RX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI3 RX"
#endif

#if STM32_SPI_USE_SPI3 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI3_TX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI3 TX"
#endif

#if STM32_SPI_USE_SPI4 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI4_RX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI4 RX"
#endif

#if STM32_SPI_USE_SPI4 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI4_TX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI4 TX"
#endif

#if STM32_SPI_USE_SPI5 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI5_RX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI5 RX"
#endif

#if STM32_SPI_USE_SPI5 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI5_TX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI5 TX"
#endif

#if STM32_SPI_USE_SPI6 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI6_RX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI6 RX"
#endif

#if STM32_SPI_USE_SPI6 &&                                                   \
    !STM32_DMA_IS_VALID_STREAM(STM32_SPI_SPI6_TX_DMA_STREAM)
#error "Invalid DMA channel assigned to SPI6 TX"
#endif

/* Devices without DMAMUX require an additional check.*/
#if STM32_ADVANCED_DMA && !STM32_DMA_SUPPORTS_DMAMUX

/* Check on the validity of the assigned DMA channels.*/
#if STM32_SPI_USE_SPI1 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI1_RX_DMA_STREAM, STM32_SPI1_RX_DMA_MSK)
#error "invalid DMA stream associated to SPI1 RX"
#endif

#if STM32_SPI_USE_SPI1 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI1_TX_DMA_STREAM, STM32_SPI1_TX_DMA_MSK)
#error "invalid DMA stream associated to SPI1 TX"
#endif

#if STM32_SPI_USE_SPI2 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI2_RX_DMA_STREAM, STM32_SPI2_RX_DMA_MSK)
#error "invalid DMA stream associated to SPI2 RX"
#endif

#if STM32_SPI_USE_SPI2 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI2_TX_DMA_STREAM, STM32_SPI2_TX_DMA_MSK)
#error "invalid DMA stream associated to SPI2 TX"
#endif

#if STM32_SPI_USE_SPI3 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI3_RX_DMA_STREAM, STM32_SPI3_RX_DMA_MSK)
#error "invalid DMA stream associated to SPI3 RX"
#endif

#if STM32_SPI_USE_SPI3 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI3_TX_DMA_STREAM, STM32_SPI3_TX_DMA_MSK)
#error "invalid DMA stream associated to SPI3 TX"
#endif

#if STM32_SPI_USE_SPI4 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI4_RX_DMA_STREAM, STM32_SPI4_RX_DMA_MSK)
#error "invalid DMA stream associated to SPI4 RX"
#endif

#if STM32_SPI_USE_SPI4 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI4_TX_DMA_STREAM, STM32_SPI4_TX_DMA_MSK)
#error "invalid DMA stream associated to SPI4 TX"
#endif

#if STM32_SPI_USE_SPI5 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI5_RX_DMA_STREAM, STM32_SPI5_RX_DMA_MSK)
#error "invalid DMA stream associated to SPI5 RX"
#endif

#if STM32_SPI_USE_SPI5 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI5_TX_DMA_STREAM, STM32_SPI5_TX_DMA_MSK)
#error "invalid DMA stream associated to SPI5 TX"
#endif

#if STM32_SPI_USE_SPI6 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI6_RX_DMA_STREAM, STM32_SPI6_RX_DMA_MSK)
#error "invalid DMA stream associated to SPI6 RX"
#endif

#if STM32_SPI_USE_SPI6 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_SPI_SPI6_TX_DMA_STREAM, STM32_SPI6_TX_DMA_MSK)
#error "invalid DMA stream associated to SPI6 TX"
#endif

#endif /* STM32_ADVANCED_DMA && !STM32_DMA_SUPPORTS_DMAMUX */

#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif

#if SPI_SELECT_MODE == SPI_SELECT_MODE_LLD
#error "SPI_SELECT_MODE_LLD not supported by this driver"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the SPI driver structure.
 */
#define spi_lld_driver_fields                                               \
  /* Pointer to the SPIx registers block.*/                                 \
  SPI_TypeDef               *spi;                                           \
  /* Receive DMA stream.*/                                                  \
  const stm32_dma_stream_t  *dmarx;                                         \
  /* Transmit DMA stream.*/                                                 \
  const stm32_dma_stream_t  *dmatx;                                         \
  /* RX DMA mode bit mask.*/                                                \
  uint32_t                  rxdmamode;                                      \
  /* TX DMA mode bit mask.*/                                                \
  uint32_t                  txdmamode;

/**
 * @brief   Low level fields of the SPI configuration structure.
 */
#define spi_lld_config_fields                                               \
  /* SPI CR1 register initialization data.*/                                \
  uint16_t                  cr1;                                            \
  /* SPI CR2 register initialization data.*/                                \
  uint16_t                  cr2;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_SPI_USE_SPI1 && !defined(__DOXYGEN__)
extern SPIDriver SPID1;
#endif

#if STM32_SPI_USE_SPI2 && !defined(__DOXYGEN__)
extern SPIDriver SPID2;
#endif

#if STM32_SPI_USE_SPI3 && !defined(__DOXYGEN__)
extern SPIDriver SPID3;
#endif

#if STM32_SPI_USE_SPI4 && !defined(__DOXYGEN__)
extern SPIDriver SPID4;
#endif

#if STM32_SPI_USE_SPI5 && !defined(__DOXYGEN__)
extern SPIDriver SPID5;
#endif

#if STM32_SPI_USE_SPI6 && !defined(__DOXYGEN__)
extern SPIDriver SPID6;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void spi_lld_init(void);
  void spi_lld_start(SPIDriver *spip);
  void spi_lld_stop(SPIDriver *spip);
#if (SPI_SELECT_MODE == SPI_SELECT_MODE_LLD) || defined(__DOXYGEN__)
  void spi_lld_select(SPIDriver *spip);
  void spi_lld_unselect(SPIDriver *spip);
#endif
  void spi_lld_ignore(SPIDriver *spip, size_t n);
  void spi_lld_exchange(SPIDriver *spip, size_t n,
                        const void *txbuf, void *rxbuf);
  void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf);
  void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf);
#if (SPI_SUPPORTS_CIRCULAR == TRUE) || defined(__DOXYGEN__)
  void spi_lld_abort(SPIDriver *spip);
#endif
  uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* HAL_SPI_LLD_H */

/** @} */
