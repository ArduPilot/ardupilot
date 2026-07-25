/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    ADUCM36x/hal_spi_lld.h
 * @brief   ADUCM SPI subsystem low level driver header.
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
#define SPI_SUPPORTS_CIRCULAR           FALSE
#define SPI_USE_CIRCULAR                FALSE

/**
 * @name    Register helpers not found in ADI headers
 * @{
 */
#define ADUCM_SPI_CON_ENABLE            0x0001U
#define ADUCM_SPI_CON_MASEN             0x0002U
#define ADUCM_SPI_CON_CPHA              0x0004U
#define ADUCM_SPI_CON_CPOL              0x0008U
#define ADUCM_SPI_CON_WOM               0x0010U
#define ADUCM_SPI_CON_LSB               0x0020U
#define ADUCM_SPI_CON_TIM               0x0040U
#define ADUCM_SPI_CON_ZEN               0x0080U
#define ADUCM_SPI_CON_RXOF              0x0100U
#define ADUCM_SPI_CON_OEN               0x0200U
#define ADUCM_SPI_CON_LOOPBACK          0x0400U
#define ADUCM_SPI_CON_CON               0x0800U
#define ADUCM_SPI_CON_RFLUSH            0x1000U
#define ADUCM_SPI_CON_TFLUSH            0x2000U
#define ADUCM_SPI_CON_MOD_MASK          0xC000U
#define ADUCM_SPI_CON_MOD_TX1RX1        0x0000U
#define ADUCM_SPI_CON_MOD_TX2RX2        0x4000U
#define ADUCM_SPI_CON_MOD_TX3RX3        0x8000U
#define ADUCM_SPI_CON_MOD_TX4RX4        0xC000U

#define ADUCM_SPI_DIV_0                 0x0001U
#define ADUCM_SPI_DIV_1                 0x0002U
#define ADUCM_SPI_DIV_2                 0x0004U
#define ADUCM_SPI_DIV_3                 0x0008U
#define ADUCM_SPI_DIV_4                 0x00F0U
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   SPI0 driver enable switch.
 * @details If set to @p TRUE the support for SPI0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ADUCM_SPI_USE_SPI0) || defined(__DOXYGEN__)
#define ADUCM_SPI_USE_SPI0                  FALSE
#endif

/**
 * @brief   SPI1 driver enable switch.
 * @details If set to @p TRUE the support for SPI1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ADUCM_SPI_USE_SPI1) || defined(__DOXYGEN__)
#define ADUCM_SPI_USE_SPI1                  FALSE
#endif

/**
 * @brief   SPI0 interrupt priority level setting.
 */
#if !defined(ADUCM_SPI_SPI0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define ADUCM_SPI_SPI0_IRQ_PRIORITY         5
#endif

/**
 * @brief   SPI1 interrupt priority level setting.
 */
#if !defined(ADUCM_SPI_SPI1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define ADUCM_SPI_SPI1_IRQ_PRIORITY         5
#endif

/**
 * @brief   Enable SPI DMA support
 */
#if !defined(ADUCM_SPI_USE_DMA) || defined(__DOXYGEN__)
#define ADUCM_SPI_USE_DMA                   FALSE
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !ADUCM_SPI_USE_SPI0 && !ADUCM_SPI_USE_SPI1
#error "SPI driver activated but no SPI peripheral assigned"
#endif

#if ADUCM_SPI_USE_SPI0 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(ADUCM_SPI_SPI0_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPI0"
#endif

#if ADUCM_SPI_USE_SPI1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(ADUCM_SPI_SPI1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPI1"
#endif

#if ADUCM_SPI_USE_DMA
#error "ADuCM SPI driver implementation does not supports DMA yet."
#endif

#if ADUCM_SPI_USE_DMA && ADUCM_SPI_USE_SPI0
#error "ADuCM SPI0 does not supports DMA."
#endif

#if (ADUCM_SPI_USE_DMA == TRUE) && !defined(ADUCM_DMA_REQUIRED)
#define ADUCM_DMA_REQUIRED
#endif

#if (SPI_USE_CIRCULAR == TRUE) && (SPI_SUPPORTS_CIRCULAR == FALSE)
#error "ADuCM SPI does not support circular mode."
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
#if ADUCM_SPI_USE_DMA
#define spi_lld_driver_fields                                               \
  /* Empty placeholder for DMA based implementation. */
#else
#define spi_lld_driver_fields                                               \
  /* Pointer to the SPIx registers block.*/                                 \
  ADI_SPI_TypeDef           *spi;                                           \
  /* Receive buffer pointer.*/                                              \
  uint8_t                   *rxbuf;                                         \
  /* Transmission buffer pointer.*/                                         \
  uint8_t                   *txbuf;                                         \
  /* Transfert size.*/                                                      \
  uint32_t                  size;
#endif /* ADUCM_SPI_USE_DMA */

/**
 * @brief   Low level fields of the SPI configuration structure.
 */
#define spi_lld_config_fields                                               \
  /* SPIxCON register initialization data.*/                                \
  uint16_t                  con;                                            \
  /* SPIxDIV register initialization data.*/                                \
  uint16_t                  div;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if ADUCM_SPI_USE_SPI0 && !defined(__DOXYGEN__)
extern SPIDriver SPID0;
#endif

#if ADUCM_SPI_USE_SPI1 && !defined(__DOXYGEN__)
extern SPIDriver SPID1;
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
  uint8_t spi_lld_polled_exchange(SPIDriver *spip, uint8_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* HAL_SPI_LLD_H */

/** @} */
