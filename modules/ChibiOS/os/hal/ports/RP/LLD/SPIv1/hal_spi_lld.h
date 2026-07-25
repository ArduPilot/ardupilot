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
 * @file    SPIv1/hal_spi_lld.h
 * @brief   RP SPI subsystem low level driver header.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef HAL_SPI_LLD_H
#define HAL_SPI_LLD_H

#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Circular mode support flag.
 */
#define SPI_SUPPORTS_CIRCULAR               FALSE

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(RP_HAS_SPI0)
#error "RP_HAS_SPI0 not defined in registry"
#endif

#if !defined(RP_HAS_SPI1)
#error "RP_HAS_SPI1 not defined in registry"
#endif

/* Mcuconf.h checks.*/
#if !defined(RP_SPI_USE_SPI0)
#error "RP_SPI_USE_SPI0 not defined in mcuconf.h"
#endif

#if !defined(RP_SPI_USE_SPI1)
#error "RP_SPI_USE_SPI1 not defined in mcuconf.h"
#endif

#if !defined(RP_IRQ_SPI0_PRIORITY)
#error "RP_IRQ_SPI0_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(RP_IRQ_SPI1_PRIORITY)
#error "RP_IRQ_SPI1_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(RP_SPI_SPI0_RX_DMA_CHANNEL)
#error "RP_SPI_SPI0_RX_DMA_CHANNEL not defined in mcuconf.h"
#endif

#if !defined(RP_SPI_SPI0_TX_DMA_CHANNEL)
#error "RP_SPI_SPI0_TX_DMA_CHANNEL not defined in mcuconf.h"
#endif

#if !defined(RP_SPI_SPI1_RX_DMA_CHANNEL)
#error "RP_SPI_SPI0_RX_DMA_CHANNEL not defined in mcuconf.h"
#endif

#if !defined(RP_SPI_SPI1_TX_DMA_CHANNEL)
#error "RP_SPI_SPI0_TX_DMA_CHANNEL not defined in mcuconf.h"
#endif

#if !defined(RP_SPI_SPI0_DMA_PRIORITY)
#error "RP_SPI_SPI0_DMA_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(RP_SPI_SPI1_DMA_PRIORITY)
#error "RP_SPI_SPI1_DMA_PRIORITY not defined in mcuconf.h"
#endif

/* Device selection checks.*/
#if RP_SPI_USE_SPI0 && !RP_HAS_SPI0
#error "SPI0 not present in the selected device"
#endif

#if RP_SPI_USE_SPI1 && !RP_HAS_SPI1
#error "SPI1 not present in the selected device"
#endif

#if !RP_SPI_USE_SPI0 && !RP_SPI_USE_SPI1
#error "SPI driver activated but no SPI peripheral assigned"
#endif

/* IRQ and DMA settings checks.*/
#if RP_SPI_USE_SPI0 &&                                                      \
    !OSAL_IRQ_IS_VALID_PRIORITY(RP_IRQ_SPI0_PRIORITY)
#error "Invalid IRQ priority assigned to SPI0"
#endif

#if RP_SPI_USE_SPI1 &&                                                      \
    !OSAL_IRQ_IS_VALID_PRIORITY(RP_IRQ_SPI1_PRIORITY)
#error "Invalid IRQ priority assigned to SPI1"
#endif

#if RP_SPI_USE_SPI0 &&                                                      \
    !RP_DMA_IS_VALID_PRIORITY(RP_SPI_SPI0_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SPI0"
#endif

#if RP_SPI_USE_SPI1 &&                                                      \
    !RP_DMA_IS_VALID_PRIORITY(RP_SPI_SPI1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SPI1"
#endif

/* Forcing inclusion of the DMA support driver.*/
#if !defined(RP_DMA_REQUIRED)
#define RP_DMA_REQUIRED
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
  const rp_dma_channel_t    *dmarx;                                         \
  /* Transmit DMA stream.*/                                                 \
  const rp_dma_channel_t    *dmatx;                                         \
  /* RX DMA mode bit mask.*/                                                \
  uint32_t                  rxdmamode;                                      \
  /* TX DMA mode bit mask.*/                                                \
  uint32_t                  txdmamode;

/**
 * @brief   Low level fields of the SPI configuration structure.
 */
#define spi_lld_config_fields                                               \
  /* SSPCR0 register initialization data.*/                                 \
  uint32_t                  SSPCR0;                                         \
  /* SSPCPSR register initialization data.*/                                \
  uint32_t                  SSPCPSR;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (RP_SPI_USE_SPI0 == TRUE) && !defined(__DOXYGEN__)
extern SPIDriver SPID0;
#endif

#if (RP_SPI_USE_SPI1 == TRUE) && !defined(__DOXYGEN__)
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
  uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI == TRUE */

#endif /* HAL_SPI_LLD_H */

/** @} */
