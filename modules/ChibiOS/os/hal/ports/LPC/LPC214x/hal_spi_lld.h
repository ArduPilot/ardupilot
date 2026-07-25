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
 * @file    LPC214x/hal_spi_lld.h
 * @brief   LPC214x low level SPI driver header.
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

/**
 * @brief   Hardware FIFO depth.
 */
#define LPC214x_SSP_FIFO_DEPTH          8

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   SPI1 driver enable switch.
 * @details If set to @p TRUE the support for SSP is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LPC214x_SPI_USE_SSP) || defined(__DOXYGEN__)
#define LPC214x_SPI_USE_SSP             TRUE
#endif

/**
 * @brief   SSP interrupt priority level setting.
 */
#if !defined(LPC214x_SPI_SSP_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define LPC214x_SPI_SSP_IRQ_PRIORITY    4
#endif

/**
 * @brief   Overflow error hook.
 * @details The default action is to stop the system.
 */
#if !defined(LPC214x_SPI_SSP_ERROR_HOOK) || defined(__DOXYGEN__)
#define LPC214x_SPI_SSP_ERROR_HOOK()    osalSysHalt("overflow")
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !LPC214x_SPI_USE_SSP
#error "SPI driver activated but no SPI peripheral assigned"
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
  /* Pointer to the SSP registers block.*/                                  \
  SSP                   *ssp;                                               \
  /* Number of bytes yet to be received.*/                                  \
  uint32_t              rxcnt;                                              \
  /* Receive pointer or @p NULL.*/                                          \
  void                  *rxptr;                                             \
  /* Number of bytes yet to be transmitted.*/                               \
  uint32_t              txcnt;                                              \
  /* Transmit pointer or @p NULL.*/                                         \
  const void            *txptr;

/**
 * @brief   Low level fields of the SPI configuration structure.
 */
#define spi_lld_config_fields                                               \
  /* SSP CR0 initialization data.*/                                         \
  uint16_t                  cr0;                                            \
  /* SSP CPSR initialization data.*/                                        \
  uint32_t                  cpsr;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if LPC214x_SPI_USE_SSP && !defined(__DOXYGEN__)
extern SPIDriver SPID1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void spi_lld_init(void);
  void spi_lld_start(SPIDriver *spip);
  void spi_lld_stop(SPIDriver *spip);
  void spi_lld_select(SPIDriver *spip);
  void spi_lld_unselect(SPIDriver *spip);
  void spi_lld_ignore(SPIDriver *spip, size_t n);
  void spi_lld_exchange(SPIDriver *spip, size_t n,
                        const void *txbuf, void *rxbuf);
  void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf);
  void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf);
  uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* HAL_SPI_LLD_H */

/** @} */
