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
 * @brief   AVR/MEGA SPI subsystem low level driver header.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef HAL_SPI_LLD_H
#define HAL_SPI_LLD_H

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/**
 * @name SPI Configuration Register
 * @{
 */
#define SPI_CR_SPIE              (1 << SPIE)

#define SPI_CR_SPE               (1 << SPE)

#define SPI_CR_DORD_MSB_FIRST    (0 << DORD)
#define SPI_CR_DORD_LSB_FIRST    (1 << DORD)

#define SPI_CR_MSTR              (1 << MSTR)

#define SPI_CR_CPOL_CPHA_MODE(n) ((n) << CPHA)

#define SPI_CR_SCK_FOSC_2        (0 << SPR0)
#define SPI_CR_SCK_FOSC_4        (0 << SPR0)
#define SPI_CR_SCK_FOSC_8        (1 << SPR0)
#define SPI_CR_SCK_FOSC_16       (1 << SPR0)
#define SPI_CR_SCK_FOSC_32       (2 << SPR0)
#define SPI_CR_SCK_FOSC_64       (2 << SPR0)
#define SPI_CR_SCK_FOSC_128      (3 << SPR0)
/** @} */

/**
 * @name SPI Status Register
 * {
 */
#define SPI_SR_SPIF              (1 << SPIF)

#define SPI_SR_WCOL              (1 << WCOL)

#define SPI_SR_SCK_FOSC_2        (1 << SPI2X)
#define SPI_SR_SCK_FOSC_4        (0 << SPI2X)
#define SPI_SR_SCK_FOSC_8        (1 << SPI2X)
#define SPI_SR_SCK_FOSC_16       (0 << SPI2X)
#define SPI_SR_SCK_FOSC_32       (1 << SPI2X)
#define SPI_SR_SCK_FOSC_64       (0 << SPI2X)
#define SPI_SR_SCK_FOSC_128      (0 << SPI2X)
/** @} */

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   SPI driver enable switch.
 * @details If set to @p TRUE the support for SPI1 is included.
 */
#if !defined(AVR_SPI_USE_SPI1) || defined(__DOXYGEN__)
#define AVR_SPI_USE_SPI1  FALSE
#endif
/** @} */

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/**
 * @brief   Circular mode support flag.
 */
#define SPI_SUPPORTS_CIRCULAR           TRUE

#define spi_lld_driver_fields                                                \
  const uint8_t             *txbuf;                                          \
  uint8_t                   *rxbuf;                                          \
  size_t                    exbytes;                                         \
  size_t                    exidx;

/**
 * @brief   Low level fields of the SPI configuration structure.
 */
#define spi_lld_config_fields                                               \
  uint8_t               spcr;                                               \
  uint8_t               spsr;

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
#define spi_lld_ignore(spip, n)     spi_lld_exchange(spip, n, NULL, NULL)

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
#define spi_lld_send(spip, n, txbuf)     spi_lld_exchange(spip, n, txbuf, NULL)

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
#define spi_lld_receive(spip, n, rxbuf)     spi_lld_exchange(spip, n, NULL, rxbuf)

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#if AVR_SPI_USE_SPI1 && !defined(__DOXYGEN__)
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
  void spi_lld_exchange(SPIDriver *spip, size_t n,
                        const void *txbuf, void *rxbuf);

#if (SPI_SUPPORTS_CIRCULAR == TRUE) || defined(__DOXYGEN__)
void spi_lld_abort(SPIDriver *spip);
#endif
#if AVR_SPI_USE_16BIT_POLLED_EXCHANGE
  uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame);
#else
  uint8_t spi_lld_polled_exchange(SPIDriver *spip, uint8_t frame);
#endif

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* HAL_SPI_LLD_H */

/** @} */
