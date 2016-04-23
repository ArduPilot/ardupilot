/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file HardwareSPI.h
 * @brief High-level SPI interface
 *
 * This is a "bare essentials" polling driver for now.
 */

/* TODO [0.1.0] Remove deprecated methods. */

#include "hal_types.h"
#include "spi.h"
#include "boards.h"

#ifndef _HARDWARESPI_H_
#define _HARDWARESPI_H_

/**
 * @brief Defines the possible SPI communication speeds.
 */
typedef enum SPIFrequency {
    SPI_18MHZ       = 0, /**< 18 MHz */
    SPI_9MHZ        = 1, /**< 9 MHz */
    SPI_4_5MHZ      = 2, /**< 4.5 MHz */
    SPI_2_25MHZ     = 3, /**< 2.25 MHz */
    SPI_1_125MHZ    = 4, /**< 1.125 MHz */
    SPI_562_500KHZ  = 5, /**< 562.500 KHz */
    SPI_281_250KHZ  = 6, /**< 281.250 KHz */
    SPI_140_625KHZ  = 7, /**< 140.625 KHz */
} SPIFrequency;

#define MAX_SPI_FREQS 8


/**
 * @brief Wirish SPI interface.
 *
 * This implementation uses software slave management, so the caller
 * is responsible for controlling the slave select line.
 */
class HardwareSPI {
public:
    /**
     * @param spiPortNumber Number of the SPI port to manage.
     */
    HardwareSPI(uint32_t spiPortNumber);

    /*
     * Set up/tear down
     */

    /**
     * @brief Turn on a SPI port and set its GPIO pin modes for use as master.
     *
     * SPI port is enabled in full duplex mode, with software slave management.
     *
     * @param frequency Communication frequency
     * @param bitOrder Either LSBFIRST (little-endian) or MSBFIRST (big-endian)
     * @param mode SPI mode to use, one of SPI_MODE_0, SPI_MODE_1,
     *             SPI_MODE_2, and SPI_MODE_3.
     */
    void begin(SPIFrequency frequency, uint32_t bitOrder, uint32_t mode);

    /**
     * @brief Equivalent to begin(SPI_1_125MHZ, MSBFIRST, 0).
     */
    void begin(void);

    /**
     * @brief Turn on a SPI port and set its GPIO pin modes for use as a slave.
     *
     * SPI port is enabled in full duplex mode, with software slave management.
     *
     * @param bitOrder Either LSBFIRST (little-endian) or MSBFIRST(big-endian)
     * @param mode SPI mode to use
     */
    void beginSlave(uint32_t bitOrder, uint32_t mode);

    /**
     * @brief Equivalent to beginSlave(MSBFIRST, 0).
     */
    void beginSlave(void);

    /**
     * @brief Disables the SPI port, but leaves its GPIO pin modes unchanged.
     */
    void end(void);

    /*
     * I/O
     */

    /**
     * @brief Return the next unread byte.
     *
     * If there is no unread byte waiting, this function will block
     * until one is received.
     */
    uint8_t read(void);

    /**
     * @brief Read length bytes, storing them into buffer.
     * @param buffer Buffer to store received bytes into.
     * @param length Number of bytes to store in buffer.  This
     *               function will block until the desired number of
     *               bytes have been read.
     */
    void read(uint8_t *buffer, uint32_t length);

    /**
     * @brief Transmit a byte.
     * @param data Byte to transmit.
     */
    void write(uint8_t data);

    /**
     * @brief Transmit multiple bytes.
     * @param buffer Bytes to transmit.
     * @param length Number of bytes in buffer to transmit.
     */
    void write(const uint8_t *buffer, uint32_t length);

    /**
     * @brief Transmit a byte, then return the next unread byte.
     *
     * This function transmits before receiving.
     *
     * @param data Byte to transmit.
     * @return Next unread byte.
     */
    uint8_t transfer(uint8_t data);

    /*
     * Pin accessors
     */

    /**
     * @brief Return the number of the MISO (master in, slave out) pin
     */
    uint8_t misoPin(void);

    /**
     * @brief Return the number of the MOSI (master out, slave in) pin
     */
    uint8_t mosiPin(void);

    /**
     * @brief Return the number of the SCK (serial clock) pin
     */
    uint8_t sckPin(void);

    /**
     * @brief Return the number of the NSS (slave select) pin
     */
    uint8_t nssPin(void);

    /* -- The following methods are deprecated --------------------------- */

    /**
     * @brief Deprecated.
     *
     * Use HardwareSPI::transfer() instead.
     *
     * @see HardwareSPI::transfer()
     */
    uint8_t send(uint8_t data);

    /**
     * @brief Deprecated.
     *
     * Use HardwareSPI::write() in combination with
     * HardwareSPI::read() (or HardwareSPI::transfer()) instead.
     *
     * @see HardwareSPI::write()
     * @see HardwareSPI::read()
     * @see HardwareSPI::transfer()
     */
    uint8_t send(uint8_t *data, uint32_t length);

    /**
     * @brief Deprecated.
     *
     * Use HardwareSPI::read() instead.
     *
     * @see HardwareSPI::read()
     */
    uint8_t recv(void);
private:
    spi_dev *spi_d;
};

#endif

