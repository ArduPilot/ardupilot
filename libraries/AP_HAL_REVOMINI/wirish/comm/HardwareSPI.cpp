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
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief Wirish SPI implementation.
 */

#include "HardwareSPI.h"

#include "timer.h"
#include "util.h"
#include "wirish.h"
#include "boards.h"

struct spi_pins {
    uint8_t nss;
    uint8_t sck;
    uint8_t miso;
    uint8_t mosi;
};

static const spi_pins* dev_to_spi_pins(spi_dev *dev);

static void enable_device(spi_dev *dev,
                          bool as_master,
                          SPIFrequency frequency,
                          uint16_t endianness,
                          spi_mode mode);

static const spi_pins board_spi_pins[] __FLASH__ = {
    {BOARD_SPI1_NSS_PIN,
     BOARD_SPI1_SCK_PIN,
     BOARD_SPI1_MISO_PIN,
     BOARD_SPI1_MOSI_PIN},
    {BOARD_SPI2_NSS_PIN,
     BOARD_SPI2_SCK_PIN,
     BOARD_SPI2_MISO_PIN,
     BOARD_SPI2_MOSI_PIN},
#ifdef STM32_HIGH_DENSITY
    {BOARD_SPI3_NSS_PIN,
     BOARD_SPI3_SCK_PIN,
     BOARD_SPI3_MISO_PIN,
     BOARD_SPI3_MOSI_PIN},
#endif
};


/*
 * Constructor
 */

HardwareSPI::HardwareSPI(uint32_t spi_num) {
    switch (spi_num) {
    case 1:
        this->spi_d = _SPI1;
        break;
    case 2:
        this->spi_d = _SPI2;
        break;
#ifdef STM32_HIGH_DENSITY
    case 3:
        this->spi_d = SPI3;
        break;
#endif
    default:
        assert_param(0);
        break;
    }
}

/*
 * Set up/tear down
 */

void HardwareSPI::begin(SPIFrequency frequency, uint32_t bitOrder, uint32_t mode) {
    if (mode >= 4) {
        assert_param(0);
        return;
    }
    spi_mode m = (spi_mode)mode;
    enable_device(this->spi_d, true, frequency, (uint16_t)bitOrder, m);
}

void HardwareSPI::begin(void) {
    this->begin(SPI_1_125MHZ, MSBFIRST, 0);
}

void HardwareSPI::beginSlave(uint32_t bitOrder, uint32_t mode) {
    if (mode >= 4) {
        assert_param(0);
        return;
    }
    spi_mode m = (spi_mode)mode;
    enable_device(this->spi_d, false, (SPIFrequency)0, (uint16_t)bitOrder, m);
}

void HardwareSPI::beginSlave(void) {
    this->beginSlave(MSBFIRST, 0);
}

void HardwareSPI::end(void) {
    if (!spi_is_enabled(this->spi_d)) {
        return;
    }

    // Follows RM0008's sequence for disabling a SPI in master/slave
    // full duplex mode.
    while (spi_is_rx_nonempty(this->spi_d)) {
        // FIXME [0.1.0] remove this once you have an interrupt based driver
        volatile uint16_t rx __attribute__((unused)) = spi_rx_reg(this->spi_d);
    }
    while (!spi_is_tx_empty(this->spi_d))
        ;
    while (spi_is_busy(this->spi_d))
        ;
    spi_peripheral_disable(this->spi_d);
}

/*
 * I/O
 */

uint8_t HardwareSPI::read(void) {
    uint8_t buf[1];
    this->read(buf, 1);
    return buf[0];
}

void HardwareSPI::read(uint8_t *buf, uint32_t len) {
    uint32_t rxed = 0;
    while (rxed < len) {
        while (!spi_is_rx_nonempty(this->spi_d))
            ;
        buf[rxed++] = (uint8_t)spi_rx_reg(this->spi_d);
    }
}

void HardwareSPI::write(uint8_t byte) {
    this->write(&byte, 1);
}

void HardwareSPI::write(const uint8_t *data, uint32_t length) {
    uint32_t txed = 0;
    while (txed < length) {
        txed += spi_tx(this->spi_d, data + txed, length - txed);
    }
}

uint8_t HardwareSPI::transfer(uint8_t byte) {
    this->write(byte);
    return this->read();
}

/*
 * Pin accessors
 */

uint8_t HardwareSPI::misoPin(void) {
    return dev_to_spi_pins(this->spi_d)->miso;
}

uint8_t HardwareSPI::mosiPin(void) {
    return dev_to_spi_pins(this->spi_d)->mosi;
}

uint8_t HardwareSPI::sckPin(void) {
    return dev_to_spi_pins(this->spi_d)->sck;
}

uint8_t HardwareSPI::nssPin(void) {
    return dev_to_spi_pins(this->spi_d)->nss;
}

/*
 * Deprecated functions
 */

uint8_t HardwareSPI::send(uint8_t data) {
    uint8_t buf[] = {data};
    return this->send(buf, 1);
}

uint8_t HardwareSPI::send(uint8_t *buf, uint32_t len) {
    uint32_t txed = 0;
    uint8_t ret = 0;
    while (txed < len) {
        this->write(buf[txed++]);
        ret = this->read();
    }
    return ret;
}

uint8_t HardwareSPI::recv(void) {
    return this->read();
}

/*
 * Auxiliary functions
 */

static void configure_gpios(spi_dev *dev, bool as_master);
static spi_baud_rate determine_baud_rate(spi_dev *dev, SPIFrequency freq);

static const spi_pins* dev_to_spi_pins(spi_dev *dev) {
    if (dev->SPIx == SPI1)
       return board_spi_pins;
    else if (dev->SPIx == SPI2)
       return board_spi_pins + 1;
#ifdef STM32_HIGH_DENSITY       
    else if (dev->SPIx == SPI3)
	  return board_spi_pins + 2;
#endif
	else 
	{
	  assert_param(0);
	  return NULL;
	}
}

/* Enables the device in master or slave full duplex mode.  If you
 * change this code, you must ensure that appropriate changes are made
 * to HardwareSPI::end(). */
static void enable_device(spi_dev *dev,
                          bool as_master,
                          SPIFrequency freq,
                          uint16_t endianness,
                          spi_mode mode) {
    spi_baud_rate baud = determine_baud_rate(dev, freq);
    spi_init(dev);
    configure_gpios(dev, as_master);
    if (as_master) {
        spi_master_enable(dev, baud, mode, endianness);
    } else {
        spi_slave_enable(dev, mode, endianness);
    }
}

//static void disable_pwm(const stm32_pin_info *i) {
//    if (i->timer_device) {
//        timer_set_mode(i->timer_device, i->timer_channel, TIMER_DISABLED);
//    }
//}

static void configure_gpios(spi_dev *dev, bool as_master) {
    const spi_pins *pins = dev_to_spi_pins(dev);

    if (!pins) {
        return;
    }

    const stm32_pin_info *nssi = &PIN_MAP[pins->nss];
    const stm32_pin_info *scki = &PIN_MAP[pins->sck];
    const stm32_pin_info *misoi = &PIN_MAP[pins->miso];
    const stm32_pin_info *mosii = &PIN_MAP[pins->mosi];

//    disable_pwm(nssi);
//    disable_pwm(scki);
//    disable_pwm(misoi);
//    disable_pwm(mosii);

    spi_gpio_cfg(dev,
				 as_master,
                 nssi->gpio_device,
                 nssi->gpio_bit,
                 scki->gpio_device,
                 scki->gpio_bit,
                 misoi->gpio_bit,
                 mosii->gpio_bit);
}

static const spi_baud_rate baud_rates[MAX_SPI_FREQS] __FLASH__ = {
    SPI_BAUD_PCLK_DIV_2,
    SPI_BAUD_PCLK_DIV_4,
    SPI_BAUD_PCLK_DIV_8,
    SPI_BAUD_PCLK_DIV_16,
    SPI_BAUD_PCLK_DIV_32,
    SPI_BAUD_PCLK_DIV_64,
    SPI_BAUD_PCLK_DIV_128,
    SPI_BAUD_PCLK_DIV_256,
};

/**
 * Lame prescaler calculator - to fix
 */
static spi_baud_rate determine_baud_rate(spi_dev *dev, SPIFrequency freq)
{

	spi_baud_rate rate;

	switch(freq)
	{
		case SPI_18MHZ:
			rate = SPI_BAUD_PCLK_DIV_8;
			break;
		case SPI_9MHZ:
			rate = SPI_BAUD_PCLK_DIV_16;
			break;
		case SPI_4_5MHZ:
			rate = SPI_BAUD_PCLK_DIV_32;
			break;
		case SPI_2_25MHZ:
			rate = SPI_BAUD_PCLK_DIV_64;
			break;
		case SPI_1_125MHZ:
			rate = SPI_BAUD_PCLK_DIV_128;
			break;
		case SPI_562_500KHZ:
			rate = SPI_BAUD_PCLK_DIV_256;
			break;
		case SPI_281_250KHZ:
			rate = SPI_BAUD_PCLK_DIV_256;
			break;
		case SPI_140_625KHZ:
			rate = SPI_BAUD_PCLK_DIV_256;
			break;
		default:
			rate = SPI_BAUD_PCLK_DIV_128;
			break;
			
	}
	return rate;
}
