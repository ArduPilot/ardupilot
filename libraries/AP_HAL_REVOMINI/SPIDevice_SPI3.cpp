/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>


#include <AP_HAL/AP_HAL.h>
#include "SPIDevices.h"
#include "GPIO.h"
#include "Semaphores.h"
#include <spi.h>
#include <io.h>

using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

REVOMINISemaphore REVOMINISPI3DeviceDriver::_semaphore;

void REVOMINISPI3DeviceDriver::init() {
    _dev = _SPI3;

    hal.gpio->pinMode(_cs_pin, OUTPUT);
    hal.gpio->write(_cs_pin, HIGH);

    //set frequency
    SPIFrequency freq = SPI_1_125MHZ;
    spi_baud_rate baud = determine_baud_rate(freq);

    //set mode
    //spi_mode m = (spi_mode)0;

    //set master
    bool as_master = true;

    //init the device
    spi_init(_dev);

    //configure gpios
    configure_gpios(_dev, as_master);

    if (as_master) {
        spi_master_enable(_dev, baud, (spi_mode)0, MSBFIRST);
    } else {
        spi_slave_enable(_dev, (spi_mode)0, MSBFIRST);
    }



}

AP_HAL::Semaphore* REVOMINISPI3DeviceDriver::get_semaphore() {
    return &_semaphore;
}

inline void REVOMINISPI3DeviceDriver::_cs_assert() {
    hal.gpio->write(_cs_pin, LOW);
}

inline void REVOMINISPI3DeviceDriver::_cs_release() {
    hal.gpio->write(_cs_pin, HIGH);
}

inline uint8_t REVOMINISPI3DeviceDriver::_transfer(uint8_t data) {
    uint8_t buf[1];

    //write 1byte
    spi_tx(this->_dev, &data, 1);

    //read one byte
    while (!spi_is_rx_nonempty(this->_dev))
            ;
    buf[0] = (uint8_t)spi_rx_reg(this->_dev);
    return buf[0];
}

uint8_t REVOMINISPI3DeviceDriver::transfer(uint8_t data) {
    return _transfer(data);
}


bool REVOMINISPI3DeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) {

    _cs_assert();
    if (rx == NULL) {
        for (uint16_t i = 0; i < len; i++) {
            _transfer(tx[i]);
        }
    } else {
        for (uint16_t i = 0; i < len; i++) {
            rx[i] = _transfer(tx[i]);
        }
    }
    _cs_release();
    return true;
}

void REVOMINISPI3DeviceDriver::transfer(const uint8_t *tx, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
            _transfer(tx[i]);
    }
}

void REVOMINISPI3DeviceDriver::set_bus_speed(REVOMINISPI3DeviceDriver::bus_speed speed)
{
/*
    if (speed == REVOMINISPI3DeviceDriver::SPI_SPEED_HIGH) {

	    //init the device
	    spi_init(_dev);

	    SPIFrequency freq = SPI_9MHZ;
	    spi_baud_rate baud = determine_baud_rate(freq);
	    spi_master_enable(_dev, baud, (spi_mode)0, MSBFIRST);

    } else {
	    //init the device
	    spi_init(_dev);

	    SPIFrequency freq = SPI_1_125MHZ;
	    spi_baud_rate baud = determine_baud_rate(freq);
	    spi_master_enable(_dev, baud, (spi_mode)0, MSBFIRST);
    }
	*/
}


void REVOMINISPI3DeviceDriver::cs_assert() {
    _cs_assert();
}

void REVOMINISPI3DeviceDriver::cs_release() {
    _cs_release();
}



const spi_pins* REVOMINISPI3DeviceDriver::dev_to_spi_pins(spi_dev *dev) {
    if (dev->SPIx == SPI1)
       return board_spi_pins;
    else if (dev->SPIx == SPI2)
       return board_spi_pins + 1;
    else if (dev->SPIx == SPI3)
	  return board_spi_pins + 2;
    else
	{
	  assert_param(0);
	  return NULL;
	}
}

void REVOMINISPI3DeviceDriver::configure_gpios(spi_dev *dev, bool as_master) {
    const spi_pins *pins = dev_to_spi_pins(dev);

    if (!pins) {
        return;
    }

    const stm32_pin_info *nssi = &PIN_MAP[pins->nss];
    const stm32_pin_info *scki = &PIN_MAP[pins->sck];
    const stm32_pin_info *misoi = &PIN_MAP[pins->miso];
    const stm32_pin_info *mosii = &PIN_MAP[pins->mosi];

    spi_gpio_cfg(dev,
		as_master,
                nssi->gpio_device,
                nssi->gpio_bit,
                scki->gpio_device,
                scki->gpio_bit,
                misoi->gpio_bit,
                mosii->gpio_bit);
}

const spi_baud_rate REVOMINISPI3DeviceDriver::determine_baud_rate(SPIFrequency freq)
{

	spi_baud_rate rate;

	switch(freq)
	{
		case SPI_18MHZ:
			rate = SPI_BAUD_PCLK_DIV_2;
			break;
		case SPI_9MHZ:
			rate = SPI_BAUD_PCLK_DIV_4;
			break;
		case SPI_4_5MHZ:
			rate = SPI_BAUD_PCLK_DIV_8;
			break;
		case SPI_2_25MHZ:
			rate = SPI_BAUD_PCLK_DIV_16;
			break;
		case SPI_1_125MHZ:
			rate = SPI_BAUD_PCLK_DIV_32;
			break;
		case SPI_562_500KHZ:
			rate = SPI_BAUD_PCLK_DIV_64;
			break;
		case SPI_281_250KHZ:
			rate = SPI_BAUD_PCLK_DIV_128;
			break;
		case SPI_140_625KHZ:
			rate = SPI_BAUD_PCLK_DIV_256;
			break;
		default:
			rate = SPI_BAUD_PCLK_DIV_32;
			break;

	}
	return rate;
}
