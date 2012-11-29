
#include <AP_HAL.h>
#include "Dataflash.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

/* flash size */
#define DF_LAST_PAGE 4096

#define DF_RESET_PIN       31 /* (PC6) */

/* AT45DB161D commands (from datasheet) */
#define DF_TRANSFER_PAGE_TO_BUFFER_1   0x53
#define DF_TRANSFER_PAGE_TO_BUFFER_2   0x55
#define DF_STATUS_REGISTER_READ   0xD7
#define DF_READ_MANUFACTURER_AND_DEVICE_ID   0x9F
#define DF_PAGE_READ   0xD2
#define DF_BUFFER_1_READ   0xD4
#define DF_BUFFER_2_READ   0xD6
#define DF_BUFFER_1_WRITE   0x84
#define DF_BUFFER_2_WRITE   0x87
#define DF_BUFFER_1_TO_PAGE_WITH_ERASE   0x83
#define DF_BUFFER_2_TO_PAGE_WITH_ERASE   0x86
#define DF_PAGE_ERASE   0x81
#define DF_BLOCK_ERASE   0x50
#define DF_SECTOR_ERASE   0x7C
#define DF_CHIP_ERASE_0   0xC7
#define DF_CHIP_ERASE_1   0x94
#define DF_CHIP_ERASE_2   0x80
#define DF_CHIP_ERASE_3   0x9A


void APM1Dataflash::init(void* machtnichts) {
    hal.gpio->pinMode(DF_RESET_PIN, GPIO_OUTPUT);

    /* Reset the dataflash chip */
    hal.gpio->write(DF_RESET_PIN, 0);
    hal.scheduler->delay(1);
    hal.gpio->write(DF_RESET_PIN, 1);

    _spi = hal.spi->device(AP_HAL::SPIDevice_Dataflash);

    _num_pages = DF_LAST_PAGE - 1;
    uint8_t status = _read_status_reg();
    _page_size = (status & 0x01) ? 512 : 528; 
}

void APM1Dataflash::read_mfg_id() {
    _spi->cs_assert();
    _spi->transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);
    _mfg = _spi->transfer(0xFF);
    _device = _spi->transfer(0xFF);
    _device = (_device << 8) | _spi->transfer(0xFF);
    /* fourth byte is dont care */
    _spi->transfer(0xFF);
    _spi->cs_release();
}

bool APM1Dataflash::media_present() {
    return true;
}

void APM1Dataflash::_wait_ready() {
    while(!_read_status());
}

void APM1Dataflash::_page_to_buffer(uint8_t buffer_num, uint16_t page_addr) {
    _spi->cs_assert();
    if (_buffer_num == 1) {
        _spi->transfer(DF_TRANSFER_PAGE_TO_BUFFER_1);
    } else {
        _spi->transfer(DF_TRANSFER_PAGE_TO_BUFFER_2);
    }

    if (_page_size == 512) {
        _spi->transfer( page_addr >> 7 );
        _spi->transfer( page_addr << 1 );
    } else {
        _spi->transfer( page_addr >> 6 );
        _spi->transfer( page_addr << 2 );
    }
    /* finally send one dont care byte */
    _spi->transfer(0x00);

    _spi->cs_release();
    _wait_ready();
}

void APM1Dataflash::_buffer_to_page(uint8_t buffer_num, uint16_t page_addr, bool wait) {
    _spi->cs_assert();
    if (_buffer_num == 1) {
        _spi->transfer(DF_BUFFER_1_TO_PAGE_WITH_ERASE);
    } else {
        _spi->transfer(DF_BUFFER_2_TO_PAGE_WITH_ERASE);
    }

    if (_page_size == 512) {
        _spi->transfer( page_addr >> 7 );
        _spi->transfer( page_addr << 1 );
    } else {
        _spi->transfer( page_addr >> 6 );
        _spi->transfer( page_addr << 2 );
    }
    /* finally send one dont care byte */
    _spi->transfer(0x00);

    _spi->cs_release();
    if (wait) {
        _wait_ready();
    }
}

void APM1Dataflash::_page_erase(uint16_t page_addr) {
    _spi->cs_assert();
    _spi->transfer(DF_PAGE_ERASE);

    if (_page_size == 512) {
        _spi->transfer( page_addr >> 7 );
        _spi->transfer( page_addr << 1 );
    } else {
        _spi->transfer( page_addr >> 6 );
        _spi->transfer( page_addr << 2 );
    }

    /* finally send one dont care byte */
    _spi->transfer(0x00);
    _spi->cs_release();
    _wait_ready();
}

void APM1Dataflash::_block_erase(uint16_t block_addr) {
    _spi->cs_assert();
    _spi->transfer(DF_BLOCK_ERASE);

    if (_page_size == 512) {
        _spi->transfer( block_addr >> 7 );
        _spi->transfer( block_addr << 1 );
    } else {
        _spi->transfer( block_addr >> 6 );
        _spi->transfer( block_addr << 2 );
    }
    /* finally send one dont care byte */
    _spi->transfer(0x00);
    _spi->cs_release();
    _wait_ready();
}

void APM1Dataflash::_chip_erase() {
    _spi->cs_assert();
    _spi->transfer(DF_CHIP_ERASE_0);
    _spi->transfer(DF_CHIP_ERASE_1);
    _spi->transfer(DF_CHIP_ERASE_2);
    _spi->transfer(DF_CHIP_ERASE_3);
    _spi->cs_release();

    while(!_read_status()) {
        hal.scheduler->delay(1);
    }
}

void APM1Dataflash::_buffer_write(uint8_t buffer_num, uint16_t page_addr, uint8_t data) {
    _spi->cs_assert();
    if (buffer_num == 1) {
        _spi->transfer(DF_BUFFER_1_WRITE);
    } else {
        _spi->transfer(DF_BUFFER_2_WRITE);
    }
    /* Don't care */
    _spi->transfer(0);
    /* Internal buffer address */
    _spi->transfer((uint8_t)(page_addr >> 8));
    _spi->transfer((uint8_t)(page_addr & 0xFF));
    /* Byte to write */
    _spi->transfer(data);
    _spi->cs_release();
}

uint8_t APM1Dataflash::_buffer_read(uint8_t buffer_num, uint16_t page_addr) {
    _spi->cs_assert();
    if (buffer_num == 1) {
        _spi->transfer(DF_BUFFER_1_READ);
    } else {
        _spi->transfer(DF_BUFFER_2_READ);
    }
    /* Don't care */
    _spi->transfer(0);
    /* Internal buffer address */
    _spi->transfer((uint8_t)(page_addr >> 8));
    _spi->transfer((uint8_t)(page_addr & 0xFF));
    /* Don't care */
    _spi->transfer(0);
    /* Read data byte */
    uint8_t res = _spi->transfer(0);
    _spi->cs_release();
    return res;
}

inline uint8_t APM1Dataflash::_read_status_reg() {
    _spi->cs_assert();
    _spi->transfer(DF_STATUS_REGISTER_READ);
    /* Read the first byte of the result */
    uint8_t res = _spi->transfer(0);
    _spi->cs_release();
    return res;
}

inline uint8_t APM1Dataflash::_read_status() {
    /* Busy status is the top bit of the status register */
    return _read_status_reg() & 0x80;
}

