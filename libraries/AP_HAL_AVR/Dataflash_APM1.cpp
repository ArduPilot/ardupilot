
#include <AP_HAL.h>
#include "Dataflash.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

/* flash size */
#define DF_LAST_PAGE 4096

/* Arduino Mega SPI pins */
#define DF_DATAOUT     51 /* MOSI */
#define DF_DATAIN      50 /* MISO */
#define DF_SPICLOCK    52 /* SCK */
#define DF_SLAVESELECT 53 /* SS (PB0) */
#define DF_RESET       31 /* RESET (PC6) */

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
    hal.gpio->pinMode(DF_DATAOUT,     GPIO_OUTPUT);
    hal.gpio->pinMode(DF_DATAIN ,     GPIO_INPUT);
    hal.gpio->pinMode(DF_SPICLOCK,    GPIO_OUTPUT);
    hal.gpio->pinMode(DF_SLAVESELECT, GPIO_OUTPUT);
    hal.gpio->pinMode(DF_RESET,       GPIO_OUTPUT);

    /* Reset the dataflash chip */
    hal.gpio->write(DF_RESET, 0);
    hal.scheduler->delay(1);
    hal.gpio->write(DF_RESET, 1);

    _cs_inactive();

    hal.spi->set_freq(8000000L);

    _num_pages = DF_LAST_PAGE - 1;
    uint8_t status = _read_status_reg();
    _page_size = (status & 0x01) ? 512 : 528; 
}

void APM1Dataflash::read_mfg_id() {
    _cs_active();
    hal.spi->transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);
    _mfg = hal.spi->transfer(0xFF);
    _device = hal.spi->transfer(0xFF);
    _device = (_device << 8) | hal.spi->transfer(0xFF);
    /* fourth byte is dont care */
    hal.spi->transfer(0xFF);
    _cs_inactive();
}

bool APM1Dataflash::media_present() {
    return true;
}

void APM1Dataflash::_wait_ready() {
    while(!_read_status());
}

void APM1Dataflash::_page_to_buffer(uint8_t buffer_num, uint16_t page_addr) {
    _cs_active();
    if (_buffer_num == 1) {
        hal.spi->transfer(DF_TRANSFER_PAGE_TO_BUFFER_1);
    } else {
        hal.spi->transfer(DF_TRANSFER_PAGE_TO_BUFFER_2);
    }

    if (_page_size == 512) {
        hal.spi->transfer( page_addr >> 7 );
        hal.spi->transfer( page_addr << 1 );
    } else {
        hal.spi->transfer( page_addr >> 6 );
        hal.spi->transfer( page_addr << 2 );
    }
    /* finally send one dont care byte */
    hal.spi->transfer(0x00);

    _cs_inactive();
    _wait_ready();
}

void APM1Dataflash::_buffer_to_page(uint8_t buffer_num, uint16_t page_addr, bool wait) {
    _cs_active();
    if (_buffer_num == 1) {
        hal.spi->transfer(DF_BUFFER_1_TO_PAGE_WITH_ERASE);
    } else {
        hal.spi->transfer(DF_BUFFER_2_TO_PAGE_WITH_ERASE);
    }

    if (_page_size == 512) {
        hal.spi->transfer( page_addr >> 7 );
        hal.spi->transfer( page_addr << 1 );
    } else {
        hal.spi->transfer( page_addr >> 6 );
        hal.spi->transfer( page_addr << 2 );
    }
    /* finally send one dont care byte */
    hal.spi->transfer(0x00);

    _cs_inactive();
    if (wait) {
        _wait_ready();
    }
}

void APM1Dataflash::_page_erase(uint16_t page_addr) {
    _cs_active();
    hal.spi->transfer(DF_PAGE_ERASE);

    if (_page_size == 512) {
        hal.spi->transfer( page_addr >> 7 );
        hal.spi->transfer( page_addr << 1 );
    } else {
        hal.spi->transfer( page_addr >> 6 );
        hal.spi->transfer( page_addr << 2 );
    }

    /* finally send one dont care byte */
    hal.spi->transfer(0x00);
    _cs_inactive();
    _wait_ready();
}

void APM1Dataflash::_block_erase(uint16_t block_addr) {
    _cs_active();
    hal.spi->transfer(DF_BLOCK_ERASE);

    if (_page_size == 512) {
        hal.spi->transfer( block_addr >> 7 );
        hal.spi->transfer( block_addr << 1 );
    } else {
        hal.spi->transfer( block_addr >> 6 );
        hal.spi->transfer( block_addr << 2 );
    }
    /* finally send one dont care byte */
    hal.spi->transfer(0x00);
    _cs_inactive();
    _wait_ready();
}

void APM1Dataflash::_chip_erase() {
    _cs_active();
    hal.spi->transfer(DF_CHIP_ERASE_0);
    hal.spi->transfer(DF_CHIP_ERASE_1);
    hal.spi->transfer(DF_CHIP_ERASE_2);
    hal.spi->transfer(DF_CHIP_ERASE_3);
    _cs_inactive();

    while(!_read_status()) {
        hal.scheduler->delay(1);
    }
}

void APM1Dataflash::_buffer_write(uint8_t buffer_num, uint16_t page_addr, uint8_t data) {
    _cs_active();
    if (buffer_num == 1) {
        hal.spi->transfer(DF_BUFFER_1_WRITE);
    } else {
        hal.spi->transfer(DF_BUFFER_2_WRITE);
    }
    /* Don't care */
    hal.spi->transfer(0);
    /* Internal buffer address */
    hal.spi->transfer((uint8_t)(page_addr >> 8));
    hal.spi->transfer((uint8_t)(page_addr & 0xFF));
    /* Byte to write */
    hal.spi->transfer(data);
    _cs_inactive();
}

uint8_t APM1Dataflash::_buffer_read(uint8_t buffer_num, uint16_t page_addr) {
    _cs_active();
    if (buffer_num == 1) {
        hal.spi->transfer(DF_BUFFER_1_READ);
    } else {
        hal.spi->transfer(DF_BUFFER_2_READ);
    }
    /* Don't care */
    hal.spi->transfer(0);
    /* Internal buffer address */
    hal.spi->transfer((uint8_t)(page_addr >> 8));
    hal.spi->transfer((uint8_t)(page_addr & 0xFF));
    /* Don't care */
    hal.spi->transfer(0);
    /* Read data byte */
    uint8_t res = hal.spi->transfer(0);
    _cs_inactive();
    return res;
}

inline uint8_t APM1Dataflash::_read_status_reg() {
    _cs_active();
    hal.spi->transfer(DF_STATUS_REGISTER_READ);
    /* Read the first byte of the result */
    uint8_t res = hal.spi->transfer(0);
    _cs_inactive();
    return res;
}

inline uint8_t APM1Dataflash::_read_status() {
    /* Busy status is the top bit of the status register */
    return _read_status_reg() & 0x80;
}

inline void APM1Dataflash::_cs_inactive() {
    hal.gpio->write(DF_SLAVESELECT, 1);
}

inline void APM1Dataflash::_cs_active() {
    hal.gpio->write(DF_SLAVESELECT, 0);
}
