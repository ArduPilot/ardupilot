
#include <avr/io.h>
#include <AP_HAL.h>
#include "Dataflash.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

/* Connected to USART3 in SPI mode */
#define DF_RESET_PIN      41  /* RESET (PG0) */
#define DF_CARDDETECT_PIN 33  /* PC4 */ 

// AT45DB321D Commands (from Datasheet)
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

#define DEBUG

#ifdef DEBUG
#define LOGD(format, ...) do { hal.console->printf_P(PSTR("DBG/Dataflash: "format), __VA_ARGS__); } while (0)
#else
#define LOGD(format, ...) do {} while(0)
#endif


void APM2Dataflash::init(void* machtnichts) {
    /* setup gpio pins */
    hal.gpio->pinMode(DF_RESET_PIN, GPIO_OUTPUT);
    hal.gpio->pinMode(DF_CARDDETECT_PIN, GPIO_INPUT);

    /* Reset device */
    hal.gpio->write(DF_RESET_PIN, 0);
    hal.scheduler->delay(1);
    hal.gpio->write(DF_RESET_PIN, 1);

    _spi = hal.spi->device(AP_HAL::SPIDevice_Dataflash);

    uint8_t status = _read_status_reg();
    _page_size = (status & 0x01) ? 512 : 528; 
    LOGD("_page_size set to %d\r\n", _page_size);

    read_mfg_id();
    /* If no card is present, MISO is usually either constant high or constant
     * low. Result is all 0 or all 1s for _mfg and _device.
     */
    if ((_mfg == 0 && _device == 0)
      ||(_mfg == 0xFF && _device == 0xFFFF)) {
      _num_pages = 0;
      return;
    }
    /* from page 22 of the spec, density code decoder: */
    uint8_t density_code = (_device >> 8) & 0x1F;
    if (density_code == 0x7) {
        /* 32 Mbit */
        _num_pages = 8191;
    } else if (density_code == 0x06) {
        /* 16 Mbit */
        _num_pages = 4095;
    } else {
        /* Unknown */
        _num_pages = 0;
    }
}

void APM2Dataflash::read_mfg_id() {
    _spi->cs_assert();
    _spi->transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);
    _mfg = _spi->transfer(0xFF);
    _device = _spi->transfer(0xFF);
    _device = (_device << 8) | _spi->transfer(0xFF);
    _spi->transfer(0xFF);
    _spi->cs_release();
}

bool APM2Dataflash::media_present() {
    /* if init detected a df chip, it set _num_pages > 0. */
    return _num_pages > 0;
}

void APM2Dataflash::_wait_ready() {
    while(!_read_status());
}

void APM2Dataflash::_page_to_buffer(uint8_t buffer_num, uint16_t page_addr) {
    LOGD("_page_to_buffer: buf: %d page: %d\r\n", (int) buffer_num, page_addr);
    _spi->cs_assert();
    if (_buffer_num == 1) {
        _spi->transfer(DF_TRANSFER_PAGE_TO_BUFFER_1);
    } else {
        _spi->transfer(DF_TRANSFER_PAGE_TO_BUFFER_2);
    }

    if (_page_size == 512) {
        _spi->transfer((uint8_t)(page_addr >> 7));
        _spi->transfer((uint8_t)(page_addr << 1));
    } else {
        _spi->transfer((uint8_t)(page_addr >> 6));
        _spi->transfer((uint8_t)(page_addr << 2));
    }
    /* finally send one dont care byte */
    _spi->transfer(0x00);

    _spi->cs_release();
    _wait_ready();
}

void APM2Dataflash::_buffer_to_page(uint8_t buffer_num, uint16_t page_addr, bool wait) {
    LOGD("_buffer_to_page buf: %d, page: %d\r\n",
            (int) buffer_num, page_addr);
    _spi->cs_assert();
    if (_buffer_num == 1) {
        _spi->transfer(DF_BUFFER_1_TO_PAGE_WITH_ERASE);
    } else {
        _spi->transfer(DF_BUFFER_2_TO_PAGE_WITH_ERASE);
    }

    if (_page_size == 512) {
        _spi->transfer((uint8_t)(page_addr >> 7));
        _spi->transfer((uint8_t)(page_addr << 1));
    } else {
        _spi->transfer((uint8_t)(page_addr >> 6));
        _spi->transfer((uint8_t)(page_addr << 2));
    }
    /* finally send one dont care byte */
    _spi->transfer(0x00);

    _spi->cs_release();
    if (wait) {
        _wait_ready();
    }
}

void APM2Dataflash::_page_erase(uint16_t page_addr) {
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

void APM2Dataflash::_block_erase(uint16_t block_addr) {
    LOGD("_block_erase %d\r\n", block_addr);
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

void APM2Dataflash::_chip_erase() {
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

void APM2Dataflash::_buffer_write(uint8_t buffer_num, uint16_t page_addr, uint8_t data) {
    LOGD("_buffer_write buf: %d page: %d data: %d\r\n",
                            (int) buffer_num, page_addr, (int) data);
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

uint8_t APM2Dataflash::_buffer_read(uint8_t buffer_num, uint16_t page_addr) {
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
    LOGD("_buffer_read num: %d pageaddr: %d result: %d\r\n",
            (int) buffer_num, (int) page_addr, (int) res);
    return res;
}

inline uint8_t APM2Dataflash::_read_status_reg() {
    _spi->cs_assert();
    _spi->transfer(DF_STATUS_REGISTER_READ);
    /* Read the first byte of the result */
    uint8_t res = _spi->transfer(0);
    _spi->cs_release();
    return res;
}

inline uint8_t APM2Dataflash::_read_status() {
    /* Busy status is the top bit of the status register */
    return _read_status_reg() & 0x80;
}

