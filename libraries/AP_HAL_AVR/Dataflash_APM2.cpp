
#include <avr/io.h>
#include <AP_HAL.h>
#include "Dataflash.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

/* Connected to USART3 in SPI mode */
#define DF_DATAOUT      14  /* MOSI */
#define DF_DATAIN       15  /* MISO */
#define DF_SPICLOCK     PJ2 /* SCK (not used via gpio) */
#define DF_SLAVESELECT  28  /* SS (PA6) */
#define DF_RESET        41  /* RESET (PG0) */
#define DF_CARDDETECT   33  /* PC4 */ 

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

#ifdef DEBUG
#define LOGD(format, ...) do { hal.console->printf_P(PSTR("DBG/Dataflash: "format), __VA_ARGS__); } while (0)
#else
#define LOGD(format, ...) do {} while(0)
#endif


void APM2Dataflash::init(void* machtnichts) {
    /* setup gpio pins */
    hal.gpio->pinMode(DF_DATAOUT,     GPIO_OUTPUT);
    hal.gpio->pinMode(DF_DATAIN,      GPIO_OUTPUT);
    hal.gpio->pinMode(DF_SLAVESELECT, GPIO_OUTPUT);
    hal.gpio->pinMode(DF_RESET,       GPIO_OUTPUT);
    hal.gpio->pinMode(DF_CARDDETECT,  GPIO_INPUT);

    /* Reset device */
    hal.gpio->write(DF_RESET, 0);
    hal.scheduler->delay(1);
    hal.gpio->write(DF_RESET, 1);

    _cs_inactive();

    /* Setup USART3 in SPI mode (MSPI), Mode 0, clock 8Mhz */
    UBRR3 = 0;
    /* DF_SPICLOCK: use XCK3 (PJ2) as output. Enables SPI master mode. */
    DDRJ |= _BV(PJ2);
    /* Set MSPI mode of operation and SPI data mode 0 */
    UCSR3C = _BV(UMSEL31) | _BV(UMSEL30);
    /* Enable transmitter and receiver */
    UCSR3B = _BV(RXEN3) | _BV(TXEN3);
    /* Set baud rate to 8Mhz */
    UBRR3 = 0;

    uint8_t status = _read_status_reg();
    _page_size = (status & 0x01) ? 512 : 528; 
    LOGD("_page_size set to %d\r\n", _page_size);

    read_mfg_id();
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
    _cs_active();
    _transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);
    _mfg = _transfer(0xFF);
    _device = _transfer(0xFF);
    _device = (_device << 8) | _transfer(0xFF);
    _transfer(0xFF);
    _cs_inactive();
}

bool APM2Dataflash::media_present() {
    /* Assume the card is present if we read a valid mfg id. */
    return _num_pages >= 4095;
}

void APM2Dataflash::_wait_ready() {
    while(!_read_status());
}

void APM2Dataflash::_page_to_buffer(uint8_t buffer_num, uint16_t page_addr) {
    LOGD("_page_to_buffer: buf: %d page: %d\r\n", (int) buffer_num, page_addr);
    _cs_active();
    if (_buffer_num == 1) {
        _transfer(DF_TRANSFER_PAGE_TO_BUFFER_1);
    } else {
        _transfer(DF_TRANSFER_PAGE_TO_BUFFER_2);
    }

    if (_page_size == 512) {
        _transfer((uint8_t)(page_addr >> 7));
        _transfer((uint8_t)(page_addr << 1));
    } else {
        _transfer((uint8_t)(page_addr >> 6));
        _transfer((uint8_t)(page_addr << 2));
    }
    /* finally send one dont care byte */
    _transfer(0x00);

    _cs_inactive();
    _wait_ready();
}

void APM2Dataflash::_buffer_to_page(uint8_t buffer_num, uint16_t page_addr, bool wait) {
    LOGD("_buffer_to_page buf: %d, page: %d\r\n",
            (int) buffer_num, page_addr);
    _cs_active();
    if (_buffer_num == 1) {
        _transfer(DF_BUFFER_1_TO_PAGE_WITH_ERASE);
    } else {
        _transfer(DF_BUFFER_2_TO_PAGE_WITH_ERASE);
    }

    if (_page_size == 512) {
        _transfer((uint8_t)(page_addr >> 7));
        _transfer((uint8_t)(page_addr << 1));
    } else {
        _transfer((uint8_t)(page_addr >> 6));
        _transfer((uint8_t)(page_addr << 2));
    }
    /* finally send one dont care byte */
    _transfer(0x00);

    _cs_inactive();
    if (wait) {
        _wait_ready();
    }
}

void APM2Dataflash::_page_erase(uint16_t page_addr) {
    _cs_active();
    _transfer(DF_PAGE_ERASE);

    if (_page_size == 512) {
        _transfer( page_addr >> 7 );
        _transfer( page_addr << 1 );
    } else {
        _transfer( page_addr >> 6 );
        _transfer( page_addr << 2 );
    }

    /* finally send one dont care byte */
    _transfer(0x00);
    _cs_inactive();
    _wait_ready();
}

void APM2Dataflash::_block_erase(uint16_t block_addr) {
    LOGD("_block_erase %d\r\n", block_addr);
    _cs_active();
    _transfer(DF_BLOCK_ERASE);

    if (_page_size == 512) {
        _transfer( block_addr >> 7 );
        _transfer( block_addr << 1 );
    } else {
        _transfer( block_addr >> 6 );
        _transfer( block_addr << 2 );
    }
    /* finally send one dont care byte */
    _transfer(0x00);
    _cs_inactive();
    _wait_ready();
}

void APM2Dataflash::_chip_erase() {
    _cs_active();
    _transfer(DF_CHIP_ERASE_0);
    _transfer(DF_CHIP_ERASE_1);
    _transfer(DF_CHIP_ERASE_2);
    _transfer(DF_CHIP_ERASE_3);
    _cs_inactive();

    while(!_read_status()) {
        hal.scheduler->delay(1);
    }
}

void APM2Dataflash::_buffer_write(uint8_t buffer_num, uint16_t page_addr, uint8_t data) {
    LOGD("_buffer_write buf: %d page: %d data: %d\r\n",
                            (int) buffer_num, page_addr, (int) data);
    _cs_active();
    if (buffer_num == 1) {
        _transfer(DF_BUFFER_1_WRITE);
    } else {
        _transfer(DF_BUFFER_2_WRITE);
    }
    /* Don't care */
    _transfer(0);
    /* Internal buffer address */
    _transfer((uint8_t)(page_addr >> 8));
    _transfer((uint8_t)(page_addr & 0xFF));
    /* Byte to write */
    _transfer(data);
    _cs_inactive();
}

uint8_t APM2Dataflash::_buffer_read(uint8_t buffer_num, uint16_t page_addr) {
    _cs_active();
    if (buffer_num == 1) {
        _transfer(DF_BUFFER_1_READ);
    } else {
        _transfer(DF_BUFFER_2_READ);
    }
    /* Don't care */
    _transfer(0);
    /* Internal buffer address */
    _transfer((uint8_t)(page_addr >> 8));
    _transfer((uint8_t)(page_addr & 0xFF));
    /* Don't care */
    _transfer(0);
    /* Read data byte */
    uint8_t res = _transfer(0);
    _cs_inactive();
    LOGD("_buffer_read num: %d pageaddr: %d result: %d\r\n",
            (int) buffer_num, (int) page_addr, (int) res);
    return res;
}

inline uint8_t APM2Dataflash::_read_status_reg() {
    _cs_active();
    _transfer(DF_STATUS_REGISTER_READ);
    /* Read the first byte of the result */
    uint8_t res = _transfer(0);
    _cs_inactive();
    return res;
}

inline uint8_t APM2Dataflash::_read_status() {
    /* Busy status is the top bit of the status register */
    return _read_status_reg() & 0x80;
}

inline void APM2Dataflash::_cs_inactive() {
    hal.gpio->write(DF_SLAVESELECT, 1);
}

inline void APM2Dataflash::_cs_active() {
    hal.gpio->write(DF_SLAVESELECT, 0);
}

/* APM2 uses USART3 in SPI mode to talk to the dataflash */ 
inline uint8_t APM2Dataflash::_transfer(uint8_t data) {
    /* Wait for empty transmit buffer */
    while (!(UCSR3A & _BV(UDRE3)));
    /* Put data into buffer to start sending */
    UDR3 = data;
    /* Wait for receive buffer to be full */
    while (!(UCSR3A & _BV(RXC3)));
    /* Return received data */
    return UDR3;
}

