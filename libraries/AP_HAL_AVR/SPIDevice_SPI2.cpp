/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <avr/io.h>

#include <AP_HAL.h>
#include "SPIDevices.h"
#include "GPIO.h"
#include "Semaphores.h"
#include "utility/pins_arduino_mega.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

AVRSemaphore AVRSPI2DeviceDriver::_semaphore;

void AVRSPI2DeviceDriver::init() {
    AVRDigitalSource spi2_miso(_BV(0), PH);
    spi2_miso.mode(GPIO_INPUT);

    AVRDigitalSource spi2_mosi(_BV(1), PH);
    spi2_mosi.mode(GPIO_OUTPUT);

    AVRDigitalSource spi2_sck(_BV(2), PH);
    spi2_sck.mode(GPIO_OUTPUT);

    /* UMSELn1 and UMSELn2: USART in SPI Master mode */
    UCSR2C = _BV(UMSEL21) | _BV(UMSEL20);
    /* Enable RX and TX. */
    UCSR2B = _BV(RXEN2) | _BV(TXEN2);

    /* Setup chip select pin */
    _cs_pin->mode(GPIO_OUTPUT);
    _cs_pin->write(1);
}

AP_HAL::Semaphore* AVRSPI2DeviceDriver::get_semaphore() {
    return &_semaphore;
}

inline void AVRSPI2DeviceDriver::_cs_assert() {
    /* set the device UCSRnC configuration bits.
     * only sets data order, clock phase, and clock polarity bits (lowest
     * three bits)  */
    const uint8_t new_ucsr2c = (UCSR2C & ~0x07) | (_ucsr2c & (0x07));
    UCSR2C = new_ucsr2c;
    /* set the device baud rate */
    UBRR2 = _ubrr2;

    _cs_pin->write(0);
}

inline void AVRSPI2DeviceDriver::_cs_release() {
    _cs_pin->write(1);
}

inline uint8_t AVRSPI2DeviceDriver::_transfer(uint8_t data) {
    /* Wait for empty transmit buffer */
    while ( !( UCSR2A & _BV(UDRE2)) ) ;

    /* Put data into buffer, sends the data */
    UDR2 = data;

    /* Wait for data to be received */
    while ( !(UCSR2A & _BV(RXC2)) ) ;

    /* Get and return received data from buffer */
    return UDR2;
}

/**
   a specialist transfer function for the APM1 ADC
 */
void AVRSPI2DeviceDriver::_transfer17(const uint8_t *tx, uint8_t *rx) 
{
#define TRANSFER1(i) do { while ( !( UCSR2A & _BV(UDRE2)) ); \
        UDR2 = tx[i]; \
        while ( !(UCSR2A & _BV(RXC2)) ) ; \
        rx[i] = UDR2; } while (0)
    TRANSFER1(0);
    TRANSFER1(1);
    TRANSFER1(2);
    TRANSFER1(3);
    TRANSFER1(4);
    TRANSFER1(5);
    TRANSFER1(6);
    TRANSFER1(7);
    TRANSFER1(8);
    TRANSFER1(9);
    TRANSFER1(10);
    TRANSFER1(11);
    TRANSFER1(12);
    TRANSFER1(13);
    TRANSFER1(14);
    TRANSFER1(15);
    TRANSFER1(16);
}

void AVRSPI2DeviceDriver::transaction(const uint8_t *tx, uint8_t *rx,
        uint16_t len) {
    _cs_assert();
    if (rx == NULL) {
        for (uint16_t i = 0; i < len; i++) {
            _transfer(tx[i]);
        }
    } else {
        while (len >= 17) {
            _transfer17(tx, rx);
            tx += 17;
            rx += 17;
            len -= 17;
        }
        for (uint16_t i = 0; i < len; i++) {
            rx[i] = _transfer(tx[i]);
        }
    }
    _cs_release();
}

void AVRSPI2DeviceDriver::cs_assert() {
    _cs_assert();
}

void AVRSPI2DeviceDriver::cs_release() {
    _cs_release();
}

uint8_t AVRSPI2DeviceDriver::transfer(uint8_t data) {
    return _transfer(data);
}

void AVRSPI2DeviceDriver::transfer(const uint8_t *data, uint16_t len) {
    while (len--)
        _transfer(*data++);
}

#endif
