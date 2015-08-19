/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)


#include <avr/io.h>

#include <AP_HAL/AP_HAL.h>
#include "SPIDevices.h"
#include "GPIO.h"
#include "Semaphores.h"

#include "utility/pins_arduino_mega.h"

using namespace AP_HAL_AVR;

#define SPI3_MOSI 14
#define SPI3_MISO 15

extern const AP_HAL::HAL& hal;

AVRSemaphore AVRSPI3DeviceDriver::_semaphore;

void AVRSPI3DeviceDriver::init() {
    /* the spi3 (USART3) sck pin PORTJ2 is not enumerated
     * by the arduino pin numbers, so we access it directly
     * with AVRDigitalSource. */
    AVRDigitalSource spi3_sck(_BV(2), PJ);
    spi3_sck.mode(HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(SPI3_MOSI, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(SPI3_MISO, HAL_GPIO_INPUT);

    /* UMSELn1 and UMSELn2: USART in SPI Master mode */
    UCSR3C = _BV(UMSEL31) | _BV(UMSEL30);
    /* Enable RX and TX. */
    UCSR3B = _BV(RXEN3) | _BV(TXEN3);

    /* Setup chip select pin */
    _cs_pin->mode(HAL_GPIO_OUTPUT);
    _cs_pin->write(1);
}

AP_HAL::Semaphore* AVRSPI3DeviceDriver::get_semaphore() {
    return &_semaphore;
}

void AVRSPI3DeviceDriver::_cs_assert() 
{
    /* set the device UCSRnC configuration bits.
     * only sets data order, clock phase, and clock polarity bits (lowest
     * three bits)  */
    const uint8_t new_ucsr3c = (UCSR3C & ~0x07) | (_ucsr3c & (0x07));
    UCSR3C = new_ucsr3c;
    /* set the device baud rate */
    UBRR3 = _ubrr3;

    _cs_pin->write(0);
}

void AVRSPI3DeviceDriver::_cs_release() {
    _cs_pin->write(1);
}

uint8_t AVRSPI3DeviceDriver::_transfer(uint8_t data) {
    /* Wait for empty transmit buffer */
    while ( !( UCSR3A & _BV(UDRE3)) ) ;

    /* Put data into buffer, sends the data */
    UDR3 = data;

    /* Wait for data to be received */
    while ( !(UCSR3A & _BV(RXC3)) ) ;

    /* Get and return received data from buffer */
    return UDR3;
}

void AVRSPI3DeviceDriver::_transfer(const uint8_t *data, uint16_t len) {
    while (len--) {
        /* Wait for empty transmit buffer */
        while ( !( UCSR3A & _BV(UDRE3)) ) ;

        /* Put data into buffer, sends the data */
        UDR3 = *data++;

        /* Wait for data to be received */
        while ( !(UCSR3A & _BV(RXC3)) ) ;

        // dummy read of UDR3 to complete
        UDR3;
    }
}

bool AVRSPI3DeviceDriver::transaction(const uint8_t *tx, uint8_t *rx,
        uint16_t len) {
    _cs_assert();
    if (rx == NULL) {
        _transfer(tx, len);
    } else {
        for (uint16_t i = 0; i < len; i++) {
            rx[i] = _transfer(tx[i]);
        }
    }
    _cs_release();
    return true;
}

void AVRSPI3DeviceDriver::cs_assert() {
    _cs_assert();
}

void AVRSPI3DeviceDriver::cs_release() {
    _cs_release();
}

uint8_t AVRSPI3DeviceDriver::transfer(uint8_t data) {
    return _transfer(data);
}

void AVRSPI3DeviceDriver::transfer(const uint8_t *data, uint16_t len) {
    _transfer(data, len);
}

#endif
