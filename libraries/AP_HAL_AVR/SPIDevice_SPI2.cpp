

#include <avr/io.h>

#include <AP_HAL.h>
#include "SPIDevices.h"
#include "GPIO.h"
#include "Semaphore.h"
#include "pins_arduino_mega.h"
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

void AVRSPI2DeviceDriver::cs_assert() {
    /* set the device UCSRnC configuration bits.
     * only sets data order, clock phase, and clock polarity bits (lowest
     * three bits)  */
    const uint8_t new_ucsr2c = UCSR2C | (_ucsr2c & (0x07));
    UCSR2C = new_ucsr2c;
    /* set the device baud rate */
    UBRR2 = _ubrr2;

    _cs_pin->write(0);
}

void AVRSPI2DeviceDriver::cs_release() {
    _cs_pin->write(1);
}

uint8_t AVRSPI2DeviceDriver::transfer(uint8_t data) {
    /* Wait for empty transmit buffer */
    while ( !( UCSR2A & _BV(UDRE2)) ) ;

    /* Put data into buffer, sends the data */
    UDR2 = data;

    /* Wait for data to be received */
    while ( !(UCSR2A & _BV(RXC2)) ) ;

    /* Get and return received data from buffer */
    return UDR2;
}

