
#include <avr/io.h>
#include "utility/pins_arduino_mega.h"

#include <AP_HAL.h>
#include "SPIDriver.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

void ArduinoSPIDriver::init(void* machtnichts) {
    hal.gpio->pinMode(SCK, GPIO_OUTPUT);
    hal.gpio->pinMode(MOSI, GPIO_OUTPUT);
    hal.gpio->pinMode(SS, GPIO_OUTPUT);

    hal.gpio->write(SCK, 0);
    hal.gpio->write(MOSI,0);
    hal.gpio->write(SS,  0);

    SPCR |= _BV(MSTR);
    SPCR |= _BV(SPE);

    set_freq( 1000000);
}

void ArduinoSPIDriver::set_freq(uint32_t freq_hz) {
    const uint16_t freq_khz = (uint16_t)(freq_hz / 1000UL);
    const uint16_t fcpu_khz = (uint16_t)(F_CPU / 1000UL);
    const int16_t div = fcpu_khz / freq_khz;
    /* can't divide clock by more than 128. */
    uint8_t b = _divider_bits( div > 128 ? 128 : ((uint8_t) div));
    _set_clock_divider_bits(b);
}

/* from Arduino SPI.h:
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV32 0x06
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
*/
uint8_t ArduinoSPIDriver::_divider_bits(uint8_t div) {
    /* Closest bits for divider without going over (ending up faster) */
    if (div > 64) return 0x03;
    if (div > 32) return 0x02; 
    if (div > 16) return 0x06;
    if (div > 8) return 0x01;
    if (div > 4) return 0x05;
    if (div > 2) return 0x00;
    return 0x04;
}

#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR

void ArduinoSPIDriver::_set_clock_divider_bits(uint8_t b) {
    SPCR = (SPCR & ~SPI_CLOCK_MASK) | ( b & SPI_CLOCK_MASK);
    SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((b >> 2) & SPI_2XCLOCK_MASK);
}

uint8_t ArduinoSPIDriver::transfer(uint8_t data) {
    SPDR = data;
    while(!(SPSR & _BV(SPIF)));
    return SPDR;
}
