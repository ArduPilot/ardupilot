#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <avr/io.h>

#include <AP_HAL.h>
#include "SPIDevices.h"
#include "GPIO.h"
#include "Semaphore.h"
#include "pins_arduino_mega.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

#define SPI0_MISO_PIN 50
#define SPI0_MOSI_PIN 51
#define SPI0_SCK_PIN  52

AVRSemaphore AVRSPI0DeviceDriver::_semaphore;

static volatile bool spi0_transferflag = false;

void AVRSPI0DeviceDriver::init() {
    hal.gpio->pinMode(SPI0_MISO_PIN, GPIO_INPUT);
    hal.gpio->pinMode(SPI0_MOSI_PIN, GPIO_OUTPUT);
    hal.gpio->pinMode(SPI0_SCK_PIN, GPIO_OUTPUT);

    _cs_pin->mode(GPIO_OUTPUT);
    _cs_pin->write(1);

    /* Enable the SPI0 peripheral as a master */
    SPCR = _BV(SPE) | _BV(MSTR);
}

AP_HAL::Semaphore* AVRSPI0DeviceDriver::get_semaphore() {
    return &_semaphore;
}

void AVRSPI0DeviceDriver::cs_assert() {
    const uint8_t valid_spcr_mask = 
        (_BV(CPOL) | _BV(CPHA) | _BV(SPR1) | _BV(SPR0));
    uint8_t new_spcr = SPCR | (_spcr & valid_spcr_mask);
    SPCR = new_spcr;  

    const uint8_t valid_spsr_mask = _BV(SPI2X);
    uint8_t new_spsr = SPSR | (_spsr & valid_spsr_mask);
    SPSR = new_spsr;

    _cs_pin->write(0);
}

void AVRSPI0DeviceDriver::cs_release() {
    _cs_pin->write(1);
}

uint8_t AVRSPI0DeviceDriver::transfer(uint8_t data) {
    if (spi0_transferflag) {
        hal.console->println_P(PSTR("PANIC: SPI0 transfer collision"));
    }
    spi0_transferflag = true;
    SPDR = data;
    if (SPSR & _BV(WCOL)) {
        hal.console->println_P(PSTR("PANIC: SPI0 write collision"));
        return 0;
    }
    while(!(SPSR & _BV(SPIF)));
    uint8_t read_spdr = SPDR;
    spi0_transferflag = false;
    return read_spdr;
}

#endif
