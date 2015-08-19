#include <AP_HAL/AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <avr/io.h>

#include <AP_HAL/AP_HAL.h>
#include "SPIDevices.h"
#include "GPIO.h"
#include "Semaphores.h"
#include "utility/pins_arduino_mega.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

#define SPI0_MISO_PIN 50
#define SPI0_MOSI_PIN 51
#define SPI0_SCK_PIN  52

AVRSemaphore AVRSPI0DeviceDriver::_semaphore;
bool AVRSPI0DeviceDriver::_force_low_speed;

static volatile bool spi0_transferflag = false;

void AVRSPI0DeviceDriver::init() {
    hal.gpio->pinMode(SPI0_MISO_PIN, HAL_GPIO_INPUT);
    hal.gpio->pinMode(SPI0_MOSI_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(SPI0_SCK_PIN, HAL_GPIO_OUTPUT);

    _cs_pin->mode(HAL_GPIO_OUTPUT);
    _cs_pin->write(1);

    /* Enable the SPI0 peripheral as a master */
    SPCR = _BV(SPE) | _BV(MSTR);
}

AP_HAL::Semaphore* AVRSPI0DeviceDriver::get_semaphore() {
    return &_semaphore;
}

void AVRSPI0DeviceDriver::_cs_assert() 
{
    const uint8_t valid_spcr_mask = 
        (_BV(CPOL) | _BV(CPHA) | _BV(SPR1) | _BV(SPR0));
    if (_force_low_speed) {
        _spcr = _spcr_lowspeed;
    }
    uint8_t new_spcr = (SPCR & ~valid_spcr_mask) | (_spcr & valid_spcr_mask);
    SPCR = new_spcr;  

    const uint8_t valid_spsr_mask = _BV(SPI2X);
    uint8_t new_spsr = (SPSR & ~valid_spsr_mask) | (_spsr & valid_spsr_mask);
    SPSR = new_spsr;

    _cs_pin->write(0);
}

void AVRSPI0DeviceDriver::_cs_release() 
{
    _cs_pin->write(1);
}

uint8_t AVRSPI0DeviceDriver::_transfer(uint8_t data) 
{
    if (spi0_transferflag) {
        hal.scheduler->panic(PSTR("PANIC: SPI0 transfer collision"));
    }
    spi0_transferflag = true;
    SPDR = data;
    if (SPSR & _BV(WCOL)) {
        hal.scheduler->panic(PSTR("PANIC: SPI0 write collision"));
        return 0;
    }
    while(!(SPSR & _BV(SPIF)));
    uint8_t read_spdr = SPDR;
    spi0_transferflag = false;
    return read_spdr;
}

/**
   a specialised transfer function for the MPU6k. This saves 2 usec
   per byte
 */
void AVRSPI0DeviceDriver::_transfer16(const uint8_t *tx, uint8_t *rx) 
{
    spi0_transferflag = true;
#define TRANSFER1(i) do { SPDR = tx[i];  while(!(SPSR & _BV(SPIF))); rx[i] = SPDR; } while(0)
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
    spi0_transferflag = false;
}

void AVRSPI0DeviceDriver::transfer(const uint8_t *tx, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
            _transfer(tx[i]);
    }
}

bool AVRSPI0DeviceDriver::transaction(const uint8_t *tx, uint8_t *rx,
        uint16_t len) {
    _cs_assert();
    if (rx == NULL) {
        for (uint16_t i = 0; i < len; i++) {
            _transfer(tx[i]);
        }
    } else {
        while (len >= 16) {
            _transfer16(tx, rx);
            tx += 16;
            rx += 16;
            len -= 16;
        }
        for (uint16_t i = 0; i < len; i++) {
            rx[i] = _transfer(tx[i]);
        }
    }
    _cs_release();
    return true;
}

void AVRSPI0DeviceDriver::cs_assert() {
    _cs_assert();
}

void AVRSPI0DeviceDriver::cs_release() {
    _cs_release();
}

uint8_t AVRSPI0DeviceDriver::transfer(uint8_t data) {
    return _transfer(data);
}

/**
   allow on the fly bus speed changes for MPU6000
 */
void AVRSPI0DeviceDriver::set_bus_speed(AVRSPI0DeviceDriver::bus_speed speed) 
{
    if (speed == AVRSPI0DeviceDriver::SPI_SPEED_HIGH) {
        _spcr = _spcr_highspeed;
        _force_low_speed = false;
    } else {
        _spcr = _spcr_lowspeed;
        _force_low_speed = true;
    }
}

#endif
