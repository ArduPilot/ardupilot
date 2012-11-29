
#ifndef __AP_HAL_AVR_SPI_DEVICES_H__
#define __AP_HAL_AVR_SPI_DEVICES_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::AVRSPI0DeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    AVRSPI0DeviceDriver(
            AP_HAL_AVR::AVRDigitalSource *cs_pin,
            uint8_t spcr,
            uint8_t spsr
    ) :
        _cs_pin(cs_pin),
        _spcr(spcr),
        _spsr(spsr)
    {}

    void init();
    AP_HAL::Semaphore* get_semaphore();
    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);

private:
    static AP_HAL_AVR::AVRSemaphore _semaphore;

    AP_HAL_AVR::AVRDigitalSource *_cs_pin;
    const uint8_t _spcr;
    const uint8_t _spsr;
};


class AP_HAL_AVR::AVRSPI2DeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    AVRSPI2DeviceDriver(
            AP_HAL_AVR::AVRDigitalSource *cs_pin,
            uint8_t ucsr2c,
            uint16_t ubrr2
    ) :
        _cs_pin(cs_pin),
        _ucsr2c(ucsr2c),
        _ubrr2(ubrr2)
    {}

    void init();
    AP_HAL::Semaphore* get_semaphore();
    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);

private:
    static AP_HAL_AVR::AVRSemaphore _semaphore;

    AP_HAL_AVR::AVRDigitalSource *_cs_pin;
    uint8_t _ucsr2c;
    uint16_t _ubrr2;
};

class AP_HAL_AVR::AVRSPI3DeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    AVRSPI3DeviceDriver(
            AP_HAL_AVR::AVRDigitalSource *cs_pin,
            uint8_t ucsr3c,
            uint16_t ubrr3
    ) :
        _cs_pin(cs_pin),
        _ucsr3c(ucsr3c),
        _ubrr3(ubrr3)
    {}

    void init();
    AP_HAL::Semaphore* get_semaphore();
    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);

private:
    static AP_HAL_AVR::AVRSemaphore _semaphore;

    AP_HAL_AVR::AVRDigitalSource *_cs_pin;
    uint8_t _ucsr3c;
    uint16_t _ubrr3;

};


#endif // __AP_HAL_AVR_SPI_DEVICES_H__
