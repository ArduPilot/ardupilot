
#ifndef __AP_HAL_AVR_SPI_DEVICES_H__
#define __AP_HAL_AVR_SPI_DEVICES_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::AVRSPI0DeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    AVRSPI0DeviceDriver(
            AP_HAL_AVR::AVRDigitalSource *cs_pin,
            uint8_t spcr_lowspeed,
            uint8_t spcr_highspeed,
            uint8_t spsr
    ) :
        _cs_pin(cs_pin),
        _spcr_lowspeed(spcr_lowspeed),
        _spcr_highspeed(spcr_highspeed),
        _spcr(spcr_lowspeed),
        _spsr(spsr)
    {}

    void init();
    AP_HAL::Semaphore* get_semaphore();

    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);
    void transfer(const uint8_t *data, uint16_t len);
    void set_bus_speed(enum bus_speed speed);

private:
    void _cs_assert();
    void _cs_release();
    uint8_t _transfer(uint8_t data);
    // used for MPU6k
    void _transfer16(const uint8_t *tx, uint8_t *rx);

    static AP_HAL_AVR::AVRSemaphore _semaphore;
    static bool _force_low_speed;

    AP_HAL_AVR::AVRDigitalSource *_cs_pin;
    const uint8_t _spcr_lowspeed;
    const uint8_t _spcr_highspeed;
    uint8_t _spcr;
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

    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);
    void transfer(const uint8_t *data, uint16_t len);

private:
    void _cs_assert();
    void _cs_release();
    uint8_t _transfer(uint8_t data);
    // used for APM1 ADC
    void _transfer17(const uint8_t *tx, uint8_t *rx);

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

    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer(uint8_t data);
    void transfer(const uint8_t *data, uint16_t len);

private:
    void _cs_assert();
    void _cs_release();
    uint8_t _transfer(uint8_t data);
    void _transfer(const uint8_t *data, uint16_t size);
    static AP_HAL_AVR::AVRSemaphore _semaphore;

    AP_HAL_AVR::AVRDigitalSource *_cs_pin;
    uint8_t _ucsr3c;
    uint16_t _ubrr3;

};


#endif // __AP_HAL_AVR_SPI_DEVICES_H__
