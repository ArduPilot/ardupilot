/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_MS5611_H__
#define __AP_BARO_MS5611_H__

#include <AP_HAL.h>
#include "AP_Baro.h"

/** Abstract serial device driver for MS5611. */
class AP_Baro_MS5611_Serial
{
public:
    /** Initialize the driver. */
    virtual void init() = 0;

    /** Read a 16-bit value from register "reg". */
    virtual uint16_t read_16bits(uint8_t reg) = 0;

    /** Read a 24-bit value from the ADC. */
    virtual uint32_t read_adc() = 0;

    /** Write a single byte command. */
    virtual void write(uint8_t reg) = 0;

    /** Acquire the internal semaphore for this device. */
    virtual void sem_get() {}

    /** Release the internal semaphore for this device. */
    virtual void sem_release() {}
};

/** SPI serial device. */
class AP_Baro_MS5611_SPI : public AP_Baro_MS5611_Serial
{
public:
    virtual void init();
    virtual uint16_t read_16bits(uint8_t reg);
    virtual uint32_t read_adc();
    virtual void write(uint8_t reg);
    virtual void sem_get();
    virtual void sem_release();

private:
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
};

/** I2C serial device. */
class AP_Baro_MS5611_I2C : public AP_Baro_MS5611_Serial
{
public:
    virtual void init();
    virtual uint16_t read_16bits(uint8_t reg);
    virtual uint32_t read_adc();
    virtual void write(uint8_t reg);
};

class AP_Baro_MS5611 : public AP_Baro
{
public:
    AP_Baro_MS5611(AP_Baro_MS5611_Serial *serial)
    {
        _serial = serial;
    }

    /* AP_Baro public interface: */
    bool            init();
    uint8_t         read();
    float           get_pressure(); // in mbar*100 units
    float           get_temperature(); // in celsius degrees * 100 units

    int32_t         get_raw_pressure();
    int32_t         get_raw_temp();

    void            _calculate();

    /* Serial port drivers to pass to "init". */
    static AP_Baro_MS5611_SPI spi;
    static AP_Baro_MS5611_I2C i2c;

private:
    /* Asynchronous handler functions: */
    static void                     _update(uint32_t );
    /* Asynchronous state: */
    static volatile bool            _updated;
    static volatile uint8_t         _d1_count;
    static volatile uint8_t         _d2_count;
    static volatile uint32_t        _s_D1, _s_D2;
    static uint8_t                  _state;
    static uint32_t                 _timer;
    static AP_Baro_MS5611_Serial   *_serial;
    /* Gates access to asynchronous state: */
    static bool                     _sync_access;

    float                           Temp;
    float                           Press;

    int32_t                         _raw_press;
    int32_t                         _raw_temp;
    // Internal calibration registers
    uint16_t                        C1,C2,C3,C4,C5,C6;
    float                           D1,D2;

};

#endif //  __AP_BARO_MS5611_H__
