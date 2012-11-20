/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_MS5611_H__
#define __AP_BARO_MS5611_H__

#include <AP_HAL.h>
#include "AP_Baro.h"

class AP_Baro_MS5611 : public AP_Baro
{
public:
    AP_Baro_MS5611() {}

    /* AP_Baro public interface: */
    bool            init();
    uint8_t         read();
    float           get_pressure(); // in mbar*100 units
    float           get_temperature(); // in celsius degrees * 100 units

    int32_t         get_raw_pressure();
    int32_t         get_raw_temp();

    void            _calculate();

private:
    /* Asynchronous handler functions: */
    static void                     _update(uint32_t );
    /* SPI device driver used from asynchronous function: */
    static AP_HAL::SPIDeviceDriver *_spi;
    static AP_HAL::Semaphore *_spi_sem;
    /* Asynchronous state: */
    static volatile bool            _updated;
    static volatile uint8_t         _d1_count;
    static volatile uint8_t         _d2_count;
    static volatile uint32_t        _s_D1, _s_D2;
    static uint8_t                  _state;
    static uint32_t                 _timer;
    /* Gates access to asynchronous state: */
    static bool                     _sync_access;

    /* Serial wrapper functions: */
    static uint8_t                  _spi_read(uint8_t reg);
    static uint16_t                 _spi_read_16bits(uint8_t reg);
    static uint32_t                 _spi_read_adc();
    static void                     _spi_write(uint8_t reg);


    float                           Temp;
    float                           Press;

    int32_t                         _raw_press;
    int32_t                         _raw_temp;
    // Internal calibration registers
    uint16_t                        C1,C2,C3,C4,C5,C6;
    float                           D1,D2;

};

#endif //  __AP_BARO_MS5611_H__
