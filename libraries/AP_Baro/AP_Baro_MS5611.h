/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Baro_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>

class AP_Baro_MS56XX : public AP_Baro_Backend
{
public:
    void update();
    void accumulate();

protected:
    AP_Baro_MS56XX(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, bool use_timer);
    void _init();

    virtual void _calculate() = 0;
    virtual bool _read_prom(uint16_t prom[8]);

    uint16_t _read_prom_word(uint8_t word);
    uint32_t _read_adc();

    void _timer();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /* Asynchronous state: */
    volatile bool            _updated;
    volatile uint8_t         _d1_count;
    volatile uint8_t         _d2_count;
    volatile uint32_t        _s_D1, _s_D2;
    uint8_t                  _state;
    uint32_t                 _last_timer;
    bool                     _timesliced;

    bool _use_timer;

    // Internal calibration registers
    uint16_t                 _c1,_c2,_c3,_c4,_c5,_c6;
    float                    _D1, _D2;
    uint8_t _instance;
};

class AP_Baro_MS5611 : public AP_Baro_MS56XX
{
public:
    AP_Baro_MS5611(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, bool use_timer);
protected:
    void _calculate();
};

class AP_Baro_MS5607 : public AP_Baro_MS56XX
{
public:
    AP_Baro_MS5607(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, bool use_timer);
protected:
    void _calculate();
};

class AP_Baro_MS5637 : public AP_Baro_MS56XX
{
public:
    AP_Baro_MS5637(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, bool use_timer);
protected:
    void _calculate();
    bool _read_prom(uint16_t prom[8]) override;
};
