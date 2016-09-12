/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Baro_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

class AP_Baro_MS56XX : public AP_Baro_Backend
{
public:
    void update();

protected:
    /*
     * Update @accum and @count with the new sample in @val, taking into
     * account a maximum number of samples given by @max_count; in case
     * maximum number is reached, @accum and @count are updated appropriately
     */
    static void _update_and_wrap_accumulator(uint32_t *accum, uint32_t val,
                                             uint8_t *count, uint8_t max_count);

    AP_Baro_MS56XX(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
    void _init();

    virtual void _calculate() = 0;
    virtual bool _read_prom(uint16_t prom[8]);

    uint16_t _read_prom_word(uint8_t word);
    uint32_t _read_adc();

    bool _timer();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /*
     * Synchronize access to _accum between thread sampling the HW and main
     * thread using the values
     */
    AP_HAL::Semaphore *_sem;

    /* Shared values between thread sampling the HW and main thread */
    struct {
        uint32_t s_D1;
        uint32_t s_D2;
        uint8_t d1_count;
        uint8_t d2_count;
    } _accum;

    uint8_t _state;
    uint8_t _instance;
    uint32_t _last_cmd_usec;

    /* Last compensated values from accumulated sample */
    float _D1, _D2;

    // Internal calibration registers
    struct {
        uint16_t c1, c2, c3, c4, c5, c6;
    } _cal_reg;
};

class AP_Baro_MS5611 : public AP_Baro_MS56XX
{
public:
    AP_Baro_MS5611(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
protected:
    void _calculate() override;
};

class AP_Baro_MS5607 : public AP_Baro_MS56XX
{
public:
    AP_Baro_MS5607(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
protected:
    void _calculate() override;
};

class AP_Baro_MS5637 : public AP_Baro_MS56XX
{
public:
    AP_Baro_MS5637(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
protected:
    void _calculate() override;
    bool _read_prom(uint16_t prom[8]) override;
};
