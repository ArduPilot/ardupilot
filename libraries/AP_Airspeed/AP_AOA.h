#pragma once
#include <AP_HAL/AP_HAL.h>
#include "AP_Airspeed_Backend.h"

class AP_AOA : public AP_Airspeed_Backend {
public: 
    AP_AOA(AP_Airspeed &fs, uint8_t inst, int8_t bus, uint8_t address);

    bool init() override;
    bool get_differential_pressure(float &pressure) override;
    bool get_airspeed(float &airspeed) override { return false; }

    private:
    void timer();      

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint8_t _address;
    float _last_pressureAOA = 0.0f;
    float _last_AoA = 0.0f;
    bool _healthy = false;

};
