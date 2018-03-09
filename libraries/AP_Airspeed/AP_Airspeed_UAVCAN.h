#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "AP_Airspeed_Backend.h"
#include <AP_UAVCAN/AP_UAVCAN.h>

class AP_Airspeed_UAVCAN : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_UAVCAN(AP_Airspeed &frontend, uint8_t _instance);
    ~AP_Airspeed_UAVCAN() override;

    // probe and initialise the sensor
    bool init(void) override;
    
    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override;
    
    // This method is called from UAVCAN thread
    virtual void handle_airspeed_msg(float pressure, float temperature) override;

private:
    bool register_uavcan_airspeed(uint8_t mgr, uint8_t node);

    AP_HAL::Semaphore *_sem_airspeed;
    bool _initialized;
    uint8_t _manager;
    float _pressure; // Pascal
    float _temperature; // Kelvin
};
