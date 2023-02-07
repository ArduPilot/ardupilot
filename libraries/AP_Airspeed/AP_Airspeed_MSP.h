/*
  MSP airspeed backend
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_MSP/msp.h>

#ifndef AP_AIRSPEED_MSP_ENABLED
#define AP_AIRSPEED_MSP_ENABLED HAL_MSP_SENSORS_ENABLED
#endif

#if AP_AIRSPEED_MSP_ENABLED

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_MSP : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_MSP(AP_Airspeed &airspeed, uint8_t instance, uint8_t msp_instance);

    bool init(void) override {
        return true;
    }

    void handle_msp(const MSP::msp_airspeed_data_message_t &pkt) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override;
    
private:
    const uint8_t msp_instance;
    float sum_pressure;
    uint8_t press_count;
    float sum_temp;
    uint8_t temp_count;
};

#endif // AP_AIRSPEED_MSP_ENABLED
