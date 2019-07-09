#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_LightWareSerial : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_LightWareSerial(RangeFinder::RangeFinder_State &_state,
                                   AP_RangeFinder_Params &_params,
                                   uint8_t serial_instance);

    // static detection function
    static bool detect(uint8_t serial_instance);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    AP_HAL::UARTDriver *uart = nullptr;
    char linebuf[10];
    uint8_t linebuf_len = 0;
    uint32_t last_init_ms;
};
