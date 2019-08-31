#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_Lanbao : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_Lanbao(RangeFinder::RangeFinder_State &_state,
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
    uint8_t buf[6];
    uint8_t buf_len = 0;
};
