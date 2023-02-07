#pragma once

#include "AP_RangeFinder_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

#ifndef AP_RANGEFINDER_USD1_CAN_ENABLED
#define AP_RANGEFINDER_USD1_CAN_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#if AP_RANGEFINDER_USD1_CAN_ENABLED

class AP_RangeFinder_USD1_CAN : public CANSensor, public AP_RangeFinder_Backend {
public:
    AP_RangeFinder_USD1_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    void update() override;

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;
    
protected:
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }
private:
    float _distance_sum;
    uint32_t _distance_count;
};

#endif  // AP_RANGEFINDER_USD1_CAN_ENABLED
