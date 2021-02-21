#pragma once

#include "AP_RangeFinder_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

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
    bool new_data;
    uint16_t _distance_cm;
    uint32_t _last_reading_ms;
};
#endif //HAL_MAX_CAN_PROTOCOL_DRIVERS

