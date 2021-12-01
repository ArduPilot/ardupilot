#pragma once

#include "AP_RangeFinder_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

class AP_RangeFinder_Benewake_CAN : public AP_RangeFinder_Backend, public CANSensor {
public:
    AP_RangeFinder_Benewake_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    void update() override;

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }
private:
    float _distance_sum_cm;
    uint32_t _distance_count;
    int32_t last_recv_id = -1;

    AP_Int32 snr_min;
    AP_Int32 receive_id;
};
#endif //HAL_MAX_CAN_PROTOCOL_DRIVERS

