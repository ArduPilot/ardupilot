#pragma once

#include "AP_RangeFinder_Backend.h"

#include <AP_CANManager/AP_CANSensor.h>

#ifndef AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
#define AP_RANGEFINDER_BENEWAKE_CAN_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#if AP_RANGEFINDER_BENEWAKE_CAN_ENABLED

class Benewake_MultiCAN;

class AP_RangeFinder_Benewake_CAN : public AP_RangeFinder_Backend {
public:
    friend class Benewake_MultiCAN;

    AP_RangeFinder_Benewake_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    void update() override;

    // handler for incoming frames. Return true if consumed
    bool handle_frame(AP_HAL::CANFrame &frame);
    bool handle_frame_H30(AP_HAL::CANFrame &frame);

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

    static Benewake_MultiCAN *multican;
    AP_RangeFinder_Benewake_CAN *next;
};

// a class to allow for multiple Benewake_CAN backends with one
// CANSensor driver
class Benewake_MultiCAN : public CANSensor {
public:
    Benewake_MultiCAN() : CANSensor("Benewake") {
        register_driver(AP_CAN::Protocol::Benewake);
    }

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    HAL_Semaphore sem;
    AP_RangeFinder_Benewake_CAN *drivers;
};

#endif  // AP_RANGEFINDER_BENEWAKE_CAN_ENABLED


