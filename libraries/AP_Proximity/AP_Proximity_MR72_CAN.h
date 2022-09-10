#pragma once
#include "AP_Proximity.h"


#ifndef AP_PROXIMITY_MR72_ENABLED
#define AP_PROXIMITY_MR72_ENABLED (HAL_PROXIMITY_ENABLED && HAL_MAX_CAN_PROTOCOL_DRIVERS)
#endif

#if (HAL_PROXIMITY_ENABLED && AP_PROXIMITY_MR72_ENABLED)

#include "AP_Proximity_Backend.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>

#define MR72_MAX_RANGE_M             50.0f   // max range of the sensor in meters
#define MR72_MIN_RANGE_M             0.2f   // min range of the sensor in meters

class MR72_MultiCAN;

class AP_Proximity_MR72_CAN : public AP_Proximity_Backend {
public:
    friend class MR72_MultiCAN;

    AP_Proximity_MR72_CAN(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_Proximity_Params& _params);

    void update() override;

    // handler for incoming frames. Return true if consumed
    bool handle_frame(AP_HAL::CANFrame &frame);

    bool parse_distance_message(AP_HAL::CANFrame &frame);

      // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return MR72_MAX_RANGE_M; }
    float distance_min() const override { return MR72_MIN_RANGE_M; }

    static const struct AP_Param::GroupInfo var_info[];

    AP_Proximity_Temp_Boundary _temp_boundary;

private:
    uint32_t _object_count;
    uint32_t _current_object_index;

    AP_Int32 receive_id;

    uint32_t last_update_ms;

    static MR72_MultiCAN *multican;
    AP_Proximity_MR72_CAN *next;
};

// a class to allow for multiple MR_72 backends with one
// CANSensor driver
class MR72_MultiCAN : public CANSensor {
public:
    MR72_MultiCAN() : CANSensor("MR72") {
        register_driver(AP_CANManager::Driver_Type_MR72);
    }

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    HAL_Semaphore sem;
    AP_Proximity_MR72_CAN *drivers;

};

#endif // HAL_PROXIMITY_ENABLED

