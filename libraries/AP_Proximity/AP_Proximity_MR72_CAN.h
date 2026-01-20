#pragma once
#include "AP_Proximity_config.h"

#if AP_PROXIMITY_MR72_DRIVER_ENABLED

#include "AP_Proximity.h"
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

    // parse a distance message from CAN frame
    bool parse_distance_message(AP_HAL::CANFrame &frame);

    // get maximum and minimum distances (in meters) of sensor
    float distance_max_m() const override { return MR72_MAX_RANGE_M; }
    float distance_min_m() const override { return MR72_MIN_RANGE_M; }

    static const struct AP_Param::GroupInfo var_info[];

    AP_Proximity_Temp_Boundary _temp_boundary;

private:

    uint32_t get_radar_id(uint32_t id) const { return ((id & 0xF0U) >> 4U); }

    uint32_t _object_count;             // total number of objects to read
    uint32_t _current_object_index;     // current object index
    uint32_t last_update_ms;            // last update time in ms

    AP_Int32 receive_id;                // ID of the sensor

    MultiCAN* multican_MR72;            // Allows for multiple CAN rangefinders on a single bus
};

#endif  // AP_PROXIMITY_MR72_DRIVER_ENABLED
