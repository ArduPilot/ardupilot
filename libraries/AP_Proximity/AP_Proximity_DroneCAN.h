#pragma once

#include "AP_Proximity_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

#ifndef AP_PROXIMITY_DRONECAN_ENABLED
#define AP_PROXIMITY_DRONECAN_ENABLED (HAL_CANMANAGER_ENABLED && HAL_PROXIMITY_ENABLED)
#endif

#if AP_PROXIMITY_DRONECAN_ENABLED
class MeasurementCb;

class AP_Proximity_DroneCAN : public AP_Proximity_Backend
{
public:
    // constructor
    using AP_Proximity_Backend::AP_Proximity_Backend;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;


   static AP_Proximity_DroneCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t address, bool create_new);


    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);

    static void handle_measurement(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MeasurementCb &cb);

private:

    uint32_t _last_update_ms;   // system time of last message received

    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;

    struct ObstacleItem {
        float yaw_deg;
        float pitch_deg;
        float distance_m;
    };

    static ObjectBuffer_TS<ObstacleItem> items;

    AP_Proximity::Status _status;
};

#endif // AP_PROXIMITY_DRONECAN_ENABLED
