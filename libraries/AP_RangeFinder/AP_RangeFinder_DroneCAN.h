#pragma once

#include "AP_RangeFinder_Backend.h"

#ifndef AP_RANGEFINDER_DRONECAN_ENABLED
#define AP_RANGEFINDER_DRONECAN_ENABLED (HAL_ENABLE_DRONECAN_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#if AP_RANGEFINDER_DRONECAN_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>

class MeasurementCb;

class AP_RangeFinder_DroneCAN : public AP_RangeFinder_Backend {
public:
    //constructor - registers instance at top RangeFinder driver
    using AP_RangeFinder_Backend::AP_RangeFinder_Backend;

    void update() override;

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static AP_RangeFinder_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t address, bool create_new);
    static AP_RangeFinder_Backend* detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    static void handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_range_sensor_Measurement &msg);

protected:
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return _sensor_type;
    }
private:
    uint8_t _instance;
    RangeFinder::Status _status;
    uint16_t _distance_cm;
    uint32_t _last_reading_ms;
    AP_DroneCAN* _ap_dronecan;
    uint8_t _node_id;
    bool new_data;
    MAV_DISTANCE_SENSOR _sensor_type;
};
#endif  // AP_RANGEFINDER_DRONECAN_ENABLED
