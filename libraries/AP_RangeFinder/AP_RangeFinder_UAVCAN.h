#pragma once

#include "AP_RangeFinder_Backend.h"

#if HAL_CANMANAGER_ENABLED
#include <AP_UAVCAN/AP_UAVCAN.h>

class MeasurementCb;

class AP_RangeFinder_UAVCAN : public AP_RangeFinder_Backend {
public:
    AP_RangeFinder_UAVCAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    void update() override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_RangeFinder_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t address, bool create_new);
    static AP_RangeFinder_Backend* detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    static void handle_measurement(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MeasurementCb &cb);

protected:
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return _sensor_type;
    }
private:
    uint8_t _instance;
    RangeFinder::Status _status;
    uint16_t _distance_cm;
    uint32_t _last_reading_ms;
    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;
    bool new_data;
    MAV_DISTANCE_SENSOR _sensor_type;
};
#endif //HAL_CANMANAGER_ENABLED
