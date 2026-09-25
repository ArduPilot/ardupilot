#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_DRONECAN_ENABLED

#include "AP_RangeFinder_Backend.h"
#include <AP_DroneCAN/AP_DroneCAN.h>

class MeasurementCb;

class AP_RangeFinder_DroneCAN : public AP_RangeFinder_Backend {
public:
    AP_RangeFinder_DroneCAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    void update() override;

    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static AP_RangeFinder_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t sensor_id);

    static void handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_range_sensor_Measurement &msg);

    static const struct AP_Param::GroupInfo var_info[];

protected:
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return _sensor_type;
    }
private:
    uint8_t _instance;
    // _status is the state received from the peripheral - or "NoData" in case of timeout
    RangeFinder::Status _status;
    float _distance_m;
    uint32_t _last_reading_ms;
    AP_DroneCAN* _ap_dronecan;
    uint8_t _node_id;
    bool new_data;
    MAV_DISTANCE_SENSOR _sensor_type;

    AP_Int8 receive_node_id;  // only accept measurements from this node; 0 for any

    // binding state is read and written from every AP_DroneCAN thread
    static HAL_Semaphore _bind_sem;

    void bind(AP_DroneCAN *ap_dronecan, uint8_t node_id, uint8_t instance);
    bool bound_to(const AP_DroneCAN *ap_dronecan, uint8_t node_id) const {
        return _ap_dronecan == ap_dronecan && _node_id == node_id;
    }
};
#endif  // AP_RANGEFINDER_DRONECAN_ENABLED
