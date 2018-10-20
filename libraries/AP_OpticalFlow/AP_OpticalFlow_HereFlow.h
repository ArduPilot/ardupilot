#pragma once
#include "OpticalFlow_backend.h"

#if HAL_WITH_UAVCAN

#include <AP_UAVCAN/AP_UAVCAN.h>

class MeasurementCb;

class AP_OpticalFlow_HereFlow : public OpticalFlow_backend {
public:
    AP_OpticalFlow_HereFlow(OpticalFlow &flow);

    void init() override {}

    void update() override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);

    static void handle_measurement(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MeasurementCb &cb);

private:

    Vector2f flowRate, bodyRate;
    uint8_t surface_quality;
    float integral_time;
    bool new_data;
    static uint8_t _node_id;

    static AP_OpticalFlow_HereFlow* _driver;
    static AP_UAVCAN* _ap_uavcan;
    void _push_state(void);

};
#endif //HAL_WITH_UAVCAN
