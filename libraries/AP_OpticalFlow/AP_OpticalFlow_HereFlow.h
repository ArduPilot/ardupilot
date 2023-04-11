#pragma once

#include "AP_OpticalFlow.h"

#ifndef AP_OPTICALFLOW_HEREFLOW_ENABLED
#define AP_OPTICALFLOW_HEREFLOW_ENABLED (AP_OPTICALFLOW_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS)
#endif

#if AP_OPTICALFLOW_HEREFLOW_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>

class AP_OpticalFlow_HereFlow : public OpticalFlow_backend {
public:
    AP_OpticalFlow_HereFlow(AP_OpticalFlow &flow);

    void init() override {}

    void update() override;

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);

    static void handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_hex_equipment_flow_Measurement &msg);

private:

    Vector2f flowRate, bodyRate;
    uint8_t surface_quality;
    float integral_time;
    bool new_data;
    static uint8_t _node_id;

    static AP_OpticalFlow_HereFlow* _driver;
    static AP_DroneCAN* _ap_dronecan;
    void _push_state(void);

};

#endif  // AP_OPTICALFLOW_HEREFLOW_ENABLED
