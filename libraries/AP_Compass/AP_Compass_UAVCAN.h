#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

class MagCb;
class Mag2Cb;

class AP_Compass_UAVCAN : public AP_Compass_Backend {
public:
    AP_Compass_UAVCAN(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id);

    void        read(void) override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_Compass_Backend* probe();

    static void handle_magnetic_field(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MagCb &cb);
    static void handle_magnetic_field_2(AP_UAVCAN* ap_uavcan, uint8_t node_id, const Mag2Cb &cb);

private:
    void init();

    // callback for UAVCAN messages
    void handle_mag_msg(const Vector3f &mag);

    static AP_Compass_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id);

    uint8_t  _instance;
    bool _initialized;

    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;
    uint8_t _sensor_id;

    // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        uint8_t sensor_id;
        AP_Compass_UAVCAN *driver;
    } _detected_modules[COMPASS_MAX_BACKEND];

    static HAL_Semaphore _sem_registry;
};
