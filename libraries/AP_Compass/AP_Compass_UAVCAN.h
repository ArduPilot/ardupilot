#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class MagCb;
class Mag2Cb;
class AP_Compass_UAVCAN : public AP_Compass_Backend {
public:
    void        read(void) override;

    AP_Compass_UAVCAN(Compass &compass, AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id);
    static void handle_magnetic_field(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MagCb &cb);
    static void handle_magnetic_field_2(AP_UAVCAN* ap_uavcan, uint8_t node_id, const Mag2Cb &cb);

    static void subscribe_compass_uavcan_messages(AP_UAVCAN* ap_uavcan);
    static AP_Compass_Backend* probe(Compass& _frontend);

private:
    uint8_t  _instance;
    int      _mag_fd;
    Vector3f _sum;
    uint32_t _count;
    uint8_t _sensor_id;

    bool _initialized;
    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;
    void init();
    static AP_Compass_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id);

    // callback for UAVCAN messages
    void handle_mag_msg(const Vector3f &mag);

    //Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        uint8_t sensor_id;
        AP_Compass_UAVCAN *driver;
    } _detected_modules[COMPASS_MAX_BACKEND];
    static AP_HAL::Semaphore *_sem_registry;
    static bool take_registry();
    static void give_registry();

    HAL_Semaphore _sem_mag;
};
