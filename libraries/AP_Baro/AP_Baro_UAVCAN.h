#pragma once

#include "AP_Baro_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

class PressureCb;
class TemperatureCb;

class AP_Baro_UAVCAN : public AP_Baro_Backend {
public:
    AP_Baro_UAVCAN(AP_Baro &baro);

    void update() override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_Baro_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new);
    static AP_Baro_Backend* probe(AP_Baro &baro);

    static void handle_pressure(AP_UAVCAN* ap_uavcan, uint8_t node_id, const PressureCb &cb);
    static void handle_temperature(AP_UAVCAN* ap_uavcan, uint8_t node_id, const TemperatureCb &cb);

private:

    static void _update_and_wrap_accumulator(float *accum, float val, uint8_t *count, const uint8_t max_count);

    uint8_t _instance;

    bool new_pressure;
    float _pressure;
    float _temperature;
    uint8_t  _pressure_count;
    HAL_Semaphore _sem_baro;

    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;

    // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        AP_Baro_UAVCAN* driver;
    } _detected_modules[BARO_MAX_DRIVERS];

    static HAL_Semaphore _sem_registry;
};
