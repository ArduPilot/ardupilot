#pragma once

#include "AP_Airspeed_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

class AirspeedCb;

class AP_Airspeed_UAVCAN : public AP_Airspeed_Backend {
public:
    AP_Airspeed_UAVCAN(AP_Airspeed &_frontend, uint8_t _instance);

    bool init(void) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);

    static AP_Airspeed_Backend* probe(AP_Airspeed &_fronted, uint8_t _instance);

private:

    static void handle_airspeed(AP_UAVCAN* ap_uavcan, uint8_t node_id, const AirspeedCb &cb);

    static AP_Airspeed_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id);

    float _pressure; // Pascal
    float _temperature; // Celcius
    uint32_t _last_sample_time_ms;

    HAL_Semaphore _sem_airspeed;

    // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        AP_Airspeed_UAVCAN *driver;
    } _detected_modules[AIRSPEED_MAX_SENSORS];

    static HAL_Semaphore _sem_registry;
    bool _have_temperature;
};
