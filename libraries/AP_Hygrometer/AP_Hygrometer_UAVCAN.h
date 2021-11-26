#pragma once

#include "AP_Hygrometer.h"

#if AP_HYGROMETER_ENABLED

#include <AP_UAVCAN/AP_UAVCAN.h>
#include "AP_Hygrometer_Backend.h"

class HygrometerCb;
class AP_Hygrometer_Backend;

class AP_Hygrometer_UAVCAN : public AP_Hygrometer_Backend
{
public:
    // constructor
    AP_Hygrometer_UAVCAN(AP_Hygrometer::Param &_params);

    bool init(void) override { return true; }

    bool get_humidity(float &humidity) override;
    bool get_temperature(float &temperature) override;

    static void handle_hygrometer(AP_UAVCAN* ap_uavcan, uint8_t node_id, const HygrometerCb &cb);

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);

    static AP_Hygrometer_Backend* probe(AP_Hygrometer::Param &_params);

private:
    float _humidity;
    float _temperature;
    uint32_t _last_sample_time_ms;

    static  AP_Hygrometer_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id);

    HAL_Semaphore _sem_hygrometer;

    // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        AP_Hygrometer_UAVCAN *driver;
    } _detected_modules[AP_HYGROMETER_MAX_SENSORS];

    static HAL_Semaphore _sem_registry;
};

#endif // AP_HYGROMETER_ENABLED
