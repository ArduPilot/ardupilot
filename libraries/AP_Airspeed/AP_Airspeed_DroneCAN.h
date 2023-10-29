#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_DRONECAN_ENABLED

#include "AP_Airspeed_Backend.h"

#include <AP_DroneCAN/AP_DroneCAN.h>

class AP_Airspeed_DroneCAN : public AP_Airspeed_Backend {
public:

    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    bool init(void) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override;

#if AP_AIRSPEED_HYGROMETER_ENABLE
    // get hygrometer data
    bool get_hygrometer(uint32_t &last_sample_ms, float &temperature, float &humidity) override;
#endif

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);

    static AP_Airspeed_Backend* probe(AP_Airspeed &_frontend, uint8_t _instance, uint32_t previous_devid);

private:

    static void handle_airspeed(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_RawAirData &msg);
    static void handle_hygrometer(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_hygrometer_Hygrometer &msg);

    static AP_Airspeed_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id);

    float _pressure; // Pascal
    float _temperature; // Celcius
    uint32_t _last_sample_time_ms;

    // hygrometer data
    struct {
        float temperature;
        float humidity;
        uint32_t last_sample_ms;
    } _hygrometer;

    HAL_Semaphore _sem_airspeed;

    // Module Detection Registry
    static struct DetectedModules {
        AP_DroneCAN* ap_dronecan;
        uint8_t node_id;
        AP_Airspeed_DroneCAN *driver;
    } _detected_modules[AIRSPEED_MAX_SENSORS];

    static HAL_Semaphore _sem_registry;
    bool _have_temperature;
};


#endif  // AP_AIRSPEED_DRONECAN_ENABLED
