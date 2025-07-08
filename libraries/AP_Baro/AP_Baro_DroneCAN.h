#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_DRONECAN_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>
#if AP_TEST_DRONECAN_DRIVERS
#include <SITL/SITL.h>
#endif

class AP_Baro_DroneCAN : public AP_Baro_Backend {
public:
    AP_Baro_DroneCAN(AP_Baro &baro);

    void update() override;

    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static AP_Baro_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, bool create_new);
    static AP_Baro_Backend* probe(AP_Baro &baro);

    static void handle_pressure(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticPressure &msg);
    static void handle_temperature(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticTemperature &msg);
#if AP_TEST_DRONECAN_DRIVERS
    void update_healthy_flag(uint8_t instance) override { _frontend.sensors[instance].healthy = !AP::sitl()->baro[instance].disable; };
#endif
private:

    static void _update_and_wrap_accumulator(float *accum, float val, uint8_t *count, const uint8_t max_count);

    uint8_t _instance;

    bool new_pressure;
    float _pressure;
    float _temperature;
    uint8_t  _pressure_count;
    HAL_Semaphore _sem_baro;

    AP_DroneCAN* _ap_dronecan;
    uint8_t _node_id;

    // Module Detection Registry
    static struct DetectedModules {
        AP_DroneCAN* ap_dronecan;
        uint8_t node_id;
        AP_Baro_DroneCAN* driver;
    } _detected_modules[BARO_MAX_DRIVERS];

    static HAL_Semaphore _sem_registry;
};

#endif  // AP_BARO_DRONECAN_ENABLED
