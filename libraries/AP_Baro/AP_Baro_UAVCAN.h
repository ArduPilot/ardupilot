#pragma once

#include "AP_Baro_Backend.h"

class PressureCb;
class TemperatureCb;
class AP_Baro_UAVCAN : public AP_Baro_Backend {
public:
    AP_Baro_UAVCAN(AP_Baro &baro);
    void update() override;

    static void handle_pressure(AP_UAVCAN* ap_uavcan, uint8_t node_id, const PressureCb &cb);
    static void handle_temperature(AP_UAVCAN* ap_uavcan, uint8_t node_id, const TemperatureCb &cb);

    static void subscribe_baro_uavcan_messages(AP_UAVCAN* ap_uavcan);
    inline void register_sensor() {
        _instance = _frontend.register_sensor();
    }
    static AP_Baro_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new);
    static AP_Baro_Backend* probe(AP_Baro &baro);
private:
    uint8_t _instance;
    float _pressure;
    float _temperature;
    uint64_t _last_timestamp;
    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;
    bool new_pressure;

    //Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        AP_Baro_UAVCAN* driver;
    } _detected_modules[BARO_MAX_DRIVERS];
    static AP_HAL::Semaphore *_sem_registry;
    static bool take_registry();
    static void give_registry();

    HAL_Semaphore _sem_baro;
};
