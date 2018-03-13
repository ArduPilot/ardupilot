#pragma once

#include "AP_Baro_Backend.h"
#include <AP_UAVCAN/AP_UAVCAN.h>

class AP_Baro_UAVCAN : public AP_Baro_Backend {
public:
    AP_Baro_UAVCAN(AP_Baro &);
    ~AP_Baro_UAVCAN() override;

    static AP_Baro_Backend *probe(AP_Baro &baro);

    void update() override;

    // This method is called from UAVCAN thread
    virtual void handle_baro_msg(float pressure, float temperature) override;

    bool register_uavcan_baro(uint8_t mgr, uint8_t node);

private:
    uint8_t _instance;
    float _pressure;
    float _temperature;
    uint64_t _last_timestamp;
    uint8_t _manager;

    bool _initialized;

    AP_HAL::Semaphore *_sem_baro;
};
