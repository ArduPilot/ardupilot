#pragma once

#include "AP_Baro_Backend.h"

#include <AP_SensorHub/AP_SensorHub.h>

#if HAL_SENSORHUB_ENABLED

class AP_Baro_SensorHub;

class BaroMessageHandler : public AP_SensorHub_Handler<BaroMessage> {
public:

    BaroMessageHandler(AP_Baro_SensorHub *backend) :
        _backend(backend)
    {}

    virtual void handle(BaroMessage::data_t *data);
    virtual bool isValid(BaroMessage::data_t *data);
private:
    AP_Baro_SensorHub *_backend;
    uint32_t _count;
    uint32_t _error;
};

class AP_Baro_SensorHub : public AP_Baro_Backend {
public:
    friend class BaroMessageHandler;

    AP_Baro_SensorHub(AP_Baro &);

    void update() override;

private:
    uint8_t _instance[BARO_MAX_INSTANCES];
    float _pressure[BARO_MAX_INSTANCES];
    float _temperature[BARO_MAX_INSTANCES];
    uint64_t _last_timestamp[BARO_MAX_INSTANCES];

    BaroMessageHandler _handler {this};
    AP_HAL::Semaphore *_sem_baro;
};
#endif