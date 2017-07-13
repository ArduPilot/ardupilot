#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#if HAL_SENSORHUB_ENABLED
class AP_Compass_SensorHub;

class CompassMessageHandler : public AP_SensorHub_Handler<CompassMessage> {
public:

    CompassMessageHandler(AP_Compass_SensorHub *backend) :
        _backend(backend)
    {}

    virtual void handle(CompassMessage::data_t *data);
    virtual bool isValid(CompassMessage::data_t *data);

private:
    AP_Compass_SensorHub *_backend;
    uint32_t _count;
    uint32_t _error;
};

class AP_Compass_SensorHub : public AP_Compass_Backend {
public:
    friend class CompassMessageHandler;

    void read(void) override;

    AP_Compass_SensorHub(Compass &compass);

    void handle_mag_msg(Vector3f &msg) {}

private:
    bool _instance[COMPASS_MAX_INSTANCES];
    Vector3f _field[COMPASS_MAX_INSTANCES];
    Vector3f _sum[COMPASS_MAX_INSTANCES];
    uint64_t _last_timestamp[COMPASS_MAX_INSTANCES];
    bool _has_sample[COMPASS_MAX_INSTANCES];
    Vector3f _mag_accum[COMPASS_MAX_INSTANCES];
    uint32_t _accum_count[COMPASS_MAX_INSTANCES];

    CompassMessageHandler _handler {this};
    AP_HAL::Semaphore *_sem_mag;

};
#endif