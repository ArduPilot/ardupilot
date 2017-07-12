#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_SENSORHUB_ENABLED

#include "Protocol.h"

using namespace SensorHub;

class AP_SensorHub;

/* Base class for IO Channel */
class AP_SensorHub_IO {
public:
    virtual void read() = 0;
    virtual bool write(Packet::packet_t *packet, size_t len) = 0;

    void setSensorHub(AP_SensorHub *shub) {
        _shub = shub;
    }

    void setId(uint8_t id) {
        _id = id;
    }

protected:
    AP_SensorHub *_shub;
    AP_HAL::Semaphore *_sem_write;
    uint32_t _write_drop;
    uint32_t _write_error;
    uint8_t _id; // ID as set by AP_SensorHub
};
#endif
