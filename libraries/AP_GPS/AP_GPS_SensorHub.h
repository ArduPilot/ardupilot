#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

#include <AP_SensorHub/AP_SensorHub.h>

#if HAL_SENSORHUB_ENABLED

class AP_GPS_SensorHub;

class GPSMessageHandler : public AP_SensorHub_Handler<GPSMessage> {
public:

    GPSMessageHandler(AP_GPS_SensorHub *backend) :
        _backend(backend)
    {}

    virtual void handle(GPSMessage::data_t *data);
    virtual bool isValid(GPSMessage::data_t *data);
private:
    AP_GPS_SensorHub *_backend;
    uint32_t _count;
    uint32_t _error;
};


class AP_GPS_SensorHub : public AP_GPS_Backend {
public:
    friend class GPSMessageHandler;

    AP_GPS_SensorHub(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    bool read() override;

    void handle_gnss_msg(const AP_GPS::GPS_State &msg) {};

    const char *name() const override { return "SENSORHUB"; }

private:
    bool _new_data;

    AP_GPS::GPS_State _interm_state;

    GPSMessageHandler _handler {this};
    AP_HAL::Semaphore *_sem_gnss;
};
#endif