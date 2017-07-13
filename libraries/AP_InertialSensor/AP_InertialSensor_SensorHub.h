#pragma once

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#if HAL_SENSORHUB_ENABLED
class AP_InertialSensor_SensorHub;

class GyroMessageHandler : public AP_SensorHub_Handler<GyroMessage> {
public:

    GyroMessageHandler(AP_InertialSensor_SensorHub *backend) :
        _backend(backend)
    {}

    virtual void handle(GyroMessage::data_t *data);
    virtual bool isValid(GyroMessage::data_t *data);

private:
    AP_InertialSensor_SensorHub *_backend;
    uint32_t _count;
    uint32_t _error;
};

class AccelMessageHandler : public AP_SensorHub_Handler<AccelMessage> {
public:

    AccelMessageHandler(AP_InertialSensor_SensorHub *backend) :
        _backend(backend)
    {}

    virtual void handle(AccelMessage::data_t *data);
    virtual bool isValid(AccelMessage::data_t *data);

private:
    AP_InertialSensor_SensorHub *_backend;
    uint32_t _count;
    uint32_t _error;
};


class AP_InertialSensor_SensorHub : public AP_InertialSensor_Backend
{
public:
    friend class GyroMessageHandler;
    friend class AccelMessageHandler;

    AP_InertialSensor_SensorHub(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    typedef struct {
        uint8_t devtype;
        bool registered;
    } sensor_info;

private:
    sensor_info _gyro_instance[INS_MAX_INSTANCES];
    sensor_info _accel_instance[INS_MAX_INSTANCES];

    Vector3f _gyro[INS_MAX_INSTANCES];
    Vector3f _accel[INS_MAX_INSTANCES];
    uint64_t _last_timestamp[INS_MAX_INSTANCES];

    GyroMessageHandler _gyro_handler {this};
    AccelMessageHandler _accel_handler {this};
    AP_HAL::Semaphore *_sem_gyro;
    AP_HAL::Semaphore *_sem_accel;
};
#endif