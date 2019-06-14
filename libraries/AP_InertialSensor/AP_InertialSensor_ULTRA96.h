#pragma once

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"


class AP_InertialSensor_ULTRA96 : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_ULTRA96(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();
	void accumulate(void) override {update();}
  
private:
    void timer_update();
    uint8_t gyro_instance;
    uint8_t accel_instance;
    volatile int32_t *data_pointer;
};
