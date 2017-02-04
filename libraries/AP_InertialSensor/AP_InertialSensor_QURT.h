#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

extern "C" {
#undef DEG_TO_RAD
#include <mpu9x50.h>
}
#include <AP_HAL/utility/RingBuffer.h>

extern ObjectBuffer<mpu9x50_data> *mpu9250_mag_buffer;

class AP_InertialSensor_QURT : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_QURT(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update() override;
    void accumulate(void) override;

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    void data_ready(void);

private:
    bool init_sensor();
    void init_mpu9250();

    uint64_t last_timestamp;
    uint32_t start_time_ms;

    uint8_t gyro_instance;
    uint8_t accel_instance;

    ObjectBuffer<mpu9x50_data> buf{100};
};

#endif
