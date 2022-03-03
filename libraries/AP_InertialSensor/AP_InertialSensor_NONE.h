#pragma once

/*
 This is a 'mock' implementation of an INS that does nothing and gives a level HUD, but does it successfully.   
 Its useful for boards that don't have any form of IMU accel/gyro etc connected just yet, but where u want to boot-up "successfully" anyway, 
 such as the ESP32, to allow wifi connectivity to startup, and alow mavlink to start streaming anyway.
 Its a rip-off of _SITL with all the sitl stuff removed or replaced with constants.
*/

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

// simulated sensor rates in Hz. This matches a pixhawk1
const uint16_t INS_NONE_SENSOR_A[] = { 1000, 1000 };


class AP_InertialSensor_NONE : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_NONE(AP_InertialSensor &imu, const uint16_t sample_rates[]);

    /* update accel and gyro state */
    bool update() override;
    void start() override;
    void accumulate() override;
    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu, const uint16_t sample_rates[]);

private:
    bool init_sensor(void);
    void timer_update();
    float gyro_drift(void);
    void generate_accel();
    void generate_gyro();
    //float rand_float(void);


    //SITL::SITL *sitl;

    const uint16_t gyro_sample_hz;
    const uint16_t accel_sample_hz;

    uint8_t gyro_instance;
    uint8_t accel_instance;
    uint64_t next_gyro_sample;
    uint64_t next_accel_sample;
    float gyro_time;
    float accel_time;
    float gyro_motor_phase[12];
    float accel_motor_phase[12];

    static uint8_t bus_id;
};
#endif // CONFIG_HAL_BOARD
