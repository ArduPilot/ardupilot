#pragma once

/*
 This is a 'mock' implementation of an INS that does nothing and gives a level HUD, but does it successfully.   
 Its useful for boards that don't have any form of IMU accel/gyro etc connected just yet, but where u want to boot-up "successfully" anyway, 
 such as the ESP32, to allow wifi connectivity to startup, and alow mavlink to start streaming anyway.
 Its a rip-off of _SITL with all the sitl stuff removed or replaced with constants.
*/

// A board with no physical IMU registers this so the vehicle finishes
// initialising and MAVLink comes up, rather than aborting in config_error() or
// blocking forever in wait_for_sample().
#include "AP_InertialSensor_config.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 || HAL_INS_ALLOW_NO_SENSORS
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

// Simulated sensor rates in Hz.
// At 1 kHz with a 100 Hz main loop i.e. 10 samples per loop, consuming ~5 ms of the 10 ms budget and leaving no time for normal-priority GCS tasks.
// For ESP32 keep 1 kHz to preserve the original behaviour.
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
const uint16_t INS_NONE_SENSOR_A[] = { 200, 200 };
#else
const uint16_t INS_NONE_SENSOR_A[] = { 1000, 1000 };
#endif


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

    uint64_t next_gyro_sample;
    uint64_t next_accel_sample;
    float gyro_time;
    float accel_time;
    float gyro_motor_phase[32];
    float accel_motor_phase[32];

    static uint8_t bus_id;
};
#endif // CONFIG_HAL_BOARD
