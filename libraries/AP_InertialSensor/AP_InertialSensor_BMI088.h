#pragma once

#include "AP_InertialSensor_Backend.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// Custom BMI088 IMU Driver
class AP_InertialSensor_BMI088_Custom : public AP_InertialSensor_Backend {
public:
    // Probe function to detect sensor
    static AP_InertialSensor_Backend* probe(AP_InertialSensor &imu);

    // Start and update lifecycle
    void start() override;
    bool update() override;

private:
    // Constructor
    AP_InertialSensor_BMI088_Custom(AP_InertialSensor &imu);

    // Initialization functions
    bool init();
    bool accel_init();
    bool gyro_init();

    // Configuration
    bool configure_accel();
    bool configure_gyro();

    // Register operations
    uint8_t read_reg(uint8_t reg);
    void write_reg(uint8_t reg, uint8_t val);

    // Sensor reading
    void read_accel();
    void read_gyro();

    // Scaling helpers
    float accel_scale(int16_t raw);
    float gyro_scale(int16_t raw);

    // SPI device
    AP_HAL::SPIDevice *_dev;

    // Sensor instances
    uint8_t accel_instance;
    uint8_t gyro_instance;

    // Raw sensor data
    int16_t _ax, _ay, _az;
    int16_t _gx, _gy, _gz;

    // Scaled data
    Vector3f accel;
    Vector3f gyro;

    // Constants
    static constexpr float ACCEL_RANGE = 24.0f;   // ±24g
    static constexpr float GYRO_RANGE  = 2000.0f; // ±2000 dps
};
