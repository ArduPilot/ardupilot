#pragma once

/*
  shared memory structures for sensor data and peripheral control on Qualcomm flight board
 */
struct DSPBuffer {
    // IMU data
    struct IMU {
        static const uint32_t max_samples = 10;
        uint32_t num_samples;
        struct BUF {
            uint64_t timestamp;
            float accel[3];
            float gyro[3];
        } buf[max_samples];
    } imu;

    // MAG data
    struct MAG {
        static const uint64_t max_samples = 10;
        uint32_t num_samples;
        struct BUF {
            uint64_t timestamp;
            int16_t mag_raw[3];
        } buf[max_samples];
    } mag;

    // baro data
    struct BARO {
        static const uint32_t max_samples = 10;
        uint32_t num_samples;
        struct BUF {
            uint64_t timestamp;
            float pressure_pa;
            float temperature_C;
        } buf[max_samples];
    } baro;
};
