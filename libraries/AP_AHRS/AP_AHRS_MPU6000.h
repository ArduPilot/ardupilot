#ifndef __AP_AHRS_MPU6000_H__
#define __AP_AHRS_MPU6000_H__
/*
 *  DCM based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

// Rate of the gyro bias from gravity correction (200Hz/4) => 50Hz
#define GYRO_BIAS_FROM_GRAVITY_RATE 4
// Initial value to detect that compass correction is not initialized
#define HEADING_UNKNOWN -9999

// max rate of gyro drift in gyro_LSB_units/s (16.4LSB = 1deg/s)
// 0.5 corresponds to 0.03 degrees/s/s;
static const float _MPU6000_gyro_drift_rate = 0.5;

class AP_AHRS_MPU6000 : public AP_AHRS
{
public:
    // Constructors
    AP_AHRS_MPU6000(AP_InertialSensor_MPU6000 *mpu6000, GPS *&gps) :
        AP_AHRS(mpu6000, gps),
        // ki and ki_yaw are experimentally derived from the simulator
        _ki(0.0087),
        _ki_yaw(0.01),
        _mpu6000(mpu6000),
        // dmp related variable initialisation
        _compass_bias_time(0),
        _gyro_bias_from_gravity_gain(0.008)
    {
        _dcm_matrix.identity();
    }

    // initialisation routine to start MPU6000's dmp
    void init();

    // return the smoothed gyro vector corrected for drift
    Vector3f get_gyro(void) {
        return _ins->get_gyro();
    }

    Matrix3f get_dcm_matrix(void) {
        return _dcm_matrix;
    }

    // return the current drift correction integrator value
    Vector3f get_gyro_drift(void) {
        return _omega_I;
    }

    // Methods
    void update(void);
    void reset(bool recover_eulers = false);

    // push offsets down from IMU to INS (required so MPU6000 can perform it's
    // own attitude estimation)
    void push_offsets_to_ins();
    void push_gains_to_dmp();

    // status reporting
    float get_error_rp(void);
    float get_error_yaw(void);

    // set_as_secondary - avoid running some steps twice (imu updates) if
    // this is a secondary ahrs
    void set_as_secondary(bool secondary) {
        _secondary_ahrs = secondary;
    }

private:
    float _ki;
    float _ki_yaw;
    AP_InertialSensor_MPU6000 *_mpu6000;

    // Methods
    void drift_correction(float deltat);

    // Compass correction variables. TO-DO: move or replace?
    // TO-DO: move wrap_PI to standard AP_AHRS methods
    float wrap_PI(float angle_in_radians);
    long _compass_bias_time;

    void  drift_correction_yaw(void);
    float yaw_error_compass();
    void  euler_angles(void);

    Vector3f _accel_filtered;
    int16_t _accel_filtered_samples;

    // bias_tracking
    float _gyro_bias[3];

    // bias correction algorithm gain
    float _gyro_bias_from_gravity_gain;
    uint8_t _gyro_bias_from_gravity_counter;

    // primary representation of attitude
    Matrix3f _dcm_matrix;
    // current accel vector
    Vector3f _accel_vector;
    // Omega Integrator correction
    Vector3f _omega_I;
    Vector3f _omega_I_sum;
    float _omega_I_sum_time;

    // state to support status reporting
    float _error_yaw_sum;
    uint16_t _error_yaw_count;
    float _error_yaw_last;

    bool _secondary_ahrs;
};

#endif // __AP_AHRS_MPU6000_H__
