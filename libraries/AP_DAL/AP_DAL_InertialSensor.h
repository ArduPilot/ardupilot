#pragma once

#include <AP_InertialSensor/AP_InertialSensor.h>

#include <AP_Logger/LogStructure.h>

class AP_DAL_InertialSensor {
public:

    // InertialSensor-like methods:

    // return the selected loop rate at which samples are made available
    uint16_t get_loop_rate_hz(void) const { return _RISH.loop_rate_hz; }

    const Vector3f &get_imu_pos_offset(uint8_t instance) const {
        return pos[instance];
    }

    // accel stuff
    uint8_t get_accel_count(void) const { return _RISH.accel_count; }
    uint8_t get_first_usable_accel(void) const { return _RISH.first_usable_accel; };

    bool use_accel(uint8_t instance) const { return _RISI[instance].use_accel; }
    const Vector3f     &get_accel(uint8_t i) const { return accel_filtered[i]; }
    bool get_delta_velocity(uint8_t i, Vector3f &delta_velocity, float &delta_velocity_dt) const {
        delta_velocity = _RISI[i].delta_velocity;
        delta_velocity_dt = _RISI[i].delta_velocity_dt;
        return _RISI[i].get_delta_velocity_ret;
    }

    // gyro stuff
    uint8_t get_gyro_count(void) const { return _RISH.gyro_count; }
    uint8_t get_first_usable_gyro(void) const { return _RISH.first_usable_gyro; };

    bool use_gyro(uint8_t instance) const { return _RISI[instance].use_gyro; }
    const Vector3f     &get_gyro(uint8_t i) const { return gyro_filtered[i]; }
    const Vector3f     &get_gyro() const { return get_gyro(_primary_gyro); }
    bool get_delta_angle(uint8_t i, Vector3f &delta_angle, float &delta_angle_dt) const {
        delta_angle = _RISI[i].delta_angle;
        delta_angle_dt = _RISI[i].delta_angle_dt;
        return _RISI[i].get_delta_angle_ret;
    }

    // return the main loop delta_t in seconds
    float get_loop_delta_t(void) const { return _RISH.loop_delta_t; }

    // AP_DAL methods:
    AP_DAL_InertialSensor();

    void start_frame();

    void handle_message(const log_RISH &msg) {
        _RISH = msg;
    }
    void handle_message(const log_RISI &msg) {
        _RISI[msg.instance] = msg;
        pos[msg.instance] = AP::ins().get_imu_pos_offset(msg.instance);
        update_filtered(msg.instance);
    }

private:
    struct log_RISH _RISH;
    struct log_RISI _RISI[INS_MAX_INSTANCES];
    float alpha;

    // sensor positions
    Vector3f pos[INS_MAX_INSTANCES];

    Vector3f gyro_filtered[INS_MAX_INSTANCES];
    Vector3f accel_filtered[INS_MAX_INSTANCES];

    uint8_t _primary_gyro;

    void update_filtered(uint8_t i);
};
