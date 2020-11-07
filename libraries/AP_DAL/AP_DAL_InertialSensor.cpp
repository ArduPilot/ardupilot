#include "AP_DAL_InertialSensor.h"

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

AP_DAL_InertialSensor::AP_DAL_InertialSensor()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RISI); i++) {
        _RISI[i].instance = i;
    }

}

void AP_DAL_InertialSensor::start_frame()
{
    const auto &ins = AP::ins();

    const log_RISH old_RISH = _RISH;

    _RISH.loop_rate_hz = ins.get_loop_rate_hz();
    _RISH.primary_gyro = ins.get_primary_gyro();
    _RISH.loop_delta_t = ins.get_loop_delta_t();
    _RISH.primary_accel = ins.get_primary_accel();
    _RISH.accel_count = ins.get_accel_count();
    _RISH.gyro_count = ins.get_gyro_count();
    WRITE_REPLAY_BLOCK_IFCHANGED(RISH, _RISH, old_RISH);

    for (uint8_t i=0; i<ARRAY_SIZE(_RISI); i++) {
        log_RISI &RISI = _RISI[i];
        const log_RISI old_RISI = RISI;

        // accel stuff
        RISI.use_accel = ins.use_accel(i);
        RISI.get_delta_velocity_ret = ins.get_delta_velocity(i, RISI.delta_velocity);
        RISI.delta_velocity_dt = ins.get_delta_velocity_dt(i);

        // gryo stuff
        RISI.use_gyro = ins.use_gyro(i);
        RISI.delta_angle_dt = ins.get_delta_angle_dt(i);
        RISI.get_delta_angle_ret = ins.get_delta_angle(i, RISI.delta_angle);

        update_filtered(i);

        WRITE_REPLAY_BLOCK_IFCHANGED(RISI, RISI, old_RISI);

        // update sensor position
        pos[i] = ins.get_imu_pos_offset(i);
    }
}

// update filtered gyro and accel
void AP_DAL_InertialSensor::update_filtered(uint8_t i)
{
    if (is_positive(_RISI[i].delta_angle_dt)) {
        gyro_filtered[i] = gyro_filtered[i] * alpha + (_RISI[i].delta_angle/_RISI[i].delta_angle_dt) * (1-alpha);
    }
    if (is_positive(_RISI[i].delta_velocity_dt)) {
        accel_filtered[i] = accel_filtered[i] * alpha + (_RISI[i].delta_velocity/_RISI[i].delta_velocity_dt) * (1-alpha);
    }
}
