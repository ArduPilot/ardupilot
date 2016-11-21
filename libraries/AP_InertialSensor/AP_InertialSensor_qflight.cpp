#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

#include "AP_InertialSensor_qflight.h"
#include <AP_HAL_Linux/qflight/qflight_util.h>
#include <AP_HAL_Linux/qflight/qflight_dsp.h>

const extern AP_HAL::HAL& hal;

AP_InertialSensor_QFLIGHT::AP_InertialSensor_QFLIGHT(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_QFLIGHT::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_QFLIGHT *sensor = new AP_InertialSensor_QFLIGHT(_imu);
    if (sensor == nullptr) {
        return nullptr;
    }
    if (!sensor->init_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_QFLIGHT::init_sensor(void) 
{
    gyro_instance = _imu.register_gyro(1000, 1);
    accel_instance = _imu.register_accel(1000, 1);

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_QFLIGHT::timer_update, void));
    return true;
}

void AP_InertialSensor_QFLIGHT::timer_update(void)
{
    if (imubuf == nullptr) {
        imubuf = QFLIGHT_RPC_ALLOCATE(DSPBuffer::IMU);
        if (imubuf == nullptr) {
            AP_HAL::panic("unable to allocate IMU buffer");
        }
    }
    int ret = qflight_get_imu_data((uint8_t *)imubuf, sizeof(*imubuf));
    if (ret != 0) {
        return;
    }
    for (uint16_t i=0; i<imubuf->num_samples; i++) {
        DSPBuffer::IMU::BUF &b = imubuf->buf[i];
        Vector3f accel(b.accel[0], b.accel[1], b.accel[2]);
        Vector3f gyro(b.gyro[0], b.gyro[1], b.gyro[2]);
        _rotate_and_correct_accel(accel_instance, accel);
        _rotate_and_correct_gyro(gyro_instance, gyro);
        _notify_new_accel_raw_sample(accel_instance, accel, b.timestamp);
        _notify_new_gyro_raw_sample(gyro_instance, gyro, b.timestamp);
    }
}

bool AP_InertialSensor_QFLIGHT::update(void) 
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

#endif // HAL_BOARD_QFLIGHT
