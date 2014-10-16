/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include "AP_InertialSensor_PX4.h"

const extern AP_HAL::HAL& hal;

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_hrt.h>

#include <stdio.h>

AP_InertialSensor_PX4::AP_InertialSensor_PX4(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu),
    _last_get_sample_timestamp(0),
    _sample_time_usec(0)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_PX4::detect(AP_InertialSensor &_imu, 
                                                         AP_InertialSensor::Sample_rate sample_rate)
{
    AP_InertialSensor_PX4 *sensor = new AP_InertialSensor_PX4(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor(sample_rate)) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_InertialSensor_PX4::_init_sensor(AP_InertialSensor::Sample_rate sample_rate) 
{
    // assumes max 3 instances
    _accel_fd[0] = open(ACCEL_DEVICE_PATH, O_RDONLY);
    _accel_fd[1] = open(ACCEL_DEVICE_PATH "1", O_RDONLY);
    _accel_fd[2] = open(ACCEL_DEVICE_PATH "2", O_RDONLY);
    _gyro_fd[0] = open(GYRO_DEVICE_PATH, O_RDONLY);
    _gyro_fd[1] = open(GYRO_DEVICE_PATH "1", O_RDONLY);
    _gyro_fd[2] = open(GYRO_DEVICE_PATH "2", O_RDONLY);

    _num_accel_instances = 0;
    _num_gyro_instances = 0;
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        if (_accel_fd[i] >= 0) {
            _num_accel_instances = i+1;
            _accel_instance[i] = _imu.register_accel();
        }
        if (_gyro_fd[i] >= 0) {
            _num_gyro_instances = i+1;
            _gyro_instance[i] = _imu.register_gyro();
        }
    }    
	if (_num_accel_instances == 0) {
        return false;
    }
	if (_num_gyro_instances == 0) {
        return false;
    }

    switch (sample_rate) {
    case AP_InertialSensor::RATE_50HZ:
        _default_filter_hz = 15;
        _sample_time_usec = 20000;
        break;
    case AP_InertialSensor::RATE_100HZ:
        _default_filter_hz = 30;
        _sample_time_usec = 10000;
        break;
    case AP_InertialSensor::RATE_200HZ:
        _default_filter_hz = 30;
        _sample_time_usec = 5000;
        break;
    case AP_InertialSensor::RATE_400HZ:
        _default_filter_hz = 30;
        _sample_time_usec = 2500;
        break;
    default:
        return false;
    }

    _set_filter_frequency(_imu.get_filter());

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
    _product_id = AP_PRODUCT_ID_PX4_V2;
#else
    _product_id = AP_PRODUCT_ID_PX4;
#endif

    return true;
}

/*
  set the filter frequency
 */
void AP_InertialSensor_PX4::_set_filter_frequency(uint8_t filter_hz)
{
    if (filter_hz == 0) {
        filter_hz = _default_filter_hz;
    }
    for (uint8_t i=0; i<_num_gyro_instances; i++) {
        ioctl(_gyro_fd[i],  GYROIOCSLOWPASS,  filter_hz);
    }
    for (uint8_t i=0; i<_num_accel_instances; i++) {
        ioctl(_accel_fd[i], ACCELIOCSLOWPASS, filter_hz);
    }
}

bool AP_InertialSensor_PX4::update(void) 
{
    // get the latest sample from the sensor drivers
    _get_sample();

    for (uint8_t k=0; k<_num_accel_instances; k++) {
        Vector3f accel = _accel_in[k];
        _rotate_and_offset_accel(_accel_instance[k], accel);
        _last_accel_update_timestamp[k] = _last_accel_timestamp[k];
    }

    for (uint8_t k=0; k<_num_gyro_instances; k++) {
        Vector3f gyro = _gyro_in[k];
        _rotate_and_offset_gyro(_gyro_instance[k], gyro);
        _last_gyro_update_timestamp[k] = _last_gyro_timestamp[k];
    }

    if (_last_filter_hz != _imu.get_filter()) {
        _set_filter_frequency(_imu.get_filter());
        _last_filter_hz = _imu.get_filter();
    }

    return true;
}

void AP_InertialSensor_PX4::_get_sample(void)
{
    for (uint8_t i=0; i<_num_accel_instances; i++) {
        struct accel_report	accel_report;
        while (_accel_fd[i] != -1 && 
               ::read(_accel_fd[i], &accel_report, sizeof(accel_report)) == sizeof(accel_report) &&
               accel_report.timestamp != _last_accel_timestamp[i]) {        
            _accel_in[i] = Vector3f(accel_report.x, accel_report.y, accel_report.z);
            _last_accel_timestamp[i] = accel_report.timestamp;
        }
    }
    for (uint8_t i=0; i<_num_gyro_instances; i++) {
        struct gyro_report	gyro_report;
        while (_gyro_fd[i] != -1 && 
               ::read(_gyro_fd[i], &gyro_report, sizeof(gyro_report)) == sizeof(gyro_report) &&
               gyro_report.timestamp != _last_gyro_timestamp[i]) {        
            _gyro_in[i] = Vector3f(gyro_report.x, gyro_report.y, gyro_report.z);
            _last_gyro_timestamp[i] = gyro_report.timestamp;
        }
    }
    _last_get_sample_timestamp = hal.scheduler->micros64();
}

bool AP_InertialSensor_PX4::gyro_sample_available(void)
{
    _get_sample();
    for (uint8_t i=0; i<_num_gyro_instances; i++) {
        if (_last_gyro_timestamp[i] != _last_gyro_update_timestamp[i]) {
            return true;
        }
    }    
    return false;
}

bool AP_InertialSensor_PX4::accel_sample_available(void)
{
    _get_sample();
    for (uint8_t i=0; i<_num_accel_instances; i++) {
        if (_last_accel_timestamp[i] != _last_accel_update_timestamp[i]) {
            return true;
        }
    }    
    return false;
}

#endif // CONFIG_HAL_BOARD

