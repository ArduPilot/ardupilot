/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "AP_InertialSensor_PX4.h"

const extern AP_HAL::HAL& hal;

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_hrt.h>

uint16_t AP_InertialSensor_PX4::_init_sensor( Sample_rate sample_rate ) 
{
    // assumes max 2 instances
    _accel_fd[0] = open(ACCEL_DEVICE_PATH, O_RDONLY);
    _accel_fd[1] = open(ACCEL_DEVICE_PATH "1", O_RDONLY);
    _gyro_fd[0] = open(GYRO_DEVICE_PATH, O_RDONLY);
    _gyro_fd[1] = open(GYRO_DEVICE_PATH "1", O_RDONLY);

	if (_accel_fd[0] < 0) {
        hal.scheduler->panic("Unable to open accel device " ACCEL_DEVICE_PATH);
    }
	if (_gyro_fd[0] < 0) {
        hal.scheduler->panic("Unable to open gyro device " GYRO_DEVICE_PATH);
    }
    if (_accel_fd[1] >= 0) {
        _num_accel_instances = 2;
    }
    if (_gyro_fd[1] >= 0) {
        _num_gyro_instances = 2;
    }

    switch (sample_rate) {
    case RATE_50HZ:
        _default_filter_hz = 15;
        _sample_time_usec = 20000;
        break;
    case RATE_100HZ:
        _default_filter_hz = 30;
        _sample_time_usec = 10000;
        break;
    case RATE_200HZ:
    default:
        _default_filter_hz = 30;
        _sample_time_usec = 5000;
        break;
    }

    _set_filter_frequency(_mpu6000_filter);

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
    return AP_PRODUCT_ID_PX4_V2;
#else
    return AP_PRODUCT_ID_PX4;
#endif
}

/*
  set the filter frequency
 */
void AP_InertialSensor_PX4::_set_filter_frequency(uint8_t filter_hz)
{
    if (filter_hz == 0) {
        filter_hz = _default_filter_hz;
    }
    for (uint8_t i=0; i<PX4_MAX_INS_INSTANCES; i++) {
        ioctl(_gyro_fd[i],  GYROIOCSLOWPASS,  filter_hz);
        ioctl(_accel_fd[i], ACCELIOCSLOWPASS, filter_hz);
    }
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

// multi-device interface
bool AP_InertialSensor_PX4::get_gyro_instance_health(uint8_t instance) const
{
    if (instance >= _num_gyro_instances) {
        return false;
    }
    if (_sample_time_usec == 0) {
        // not initialised yet, show as healthy to prevent scary GCS
        // warnings
        return true;
    }
    uint64_t tnow = hrt_absolute_time();

    if ((tnow - _last_gyro_timestamp[instance]) > 2*_sample_time_usec) {
        // gyros have not updated
        return false;
    }
    return true;
}

uint8_t AP_InertialSensor_PX4::get_gyro_count(void) const
{
    return _num_gyro_instances;
}

bool AP_InertialSensor_PX4::get_gyro_instance(uint8_t instance, Vector3f &gyro) const
{
    if (instance >= _num_gyro_instances) {
        return false;
    }
    gyro = _gyro_in[instance];
    gyro.rotate(_board_orientation);
    gyro -= _gyro_offset;
    return true;
}

bool AP_InertialSensor_PX4::get_accel_instance_health(uint8_t instance) const
{
    if (instance >= _num_accel_instances) {
        return false;
    }
    if (_sample_time_usec == 0) {
        // not initialised yet, show as healthy to prevent scary GCS
        // warnings
        return true;
    }
    uint64_t tnow = hrt_absolute_time();

    if ((tnow - _last_accel_timestamp[instance]) > 2*_sample_time_usec) {
        // accels have not updated
        return false;
    }
    if (fabsf(_accel.x) > 30 && fabsf(_accel.y) > 30 && fabsf(_accel.z) > 30 &&
        (_previous_accels[instance] - _accel_in[instance]).length() < 0.01f) {
        // unchanging accel, large in all 3 axes. This is a likely
        // accelerometer failure of the LSM303d
        return false;
    }
    return true;
    
}
uint8_t AP_InertialSensor_PX4::get_accel_count(void) const
{
    return _num_accel_instances;
}

bool AP_InertialSensor_PX4::get_accel_instance(uint8_t instance, Vector3f &accel) const
{
    if (instance >= _num_accel_instances) {
        return false;
    }
    accel = _accel_in[instance];
    accel.rotate(_board_orientation);
    accel.x *= _accel_scale.get().x;
    accel.y *= _accel_scale.get().y;
    accel.z *= _accel_scale.get().z;
    accel   -= _accel_offset;
    return true;    
}

bool AP_InertialSensor_PX4::update(void) 
{
    // get the latest sample from the sensor drivers
    _get_sample();

    _previous_accel = _accel;

    get_accel_instance(0, _accel);
    get_gyro_instance(0, _gyro);

    if (_last_filter_hz != _mpu6000_filter) {
        _set_filter_frequency(_mpu6000_filter);
        _last_filter_hz = _mpu6000_filter;
    }

    _have_sample_available = false;

    return true;
}

float AP_InertialSensor_PX4::get_delta_time(void)
{
    return _sample_time_usec * 1.0e-6f;
}

float AP_InertialSensor_PX4::get_gyro_drift_rate(void) 
{
    // assume 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

void AP_InertialSensor_PX4::_get_sample(void)
{
    for (uint8_t i=0; i<PX4_MAX_INS_INSTANCES; i++) {
        struct accel_report	accel_report;
        while (_accel_fd[i] != -1 && 
               ::read(_accel_fd[i], &accel_report, sizeof(accel_report)) == sizeof(accel_report) &&
               accel_report.timestamp != _last_accel_timestamp[i]) {        
            _previous_accels[i] = _accel_in[i];
            _accel_in[i] = Vector3f(accel_report.x, accel_report.y, accel_report.z);
            _last_accel_timestamp[i] = accel_report.timestamp;
        }
    }
    for (uint8_t i=0; i<PX4_MAX_INS_INSTANCES; i++) {
        struct gyro_report	gyro_report;
        while (_gyro_fd[i] != -1 && 
               ::read(_gyro_fd[i], &gyro_report, sizeof(gyro_report)) == sizeof(gyro_report) &&
               gyro_report.timestamp != _last_gyro_timestamp[i]) {        
            _gyro_in[i] = Vector3f(gyro_report.x, gyro_report.y, gyro_report.z);
            _last_gyro_timestamp[i] = gyro_report.timestamp;
        }
    }
}

bool AP_InertialSensor_PX4::_sample_available(void)
{
    uint64_t tnow = hrt_absolute_time();
    while (tnow - _last_sample_timestamp > _sample_time_usec) {
        _have_sample_available = true;
        _last_sample_timestamp += _sample_time_usec;
    }
    return _have_sample_available;
}

bool AP_InertialSensor_PX4::wait_for_sample(uint16_t timeout_ms)
{
    if (_sample_available()) {
        return true;
    }
    uint32_t start = hal.scheduler->millis();
    while ((hal.scheduler->millis() - start) < timeout_ms) {
        uint64_t tnow = hrt_absolute_time();
        // we spin for the last timing_lag microseconds. Before that
        // we yield the CPU to allow IO to happen
        const uint16_t timing_lag = 400;
        if (_last_sample_timestamp + _sample_time_usec > tnow+timing_lag) {
            hal.scheduler->delay_microseconds(_last_sample_timestamp + _sample_time_usec - (tnow+timing_lag));
        }
        if (_sample_available()) {
            return true;
        }
    }
    return false;
}

/**
   try to detect bad accel/gyro sensors
 */
bool AP_InertialSensor_PX4::healthy(void) const
{
    return get_gyro_instance_health(0) && get_accel_instance_health(0);
}

#endif // CONFIG_HAL_BOARD

