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

    _delta_time = _sample_time_usec * 1.0e-6f;

	// init accelerometers
	_accel_fd = open(ACCEL_DEVICE_PATH, O_RDONLY);
	if (_accel_fd < 0) {
        hal.scheduler->panic("Unable to open accel device " ACCEL_DEVICE_PATH);
    }

	_gyro_fd = open(GYRO_DEVICE_PATH, O_RDONLY);
	if (_gyro_fd < 0) {
        hal.scheduler->panic("Unable to open gyro device " GYRO_DEVICE_PATH);
    }

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
    uint32_t driver_rate = 1000;
#else
    uint32_t driver_rate = 800;
#endif

    /* 
     * set the accel and gyro sampling rate. 
     */
    ioctl(_accel_fd, ACCELIOCSSAMPLERATE, driver_rate);
    ioctl(_accel_fd, SENSORIOCSPOLLRATE,  driver_rate);
    ioctl(_gyro_fd,  GYROIOCSSAMPLERATE,  driver_rate);
    ioctl(_gyro_fd,  SENSORIOCSPOLLRATE,  driver_rate);

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
    ioctl(_gyro_fd,  GYROIOCSLOWPASS,  filter_hz);
    ioctl(_accel_fd, ACCELIOCSLOWPASS, filter_hz);
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_PX4::update(void) 
{
    Vector3f accel_scale = _accel_scale.get();

    // get the latest sample from the sensor drivers
    _get_sample();

    _accel = _accel_in;
    _gyro  = _gyro_in;

    // add offsets and rotation
    _accel.rotate(_board_orientation);
    _accel.x *= accel_scale.x;
    _accel.y *= accel_scale.y;
    _accel.z *= accel_scale.z;
    _accel   -= _accel_offset;

    _gyro.rotate(_board_orientation);
    _gyro -= _gyro_offset;

    if (_last_filter_hz != _mpu6000_filter) {
        _set_filter_frequency(_mpu6000_filter);
        _last_filter_hz = _mpu6000_filter;
    }

    _num_samples_available = 0;

    return true;
}

float AP_InertialSensor_PX4::get_delta_time(void)
{
    return _delta_time;
}

float AP_InertialSensor_PX4::get_gyro_drift_rate(void) 
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

void AP_InertialSensor_PX4::_get_sample(void)
{
    struct accel_report	accel_report;
    struct gyro_report	gyro_report;

    while (::read(_accel_fd, &accel_report, sizeof(accel_report)) == sizeof(accel_report) &&
        accel_report.timestamp != _last_accel_timestamp) {        
        _accel_in = Vector3f(accel_report.x, accel_report.y, accel_report.z);
        _last_accel_timestamp = accel_report.timestamp;
	}

    while (::read(_gyro_fd, &gyro_report, sizeof(gyro_report)) == sizeof(gyro_report) &&
        gyro_report.timestamp != _last_gyro_timestamp) {        
        _gyro_in = Vector3f(gyro_report.x, gyro_report.y, gyro_report.z);
        _last_gyro_timestamp = gyro_report.timestamp;
	}
}

uint16_t AP_InertialSensor_PX4::num_samples_available(void)
{
    uint64_t tnow = hrt_absolute_time();
    if (tnow - _last_sample_timestamp > _sample_time_usec) {
        _num_samples_available++;
        _last_sample_timestamp = tnow;
    }
    return _num_samples_available;
}

#endif // CONFIG_HAL_BOARD

