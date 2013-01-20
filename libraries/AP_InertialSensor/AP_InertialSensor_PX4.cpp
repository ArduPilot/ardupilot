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

uint16_t AP_InertialSensor_PX4::_init_sensor( Sample_rate sample_rate ) 
{
    switch (sample_rate) {
    case RATE_50HZ:
        _sample_divider = 4;
        break;
    case RATE_100HZ:
        _sample_divider = 2;
        break;
    case RATE_200HZ:
    default:
        _sample_divider = 1;
        break;
    }

	// init accelerometers
	_accel_fd = open(ACCEL_DEVICE_PATH, O_RDONLY);
	if (_accel_fd < 0) {
        hal.scheduler->panic("Unable to open accel device " ACCEL_DEVICE_PATH);
    }

	_gyro_fd = open(GYRO_DEVICE_PATH, O_RDONLY);
	if (_gyro_fd < 0) {
        hal.scheduler->panic("Unable to open gyro device " GYRO_DEVICE_PATH);
    }

    /* 
     * set the accel and gyro sampling rate. We always set these to
     * 200 then average in this driver
     */
    ioctl(_accel_fd, ACCELIOCSSAMPLERATE, 200);
    ioctl(_accel_fd, SENSORIOCSPOLLRATE,  200);
    ioctl(_gyro_fd,  GYROIOCSSAMPLERATE,  200);
    ioctl(_gyro_fd,  SENSORIOCSPOLLRATE,  200);

    // ask for a 10 sample buffer. The mpu6000 PX4 driver doesn't
    // support this yet, but when it does we want to use it
    ioctl(_accel_fd, SENSORIOCSQUEUEDEPTH, 10);
    ioctl(_gyro_fd,  SENSORIOCSQUEUEDEPTH, 10);

    return AP_PRODUCT_ID_PX4;
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_PX4::update(void) 
{
    while (num_samples_available() == 0) {
        hal.scheduler->delay(1);
    }
    uint32_t now = hal.scheduler->micros();

    // the current mpu6000 PX4 driver does not buffer samples, so
    // using the sample count times 5ms would produce a bad delta time
    // if we missed one. For now we need to use the clock to get the
    // delta time
    _delta_time = (now - _last_update_usec) * 1.0e-6f;
    _last_update_usec = now;

    Vector3f accel_scale = _accel_scale.get();

    _accel   = _accel_sum / _accel_sum_count;
    _accel.rotate(_board_orientation);
    _accel.x *= accel_scale.x;
    _accel.y *= accel_scale.y;
    _accel.z *= accel_scale.z;
    _accel   -= _accel_offset;

    _gyro    = _gyro_sum / _gyro_sum_count;
    _gyro.rotate(_board_orientation);
    _gyro   -= _gyro_offset;

    _accel_sum.zero();
    _accel_sum_count = 0;
    _gyro_sum.zero();
    _gyro_sum_count = 0;

    return true;
}

bool AP_InertialSensor_PX4::new_data_available(void) 
{
    return num_samples_available() > 0;
}


float AP_InertialSensor_PX4::temperature(void) 
{
    return 0.0;
}

float AP_InertialSensor_PX4::get_delta_time(void) 
{
    return _delta_time;
}

uint32_t AP_InertialSensor_PX4::get_last_sample_time_micros(void) 
{
    return _last_update_usec;
}

float AP_InertialSensor_PX4::get_gyro_drift_rate(void) 
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

uint16_t AP_InertialSensor_PX4::num_samples_available(void)
{
    struct accel_report	accel_report;
    struct gyro_report	gyro_report;
    
    if (::read(_accel_fd, &accel_report, sizeof(accel_report)) == sizeof(accel_report)) {
        _accel_sum += Vector3f(accel_report.x, accel_report.y, accel_report.z);
        _accel_sum_count++;
	}

    if (::read(_gyro_fd, &gyro_report, sizeof(gyro_report)) == sizeof(gyro_report)) {
        _gyro_sum += Vector3f(gyro_report.x, gyro_report.y, gyro_report.z);
        _gyro_sum_count++;
	}

    return min(_accel_sum_count, _gyro_sum_count) / _sample_divider;
}

#endif // CONFIG_HAL_BOARD

