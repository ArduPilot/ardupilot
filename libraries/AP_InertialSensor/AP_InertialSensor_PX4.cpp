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
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

uint16_t AP_InertialSensor_PX4::_init_sensor( Sample_rate sample_rate ) {
    uint16_t rate_hz;
    int fd;

    switch (sample_rate) {
    case RATE_50HZ:
        rate_hz = 50;
        break;
    case RATE_100HZ:
        rate_hz = 100;
        break;
    case RATE_200HZ:
    default:
        rate_hz = 200;
        break;
    }

	// init accelerometers
	fd = open(ACCEL_DEVICE_PATH, 0);
	if (fd < 0) {
        hal.scheduler->panic("Unable to open accel device " ACCEL_DEVICE_PATH);
    }

    /* set the accel internal sampling rate */
    ioctl(fd, ACCELIOCSSAMPLERATE, rate_hz);

    /* set the driver poll rate */
    ioctl(fd, SENSORIOCSPOLLRATE, rate_hz);

    close(fd);
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));

	// init gyros
	fd = open(GYRO_DEVICE_PATH, 0);
	if (fd < 0) {
        hal.scheduler->panic("Unable to open gyro device " GYRO_DEVICE_PATH);
    }

    /* set the gyro internal sampling rate */
    ioctl(fd, GYROIOCSSAMPLERATE, rate_hz);

    /* set the driver poll rate  */
    ioctl(fd, SENSORIOCSPOLLRATE, rate_hz);

    close(fd);
	_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));

    return AP_PRODUCT_ID_PX4;
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_PX4::update(void) 
{
    while (num_samples_available() == 0) {
        hal.scheduler->delay_microseconds(1);
    }
    uint32_t now = hal.scheduler->micros();
    _delta_time_usec = now - _last_update_usec;
    _last_update_usec = now;

    Vector3f accel_scale = _accel_scale.get();

    _accel.x =   accel_scale.x * _raw_sensors.accelerometer_m_s2[0] / _raw_sensors.accelerometer_counter;
    _accel.y = - accel_scale.y * _raw_sensors.accelerometer_m_s2[1] / _raw_sensors.accelerometer_counter;
    _accel.z = - accel_scale.z * _raw_sensors.accelerometer_m_s2[2] / _raw_sensors.accelerometer_counter;
    _accel -= _accel_offset;

    _gyro.x =   _raw_sensors.gyro_rad_s[0] / _raw_sensors.gyro_counter;
    _gyro.y = - _raw_sensors.gyro_rad_s[1] / _raw_sensors.gyro_counter;
    _gyro.z = - _raw_sensors.gyro_rad_s[2] / _raw_sensors.gyro_counter;
    _gyro -= _gyro_offset;

    memset(&_raw_sensors, 0, sizeof(_raw_sensors));

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

uint32_t AP_InertialSensor_PX4::get_delta_time_micros(void) 
{
    return _delta_time_usec;
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
	bool accel_updated=false;
	bool gyro_updated =false;

	orb_check(_accel_sub, &accel_updated);
	if (accel_updated) {
		struct accel_report	accel_report;

		orb_copy(ORB_ID(sensor_accel), _accel_sub, &accel_report);

		_raw_sensors.accelerometer_m_s2[0] += accel_report.x;
		_raw_sensors.accelerometer_m_s2[1] += accel_report.y;
		_raw_sensors.accelerometer_m_s2[2] += accel_report.z;

		_raw_sensors.accelerometer_raw[0] = accel_report.x_raw;
		_raw_sensors.accelerometer_raw[1] = accel_report.y_raw;
		_raw_sensors.accelerometer_raw[2] = accel_report.z_raw;

		_raw_sensors.accelerometer_counter++;
	}

	orb_check(_gyro_sub, &gyro_updated);

	if (gyro_updated) {
		struct gyro_report	gyro_report;

		orb_copy(ORB_ID(sensor_gyro), _gyro_sub, &gyro_report);

		_raw_sensors.gyro_rad_s[0] += gyro_report.x;
		_raw_sensors.gyro_rad_s[1] += gyro_report.y;
		_raw_sensors.gyro_rad_s[2] += gyro_report.z;

		_raw_sensors.gyro_raw[0] = gyro_report.x_raw;
		_raw_sensors.gyro_raw[1] = gyro_report.y_raw;
		_raw_sensors.gyro_raw[2] = gyro_report.z_raw;

		_raw_sensors.gyro_counter++;
	}

    return min(_raw_sensors.accelerometer_counter, _raw_sensors.gyro_counter);
}

#endif // CONFIG_HAL_BOARD

