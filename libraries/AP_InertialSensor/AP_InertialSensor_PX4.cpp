/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
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
    AP_InertialSensor_Backend(imu)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_PX4::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_PX4 *sensor = new AP_InertialSensor_PX4(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

/*
  calculate the right queue depth for a device with the given sensor
  sample rate
 */
uint8_t AP_InertialSensor_PX4::_queue_depth(uint16_t sensor_sample_rate) const
{
    uint16_t requested_sample_rate = get_sample_rate_hz();
    uint8_t min_depth = (sensor_sample_rate+requested_sample_rate-1)/requested_sample_rate;
    // add 5ms more worth of queue to account for possible timing jitter
    uint8_t ret = min_depth + (5 * sensor_sample_rate) / 1000;
    return ret;
}

bool AP_InertialSensor_PX4::_init_sensor(void) 
{
    // assumes max 3 instances
    _accel_fd[0] = open(ACCEL_BASE_DEVICE_PATH "0", O_RDONLY);
    _accel_fd[1] = open(ACCEL_BASE_DEVICE_PATH "1", O_RDONLY);
    _accel_fd[2] = open(ACCEL_BASE_DEVICE_PATH "2", O_RDONLY);
    _gyro_fd[0] = open(GYRO_BASE_DEVICE_PATH "0", O_RDONLY);
    _gyro_fd[1] = open(GYRO_BASE_DEVICE_PATH "1", O_RDONLY);
    _gyro_fd[2] = open(GYRO_BASE_DEVICE_PATH "2", O_RDONLY);

    _num_accel_instances = 0;
    _num_gyro_instances = 0;
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        if (_accel_fd[i] >= 0) {
            _num_accel_instances = i+1;
        }
        if (_gyro_fd[i] >= 0) {
            _num_gyro_instances = i+1;
        }
    }
    if (_num_accel_instances == 0) {
        return false;
    }
    if (_num_gyro_instances == 0) {
        return false;
    }

    for (uint8_t i=0; i<_num_gyro_instances; i++) {
        int fd = _gyro_fd[i];
        int devid = (ioctl(fd, DEVIOCGDEVICEID, 0) & 0x00FF0000)>>16;

        // software LPF off
        ioctl(fd, GYROIOCSLOWPASS, 0);
        // 2000dps range
        ioctl(fd, GYROIOCSRANGE, 2000);

        switch(devid) {
            case DRV_GYR_DEVTYPE_MPU6000:
            case DRV_GYR_DEVTYPE_MPU9250:
                // hardware LPF off
                ioctl(fd, GYROIOCSHWLOWPASS, 256);
                // khz sampling
                ioctl(fd, GYROIOCSSAMPLERATE, 1000);
                // set queue depth
                ioctl(fd, SENSORIOCSQUEUEDEPTH, _queue_depth(1000));
                break;
            case DRV_GYR_DEVTYPE_L3GD20:
                // hardware LPF as high as possible
                ioctl(fd, GYROIOCSHWLOWPASS, 100);
                // ~khz sampling
                ioctl(fd, GYROIOCSSAMPLERATE, 800);
                // 10ms queue depth
                ioctl(fd, SENSORIOCSQUEUEDEPTH, _queue_depth(800));
                break;
            default:
                break;
        }
        // calculate gyro sample time
        int samplerate = ioctl(fd,  GYROIOCGSAMPLERATE, 0);
        if (samplerate < 100 || samplerate > 10000) {
            AP_HAL::panic("Invalid gyro sample rate");
        }
        _gyro_instance[i] = _imu.register_gyro(samplerate);
        _gyro_sample_time[i] = 1.0f / samplerate;
    }

    for (uint8_t i=0; i<_num_accel_instances; i++) {
        int fd = _accel_fd[i];
        int devid = (ioctl(fd, DEVIOCGDEVICEID, 0) & 0x00FF0000)>>16;

        // software LPF off
        ioctl(fd, ACCELIOCSLOWPASS, 0);
        // 16g range
        ioctl(fd, ACCELIOCSRANGE, 16);

        switch(devid) {
            case DRV_ACC_DEVTYPE_MPU6000:
            case DRV_ACC_DEVTYPE_MPU9250:
                // hardware LPF off
                ioctl(fd, ACCELIOCSHWLOWPASS, 256);
                // khz sampling
                ioctl(fd, ACCELIOCSSAMPLERATE, 1000);
                // 10ms queue depth
                ioctl(fd, SENSORIOCSQUEUEDEPTH, _queue_depth(1000));
                break;
            case DRV_ACC_DEVTYPE_LSM303D:
                // hardware LPF to ~1/10th sample rate for antialiasing
                ioctl(fd, ACCELIOCSHWLOWPASS, 194);
                // ~khz sampling
                ioctl(fd, ACCELIOCSSAMPLERATE, 1600);
                ioctl(fd,SENSORIOCSPOLLRATE, 1600);
                // 10ms queue depth
                ioctl(fd, SENSORIOCSQUEUEDEPTH, _queue_depth(1600));
                break;
            default:
                break;
        }
        // calculate accel sample time
        int samplerate = ioctl(fd,  ACCELIOCGSAMPLERATE, 0);
        if (samplerate < 100 || samplerate > 10000) {
            AP_HAL::panic("Invalid accel sample rate");
        }
        _accel_instance[i] = _imu.register_accel(samplerate);
        _accel_sample_time[i] = 1.0f / samplerate;
    }

#if  CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    _product_id = AP_PRODUCT_ID_VRBRAIN;
#else
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
    _product_id = AP_PRODUCT_ID_PX4_V2;
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
    _product_id = AP_PRODUCT_ID_PX4_V4;
#else
    _product_id = AP_PRODUCT_ID_PX4;
#endif
#endif
    return true;
}

bool AP_InertialSensor_PX4::update(void) 
{
    // get the latest sample from the sensor drivers
    _get_sample();

    for (uint8_t k=0; k<_num_accel_instances; k++) {
        update_accel(_accel_instance[k]);
    }

    for (uint8_t k=0; k<_num_gyro_instances; k++) {
        update_gyro(_gyro_instance[k]);
    }
    
    return true;
}

void AP_InertialSensor_PX4::_new_accel_sample(uint8_t i, accel_report &accel_report)
{
    Vector3f accel = Vector3f(accel_report.x, accel_report.y, accel_report.z);
    uint8_t frontend_instance = _accel_instance[i];

    // apply corrections
    _rotate_and_correct_accel(frontend_instance, accel);
    _notify_new_accel_raw_sample(frontend_instance, accel, accel_report.timestamp);

    // save last timestamp
    _last_accel_timestamp[i] = accel_report.timestamp;

    // report error count
    _set_accel_error_count(frontend_instance, accel_report.error_count);

    // publish a temperature (for logging purposed only)
    _publish_temperature(frontend_instance, accel_report.temperature);

#ifdef AP_INERTIALSENSOR_PX4_DEBUG
    // get time since last sample
    float dt = _accel_sample_time[i];

    _accel_dt_max[i] = MAX(_accel_dt_max[i],dt);

    _accel_meas_count[i] ++;

    if(_accel_meas_count[i] >= 10000) {
        uint32_t tnow = AP_HAL::micros();

        ::printf("a%d %.2f Hz max %.8f s\n", frontend_instance, 10000.0f/((tnow-_accel_meas_count_start_us[i])*1.0e-6f),_accel_dt_max[i]);

        _accel_meas_count_start_us[i] = tnow;
        _accel_meas_count[i] = 0;
        _accel_dt_max[i] = 0;
    }
#endif // AP_INERTIALSENSOR_PX4_DEBUG
}

void AP_InertialSensor_PX4::_new_gyro_sample(uint8_t i, gyro_report &gyro_report)
{
    Vector3f gyro = Vector3f(gyro_report.x, gyro_report.y, gyro_report.z);
    uint8_t frontend_instance = _gyro_instance[i];

    // apply corrections
    _rotate_and_correct_gyro(frontend_instance, gyro);
    _notify_new_gyro_raw_sample(frontend_instance, gyro, gyro_report.timestamp);

    // save last timestamp
    _last_gyro_timestamp[i] = gyro_report.timestamp;

    // report error count
    _set_gyro_error_count(_gyro_instance[i], gyro_report.error_count);

#ifdef AP_INERTIALSENSOR_PX4_DEBUG
    // get time since last sample
    float dt = _gyro_sample_time[i];

    _gyro_dt_max[i] = MAX(_gyro_dt_max[i],dt);

    _gyro_meas_count[i] ++;

    if(_gyro_meas_count[i] >= 10000) {
        uint32_t tnow = AP_HAL::micros();

        ::printf("g%d %.2f Hz max %.8f s\n", frontend_instance, 10000.0f/((tnow-_gyro_meas_count_start_us[i])*1.0e-6f), _gyro_dt_max[i]);

        _gyro_meas_count_start_us[i] = tnow;
        _gyro_meas_count[i] = 0;
        _gyro_dt_max[i] = 0;
    }
#endif // AP_INERTIALSENSOR_PX4_DEBUG
}

void AP_InertialSensor_PX4::_get_sample()
{
    for (uint8_t i=0; i<MAX(_num_accel_instances,_num_gyro_instances);i++) {
        struct accel_report accel_report;
        struct gyro_report gyro_report;

        bool gyro_valid = _get_gyro_sample(i,gyro_report);
        bool accel_valid = _get_accel_sample(i,accel_report);

        while(gyro_valid || accel_valid) {
            // interleave accel and gyro samples by time - this will allow sculling corrections later
            // check the next gyro measurement to see if it needs to be integrated first
            if(gyro_valid && accel_valid && gyro_report.timestamp <= accel_report.timestamp) {
                _new_gyro_sample(i,gyro_report);
                gyro_valid = _get_gyro_sample(i,gyro_report);
                continue;
            }
            // if not, try to integrate an accelerometer sample
            if(accel_valid) {
                _new_accel_sample(i,accel_report);
                accel_valid = _get_accel_sample(i,accel_report);
                continue;
            }
            // if not, we've only got gyro samples left in the buffer
            if(gyro_valid) {
                _new_gyro_sample(i,gyro_report);
                gyro_valid = _get_gyro_sample(i,gyro_report);
            }
        }
    }
}

bool AP_InertialSensor_PX4::_get_accel_sample(uint8_t i, struct accel_report &accel_report) 
{
    if (i<_num_accel_instances && 
        _accel_fd[i] != -1 && 
        ::read(_accel_fd[i], &accel_report, sizeof(accel_report)) == sizeof(accel_report) && 
        accel_report.timestamp != _last_accel_timestamp[i]) {
        return true;
    }
    return false;
}

bool AP_InertialSensor_PX4::_get_gyro_sample(uint8_t i, struct gyro_report &gyro_report) 
{
    if (i<_num_gyro_instances && 
        _gyro_fd[i] != -1 && 
        ::read(_gyro_fd[i], &gyro_report, sizeof(gyro_report)) == sizeof(gyro_report) && 
        gyro_report.timestamp != _last_gyro_timestamp[i]) {
        return true;
    }
    return false;
}

#endif // CONFIG_HAL_BOARD

