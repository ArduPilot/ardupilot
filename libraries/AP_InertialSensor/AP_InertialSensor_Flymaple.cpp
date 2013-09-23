/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Flymaple port by Mike McCauley
 */

// Interface to the Flymaple sensors:
// ITG3205 Gyroscope
// ADXL345 Accelerometer

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
#include "AP_InertialSensor_Flymaple.h"

const extern AP_HAL::HAL& hal;


Vector3f AP_InertialSensor_Flymaple::_accel_sum;
uint32_t AP_InertialSensor_Flymaple::_accel_sum_count;
Vector3f AP_InertialSensor_Flymaple::_gyro_sum;
uint32_t AP_InertialSensor_Flymaple::_gyro_sum_count;
volatile bool AP_InertialSensor_Flymaple::_in_accumulate;
uint64_t AP_InertialSensor_Flymaple::_last_accel_timestamp;
uint64_t AP_InertialSensor_Flymaple::_last_gyro_timestamp;
int AP_InertialSensor_Flymaple::_accel_fd;
int AP_InertialSensor_Flymaple::_gyro_fd;

///////
/// Accelerometer ADXL345 definitions
#define FLYMAPLE_ACCELEROMETER_ADDRESS              0x53
#define FLYMAPLE_ACCELEROMETER_XL345_DEVID          0xe5
#define FLYMAPLE_ACCELEROMETER_ADXLREG_BW_RATE      0x2c
#define FLYMAPLE_ACCELEROMETER_ADXLREG_POWER_CTL    0x2d
#define FLYMAPLE_ACCELEROMETER_ADXLREG_DATA_FORMAT  0x31
#define FLYMAPLE_ACCELEROMETER_ADXLREG_DEVID        0x00
#define FLYMAPLE_ACCELEROMETER_ADXLREG_DATAX0       0x32
#define FLYMAPLE_ACCELEROMETER_GRAVITY              248

/// Gyro ITG3205 definitions
#define FLYMAPLE_GYRO_ADDRESS                       0x68
#define FLYMAPLE_GYRO_PWR_MGM                       0x3e
#define FLYMAPLE_GYRO_DLPF_FS                       0x16
#define FLYMAPLE_GYRO_INT_CFG                       0x17
#define FLYMAPLE_GYRO_SMPLRT_DIV                    0x15
#define FLYMAPLE_GYRO_GYROX_H                       0x1d

uint16_t AP_InertialSensor_Flymaple::_init_sensor( Sample_rate sample_rate ) 
{
    switch (sample_rate) {
    case RATE_50HZ:
        _sample_divider = 4;
        _default_filter_hz = 10;
        break;
    case RATE_100HZ:
        _sample_divider = 2;
        _default_filter_hz = 20;
        break;
    case RATE_200HZ:
    default:
        _sample_divider = 1;
        _default_filter_hz = 20;
        break;
    }

    // Init the accelerometer
    uint8_t data;
    hal.i2c->readRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_DEVID, &data);
    if (data != FLYMAPLE_ACCELEROMETER_XL345_DEVID)
        hal.scheduler->panic(PSTR("AP_InertialSensor_Flymaple: could not find ADXL345 accelerometer sensor"));
    hal.i2c->writeRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_POWER_CTL, 0x00);
    hal.scheduler->delay(5);
    hal.i2c->writeRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_POWER_CTL, 0xff);
    hal.scheduler->delay(5);
    hal.i2c->writeRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_POWER_CTL, 0x08);
    hal.scheduler->delay(5);
    hal.i2c->writeRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_DATA_FORMAT, 0x08);
    hal.scheduler->delay(5);
    hal.i2c->writeRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_BW_RATE, 0x09);
    hal.scheduler->delay(5);

    /// Init the Gyro
    hal.i2c->writeRegister(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_PWR_MGM, 0x00);
    hal.scheduler->delay(1);
    hal.i2c->writeRegister(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_SMPLRT_DIV, 0x07);
    hal.scheduler->delay(1);
    hal.i2c->writeRegister(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_DLPF_FS,0x1e);
    hal.scheduler->delay(1);
    hal.i2c->writeRegister(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_INT_CFG, 0x00);
    hal.scheduler->delay(1);

    return AP_PRODUCT_ID_FLYMAPLE;
}

/*
  set the filter frequency
 */
void AP_InertialSensor_Flymaple::_set_filter_frequency(uint8_t filter_hz)
{
    if (filter_hz == 0) {
        filter_hz = _default_filter_hz;
    }

/// TODO ...
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_Flymaple::update(void) 
{
    while (num_samples_available() == 0) {
        hal.scheduler->delay(1);
    }
    Vector3f accel_scale = _accel_scale.get();

    hal.scheduler->suspend_timer_procs();

    // base the time on the gyro timestamp, as that is what is
    // multiplied by time to integrate in DCM
    _delta_time = (_last_gyro_timestamp - _last_update_usec) * 1.0e-6f;
    _last_update_usec = _last_gyro_timestamp;

    _accel = _accel_sum / _accel_sum_count;
    _accel_sum.zero();
    _accel_sum_count = 0;

    _gyro = _gyro_sum / _gyro_sum_count;
    _gyro_sum.zero();
    _gyro_sum_count = 0;

    hal.scheduler->resume_timer_procs();

    // add offsets and rotation
    _accel.rotate(_board_orientation);
    _accel.x *= accel_scale.x;
    _accel.y *= accel_scale.y;
    _accel.z *= accel_scale.z;
    _accel   -= _accel_offset;

    _gyro.rotate(_board_orientation);
    _gyro *= (1.0 / 14.375) * (3.1415926 / 180); // ITG3200 14.375 LSB degrees/sec with FS_SEL=3
    _gyro -= _gyro_offset;

#if 0
// whats this all about????
    if (_last_filter_hz != _mpu6000_filter) {
        _set_filter_frequency(_mpu6000_filter);
        _last_filter_hz = _mpu6000_filter;
    }
#endif
    return true;
}

float AP_InertialSensor_Flymaple::get_delta_time(void) 
{
    return _delta_time;
}

uint32_t AP_InertialSensor_Flymaple::get_last_sample_time_micros(void) 
{
    return _last_update_usec;
}

float AP_InertialSensor_Flymaple::get_gyro_drift_rate(void) 
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

void AP_InertialSensor_Flymaple::_accumulate(void)
{
    if (_in_accumulate) {
        return;
    }
    _in_accumulate = true;

    // Read accelerometer
    uint8_t buffer[8];
    if (hal.i2c->readRegisters(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_DATAX0, 8, buffer) == 0)
    {
        int16_t x = -((((int16_t)buffer[1]) << 8) | buffer[0]);    // X axis
        int16_t y = (((int16_t)buffer[3]) << 8) | buffer[2];    // Y axis
        int16_t z = -((((int16_t)buffer[5]) << 8) | buffer[4]);    // Z axis
        _accel_sum += Vector3f(x, y, z);
        _accel_sum_count++;
        _last_accel_timestamp = hal.scheduler->micros();
    }

    // Read gyro
    if (hal.i2c->readRegisters(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_GYROX_H, 6, buffer) == 0)
    {
        int16_t x = (((int16_t)buffer[0]) << 8) | buffer[1];    // X axis
        int16_t y = (((int16_t)buffer[2]) << 8) | buffer[3];    // Y axis
        int16_t z = -((((int16_t)buffer[4]) << 8) | buffer[5]);    // Z axis
        _gyro_sum += Vector3f(x, y, z);
        _gyro_sum_count++;
        _last_gyro_timestamp = hal.scheduler->micros();
    }

    _in_accumulate = false;
}

void AP_InertialSensor_Flymaple::_ins_timer(uint32_t now)
{
    _accumulate();
}

uint16_t AP_InertialSensor_Flymaple::num_samples_available(void)
{
    _accumulate();
    return min(_accel_sum_count, _gyro_sum_count) / _sample_divider;
}

#endif // CONFIG_HAL_BOARD

