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
// ITG3205 Gyroscope  http://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
// ADXL345 Accelerometer http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include "AP_InertialSensor_Flymaple.h"

const extern AP_HAL::HAL& hal;

/// Statics
Vector3f AP_InertialSensor_Flymaple::_accel_filtered;
uint32_t AP_InertialSensor_Flymaple::_accel_samples;
Vector3f AP_InertialSensor_Flymaple::_gyro_filtered;
uint32_t AP_InertialSensor_Flymaple::_gyro_samples;
uint64_t AP_InertialSensor_Flymaple::_last_accel_timestamp;
uint64_t AP_InertialSensor_Flymaple::_last_gyro_timestamp;
LowPassFilter2p AP_InertialSensor_Flymaple::_accel_filter_x(800, 10);
LowPassFilter2p AP_InertialSensor_Flymaple::_accel_filter_y(800, 10);
LowPassFilter2p AP_InertialSensor_Flymaple::_accel_filter_z(800, 10);
LowPassFilter2p AP_InertialSensor_Flymaple::_gyro_filter_x(800, 10);
LowPassFilter2p AP_InertialSensor_Flymaple::_gyro_filter_y(800, 10);
LowPassFilter2p AP_InertialSensor_Flymaple::_gyro_filter_z(800, 10);

// This is how often we wish to make raw samples of the sensors in Hz
const uint32_t  raw_sample_rate_hz = 800;
// And the equivalent time between samples in microseconds
const uint32_t  raw_sample_interval_us = (1000000 / raw_sample_rate_hz);

///////
/// Accelerometer ADXL345 register definitions
#define FLYMAPLE_ACCELEROMETER_ADDRESS              0x53
#define FLYMAPLE_ACCELEROMETER_XL345_DEVID          0xe5
#define FLYMAPLE_ACCELEROMETER_ADXLREG_BW_RATE      0x2c
#define FLYMAPLE_ACCELEROMETER_ADXLREG_POWER_CTL    0x2d
#define FLYMAPLE_ACCELEROMETER_ADXLREG_DATA_FORMAT  0x31
#define FLYMAPLE_ACCELEROMETER_ADXLREG_DEVID        0x00
#define FLYMAPLE_ACCELEROMETER_ADXLREG_DATAX0       0x32
#define FLYMAPLE_ACCELEROMETER_GRAVITY              248

// ADXL345 accelerometer scaling
// Result will be scaled to 1m/s/s
// ADXL345 in Full resolution mode (any g scaling) is 256 counts/g, so scale by 9.81/256 = 0.038320312
#define FLYMAPLE_ACCELEROMETER_SCALE_M_S    (GRAVITY_MSS / 256.0f)

/// Gyro ITG3205 register definitions
#define FLYMAPLE_GYRO_ADDRESS                       0x68
#define FLYMAPLE_GYRO_WHO_AM_I                      0x00
#define FLYMAPLE_GYRO_PWR_MGM                       0x3e
#define FLYMAPLE_GYRO_DLPF_FS                       0x16
#define FLYMAPLE_GYRO_INT_CFG                       0x17
#define FLYMAPLE_GYRO_SMPLRT_DIV                    0x15
#define FLYMAPLE_GYRO_GYROX_H                       0x1d

// ITG3200 Gyroscope scaling
// ITG3200 is 14.375 LSB degrees/sec with FS_SEL=3
// Result wil be radians/sec
#define FLYMAPLE_GYRO_SCALE_R_S (1.0f / 14.375f) * (3.1415926f / 180.0f)

uint16_t AP_InertialSensor_Flymaple::_init_sensor( Sample_rate sample_rate ) 
{
    // Sensors are raw sampled at 800Hz.
    // Here we figure the divider to get the rate that update should be called
    switch (sample_rate) {
    case RATE_50HZ:
        _sample_divider = raw_sample_rate_hz / 50;
        _default_filter_hz = 10;
        break;
    case RATE_100HZ:
        _sample_divider = raw_sample_rate_hz / 100;
        _default_filter_hz = 20;
        break;
    case RATE_200HZ:
    default:
        _sample_divider = raw_sample_rate_hz / 200;
        _default_filter_hz = 20;
        break;
    }

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))
        return false;

    // Init the accelerometer
    uint8_t data;
    hal.i2c->readRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_DEVID, &data);
    if (data != FLYMAPLE_ACCELEROMETER_XL345_DEVID)
        hal.scheduler->panic(PSTR("AP_InertialSensor_Flymaple: could not find ADXL345 accelerometer sensor"));
    hal.i2c->writeRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_POWER_CTL, 0x00);
    hal.scheduler->delay(5);
    hal.i2c->writeRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_POWER_CTL, 0xff);
    hal.scheduler->delay(5);
    // Measure mode:
    hal.i2c->writeRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_POWER_CTL, 0x08);
    hal.scheduler->delay(5);
    // Full resolution, 8g:
    // Caution, this must agree with FLYMAPLE_ACCELEROMETER_SCALE_1G
    // In full resoution mode, the scale factor need not change
    hal.i2c->writeRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_DATA_FORMAT, 0x08);
    hal.scheduler->delay(5);
    // Normal power, 800Hz Output Data Rate, 400Hz bandwidth:
    hal.i2c->writeRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_BW_RATE, 0x0d);
    hal.scheduler->delay(5);
    // Power up default is FIFO bypass mode. FIFO is not used by the chip

    // Init the Gyro
    // Expect to read the same as the Gyro I2C adress:
    hal.i2c->readRegister(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_WHO_AM_I, &data);
    if (data != FLYMAPLE_GYRO_ADDRESS)
        hal.scheduler->panic(PSTR("AP_InertialSensor_Flymaple: could not find ITG-3200 accelerometer sensor"));
    hal.i2c->writeRegister(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_PWR_MGM, 0x00);
    hal.scheduler->delay(1);
    // Sample rate divider: with 8kHz internal clock (see FLYMAPLE_GYRO_DLPF_FS), 
    // get 500Hz sample rate, 2 samples
    hal.i2c->writeRegister(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_SMPLRT_DIV, 0x0f);
    hal.scheduler->delay(1);
    // 2000 degrees/sec, 256Hz LPF, 8kHz internal sample rate
    // This is the least amount of filtering we can configure for this device
    hal.i2c->writeRegister(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_DLPF_FS, 0x18);
    hal.scheduler->delay(1);
    // No interrupts
    hal.i2c->writeRegister(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_INT_CFG, 0x00);
    hal.scheduler->delay(1);

    // Set up the filter desired
    _set_filter_frequency(_mpu6000_filter);

   // give back i2c semaphore
    i2c_sem->give();

    return AP_PRODUCT_ID_FLYMAPLE;
}

/*
  set the filter frequency
 */
void AP_InertialSensor_Flymaple::_set_filter_frequency(uint8_t filter_hz)
{
    if (filter_hz == 0)
        filter_hz = _default_filter_hz;

    _accel_filter_x.set_cutoff_frequency(raw_sample_rate_hz, filter_hz);
    _accel_filter_y.set_cutoff_frequency(raw_sample_rate_hz, filter_hz);
    _accel_filter_z.set_cutoff_frequency(raw_sample_rate_hz, filter_hz);
    _gyro_filter_x.set_cutoff_frequency(raw_sample_rate_hz, filter_hz);
    _gyro_filter_y.set_cutoff_frequency(raw_sample_rate_hz, filter_hz);
    _gyro_filter_z.set_cutoff_frequency(raw_sample_rate_hz, filter_hz);
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

// This takes about 20us to run
bool AP_InertialSensor_Flymaple::update(void) 
{
    if (!wait_for_sample(100)) {
        return false;
    }
    Vector3f accel_scale = _accel_scale[0].get();

    // Not really needed since Flymaple _accumulate runs in the main thread
    hal.scheduler->suspend_timer_procs();

    // base the time on the gyro timestamp, as that is what is
    // multiplied by time to integrate in DCM
    _delta_time = (_last_gyro_timestamp - _last_update_usec) * 1.0e-6f;
    _last_update_usec = _last_gyro_timestamp;

    _previous_accel[0] = _accel[0];

    _accel[0] = _accel_filtered;
    _accel_samples = 0;

    _gyro[0] = _gyro_filtered;
    _gyro_samples = 0;

    hal.scheduler->resume_timer_procs();

    // add offsets and rotation
    _accel[0].rotate(_board_orientation);

    // Adjust for chip scaling to get m/s/s
    _accel[0] *= FLYMAPLE_ACCELEROMETER_SCALE_M_S;

    // Now the calibration scale factor
    _accel[0].x *= accel_scale.x;
    _accel[0].y *= accel_scale.y;
    _accel[0].z *= accel_scale.z;
    _accel[0]   -= _accel_offset[0];

    _gyro[0].rotate(_board_orientation);

    // Adjust for chip scaling to get radians/sec
    _gyro[0] *= FLYMAPLE_GYRO_SCALE_R_S;
    _gyro[0] -= _gyro_offset[0];

    if (_last_filter_hz != _mpu6000_filter) {
        _set_filter_frequency(_mpu6000_filter);
        _last_filter_hz = _mpu6000_filter;
    }

    return true;
}

float AP_InertialSensor_Flymaple::get_delta_time(void) 
{
    return _delta_time;
}

float AP_InertialSensor_Flymaple::get_gyro_drift_rate(void) 
{
    // Dont really know this for the ITG-3200.
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

// This needs to get called as often as possible.
// Its job is to accumulate samples as fast as is reasonable for the accel and gyro
// sensors.
// Cant call this from within the system timers, since the long I2C reads (up to 1ms) 
// with interrupts disabled breaks lots of things
// Therefore must call this as often as possible from
// within the mainline and thropttle the reads here to suit the sensors
void AP_InertialSensor_Flymaple::_accumulate(void)
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))
        return;

    // Read accelerometer
    // ADXL345 is in the default FIFO bypass mode, so the FIFO is not used
    uint8_t buffer[6];
    uint64_t now = hal.scheduler->micros();
    // This takes about 250us at 400kHz I2C speed
    if ((now - _last_accel_timestamp) >= raw_sample_interval_us
        && hal.i2c->readRegisters(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_DATAX0, 6, buffer) == 0)
    {
        // The order is a bit wierd here since the standard we have adopted for Flymaple 
        // sensor orientation is different to what the board designers intended
        // Caution, to support alternative chip orientations on other bords, may 
        // need to add a chip orientation rotate
        int16_t y = -((((int16_t)buffer[1]) << 8) | buffer[0]);    // chip X axis
        int16_t x = -((((int16_t)buffer[3]) << 8) | buffer[2]);    // chip Y axis
        int16_t z = -((((int16_t)buffer[5]) << 8) | buffer[4]);    // chip Z axis
        _accel_filtered = Vector3f(_accel_filter_x.apply(x),
                                   _accel_filter_y.apply(y),
                                   _accel_filter_z.apply(z));
        _accel_samples++;
        _last_accel_timestamp = now;
    }

    // Read gyro
    now = hal.scheduler->micros();
    // This takes about 250us at 400kHz I2C speed
    if ((now - _last_gyro_timestamp) >= raw_sample_interval_us
        && hal.i2c->readRegisters(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_GYROX_H, 6, buffer) == 0)
    {
        // See above re order of samples in buffer
        int16_t y = -((((int16_t)buffer[0]) << 8) | buffer[1]);    // chip X axis
        int16_t x = -((((int16_t)buffer[2]) << 8) | buffer[3]);    // chip Y axis
        int16_t z = -((((int16_t)buffer[4]) << 8) | buffer[5]);    // chip Z axis
        _gyro_filtered = Vector3f(_gyro_filter_x.apply(x),
                                  _gyro_filter_y.apply(y),
                                  _gyro_filter_z.apply(z));
        _gyro_samples++;
        _last_gyro_timestamp = now;
    }

    // give back i2c semaphore
    i2c_sem->give();
}

bool AP_InertialSensor_Flymaple::_sample_available(void)
{
    _accumulate();
    return min(_accel_samples, _gyro_samples) / _sample_divider > 0;
}

bool AP_InertialSensor_Flymaple::wait_for_sample(uint16_t timeout_ms)
{
    if (_sample_available()) {
        return true;
    }
    uint32_t start = hal.scheduler->millis();
    while ((hal.scheduler->millis() - start) < timeout_ms) {
        hal.scheduler->delay_microseconds(100);
        if (_sample_available()) {
            return true;
        }
    }
    return false;
}

#endif // CONFIG_HAL_BOARD

