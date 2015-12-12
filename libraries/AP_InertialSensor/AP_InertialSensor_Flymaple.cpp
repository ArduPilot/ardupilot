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
  Flymaple IMU driver by Mike McCauley
 */

// Interface to the Flymaple sensors:
// ITG3205 Gyroscope  http://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
// ADXL345 Accelerometer http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include "AP_InertialSensor_Flymaple.h"

const extern AP_HAL::HAL& hal;

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

AP_InertialSensor_Flymaple::AP_InertialSensor_Flymaple(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_Flymaple::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_Flymaple *sensor = new AP_InertialSensor_Flymaple(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_InertialSensor_Flymaple::_init_sensor(void) 
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))
        return false;

    // Init the accelerometer
    uint8_t data;
    hal.i2c->readRegister(FLYMAPLE_ACCELEROMETER_ADDRESS, FLYMAPLE_ACCELEROMETER_ADXLREG_DEVID, &data);
    if (data != FLYMAPLE_ACCELEROMETER_XL345_DEVID)
        AP_HAL::panic("AP_InertialSensor_Flymaple: could not find ADXL345 accelerometer sensor");
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
        AP_HAL::panic("AP_InertialSensor_Flymaple: could not find ITG-3200 accelerometer sensor");
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

    // give back i2c semaphore
    i2c_sem->give();

    _gyro_instance = _imu.register_gyro(raw_sample_rate_hz);
    _accel_instance = _imu.register_accel(raw_sample_rate_hz);

    _product_id = AP_PRODUCT_ID_FLYMAPLE;

    return true;
}

// This takes about 20us to run
bool AP_InertialSensor_Flymaple::update(void) 
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);

    return true;
}

// This needs to get called as often as possible.
// Its job is to accumulate samples as fast as is reasonable for the accel and gyro
// sensors.
// Note that this is called from gyro_sample_available() and
// accel_sample_available(), which is really not good enough for
// 800Hz, as it is common for the main loop to take more than 1.5ms
// before wait_for_sample() is called again. We can't just call this
// from a timer as timers run with interrupts disabled, and the I2C
// operations take too long
// So we are stuck with a suboptimal solution. The results are not so
// good in terms of timing. It may be better with the FIFOs enabled
void AP_InertialSensor_Flymaple::accumulate(void)
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))
        return;

    // Read accelerometer
    // ADXL345 is in the default FIFO bypass mode, so the FIFO is not used
    uint8_t buffer[6];
    uint32_t now = AP_HAL::micros();
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
        Vector3f accel = Vector3f(x,y,z);
        // Adjust for chip scaling to get m/s/s
        accel *= FLYMAPLE_ACCELEROMETER_SCALE_M_S;
        _rotate_and_correct_accel(_accel_instance, accel);
        _notify_new_accel_raw_sample(_accel_instance, accel);
        _last_accel_timestamp = now;
    }

    // Read gyro
    now = AP_HAL::micros();
    // This takes about 250us at 400kHz I2C speed
    if ((now - _last_gyro_timestamp) >= raw_sample_interval_us
        && hal.i2c->readRegisters(FLYMAPLE_GYRO_ADDRESS, FLYMAPLE_GYRO_GYROX_H, 6, buffer) == 0)
    {
        // See above re order of samples in buffer
        int16_t y = -((((int16_t)buffer[0]) << 8) | buffer[1]);    // chip X axis
        int16_t x = -((((int16_t)buffer[2]) << 8) | buffer[3]);    // chip Y axis
        int16_t z = -((((int16_t)buffer[4]) << 8) | buffer[5]);    // chip Z axis
        Vector3f gyro = Vector3f(x,y,z);
        // Adjust for chip scaling to get radians/sec
        gyro *= FLYMAPLE_GYRO_SCALE_R_S;
        _rotate_and_correct_gyro(_gyro_instance, gyro);
        _notify_new_gyro_raw_sample(_gyro_instance, gyro);
    }

    // give back i2c semaphore
    i2c_sem->give();
}

#endif // CONFIG_HAL_BOARD
