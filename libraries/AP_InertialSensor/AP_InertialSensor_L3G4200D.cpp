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
  This is an INS driver for the combination L3G4200D gyro and ADXL345 accelerometer.
  This combination is available as a cheap 10DOF sensor on ebay

  This sensor driver is an example only - it should not be used in any
  serious autopilot as the latencies on I2C prevent good timing at
  high sample rates. It is useful when doing an initial port of
  ardupilot to a board where only I2C is available, and a cheap sensor
  can be used.

Datasheets:
  ADXL345 Accelerometer http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf
  L3G4200D gyro http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00265057.pdf
*/

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_Math.h>
#include "AP_InertialSensor_L3G4200D.h"
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <sched.h>
#include <linux/rtc.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <sched.h>
#include <linux/rtc.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>

const extern AP_HAL::HAL& hal;

///////
/// Accelerometer ADXL345 register definitions
#define ADXL345_ACCELEROMETER_ADDRESS                  0x53
#define ADXL345_ACCELEROMETER_XL345_DEVID              0xe5
#define ADXL345_ACCELEROMETER_ADXLREG_BW_RATE          0x2c
#define ADXL345_ACCELEROMETER_ADXLREG_POWER_CTL        0x2d
#define ADXL345_ACCELEROMETER_ADXLREG_DATA_FORMAT      0x31
#define ADXL345_ACCELEROMETER_ADXLREG_DEVID            0x00
#define ADXL345_ACCELEROMETER_ADXLREG_DATAX0           0x32
#define ADXL345_ACCELEROMETER_ADXLREG_FIFO_CTL         0x38
#define ADXL345_ACCELEROMETER_ADXLREG_FIFO_CTL_STREAM     0x9F
#define ADXL345_ACCELEROMETER_ADXLREG_FIFO_STATUS      0x39

// ADXL345 accelerometer scaling
// Result will be scaled to 1m/s/s
// ADXL345 in Full resolution mode (any g scaling) is 256 counts/g, so scale by 9.81/256 = 0.038320312
#define ADXL345_ACCELEROMETER_SCALE_M_S    (GRAVITY_MSS / 256.0f)

/// Gyro ITG3205 register definitions
#define L3G4200D_I2C_ADDRESS                       0x69

#define L3G4200D_REG_WHO_AM_I                      0x0f
#define L3G4200D_REG_WHO_AM_I_VALUE                     0xd3

#define L3G4200D_REG_CTRL_REG1                     0x20
#define L3G4200D_REG_CTRL_REG1_DRBW_800_110             0xf0
#define L3G4200D_REG_CTRL_REG1_PD                       0x08
#define L3G4200D_REG_CTRL_REG1_XYZ_ENABLE               0x07

#define L3G4200D_REG_CTRL_REG4                     0x23
#define L3G4200D_REG_CTRL_REG4_FS_2000                  0x30

#define L3G4200D_REG_CTRL_REG5                     0x24
#define L3G4200D_REG_CTRL_REG5_FIFO_EN                  0x40

#define L3G4200D_REG_FIFO_CTL                      0x2e
#define L3G4200D_REG_FIFO_CTL_STREAM                    0x40

#define L3G4200D_REG_FIFO_SRC                      0x2f
#define L3G4200D_REG_FIFO_SRC_ENTRIES_MASK              0x1f
#define L3G4200D_REG_FIFO_SRC_EMPTY                     0x20
#define L3G4200D_REG_FIFO_SRC_OVERRUN                   0x40

#define L3G4200D_REG_XL                            0x28

// this bit is ORd into the register to enable auto-increment mode
#define L3G4200D_REG_AUTO_INCREMENT		           0x80

// L3G4200D Gyroscope scaling
// running at 2000 DPS full range, 16 bit signed data, datasheet 
// specifies 70 mdps per bit
#define L3G4200D_GYRO_SCALE_R_S (DEG_TO_RAD * 70.0f * 0.001f)

// constructor
AP_InertialSensor_L3G4200D::AP_InertialSensor_L3G4200D(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu),
    _have_gyro_sample(false),
    _have_accel_sample(false),
    _accel_filter_x(800, 10),
    _accel_filter_y(800, 10),
    _accel_filter_z(800, 10),
    _gyro_filter_x(800, 10),
    _gyro_filter_y(800, 10),
    _gyro_filter_z(800, 10)
{}

bool AP_InertialSensor_L3G4200D::_init_sensor(void) 
{
    _default_filter_hz = _default_filter();

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))
        return false;

    // Init the accelerometer
    uint8_t data = 0;
    hal.i2c->readRegister(ADXL345_ACCELEROMETER_ADDRESS, ADXL345_ACCELEROMETER_ADXLREG_DEVID, &data);
    if (data != ADXL345_ACCELEROMETER_XL345_DEVID) {
        hal.scheduler->panic(PSTR("AP_InertialSensor_L3G4200D: could not find ADXL345 accelerometer sensor"));
    }
    hal.i2c->writeRegister(ADXL345_ACCELEROMETER_ADDRESS, ADXL345_ACCELEROMETER_ADXLREG_POWER_CTL, 0x00);
    hal.scheduler->delay(5);
    hal.i2c->writeRegister(ADXL345_ACCELEROMETER_ADDRESS, ADXL345_ACCELEROMETER_ADXLREG_POWER_CTL, 0xff);
    hal.scheduler->delay(5);
    // Measure mode:
    hal.i2c->writeRegister(ADXL345_ACCELEROMETER_ADDRESS, ADXL345_ACCELEROMETER_ADXLREG_POWER_CTL, 0x08);
    hal.scheduler->delay(5);

    // Full resolution, 8g:
    // Caution, this must agree with ADXL345_ACCELEROMETER_SCALE_1G
    // In full resoution mode, the scale factor need not change
    hal.i2c->writeRegister(ADXL345_ACCELEROMETER_ADDRESS, ADXL345_ACCELEROMETER_ADXLREG_DATA_FORMAT, 0x08);
    hal.scheduler->delay(5);

    // Normal power, 800Hz Output Data Rate, 400Hz bandwidth:
    hal.i2c->writeRegister(ADXL345_ACCELEROMETER_ADDRESS, ADXL345_ACCELEROMETER_ADXLREG_BW_RATE, 0x0d);
    hal.scheduler->delay(5);

    // enable FIFO in stream mode. This will allow us to read the accelerometers much less frequently
    hal.i2c->writeRegister(ADXL345_ACCELEROMETER_ADDRESS, 
                           ADXL345_ACCELEROMETER_ADXLREG_FIFO_CTL, 
                           ADXL345_ACCELEROMETER_ADXLREG_FIFO_CTL_STREAM);

    // Init the Gyro
    // Expect to read the right 'WHO_AM_I' value
    hal.i2c->readRegister(L3G4200D_I2C_ADDRESS, L3G4200D_REG_WHO_AM_I, &data);
    if (data != L3G4200D_REG_WHO_AM_I_VALUE)
        hal.scheduler->panic(PSTR("AP_InertialSensor_L3G4200D: could not find L3G4200D gyro sensor"));

    // setup for 800Hz sampling with 110Hz filter
    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS, 
                           L3G4200D_REG_CTRL_REG1, 
                           L3G4200D_REG_CTRL_REG1_DRBW_800_110 |
                           L3G4200D_REG_CTRL_REG1_PD |
                           L3G4200D_REG_CTRL_REG1_XYZ_ENABLE);
    hal.scheduler->delay(1);

    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS, 
                           L3G4200D_REG_CTRL_REG1, 
                           L3G4200D_REG_CTRL_REG1_DRBW_800_110 |
                           L3G4200D_REG_CTRL_REG1_PD |
                           L3G4200D_REG_CTRL_REG1_XYZ_ENABLE);
    hal.scheduler->delay(1);

    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS, 
                           L3G4200D_REG_CTRL_REG1, 
                           L3G4200D_REG_CTRL_REG1_DRBW_800_110 |
                           L3G4200D_REG_CTRL_REG1_PD |
                           L3G4200D_REG_CTRL_REG1_XYZ_ENABLE);
    hal.scheduler->delay(1);

    // setup for 2000 degrees/sec full range
    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS, 
                           L3G4200D_REG_CTRL_REG4, 
                           L3G4200D_REG_CTRL_REG4_FS_2000);
    hal.scheduler->delay(1);

    // enable FIFO
    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS, 
                           L3G4200D_REG_CTRL_REG5, 
                           L3G4200D_REG_CTRL_REG5_FIFO_EN);
    hal.scheduler->delay(1);

    // enable FIFO in stream mode. This will allow us to read the gyros much less frequently
    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS,
                           L3G4200D_REG_FIFO_CTL,
                           L3G4200D_REG_FIFO_CTL_STREAM);
    hal.scheduler->delay(1);
                           

    // Set up the filter desired
    _set_filter_frequency(_imu.get_filter());

    // give back i2c semaphore
    i2c_sem->give();

    // start the timer process to read samples
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_InertialSensor_L3G4200D::_accumulate));

    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();

    _product_id = AP_PRODUCT_ID_L3G4200D;

    return true;
}

/*
  set the filter frequency
 */
void AP_InertialSensor_L3G4200D::_set_filter_frequency(uint8_t filter_hz)
{
    if (filter_hz == 0)
        filter_hz = _default_filter_hz;

    _accel_filter_x.set_cutoff_frequency(800, filter_hz);
    _accel_filter_y.set_cutoff_frequency(800, filter_hz);
    _accel_filter_z.set_cutoff_frequency(800, filter_hz);
    _gyro_filter_x.set_cutoff_frequency(800, filter_hz);
    _gyro_filter_y.set_cutoff_frequency(800, filter_hz);
    _gyro_filter_z.set_cutoff_frequency(800, filter_hz);
}

/*
  copy filtered data to the frontend
 */
bool AP_InertialSensor_L3G4200D::update(void) 
{
    Vector3f accel, gyro;

    hal.scheduler->suspend_timer_procs();
    accel = _accel_filtered;
    gyro = _gyro_filtered;
    _have_gyro_sample = false;
    _have_accel_sample = false;
    hal.scheduler->resume_timer_procs();

    // Adjust for chip scaling to get m/s/s
    accel *= ADXL345_ACCELEROMETER_SCALE_M_S;
    _rotate_and_offset_accel(_accel_instance, accel);

    // Adjust for chip scaling to get radians/sec
    gyro *= L3G4200D_GYRO_SCALE_R_S;
    _rotate_and_offset_gyro(_gyro_instance, gyro);

    if (_last_filter_hz != _imu.get_filter()) {
        _set_filter_frequency(_imu.get_filter());
        _last_filter_hz = _imu.get_filter();
    }

    return true;
}

// Accumulate values from accels and gyros
void AP_InertialSensor_L3G4200D::_accumulate(void)
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take_nonblocking())
        return;

    uint8_t num_samples_available;
    uint8_t fifo_status = 0;

    // Read gyro FIFO status
    fifo_status = 0;
    hal.i2c->readRegister(L3G4200D_I2C_ADDRESS,
                          L3G4200D_REG_FIFO_SRC,
                          &fifo_status);
    if (fifo_status & L3G4200D_REG_FIFO_SRC_OVERRUN) {
        // FIFO is full
        num_samples_available = 32;
    } else if (fifo_status & L3G4200D_REG_FIFO_SRC_EMPTY) {
        // FIFO is empty
        num_samples_available = 0;
    } else {
        // FIFO is partly full
        num_samples_available = fifo_status & L3G4200D_REG_FIFO_SRC_ENTRIES_MASK;
    }

    if (num_samples_available > 0) {
        // read all the entries in one go, using AUTO_INCREMENT. This saves a lot of time on I2C setup
        int16_t buffer[num_samples_available][3];
        if (hal.i2c->readRegisters(L3G4200D_I2C_ADDRESS, L3G4200D_REG_XL | L3G4200D_REG_AUTO_INCREMENT, 
                                   sizeof(buffer), (uint8_t *)&buffer[0][0]) == 0) {
            for (uint8_t i=0; i<num_samples_available; i++) {
                _gyro_filtered = Vector3f(_gyro_filter_x.apply(buffer[i][0]), 
                                          _gyro_filter_y.apply(-buffer[i][1]), 
                                          _gyro_filter_z.apply(-buffer[i][2]));
                _have_gyro_sample = true;
            }
        }
    }

    // Read accelerometer FIFO to find out how many samples are available
    hal.i2c->readRegister(ADXL345_ACCELEROMETER_ADDRESS,
                          ADXL345_ACCELEROMETER_ADXLREG_FIFO_STATUS,
                          &fifo_status);
    num_samples_available = fifo_status & 0x3F;

    // read the samples and apply the filter
    if (num_samples_available > 0) {
        int16_t buffer[num_samples_available][3];
        if (hal.i2c->readRegistersMultiple(ADXL345_ACCELEROMETER_ADDRESS, 
                                           ADXL345_ACCELEROMETER_ADXLREG_DATAX0, 
                                           sizeof(buffer[0]), num_samples_available,
                                           (uint8_t *)&buffer[0][0]) == 0) {
            for (uint8_t i=0; i<num_samples_available; i++) {
                _accel_filtered = Vector3f(_accel_filter_x.apply(buffer[i][0]),
                                           _accel_filter_y.apply(-buffer[i][1]),
                                           _accel_filter_z.apply(-buffer[i][2]));
                _have_accel_sample = true;
            }
        }
    }

    // give back i2c semaphore
    i2c_sem->give();
}

#endif // CONFIG_HAL_BOARD

