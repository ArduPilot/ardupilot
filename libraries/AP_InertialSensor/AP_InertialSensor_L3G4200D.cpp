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
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AP_InertialSensor_L3G4200D.h"

#include <inttypes.h>
#include <utility>

const extern AP_HAL::HAL &hal;

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
AP_InertialSensor_L3G4200D::AP_InertialSensor_L3G4200D(AP_InertialSensor &imu,
                                                       AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
{
}

AP_InertialSensor_L3G4200D::~AP_InertialSensor_L3G4200D()
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_L3G4200D::probe(AP_InertialSensor &imu,
                                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    AP_InertialSensor_L3G4200D *sensor
        = new AP_InertialSensor_L3G4200D(imu, std::move(dev));
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_L3G4200D::_init_sensor(void)
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // Init the accelerometer
    uint8_t data = 0;
    _dev->read_registers(ADXL345_ACCELEROMETER_ADXLREG_DEVID, &data, 1);
    if (data != ADXL345_ACCELEROMETER_XL345_DEVID) {
        AP_HAL::panic("AP_InertialSensor_L3G4200D: could not find ADXL345 accelerometer sensor");
    }

    _dev->write_register(ADXL345_ACCELEROMETER_ADXLREG_POWER_CTL, 0x00);
    hal.scheduler->delay(5);
    _dev->write_register(ADXL345_ACCELEROMETER_ADXLREG_POWER_CTL, 0xff);
    hal.scheduler->delay(5);
    // Measure mode:
    _dev->write_register(ADXL345_ACCELEROMETER_ADXLREG_POWER_CTL, 0x08);
    hal.scheduler->delay(5);

    // Full resolution, 8g:
    // Caution, this must agree with ADXL345_ACCELEROMETER_SCALE_1G
    // In full resoution mode, the scale factor need not change
    _dev->write_register(ADXL345_ACCELEROMETER_ADXLREG_DATA_FORMAT, 0x08);
    hal.scheduler->delay(5);

    // Normal power, 800Hz Output Data Rate, 400Hz bandwidth:
    _dev->write_register(ADXL345_ACCELEROMETER_ADXLREG_BW_RATE, 0x0d);
    hal.scheduler->delay(5);

    // enable FIFO in stream mode. This will allow us to read the accelerometers
    // much less frequently
    _dev->write_register(ADXL345_ACCELEROMETER_ADXLREG_FIFO_CTL,
                         ADXL345_ACCELEROMETER_ADXLREG_FIFO_CTL_STREAM);

    // Init the Gyro
    // Expect to read the right 'WHO_AM_I' value
    _dev->read_registers(L3G4200D_REG_WHO_AM_I, &data, 1);
    if (data != L3G4200D_REG_WHO_AM_I_VALUE) {
        AP_HAL::panic("AP_InertialSensor_L3G4200D: could not find L3G4200D gyro sensor");
    }

    // setup for 800Hz sampling with 110Hz filter
    _dev->write_register(L3G4200D_REG_CTRL_REG1,
                         L3G4200D_REG_CTRL_REG1_DRBW_800_110 |
                         L3G4200D_REG_CTRL_REG1_PD |
                         L3G4200D_REG_CTRL_REG1_XYZ_ENABLE);
    hal.scheduler->delay(1);

    _dev->write_register(L3G4200D_REG_CTRL_REG1,
                         L3G4200D_REG_CTRL_REG1_DRBW_800_110 |
                         L3G4200D_REG_CTRL_REG1_PD |
                         L3G4200D_REG_CTRL_REG1_XYZ_ENABLE);
    hal.scheduler->delay(1);

    _dev->write_register(L3G4200D_REG_CTRL_REG1,
                         L3G4200D_REG_CTRL_REG1_DRBW_800_110 |
                         L3G4200D_REG_CTRL_REG1_PD |
                         L3G4200D_REG_CTRL_REG1_XYZ_ENABLE);
    hal.scheduler->delay(1);

    // setup for 2000 degrees/sec full range
    _dev->write_register(L3G4200D_REG_CTRL_REG4,
                         L3G4200D_REG_CTRL_REG4_FS_2000);
    hal.scheduler->delay(1);

    // enable FIFO
    _dev->write_register(L3G4200D_REG_CTRL_REG5,
                         L3G4200D_REG_CTRL_REG5_FIFO_EN);
    hal.scheduler->delay(1);

    // enable FIFO in stream mode. This will allow us to read the gyros much less frequently
    _dev->write_register(L3G4200D_REG_FIFO_CTL,
                         L3G4200D_REG_FIFO_CTL_STREAM);
    hal.scheduler->delay(1);

    _dev->get_semaphore()->give();

    _gyro_instance = _imu.register_gyro(800);
    _accel_instance = _imu.register_accel(800);
    _product_id = AP_PRODUCT_ID_L3G4200D;

    // start the timer process to read samples
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_L3G4200D::_accumulate, void));

    return true;
}

/*
  copy filtered data to the frontend
 */
bool AP_InertialSensor_L3G4200D::update(void)
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);

    return true;
}

// Accumulate values from accels and gyros
void AP_InertialSensor_L3G4200D::_accumulate(void)
{
    if (!_dev->get_semaphore()->take_nonblocking()) {
        return;
    }

    uint8_t num_samples_available;
    uint8_t fifo_status = 0;

    // Read gyro FIFO status
    fifo_status = 0;
    _dev->read_registers(L3G4200D_REG_FIFO_SRC, &fifo_status, 1);
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
        if (!_dev->read_registers(L3G4200D_REG_XL | L3G4200D_REG_AUTO_INCREMENT,
                                  (uint8_t *)&buffer, sizeof(buffer))) {
            for (uint8_t i=0; i < num_samples_available; i++) {
                Vector3f gyro = Vector3f(buffer[i][0], -buffer[i][1], -buffer[i][2]);
                // Adjust for chip scaling to get radians/sec
                gyro *= L3G4200D_GYRO_SCALE_R_S;
                _rotate_and_correct_gyro(_gyro_instance, gyro);
                _notify_new_gyro_raw_sample(_gyro_instance, gyro);
            }
        }
    }

    // Read accelerometer FIFO to find out how many samples are available
    _dev->read_registers(ADXL345_ACCELEROMETER_ADXLREG_FIFO_STATUS,
                         &fifo_status, 1);
    num_samples_available = fifo_status & 0x3F;

    // read the samples and apply the filter
    if (num_samples_available > 0) {
        int16_t buffer[num_samples_available][3];
        if (!_dev->read_registers_multiple(ADXL345_ACCELEROMETER_ADXLREG_DATAX0,
                                           (uint8_t *)buffer, sizeof(buffer[0]),
                                           num_samples_available)) {
            for (uint8_t i=0; i<num_samples_available; i++) {
                Vector3f accel = Vector3f(buffer[i][0], -buffer[i][1], -buffer[i][2]);
                // Adjust for chip scaling to get m/s/s
                accel *= ADXL345_ACCELEROMETER_SCALE_M_S;
                _rotate_and_correct_accel(_accel_instance, accel);
                _notify_new_accel_raw_sample(_accel_instance, accel);
            }
        }
    }

    // give back i2c semaphore
    _dev->get_semaphore()->give();
}

#endif
