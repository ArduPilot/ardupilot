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

    -- Coded by Victor Mayoral Vilches --
*/

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AP_InertialSensor_MPU9250.h"
#include "../AP_HAL_Linux/GPIO.h"

extern const AP_HAL::HAL& hal;

// MPU6000 accelerometer scaling
#define MPU9250_ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f)

#define MPUREG_XG_OFFS_TC                               0x00
#define MPUREG_YG_OFFS_TC                               0x01
#define MPUREG_ZG_OFFS_TC                               0x02
#define MPUREG_X_FINE_GAIN                              0x03
#define MPUREG_Y_FINE_GAIN                              0x04
#define MPUREG_Z_FINE_GAIN                              0x05

// MPU9250 registers
#define MPUREG_XA_OFFS_H                                0x77    // X axis accelerometer offset (high byte)
#define MPUREG_XA_OFFS_L                                0x78    // X axis accelerometer offset (low byte)
#define MPUREG_YA_OFFS_H                                0x7A    // Y axis accelerometer offset (high byte)
#define MPUREG_YA_OFFS_L                                0x0B    // Y axis accelerometer offset (low byte)
#define MPUREG_ZA_OFFS_H                                0x0D    // Z axis accelerometer offset (high byte)
#define MPUREG_ZA_OFFS_L                                0x0E    // Z axis accelerometer offset (low byte)

// MPU6000 & MPU9250 registers
// not sure if present in MPU9250
// #define MPUREG_PRODUCT_ID                               0x0C    // Product ID Register
#define MPUREG_XG_OFFS_USRH                     0x13    // X axis gyro offset (high byte)
#define MPUREG_XG_OFFS_USRL                     0x14    // X axis gyro offset (low byte)
#define MPUREG_YG_OFFS_USRH                     0x15    // Y axis gyro offset (high byte)
#define MPUREG_YG_OFFS_USRL                     0x16    // Y axis gyro offset (low byte)
#define MPUREG_ZG_OFFS_USRH                     0x17    // Z axis gyro offset (high byte)
#define MPUREG_ZG_OFFS_USRL                     0x18    // Z axis gyro offset (low byte)
#define MPUREG_SMPLRT_DIV                               0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
#       define MPUREG_SMPLRT_1000HZ                             0x00
#       define MPUREG_SMPLRT_500HZ                              0x01
#       define MPUREG_SMPLRT_250HZ                              0x03
#       define MPUREG_SMPLRT_200HZ                              0x04
#       define MPUREG_SMPLRT_100HZ                              0x09
#       define MPUREG_SMPLRT_50HZ                               0x13
#define MPUREG_CONFIG                                           0x1A
#define MPUREG_GYRO_CONFIG                                      0x1B
// bit definitions for MPUREG_GYRO_CONFIG
#       define BITS_GYRO_FS_250DPS                              0x00
#       define BITS_GYRO_FS_500DPS                              0x08
#       define BITS_GYRO_FS_1000DPS                             0x10
#       define BITS_GYRO_FS_2000DPS                             0x18
#       define BITS_GYRO_FS_MASK                                0x18    // only bits 3 and 4 are used for gyro full scale so use this to mask off other bits
#       define BITS_GYRO_ZGYRO_SELFTEST                 0x20
#       define BITS_GYRO_YGYRO_SELFTEST                 0x40
#       define BITS_GYRO_XGYRO_SELFTEST                 0x80
#define MPUREG_ACCEL_CONFIG                             0x1C
#define MPUREG_MOT_THR                                  0x1F    // detection threshold for Motion interrupt generation.  Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
#define MPUREG_MOT_DUR                                  0x20    // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms
#define MPUREG_ZRMOT_THR                                0x21    // detection threshold for Zero Motion interrupt generation.
#define MPUREG_ZRMOT_DUR                                0x22    // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms.
#define MPUREG_FIFO_EN                                  0x23
#define MPUREG_INT_PIN_CFG                              0x37
#       define BIT_INT_RD_CLEAR                                 0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                                 0x20    // latch data ready pin
#define MPUREG_INT_ENABLE                               0x38
// bit definitions for MPUREG_INT_ENABLE
#       define BIT_RAW_RDY_EN                                   0x01
#       define BIT_DMP_INT_EN                                   0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#       define BIT_UNKNOWN_INT_EN                               0x04
#       define BIT_I2C_MST_INT_EN                               0x08
#       define BIT_FIFO_OFLOW_EN                                0x10
#       define BIT_ZMOT_EN                                              0x20
#       define BIT_MOT_EN                                               0x40
#       define BIT_FF_EN                                                0x80
#define MPUREG_INT_STATUS                               0x3A
// bit definitions for MPUREG_INT_STATUS (same bit pattern as above because this register shows what interrupt actually fired)
#       define BIT_RAW_RDY_INT                                  0x01
#       define BIT_DMP_INT                                              0x02
#       define BIT_UNKNOWN_INT                                  0x04
#       define BIT_I2C_MST_INT                                  0x08
#       define BIT_FIFO_OFLOW_INT                               0x10
#       define BIT_ZMOT_INT                                             0x20
#       define BIT_MOT_INT                                              0x40
#       define BIT_FF_INT                                               0x80
#define MPUREG_ACCEL_XOUT_H                             0x3B
#define MPUREG_ACCEL_XOUT_L                             0x3C
#define MPUREG_ACCEL_YOUT_H                             0x3D
#define MPUREG_ACCEL_YOUT_L                             0x3E
#define MPUREG_ACCEL_ZOUT_H                             0x3F
#define MPUREG_ACCEL_ZOUT_L                             0x40
#define MPUREG_TEMP_OUT_H                               0x41
#define MPUREG_TEMP_OUT_L                               0x42
#define MPUREG_GYRO_XOUT_H                              0x43
#define MPUREG_GYRO_XOUT_L                              0x44
#define MPUREG_GYRO_YOUT_H                              0x45
#define MPUREG_GYRO_YOUT_L                              0x46
#define MPUREG_GYRO_ZOUT_H                              0x47
#define MPUREG_GYRO_ZOUT_L                              0x48
#define MPUREG_USER_CTRL                                0x6A
// bit definitions for MPUREG_USER_CTRL
#       define BIT_USER_CTRL_SIG_COND_RESET             0x01            // resets signal paths and results registers for all sensors (gyros, accel, temp)
#       define BIT_USER_CTRL_I2C_MST_RESET              0x02            // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_FIFO_RESET                 0x04            // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET                  0x08            // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS                 0x10            // Disable primary I2C interface and enable hal.spi->interface
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20            // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                    0x40            // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN                             0x80            // Enable DMP operations
#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_XGYRO                 0x01            // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO                 0x02            // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO                 0x03            // PLL with Z axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                             0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                             0x40            // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device
#define MPUREG_PWR_MGMT_2                               0x6C            // allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define MPUREG_BANK_SEL                                 0x6D            // DMP bank selection register (used to indirectly access DMP registers)
#define MPUREG_MEM_START_ADDR                   0x6E            // DMP memory start address (used to indirectly write to dmp memory)
#define MPUREG_MEM_R_W                                  0x6F            // DMP related register
#define MPUREG_DMP_CFG_1                                0x70            // DMP related register
#define MPUREG_DMP_CFG_2                                0x71            // DMP related register
#define MPUREG_FIFO_COUNTH                              0x72
#define MPUREG_FIFO_COUNTL                              0x73
#define MPUREG_FIFO_R_W                                 0x74
#define MPUREG_WHOAMI                                   0x75


// Configuration bits MPU 3000, MPU 6000 and MPU9250
#define BITS_DLPF_CFG_256HZ_NOLPF2              0x00
#define BITS_DLPF_CFG_188HZ                             0x01
#define BITS_DLPF_CFG_98HZ                              0x02
#define BITS_DLPF_CFG_42HZ                              0x03
#define BITS_DLPF_CFG_20HZ                              0x04
#define BITS_DLPF_CFG_10HZ                              0x05
#define BITS_DLPF_CFG_5HZ                               0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF              0x07
#define BITS_DLPF_CFG_MASK                              0x07

/*
 *  PS-MPU-9250A-00.pdf, page 8, lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
#define GYRO_SCALE (0.0174532f / 16.4f)

/*
 *  PS-MPU-9250A-00.pdf, page 9, lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPUXk
 *  variants however
 */

AP_InertialSensor_MPU9250::AP_InertialSensor_MPU9250(AP_InertialSensor &imu) :
	AP_InertialSensor_Backend(imu),
    _last_filter_hz(-1),
    _shared_data_idx(0),
    _accel_filter_x(1000, 15),
    _accel_filter_y(1000, 15),
    _accel_filter_z(1000, 15),
    _gyro_filter_x(1000, 15),
    _gyro_filter_y(1000, 15),
    _gyro_filter_z(1000, 15),
    _have_sample_available(false)
{
}


/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_MPU9250::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_MPU9250 *sensor = new AP_InertialSensor_MPU9250(_imu);
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
  initialise the sensor
 */
bool AP_InertialSensor_MPU9250::_init_sensor(void)
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_MPU9250);
    _spi_sem = _spi->get_semaphore();

    // we need to suspend timers to prevent other SPI drivers grabbing
    // the bus while we do the long initialisation
    hal.scheduler->suspend_timer_procs();

    uint8_t whoami = _register_read(MPUREG_WHOAMI);
    if (whoami != 0x71) {
        // TODO: we should probably accept multiple chip
        // revisions. This is the one on the PXF
        hal.console->printf("MPU9250: unexpected WHOAMI 0x%x\n", (unsigned)whoami);
        return false;
    }

    uint8_t tries = 0;
    do {
        bool success = _hardware_init();
        if (success) {
            hal.scheduler->delay(10);
            if (!_spi_sem->take(100)) {
                hal.console->printf("MPU9250: Unable to get semaphore");
                return false;
            }
            uint8_t status = _register_read(MPUREG_INT_STATUS);
            if ((status & BIT_RAW_RDY_INT) != 0) {
                _spi_sem->give();
                break;
            }
            _spi_sem->give();
        }
        if (tries++ > 5) {
            return false;
        }
    } while (1);

    hal.scheduler->resume_timer_procs();

    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();

    _product_id = AP_PRODUCT_ID_MPU9250;

    // start the timer process to read samples
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_InertialSensor_MPU9250::_poll_data));

#if MPU9250_DEBUG
    _dump_registers();
#endif
    return true;
}

/*
  update the accel and gyro vectors
 */
bool AP_InertialSensor_MPU9250::update( void )
{
    // pull the data from the timer shared data buffer
    uint8_t idx = _shared_data_idx;
    Vector3f gyro = _shared_data[idx]._gyro_filtered;
    Vector3f accel = _shared_data[idx]._accel_filtered;

    _have_sample_available = false;

    accel *= MPU9250_ACCEL_SCALE_1G;
    gyro *= GYRO_SCALE;

    // rotate for bbone default
    accel.rotate(ROTATION_ROLL_180_YAW_90);
    gyro.rotate(ROTATION_ROLL_180_YAW_90);

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
    // PXF has an additional YAW 180
    accel.rotate(ROTATION_YAW_180);
    gyro.rotate(ROTATION_YAW_180);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
    // NavIO has different orientation, assuming RaspberryPi is right
    // way up, and PWM pins on NavIO are at the back of the aircraft
    accel.rotate(ROTATION_ROLL_180_YAW_90);
    gyro.rotate(ROTATION_ROLL_180_YAW_90);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    accel.rotate(ROTATION_ROLL_180);
    gyro.rotate(ROTATION_ROLL_180);
#endif

    _rotate_and_offset_gyro(_gyro_instance, gyro);
    _rotate_and_offset_accel(_accel_instance, accel);

    if (_last_filter_hz != _imu.get_filter()) {
        _set_filter(_imu.get_filter());
        _last_filter_hz = _imu.get_filter();
    }

    return true;
}

/*================ HARDWARE FUNCTIONS ==================== */

/**
 * Timer process to poll for new data from the MPU9250.
 */
void AP_InertialSensor_MPU9250::_poll_data(void)
{
    if (!_spi_sem->take_nonblocking()) {
        /*
          the semaphore being busy is an expected condition when the
          mainline code is calling wait_for_sample() which will
          grab the semaphore. We return now and rely on the mainline
          code grabbing the latest sample.
        */
        return;
    }
    _read_data_transaction();
    _spi_sem->give();
}


/*
  read from the data registers and update filtered data
 */
void AP_InertialSensor_MPU9250::_read_data_transaction() 
{
    /* one resister address followed by seven 2-byte registers */
    struct PACKED {
        uint8_t cmd;
        uint8_t int_status;
        uint8_t v[14];
    } rx, tx = { cmd : MPUREG_INT_STATUS | 0x80, };

    _spi->transaction((const uint8_t *)&tx, (uint8_t *)&rx, sizeof(rx));

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))

    Vector3f _accel_filtered = Vector3f(_accel_filter_x.apply(int16_val(rx.v, 1)), 
                                        _accel_filter_y.apply(int16_val(rx.v, 0)), 
                                        _accel_filter_z.apply(-int16_val(rx.v, 2)));

    Vector3f _gyro_filtered = Vector3f(_gyro_filter_x.apply(int16_val(rx.v, 5)), 
                                       _gyro_filter_y.apply(int16_val(rx.v, 4)), 
                                       _gyro_filter_z.apply(-int16_val(rx.v, 6)));
    // update the shared buffer
    uint8_t idx = _shared_data_idx ^ 1;
    _shared_data[idx]._accel_filtered = _accel_filtered;
    _shared_data[idx]._gyro_filtered = _gyro_filtered;
    _shared_data_idx = idx;

    _have_sample_available = true;
}

/*
  read an 8 bit register
 */
uint8_t AP_InertialSensor_MPU9250::_register_read( uint8_t reg )
{
    uint8_t addr = reg | 0x80; // Set most significant bit

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _spi->transaction(tx, rx, 2);
    return rx[1];
}

/*
  write an 8 bit register
 */
void AP_InertialSensor_MPU9250::_register_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg;
    tx[1] = val;
    _spi->transaction(tx, rx, 2);
}

/*
  set the accel/gyro filter frequency
 */
void AP_InertialSensor_MPU9250::_set_filter(uint8_t filter_hz)
{
    if (filter_hz == 0) {
        filter_hz = _default_filter_hz;
    }

    _accel_filter_x.set_cutoff_frequency(1000, filter_hz);
    _accel_filter_y.set_cutoff_frequency(1000, filter_hz);
    _accel_filter_z.set_cutoff_frequency(1000, filter_hz);

    _gyro_filter_x.set_cutoff_frequency(1000, filter_hz);
    _gyro_filter_y.set_cutoff_frequency(1000, filter_hz);
    _gyro_filter_z.set_cutoff_frequency(1000, filter_hz);    
}


/*
  initialise the sensor configuration registers
 */
bool AP_InertialSensor_MPU9250::_hardware_init(void)
{
    if (!_spi_sem->take(100)) {
        hal.console->printf("MPU9250: Unable to get semaphore");
        return false;
    }

    // initially run the bus at low speed
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries<5; tries++) {
#if HAL_COMPASS_DEFAULT != HAL_COMPASS_AK8963 
        /* Prevent reseting if internal AK8963 is selected, because it may corrupt
         * AK8963's initialisation.  */
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
#endif
        hal.scheduler->delay(100);

        // Wake up device and select GyroZ clock. Note that the
        // MPU6000 starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }
#if MPU9250_DEBUG
        _dump_registers();
#endif
    }
    if (tries == 5) {
        hal.console->println_P(PSTR("Failed to boot MPU9250 5 times"));
        _spi_sem->give();
        return false;
    }

    _register_write(MPUREG_PWR_MGMT_2, 0x00);            // only used for wake-up in accelerometer only low power mode

    // Disable I2C bus (recommended on datasheet)
#if HAL_COMPASS_DEFAULT != HAL_COMPASS_AK8963 
     /* Prevent disabling if internal AK8963 is selected. If internal AK8963 is not used
      * it's ok to disable I2C slaves*/
    _register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_IF_DIS);
#endif

    _default_filter_hz = _default_filter();

    // used a fixed filter of 42Hz on the sensor, then filter using
    // the 2-pole software filter
    _register_write(MPUREG_CONFIG, BITS_DLPF_CFG_42HZ);

    // set sample rate to 1kHz, and use the 2 pole filter to give the
    // desired rate
    _register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_1000HZ);
    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);  // Gyro scale 2000º/s

    // RM-MPU-9250A-00.pdf, pg. 15, select accel full scale 8g
    _register_write(MPUREG_ACCEL_CONFIG,2<<3);

    // configure interrupt to fire when new data arrives
    _register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    _register_write(MPUREG_INT_PIN_CFG, BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN);

    // now that we have initialised, we set the SPI bus speed to high
    // (8MHz on APM2)
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);

    _spi_sem->give();

    return true;
}

#if MPU9250_DEBUG
// dump all config registers - used for debug
void AP_InertialSensor_MPU9250::_dump_registers(void)
{
    hal.console->println_P(PSTR("MPU9250 registers"));
    for (uint8_t reg=0; reg<=126; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
        if ((reg - (MPUREG_PRODUCT_ID-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();
}
#endif


#endif // CONFIG_HAL_BOARD
