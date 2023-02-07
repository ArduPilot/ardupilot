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
  IMU driver for Robsense PhenixPro Devkit board including i3g4250d and iis328dq
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ
#include "AP_InertialSensor_RST.h"
#include <AP_Math/AP_Math.h>

#include <inttypes.h>
#include <utility>
#include <math.h>
#include <stdio.h>

const extern AP_HAL::HAL &hal;

#define ADDR_INCREMENT                (1<<6)

/************************************iis328dq register addresses *******************************************/
#define ACCEL_WHO_AM_I             0x0F
#define ACCEL_WHO_I_AM             0x32

#define ACCEL_CTRL_REG1            0x20
/* keep lowpass low to avoid noise issues */
#define RATE_50HZ_LP_37HZ          (0<<4) | (0<<3)
#define RATE_100HZ_LP_74HZ         (0<<4) | (1<<3)
#define RATE_400HZ_LP_292HZ        (1<<4) | (0<<3)
#define RATE_1000HZ_LP_780HZ       (1<<4) | (1<<3)

#define ACCEL_CTRL_REG2            0x21
#define ACCEL_CTRL_REG3            0x22
#define ACCEL_CTRL_REG4            0x23

#define ACCEL_CTRL_REG5            0x24
#define ACCEL_HP_FILTER_RESETE     0x25
#define ACCEL_OUT_REFERENCE        0x26
#define ACCEL_STATUS_REG           0x27
#define ACCEL_OUT_X_L              0x28
#define ACCEL_OUT_X_H              0x29
#define ACCEL_OUT_Y_L              0x2A
#define ACCEL_OUT_Y_H              0x2B
#define ACCEL_OUT_Z_L              0x2C
#define ACCEL_OUT_Z_H              0x2D
#define ACCEL_INT1_CFG             0x30
#define ACCEL_INT1_SRC             0x31
#define ACCEL_INT1_TSH             0x32
#define ACCEL_INT1_DURATION        0x33
#define ACCEL_INT2_CFG             0x34
#define ACCEL_INT2_SRC             0x35
#define ACCEL_INT2_TSH             0x36
#define ACCEL_INT2_DURATION        0x37


/* Internal configuration values */
#define ACCEL_REG1_POWER_NORMAL    ((0<<7) | (0<<6) | (1<<5))
#define ACCEL_REG1_Z_ENABLE        (1<<2)
#define ACCEL_REG1_Y_ENABLE        (1<<1)
#define ACCEL_REG1_X_ENABLE        (1<<0)

#define ACCEL_REG4_BDU             (1<<7)
#define ACCEL_REG4_BLE             (1<<6)
#define ACCEL_REG4_FULL_SCALE_BITS ((1<<5) | (1<<4))
#define ACCEL_REG4_FULL_SCALE_2G   ((0<<5) | (0<<4))
#define ACCEL_REG4_FULL_SCALE_4G   ((0<<5) | (1<<4))
#define ACCEL_REG4_FULL_SCALE_8G   ((1<<5) | (1<<4))

#define ACCEL_STATUS_ZYXOR         (1<<7)
#define ACCEL_STATUS_ZOR           (1<<6)
#define ACCEL_STATUS_YOR           (1<<5)
#define ACCEL_STATUS_XOR           (1<<4)
#define ACCEL_STATUS_ZYXDA         (1<<3)
#define ACCEL_STATUS_ZDA           (1<<2)
#define ACCEL_STATUS_YDA           (1<<1)
#define ACCEL_STATUS_XDA           (1<<0)

#define ACCEL_DEFAULT_RANGE_G               8
#define ACCEL_DEFAULT_RATE                  1000
#define ACCEL_DEFAULT_ONCHIP_FILTER_FREQ    780
#define ACCEL_ONE_G                         GRAVITY_MSS

/************************************i3g4250d register addresses *******************************************/
#define GYRO_WHO_AM_I             0x0F
#define GYRO_WHO_I_AM             0xD3

#define GYRO_CTRL_REG1            0x20
/* keep lowpass low to avoid noise issues */
#define RATE_100HZ_LP_25HZ        ((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_200HZ_LP_25HZ        ((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_200HZ_LP_50HZ        ((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_200HZ_LP_70HZ        ((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define RATE_400HZ_LP_20HZ        ((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_400HZ_LP_25HZ        ((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_400HZ_LP_50HZ        ((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_400HZ_LP_100HZ       ((1<<7) | (0<<6) | (1<<5) | (1<<4))
#define RATE_800HZ_LP_30HZ        ((1<<7) | (1<<6) | (0<<5) | (0<<4))
#define RATE_800HZ_LP_35HZ        ((1<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_800HZ_LP_50HZ        ((1<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_800HZ_LP_100HZ       ((1<<7) | (1<<6) | (1<<5) | (1<<4))

#define GYRO_CTRL_REG2            0x21
#define GYRO_CTRL_REG3            0x22
#define GYRO_CTRL_REG4            0x23
#define RANGE_250DPS              (0<<4)
#define RANGE_500DPS              (1<<4)
#define RANGE_2000DPS             (3<<4)

#define GYRO_CTRL_REG5            0x24
#define GYRO_REFERENCE            0x25
#define GYRO_OUT_TEMP             0x26
#define GYRO_STATUS_REG           0x27
#define GYRO_OUT_X_L              0x28
#define GYRO_OUT_X_H              0x29
#define GYRO_OUT_Y_L              0x2A
#define GYRO_OUT_Y_H              0x2B
#define GYRO_OUT_Z_L              0x2C
#define GYRO_OUT_Z_H              0x2D
#define GYRO_FIFO_CTRL_REG        0x2E
#define GYRO_FIFO_SRC_REG         0x2F
#define GYRO_INT1_CFG             0x30
#define GYRO_INT1_SRC             0x31
#define GYRO_INT1_TSH_XH          0x32
#define GYRO_INT1_TSH_XL          0x33
#define GYRO_INT1_TSH_YH          0x34
#define GYRO_INT1_TSH_YL          0x35
#define GYRO_INT1_TSH_ZH          0x36
#define GYRO_INT1_TSH_ZL          0x37
#define GYRO_INT1_DURATION        0x38
#define GYRO_LOW_ODR              0x39


/* Internal configuration values */
#define GYRO_REG1_POWER_NORMAL    (1<<3)
#define GYRO_REG1_Z_ENABLE        (1<<2)
#define GYRO_REG1_Y_ENABLE        (1<<1)
#define GYRO_REG1_X_ENABLE        (1<<0)

#define GYRO_REG4_BLE             (1<<6)

#define GYRO_REG5_FIFO_ENABLE     (1<<6)
#define GYRO_REG5_REBOOT_MEMORY   (1<<7)

#define GYRO_STATUS_ZYXOR         (1<<7)
#define GYRO_STATUS_ZOR           (1<<6)
#define GYRO_STATUS_YOR           (1<<5)
#define GYRO_STATUS_XOR           (1<<4)
#define GYRO_STATUS_ZYXDA         (1<<3)
#define GYRO_STATUS_ZDA           (1<<2)
#define GYRO_STATUS_YDA           (1<<1)
#define GYRO_STATUS_XDA           (1<<0)

#define GYRO_FIFO_CTRL_BYPASS_MODE              (0<<5)
#define GYRO_FIFO_CTRL_FIFO_MODE                (1<<5)
#define GYRO_FIFO_CTRL_STREAM_MODE              (1<<6)
#define GYRO_FIFO_CTRL_STREAM_TO_FIFO_MODE      (3<<5)
#define GYRO_FIFO_CTRL_BYPASS_TO_STREAM_MODE    (1<<7)

//data output frequency
#define GYRO_DEFAULT_RATE               800
#define GYRO_DEFAULT_RANGE_DPS          2000
#define GYRO_DEFAULT_FILTER_FREQ        35
#define GYRO_TEMP_OFFSET_CELSIUS        40


// constructor
AP_InertialSensor_RST::AP_InertialSensor_RST(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                              enum Rotation rotation_g,
                              enum Rotation rotation_a)
    : AP_InertialSensor_Backend(imu)
    , _dev_gyro(std::move(dev_gyro))
    , _dev_accel(std::move(dev_accel))
    , _rotation_g(rotation_g)
    , _rotation_a(rotation_a)
{
}

AP_InertialSensor_RST::~AP_InertialSensor_RST()
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_RST::probe(AP_InertialSensor &imu,
                                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                              enum Rotation rotation_g,
                                              enum Rotation rotation_a)
{
    if (!dev_gyro && !dev_accel) {
        return nullptr;
    }
    AP_InertialSensor_RST *sensor
        = new AP_InertialSensor_RST(imu, std::move(dev_gyro), std::move(dev_accel), rotation_g, rotation_a);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/*
 * init gyro
 */
bool AP_InertialSensor_RST::_init_gyro(void)
{
    uint8_t whoami;

    _dev_gyro->get_semaphore()->take_blocking();

    // set flag for reading registers
    _dev_gyro->set_read_flag(0x80);

    _dev_gyro->set_speed(AP_HAL::Device::SPEED_HIGH);

    _dev_gyro->read_registers(GYRO_WHO_AM_I, &whoami, sizeof(whoami));
    if (whoami != GYRO_WHO_I_AM) {
        hal.console->printf("RST: unexpected gyro WHOAMI 0x%x\n", (unsigned)whoami);
        printf("RST: unexpected gyro WHOAMI 0x%x\n", (unsigned)whoami);
        goto fail_whoami;
    }

    printf("detect i3g4250d\n");

    //enter power-down mode first
    _dev_gyro->write_register(GYRO_CTRL_REG1, 0);

    hal.scheduler->delay(100);

    _dev_gyro->write_register(GYRO_CTRL_REG1,
                              GYRO_REG1_POWER_NORMAL |
                              GYRO_REG1_Z_ENABLE | GYRO_REG1_X_ENABLE | GYRO_REG1_Y_ENABLE |
                              RATE_800HZ_LP_50HZ);

    /* disable high-pass filters */
    _dev_gyro->write_register(GYRO_CTRL_REG2, 0);

    /* DRDY disable */
    _dev_gyro->write_register(GYRO_CTRL_REG3, 0x0);
    _dev_gyro->write_register(GYRO_CTRL_REG4, RANGE_2000DPS);

    /* disable wake-on-interrupt */
    _dev_gyro->write_register(GYRO_CTRL_REG5, GYRO_REG5_FIFO_ENABLE);

    /* disable FIFO. This makes things simpler and ensures we
     * aren't getting stale data. It means we must run the hrt
     * callback fast enough to not miss data. */
    _dev_gyro->write_register(GYRO_FIFO_CTRL_REG, GYRO_FIFO_CTRL_BYPASS_MODE);

    _gyro_scale = 70e-3f / 180.0f * M_PI;

    hal.scheduler->delay(100);

    _dev_gyro->get_semaphore()->give();

    return true;

fail_whoami:
    _dev_gyro->get_semaphore()->give();
    return false;
}

/*
 * init accel
 */
bool AP_InertialSensor_RST::_init_accel(void)
{
    uint8_t whoami;

    _dev_accel->get_semaphore()->take_blocking();

    _dev_accel->set_speed(AP_HAL::Device::SPEED_HIGH);

    _dev_accel->set_read_flag(0x80);

    _dev_accel->read_registers(ACCEL_WHO_AM_I, &whoami, sizeof(whoami));
    if (whoami != ACCEL_WHO_I_AM) {
        DEV_PRINTF("RST: unexpected accel WHOAMI 0x%x\n", (unsigned)whoami);
        printf("RST: unexpected accel WHOAMI 0x%x\n", (unsigned)whoami);
        goto fail_whoami;
    }

    _dev_accel->write_register(ACCEL_CTRL_REG1,
                               ACCEL_REG1_POWER_NORMAL |
                               ACCEL_REG1_Z_ENABLE | ACCEL_REG1_Y_ENABLE | ACCEL_REG1_X_ENABLE |
                               RATE_1000HZ_LP_780HZ);

    /* disable high-pass filters */
    _dev_accel->write_register(ACCEL_CTRL_REG2, 0);

    /* DRDY enable */
    _dev_accel->write_register(ACCEL_CTRL_REG3, 0x02);
    _dev_accel->write_register(ACCEL_CTRL_REG4,
                               ACCEL_REG4_BDU | ACCEL_REG4_FULL_SCALE_8G);

    _accel_scale = 0.244e-3f * ACCEL_ONE_G;

    _dev_accel->get_semaphore()->give();

    return true;

fail_whoami:
    _dev_accel->get_semaphore()->give();
    return false;


}

bool AP_InertialSensor_RST::_init_sensor(void)
{
    if (!_init_gyro() || !_init_accel()) {
        return false;
    }

    return true;
}

/*
  startup the sensor
 */
void AP_InertialSensor_RST::start(void)
{
    if (!_imu.register_gyro(_gyro_instance, 800, _dev_gyro->get_bus_id_devtype(DEVTYPE_GYR_I3G4250D)) ||
        !_imu.register_accel(_accel_instance, 1000, _dev_accel->get_bus_id_devtype(DEVTYPE_ACC_IIS328DQ))) {
        return;
    }

    set_gyro_orientation(_gyro_instance, _rotation_g);
    set_accel_orientation(_accel_instance, _rotation_a);

    // start the timer process to read samples
    _dev_gyro->register_periodic_callback(1150, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_RST::gyro_measure, void));
    _dev_accel->register_periodic_callback(800, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_RST::accel_measure, void));
}

/*
  copy filtered data to the frontend
 */
bool AP_InertialSensor_RST::update(void)
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);

    return true;
}

// Accumulate values from gyros
void AP_InertialSensor_RST::gyro_measure(void)
{
    Vector3f gyro;
    uint8_t status = 0;
    int16_t raw_data[3];

    _dev_gyro->read_registers(GYRO_STATUS_REG, &status, sizeof(status));

    if ((status & GYRO_STATUS_ZYXDA) == 0) {
        return;
    }

    if (_dev_gyro->read_registers(GYRO_OUT_X_L | ADDR_INCREMENT, (uint8_t *)raw_data, sizeof(raw_data))) {
        gyro = Vector3f(raw_data[0], raw_data[1], raw_data[2]);
        gyro *= _gyro_scale;
        _rotate_and_correct_gyro(_gyro_instance, gyro);
        _notify_new_gyro_raw_sample(_gyro_instance, gyro);
    }
}

// Accumulate values from accels
void AP_InertialSensor_RST::accel_measure(void)
{
    Vector3f accel;
    uint8_t status = 0;
    int16_t raw_data[3];

    _dev_accel->read_registers(ACCEL_STATUS_REG, &status, sizeof(status));

    if ((status & ACCEL_STATUS_ZYXDA) == 0) {
        return;
    }

    if (_dev_accel->read_registers(ACCEL_OUT_X_L | ADDR_INCREMENT, (uint8_t *)raw_data, sizeof(raw_data))) {
        accel = Vector3f(raw_data[0], raw_data[1], raw_data[2]);
        accel *= _accel_scale;
        _rotate_and_correct_accel(_accel_instance, accel);
        _notify_new_accel_raw_sample(_accel_instance, accel);
    }
}

#endif
