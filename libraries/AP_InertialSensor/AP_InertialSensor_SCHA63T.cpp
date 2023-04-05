/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * sensor information url
 * <https://www.murata.com/ja-jp/products/sensor/gyro/overview/lineup/scha600>.
 */

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_SCHA63T.h"
#include <GCS_MAVLink/GCS.h>

#if defined(HAL_GPIO_PIN_SCHA63T_RESET)
#include <hal.h>
#endif

#define BACKEND_SAMPLE_RATE       1000
#define BACKEND_SAMPLE_RATE_MAX   4000

extern const AP_HAL::HAL& hal;

#define CONSTANTS_ONE_G             (9.80665f)						// m/s^2
#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))

#define SCHA63T_UNO 0
#define SCHA63T_DUE 1

AP_InertialSensor_SCHA63T::AP_InertialSensor_SCHA63T(AP_InertialSensor &imu,
        AP_HAL::OwnPtr<AP_HAL::Device> _dev_accel,
        AP_HAL::OwnPtr<AP_HAL::Device> _dev_gyro,
        enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev_accel(std::move(_dev_accel))
    , dev_gyro(std::move(_dev_gyro))
    , rotation(_rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_SCHA63T::probe(AP_InertialSensor &imu,
                                 AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                 AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                 enum Rotation rotation)
{
    if (!dev_accel || !dev_gyro) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_SCHA63T(imu, std::move(dev_accel), std::move(dev_gyro), rotation);

    if (!sensor) {
        return nullptr;
    }

#if defined(HAL_GPIO_PIN_SCHA63T_RESET)
    palSetLine(HAL_GPIO_PIN_SCHA63T_RESET);
#endif

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_SCHA63T::start()
{
    if (!_imu.register_accel(accel_instance, BACKEND_SAMPLE_RATE, dev_accel->get_bus_id_devtype(DEVTYPE_INS_SCHA63T)) ||
        !_imu.register_gyro(gyro_instance, BACKEND_SAMPLE_RATE, dev_gyro->get_bus_id_devtype(DEVTYPE_INS_SCHA63T))) {
        return;
    }

    // set backend rate
    backend_rate_hz = BACKEND_SAMPLE_RATE;
    if (enable_fast_sampling(accel_instance) && get_fast_sampling_rate() > 1) {
        fast_sampling = dev_accel->bus_type() == AP_HAL::Device::BUS_TYPE_SPI;
        if (fast_sampling) {
            // constrain the gyro rate to be a 2^N multiple
            uint8_t fast_sampling_rate = constrain_int16(get_fast_sampling_rate(), 1, 4);
            // calculate rate we will be giving samples to the backend
            backend_rate_hz = constrain_int16(backend_rate_hz * fast_sampling_rate, backend_rate_hz, BACKEND_SAMPLE_RATE_MAX);
        }
    }
    backend_period_us = 1000000UL / backend_rate_hz;

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    // setup callbacks
    dev_accel->register_periodic_callback(backend_period_us, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SCHA63T::read_accel, void));
    dev_gyro->register_periodic_callback(backend_period_us, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SCHA63T::read_gyro, void));
}

/*
  probe and initialise accelerometer
 */
bool AP_InertialSensor_SCHA63T::init()
{
    WITH_SEMAPHORE(dev_accel->get_semaphore());
    WITH_SEMAPHORE(dev_gyro->get_semaphore());

    error_scha63t = false;

    // wait 25ms for non-volatile memory (NVM) read
    hal.scheduler->delay(25);

    // set DUE operation mode on (must be less than 1ms)
    error_scha63t |= RegisterWrite(SCHA63T_DUE, MODE, MODE_NORM);
    error_scha63t |= RegisterWrite(SCHA63T_DUE, MODE, MODE_NORM);
    // set UNO operation mode on
    error_scha63t |= RegisterWrite(SCHA63T_UNO, MODE, MODE_NORM);
    // wait 70ms initial startup
    hal.scheduler->delay(70);

    // set UNO configuration (data filter, flag filter)
    error_scha63t |= RegisterWrite(SCHA63T_UNO, G_FILT_DYN, G_FILT);
    error_scha63t |= RegisterWrite(SCHA63T_UNO, A_FILT_DYN, A_FILT);

    // reset DUE write (0001h) to register 18h
    error_scha63t |= RegisterWrite(SCHA63T_DUE, RESCTRL, HW_RES);
    // wait 25ms for non-volatile memory (NVM) read
    hal.scheduler->delay(25);

    // set DUE operation mode on (must be less than 1ms)
    error_scha63t |= RegisterWrite(SCHA63T_DUE, MODE, MODE_NORM);
    error_scha63t |= RegisterWrite(SCHA63T_DUE, MODE, MODE_NORM);
    // wait 1ms (50ms has already passed)
    hal.scheduler->delay(1);

    // set DUE configuration (data filter, flag filter)
    error_scha63t |= RegisterWrite(SCHA63T_DUE, G_FILT_DYN, G_FILT);

    // startup clear (startup_attempt = 0)
    if (!check_startup()) {
        // system in FAILURE mode (startup_attempt not equl 0 startup_attempt = 1)
        // reset UNO write (0001h) to register 18h
        error_scha63t |= RegisterWrite(SCHA63T_UNO, RESCTRL, HW_RES);
        // reset DUE write (0001h) to register 18h
        error_scha63t |= RegisterWrite(SCHA63T_DUE, RESCTRL, HW_RES);
        // wait 25ms for non-volatile memory (NVM) read
        hal.scheduler->delay(25);

        // set DUE operation mode on (must be less than 1ms)
        error_scha63t |= RegisterWrite(SCHA63T_DUE, MODE, MODE_NORM);
        error_scha63t |= RegisterWrite(SCHA63T_DUE, MODE, MODE_NORM);
        // set UNO operation mode on
        error_scha63t |= RegisterWrite(SCHA63T_UNO, MODE, MODE_NORM);
        // wait 70ms initial startup
        hal.scheduler->delay(50);

        // set UNO configuration (data filter, flag filter)
        error_scha63t |= RegisterWrite(SCHA63T_UNO, G_FILT_DYN, G_FILT);
        error_scha63t |= RegisterWrite(SCHA63T_UNO, A_FILT_DYN, A_FILT);
        // set DUE configuration (data filter, flag filter)
        error_scha63t |= RegisterWrite(SCHA63T_DUE, G_FILT_DYN, G_FILT);

        // wait 45ms (adjust restart duration to 500ms)
        hal.scheduler->delay(45);

        if (!check_startup()) {
            return false;    // check FAILED
        }
    }

    // check ok
    return true;
}

bool AP_InertialSensor_SCHA63T::check_startup()
{
    uint8_t val[4];
    bool read_summary_error = false;

    // wait 405ms (300Hz filter)
    hal.scheduler->delay(405);

    // start EOI = 1
    error_scha63t |= RegisterWrite(SCHA63T_UNO, RESCTRL, RES_EOI);
    error_scha63t |= RegisterWrite(SCHA63T_DUE, RESCTRL, RES_EOI);

    // first read summary status
    error_scha63t |= RegisterRead(SCHA63T_UNO, S_SUM, val);
    error_scha63t |= RegisterRead(SCHA63T_DUE, S_SUM, val);
    // 2.5ms or more
    hal.scheduler->delay(3);

    // second read summary status
    error_scha63t |= RegisterRead(SCHA63T_UNO, S_SUM, val);
    error_scha63t |= RegisterRead(SCHA63T_DUE, S_SUM, val);
    // 2.5ms or more
    hal.scheduler->delay(3);

    // read summary status
    error_scha63t |= RegisterRead(SCHA63T_UNO, S_SUM, val);
    // check UNO summary status
    if (!((val[1] & 0x9e) && (val[2] & 0xda))) {
        read_summary_error = true;
    }
    error_scha63t |= RegisterRead(SCHA63T_DUE, S_SUM, val);
    // check DUE summary status
    if (!((val[1] & 0xf8) && (val[2] & 0x03))) {
        read_summary_error = true;
    }

    // check error
    if (read_summary_error) {
        return false;
    }

    return true;
}

/*
  read accel fifo
 */
void AP_InertialSensor_SCHA63T::read_accel(void)
{
    uint8_t rsp_accl_x[3];
    uint8_t rsp_accl_y[3];
    uint8_t rsp_accl_z[3];
    uint8_t rsp_temper[3];

    int16_t accel_x = 0;
    int16_t accel_y = 0;
    int16_t accel_z = 0;
    int16_t uno_temp = 0;

    // ACCL_X Cmd Send (This Response rsp_accl_x is Dust!!)
    error_scha63t |= RegisterRead(SCHA63T_UNO, ACC_X, rsp_accl_x);
    // ACCL_Y Cmd Send + ACCL_X Response Receive
    error_scha63t |= RegisterRead(SCHA63T_UNO, ACC_Y, rsp_accl_x);
    // ACCL_Z Cmd Send + ACCL_Y Response Receive
    error_scha63t |= RegisterRead(SCHA63T_UNO, ACC_Z, rsp_accl_y);
    // TEMPER Cmd Send + RATE_X Response Receive
    error_scha63t |= RegisterRead(SCHA63T_UNO, TEMP, rsp_accl_z);
    // TEMPER Cmd Send + TEMPRE Response Receive
    error_scha63t |= RegisterRead(SCHA63T_UNO, TEMP, rsp_temper);

    // response data address check
    if (((rsp_accl_x[0] & 0x7C) >> 2) == ACC_X) {
        accel_x = combine(rsp_accl_x[1], rsp_accl_x[2]);
    } else {
        error_scha63t = 1;
    }
    if (((rsp_accl_y[0] & 0x7C) >> 2) == ACC_Y) {
        accel_y = combine(rsp_accl_y[1], rsp_accl_y[2]);
    } else {
        error_scha63t = 1;
    }
    if (((rsp_accl_z[0] & 0x7C) >> 2) == ACC_Z) {
        accel_z = combine(rsp_accl_z[1], rsp_accl_z[2]);
    } else {
        error_scha63t = 1;
    }
    if (((rsp_temper[0] & 0x7C) >> 2) == TEMP) {
        uno_temp = combine(rsp_temper[1], rsp_temper[2]);
    } else {
        error_scha63t = 1;
    }
    set_temperature(accel_instance, uno_temp);

    // change coordinate system from left hand too right hand
    accel_z = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

    Vector3f accel(accel_x, accel_y, accel_z);
    accel *= (CONSTANTS_ONE_G / 4905.f); // 4905 LSB/g, 0.204mg/LSB

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    AP_HAL::Device::checkreg reg;
    if (!dev_accel->check_next_register(reg)) {
        log_register_change(dev_accel->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
    }
}

/*
  read gyro fifo
 */
void AP_InertialSensor_SCHA63T::read_gyro(void)
{
    uint8_t rsp_rate_x[3];
    uint8_t rsp_rate_y[3];
    uint8_t rsp_rate_z[3];
    uint8_t rsp_uno_temper[3];
    uint8_t rsp_due_temper[3];

    int16_t gyro_x = 0;
    int16_t gyro_y = 0;
    int16_t gyro_z = 0;
    int16_t uno_temp = 0;
    int16_t due_temp = 0;

    // RATE_Y Cmd Send (This Response rsp_rate_y is Dust!!)
    error_scha63t |= RegisterRead(SCHA63T_DUE, RATE_Y, rsp_rate_y);
    // RATE_Z Cmd Send + RATE_Y Response Receive
    error_scha63t |= RegisterRead(SCHA63T_DUE, RATE_XZ, rsp_rate_y);
    // TEMPER Cmd Send + RATE_Z Response Receive
    error_scha63t |= RegisterRead(SCHA63T_DUE, TEMP, rsp_rate_z);
    // TEMPER Cmd Send + TEMPRE Response Receive
    error_scha63t |= RegisterRead(SCHA63T_DUE, TEMP, rsp_due_temper);
    // RATE_X Cmd Send + ACCL_Z Response Receive
    error_scha63t |= RegisterRead(SCHA63T_UNO, RATE_XZ, rsp_rate_x);
    // TEMPER Cmd Send + TEMPRE Response Receive
    error_scha63t |= RegisterRead(SCHA63T_UNO, TEMP, rsp_rate_x);
    // TEMPER Cmd Send + TEMPRE Response Receive
    error_scha63t |= RegisterRead(SCHA63T_UNO, TEMP, rsp_uno_temper);

    // response data address check
    if (((rsp_rate_x[0] & 0x7C) >> 2) == RATE_XZ) {
        gyro_x = combine(rsp_rate_x[1], rsp_rate_x[2]);
    } else {
        error_scha63t = 1;
    }
    if (((rsp_rate_y[0] & 0x7C) >> 2) == RATE_Y) {
        gyro_y = combine(rsp_rate_y[1], rsp_rate_y[2]);
    } else {
        error_scha63t = 1;
    }
    if (((rsp_rate_z[0] & 0x7C) >> 2) == RATE_XZ) {
        gyro_z = combine(rsp_rate_z[1], rsp_rate_z[2]);
    } else {
        error_scha63t = 1;
    }
    if (((rsp_uno_temper[0] & 0x7C) >> 2) == TEMP) {
        uno_temp = combine(rsp_uno_temper[1], rsp_uno_temper[2]);
    } else {
        error_scha63t = 1;
    }
    if (((rsp_due_temper[0] & 0x7C) >> 2) == TEMP) {
        due_temp = combine(rsp_due_temper[1], rsp_due_temper[2]);
    } else {
        error_scha63t = 1;
    }
    set_temperature(gyro_instance, (uno_temp + due_temp) / 2);

    // change coordinate system from left hand too right hand
    gyro_z = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

    Vector3f gyro(gyro_x, gyro_y, gyro_z);
    gyro *= radians(1.f / 80.f);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    AP_HAL::Device::checkreg reg;
    if (!dev_gyro->check_next_register(reg)) {
        log_register_change(dev_gyro->get_bus_id(), reg);
        _inc_gyro_error_count(gyro_instance);
    }
}

void AP_InertialSensor_SCHA63T::set_temperature(uint8_t instance, uint16_t temper)
{
    float temperature = 25.0f + ( temper / 30 );
    float temp_degc = (0.5f * temperature) + 23.0f;
    _publish_temperature(instance, temp_degc);
}

bool AP_InertialSensor_SCHA63T::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

bool AP_InertialSensor_SCHA63T::RegisterRead(int uno_due, reg_scha63t reg_addr, uint8_t* val)
{
    bool ret = false;
    uint8_t cmd[4];
    uint8_t bCrc;

    cmd[1] = cmd[2] = 0;
    cmd[0] = reg_addr << 2;
    cmd[0] &= 0x7f;
    cmd[3] = crc8_sae(cmd, 3);

    switch ( uno_due ) {
    case SCHA63T_UNO:
        ret = dev_accel->transfer_fullduplex(cmd, val, 4);
        break;
    case SCHA63T_DUE:
        ret = dev_gyro->transfer_fullduplex(cmd, val, 4);
        break;
    default:
        break;
    }

    if (ret == true) {
        bCrc = crc8_sae(val, 3);
        if ( bCrc != val[3] ) {
            ret = false;
        }
    }

    return ret;
}

bool AP_InertialSensor_SCHA63T::RegisterWrite(int uno_due, reg_scha63t reg_addr, uint16_t val)
{
    bool ret = false;
    uint8_t res[4];
    uint8_t cmd[4];

    cmd[0] = reg_addr << 2;
    cmd[0] |= 0x80;
    cmd[1] = (val >> 8);
    cmd[2] = val;
    cmd[3] = crc8_sae(cmd, 3);

    switch ( uno_due ) {
    case SCHA63T_UNO:
        ret = dev_accel->transfer_fullduplex(cmd, res, 4);
        break;
    case SCHA63T_DUE:
        ret = dev_gyro->transfer_fullduplex(cmd, res, 4);
        break;
    default:
        break;
    }

    return ret;
}
