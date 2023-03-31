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

#if defined(HAL_GPIO_PIN_RESET)
#include <hal.h>
#endif

#define ACCEL_BACKEND_SAMPLE_RATE   1000
#define GYRO_BACKEND_SAMPLE_RATE    1000

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

#if defined(HAL_GPIO_PIN_RESET)
    palSetLine(HAL_GPIO_PIN_RESET);
#endif

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_SCHA63T::start()
{
    if (!_imu.register_accel(accel_instance, ACCEL_BACKEND_SAMPLE_RATE, dev_accel->get_bus_id_devtype(DEVTYPE_INS_SCHA63T)) ||
        !_imu.register_gyro(gyro_instance, GYRO_BACKEND_SAMPLE_RATE,   dev_gyro->get_bus_id_devtype(DEVTYPE_INS_SCHA63T))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    // setup callbacks
    dev_accel->register_periodic_callback(1000000UL / ACCEL_BACKEND_SAMPLE_RATE,
                                          FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SCHA63T::read_fifo_accel, void));
    dev_gyro->register_periodic_callback(1000000UL / GYRO_BACKEND_SAMPLE_RATE,
                                         FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SCHA63T::read_fifo_gyro, void));
}

/*
  probe and initialise accelerometer
 */
bool AP_InertialSensor_SCHA63T::init()
{
    WITH_SEMAPHORE(dev_accel->get_semaphore());
    WITH_SEMAPHORE(dev_gyro->get_semaphore());

#if 0
    uint8_t v[4];

    hal.scheduler->delay(25);

    RegisterWrite(0, SEL_BANK, 0);
    RegisterWrite(0, RESCTRL, 0);
    hal.scheduler->delay(1);
    RegisterWrite(1, SEL_BANK, 0);
    RegisterWrite(1, RESCTRL, 0);

    hal.scheduler->delay(50);

    RegisterWrite(0, MODE, 0);
    RegisterWrite(0, G_FILT_DYN, 0);
    RegisterWrite(0, A_FILT_DYN, 0);
    hal.scheduler->delay(1);
    RegisterWrite(1, MODE, 0);
    RegisterWrite(1, MODE, 0);
    RegisterWrite(1, G_FILT_DYN, 0);

    hal.scheduler->delay(500);

    RegisterWrite(0, SET_EOI, 0);
    RegisterWrite(1, SET_EOI, 0);

    hal.scheduler->delay(50);

    RegisterRead(0, S_SUM, v);
    RegisterRead(1, S_SUM, v);
    hal.scheduler->delay(50);
    RegisterRead(0, S_SUM, v);
    RegisterRead(1, S_SUM, v);
    hal.scheduler->delay(50);
    RegisterRead(0, S_SUM, v);
    RegisterRead(1, S_SUM, v);
    hal.scheduler->delay(50);
    int ret_uno = RegisterRead(0, S_SUM, v);
    int ret_due = RegisterRead(1, S_SUM, v);

    return ret_uno && ret_due;

#else
    uint8_t val[4];

    // wait 25ms for non-volatile memory (NVM) read
    hal.scheduler->delay(25);

    // set DUE operation mode on (must be less than 1ms)
    RegisterWrite(SCHA63T_DUE, MODE);
    RegisterWrite(SCHA63T_DUE, MODE);

    // set UNO operation mode on
    RegisterWrite(SCHA63T_UNO, MODE);

    // wait 70ms initial startup
    hal.scheduler->delay(70);

    // set UNO configuration (data filter, flag filter)
    RegisterWrite(SCHA63T_UNO, G_FILT_DYN);
    RegisterWrite(SCHA63T_UNO, A_FILT_DYN);

    // reset DUE write (0001h) to register 18h
    RegisterWrite(SCHA63T_DUE, RESCTRL);

    // wait 25ms for non-volatile memory (NVM) read
    hal.scheduler->delay(25);

    // set DUE operation mode on (must be less than 1ms)
    RegisterWrite(SCHA63T_DUE, MODE);
    RegisterWrite(SCHA63T_DUE, MODE);

    // wait 1ms (50ms has already passed)
    hal.scheduler->delay(1);

    // set DUE configuration (data filter, flag filter)
    RegisterWrite(SCHA63T_DUE, G_FILT_DYN);

    // wait 405ms (300Hz filter)
    hal.scheduler->delay(405);

    // start EOI = 1
    RegisterWrite(SCHA63T_UNO, SET_EOI);
    RegisterWrite(SCHA63T_DUE, SET_EOI);

    // first read summary status
    RegisterRead(SCHA63T_UNO, S_SUM, val);
    RegisterRead(SCHA63T_DUE, S_SUM, val);
    // 2.5ms or more
    hal.scheduler->delay(3);

    // second read summary status
    RegisterRead(SCHA63T_UNO, S_SUM, val);
    RegisterRead(SCHA63T_DUE, S_SUM, val);
    // 2.5ms or more
    hal.scheduler->delay(3);

    // read summary status
    int ret_uno = RegisterRead(SCHA63T_UNO, S_SUM, val);
    int ret_due = RegisterRead(SCHA63T_DUE, S_SUM, val);

    return ret_uno && ret_due;
#endif
}

/*
  read accel fifo
 */
void AP_InertialSensor_SCHA63T::read_fifo_accel(void)
{
#if 0
    static const uint8_t cmd_accl_x[4] = { 0x10, 0x00, 0x00, 0xE9 }; // ACCL_X
    static const uint8_t cmd_accl_y[4] = { 0x14, 0x00, 0x00, 0xEF }; // ACCL_Y
    static const uint8_t cmd_accl_z[4] = { 0x18, 0x00, 0x00, 0xE5 }; // ACCL_Z
    static const uint8_t cmd_rate_x[4] = { 0x04, 0x00, 0x00, 0xF7 }; // RATE_X
    static const uint8_t cmd_temper[4] = { 0x1C, 0x00, 0x00, 0xE3 }; // TEMPER

    uint8_t rsp_accl_x[4];
    uint8_t rsp_accl_y[4];
    uint8_t rsp_accl_z[4];
    uint8_t rsp_rate_x[4];
    uint8_t rsp_temper[4];

    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    // #### ACCL_X Cmd Send (This Response rsp_accl_x is Dust!!)
    dev_accel->transfer_fullduplex(cmd_accl_x, rsp_accl_x, 4);

    // #### ACCL_Y Cmd Send + ACCL_X Response Receive
    dev_accel->transfer_fullduplex(cmd_accl_y, rsp_accl_x, 4);

    // #### ACCL_Z Cmd Send + ACCL_Y Response Receive
    dev_accel->transfer_fullduplex(cmd_accl_z, rsp_accl_y, 4);

    // ##### RATE_X Cmd Send + ACCL_Z Response Receive
    dev_accel->transfer_fullduplex(cmd_rate_x, rsp_accl_z, 4);

    // ##### TEMPER Cmd Send + RATE_X Response Receive
    dev_accel->transfer_fullduplex(cmd_temper, rsp_rate_x, 4);

    // ##### TEMPER Cmd Send + TEMPRE Response Receive
    dev_accel->transfer_fullduplex(cmd_temper, rsp_temper, 4);

    accel_x = combine(rsp_accl_x[1], rsp_accl_x[2]);
    accel_y = combine(rsp_accl_y[1], rsp_accl_y[2]);
    accel_z = combine(rsp_accl_z[1], rsp_accl_z[2]);

    accel_z = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

    Vector3f accel(accel_x, accel_y, accel_z);
    accel *= (CONSTANTS_ONE_G / 4905.f); // 4905 LSB/g, 0.204mg/LSB

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    set_temperature(accel_instance, rsp_temper[1], rsp_temper[2]);

    AP_HAL::Device::checkreg reg;
    if (!dev_accel->check_next_register(reg)) {
        log_register_change(dev_accel->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
    }

    gyro_x = combine(rsp_rate_x[1], rsp_rate_x[2]);

#else
    static const uint8_t cmd_accl_x[4] = { 0x10, 0x00, 0x00, 0xE9 }; // ACCL_X
    static const uint8_t cmd_accl_y[4] = { 0x14, 0x00, 0x00, 0xEF }; // ACCL_Y
    static const uint8_t cmd_accl_z[4] = { 0x18, 0x00, 0x00, 0xE5 }; // ACCL_Z
    static const uint8_t cmd_rate_x[4] = { 0x04, 0x00, 0x00, 0xF7 }; // RATE_X
    static const uint8_t cmd_temper[4] = { 0x1C, 0x00, 0x00, 0xE3 }; // TEMPER

    uint8_t rsp_accl_x[4];
    uint8_t rsp_accl_y[4];
    uint8_t rsp_accl_z[4];
    uint8_t rsp_rate_x[4];
    uint8_t rsp_temper[4];

    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    uint8_t buf[4];

    // #### ACCL_X Cmd Send (This Response rsp_accl_x is Dust!!)
//    dev_accel->transfer_fullduplex(cmd_accl_x, rsp_accl_x, 4);
    memcpy(buf, cmd_accl_x, 4);
    dev_accel->transfer(buf, 4, buf, 4);
    memcpy(rsp_accl_x, buf, 4);

    // #### ACCL_Y Cmd Send + ACCL_X Response Receive
//    dev_accel->transfer_fullduplex(cmd_accl_y, rsp_accl_x, 4);
    memcpy(buf, cmd_accl_y, 4);
    dev_accel->transfer(buf, 4, buf, 4);
    memcpy(rsp_accl_x, buf, 4);

    // #### ACCL_Z Cmd Send + ACCL_Y Response Receive
//    dev_accel->transfer_fullduplex(cmd_accl_z, rsp_accl_y, 4);
    memcpy(buf, cmd_accl_z, 4);
    dev_accel->transfer(buf, 4, buf, 4);
    memcpy(rsp_accl_y, buf, 4);

    // ##### RATE_X Cmd Send + ACCL_Z Response Receive
//    dev_accel->transfer_fullduplex(cmd_rate_x, rsp_accl_z, 4);
    memcpy(buf, cmd_rate_x, 4);
    dev_accel->transfer(buf, 4, buf, 4);
    memcpy(rsp_accl_z, buf, 4);

    // ##### TEMPER Cmd Send + RATE_X Response Receive
//    dev_accel->transfer_fullduplex(cmd_temper, rsp_rate_x, 4);
    memcpy(buf, cmd_temper, 4);
    dev_accel->transfer(buf, 4, buf, 4);
    memcpy(rsp_rate_x, buf, 4);

    // ##### TEMPER Cmd Send + TEMPRE Response Receive
//    dev_accel->transfer_fullduplex(cmd_temper, rsp_temper, 4);
    memcpy(buf, cmd_temper, 4);
    dev_accel->transfer(buf, 4, buf, 4);
    memcpy(rsp_temper, buf, 4);


//GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MP : %x,%x,%x,%x", rsp_accl_x[0], rsp_accl_x[1], rsp_accl_x[2], rsp_accl_x[3]);


    accel_x = combine(rsp_accl_x[1], rsp_accl_x[2]);
    accel_y = combine(rsp_accl_y[1], rsp_accl_y[2]);
    accel_z = combine(rsp_accl_z[1], rsp_accl_z[2]);

    accel_z = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

    Vector3f accel(accel_x, accel_y, accel_z);
    accel *= (CONSTANTS_ONE_G / 4905.f); // 4905 LSB/g, 0.204mg/LSB

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    set_temperature(accel_instance, rsp_temper[1], rsp_temper[2]);

    AP_HAL::Device::checkreg reg;
    if (!dev_accel->check_next_register(reg)) {
        log_register_change(dev_accel->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
    }

    gyro_x = combine(rsp_rate_x[1], rsp_rate_x[2]);
#endif
}

/*
  read gyro fifo
 */
void AP_InertialSensor_SCHA63T::read_fifo_gyro(void)
{
#if 0
    static const uint8_t cmd_rate_y[4] = { 0x0C, 0x00, 0x00, 0xFB }; // RATE_Y
    static const uint8_t cmd_rate_z[4] = { 0x04, 0x00, 0x00, 0xF7 }; // RATE_Z
    static const uint8_t cmd_temper[4] = { 0x1C, 0x00, 0x00, 0xE3 }; // TEMPER

    uint8_t rsp_rate_y[4];
    uint8_t rsp_rate_z[4];
    uint8_t rsp_temper[4];

    int16_t gyro_y;
    int16_t gyro_z;

    // ####### RATE_Y Cmd Send (This Response rsp_rate_y is Dust!!)
    dev_gyro->transfer_fullduplex(cmd_rate_y, rsp_rate_y, 4);

    // ####### RATE_Z Cmd Send + RATE_Y Response Receive
    dev_gyro->transfer_fullduplex(cmd_rate_z, rsp_rate_y, 4);

    // ####### TEMPER Cmd Send + RATE_Z Response Receive
    dev_gyro->transfer_fullduplex(cmd_temper, rsp_rate_z, 4);

    // ##### TEMPER Cmd Send + TEMPRE Response Receive
    dev_gyro->transfer_fullduplex(cmd_temper, rsp_temper, 4);

    gyro_y = combine(rsp_rate_y[1], rsp_rate_y[2]);
    gyro_z = combine(rsp_rate_z[1], rsp_rate_z[2]);

    gyro_z = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

    Vector3f gyro(gyro_x, gyro_y, gyro_z);
    gyro *= radians(1.f / 80.f);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    set_temperature(gyro_instance, rsp_temper[1], rsp_temper[2]);

    AP_HAL::Device::checkreg reg;
    if (!dev_gyro->check_next_register(reg)) {
        log_register_change(dev_gyro->get_bus_id(), reg);
        _inc_gyro_error_count(gyro_instance);
    }

#else
    static const uint8_t cmd_rate_y[4] = { 0x0C, 0x00, 0x00, 0xFB }; // RATE_Y
    static const uint8_t cmd_rate_z[4] = { 0x04, 0x00, 0x00, 0xF7 }; // RATE_Z
    static const uint8_t cmd_temper[4] = { 0x1C, 0x00, 0x00, 0xE3 }; // TEMPER

    uint8_t rsp_rate_y[4];
    uint8_t rsp_rate_z[4];
    uint8_t rsp_temper[4];

    int16_t gyro_y;
    int16_t gyro_z;

    uint8_t buf[4];

    // ####### RATE_Y Cmd Send (This Response rsp_rate_y is Dust!!)
//    dev_gyro->transfer_fullduplex(cmd_rate_y, rsp_rate_y, 4);
    memcpy(buf, cmd_rate_y, 4);
    dev_accel->transfer(buf, 4, buf, 4);
    memcpy(rsp_rate_y, buf, 4);

    // ####### RATE_Z Cmd Send + RATE_Y Response Receive
//    dev_gyro->transfer_fullduplex(cmd_rate_z, rsp_rate_y, 4);
    memcpy(buf, cmd_rate_z, 4);
    dev_accel->transfer(buf, 4, buf, 4);
    memcpy(rsp_rate_y, buf, 4);

    // ####### TEMPER Cmd Send + RATE_Z Response Receive
//    dev_gyro->transfer_fullduplex(cmd_temper, rsp_rate_z, 4);
    memcpy(buf, cmd_temper, 4);
    dev_accel->transfer(buf, 4, buf, 4);
    memcpy(rsp_rate_z, buf, 4);

    // ##### TEMPER Cmd Send + TEMPRE Response Receive
//    dev_gyro->transfer_fullduplex(cmd_temper, rsp_temper, 4);
    memcpy(buf, cmd_temper, 4);
    dev_accel->transfer(buf, 4, buf, 4);
    memcpy(rsp_temper, buf, 4);

    gyro_y = combine(rsp_rate_y[1], rsp_rate_y[2]);
    gyro_z = combine(rsp_rate_z[1], rsp_rate_z[2]);

    gyro_z = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

    Vector3f gyro(gyro_x, gyro_y, gyro_z);
    gyro *= radians(1.f / 80.f);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    set_temperature(gyro_instance, rsp_temper[1], rsp_temper[2]);

    AP_HAL::Device::checkreg reg;
    if (!dev_gyro->check_next_register(reg)) {
        log_register_change(dev_gyro->get_bus_id(), reg);
        _inc_gyro_error_count(gyro_instance);
    }
#endif
}

void AP_InertialSensor_SCHA63T::set_temperature(uint8_t instance, uint8_t temper1, uint8_t temper2)
{
    int16_t temper  = combine(temper1, temper2);
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

bool AP_InertialSensor_SCHA63T::RegisterRead(int tp, reg_scha63t reg, uint8_t* val)
{
    bool ret = false;
#if 1
    uint8_t buf[4];
#endif    
    const uint8_t *cmd;
    static const uint8_t acc0E[4] = { 0x38, 0x00, 0x00, 0xD5 };
    static const uint8_t acc10[4] = { 0x40, 0x00, 0x00, 0x91 };
    static const uint8_t acc12[4] = { 0x48, 0x00, 0x00, 0x9D };
    static const uint8_t acc14[4] = { 0x50, 0x00, 0x00, 0x89 };
    static const uint8_t acc15[4] = { 0x54, 0x00, 0x00, 0x8F };

    cmd = acc15;
    switch ( reg ) {
    case S_SUM:  /* 0x0E */
        cmd = acc0E;
        break;
    case R_S1:   /* 0x10 */
        cmd = acc10;
        break;
    case A_S1:   /* 0x12 */
        cmd = acc12;
        break;
    case C_S1:   /* 0x14 */
        cmd = acc14;
        break;
    case C_S2:   /* 0x15 */
        cmd = acc15;
        break;
    default:
        break;
    }

    //transfer(cmd, rrsp, 4);
    switch ( tp ) {
    case SCHA63T_UNO:
#if 0
        ret = dev_accel->transfer_fullduplex(cmd, val, 4);
#else
        memcpy(buf, cmd, 4);
        dev_accel->transfer(buf, 4, buf, 4);
        memcpy(val, buf, 4);
#endif
        break;
    case SCHA63T_DUE:
#if 0
        ret = dev_gyro->transfer_fullduplex(cmd, val, 4);
#else
        memcpy(buf, cmd, 4);
        dev_accel->transfer(buf, 4, buf, 4);
        memcpy(val, buf, 4);
#endif
        break;
    default:
        break;
    }

    if (ret == true) {
        uint8_t bCrc = crc8_sae(val, 3);
        if ( bCrc != val[3] ) {
            ret = false;
        }
    }

    return ret;
}

bool AP_InertialSensor_SCHA63T::RegisterWrite(int tp, reg_scha63t reg, uint8_t val)
{
    bool ret = false;
    uint8_t res[4];
#if 1
    uint8_t buf[4];
#endif
    const uint8_t *cmd;
    static const uint8_t acc16[4] = { 0xD8, 0x24, 0x24, 0xEE };
    static const uint8_t acc18_res[4] = { 0xE0, 0x00, 0x01, 0x7C };
    static const uint8_t acc18_eoi[4] = { 0xE0, 0x00, 0x02, 0x5B };
    static const uint8_t acc19[4] = { 0xE4, 0x00, 0x00, 0x67 };
    static const uint8_t acc1A[4] = { 0xE8, 0x04, 0x44, 0x27 };
    static const uint8_t acc1F[4] = { 0xFC, 0x00, 0x00, 0x73 };

    static const uint8_t set_d[4] = { 0x74, 0x00, 0x02, 0xB8 };
    static const uint8_t set_e[4] = { 0x78, 0x00, 0x02, 0x1E };
    static const uint8_t set_c[4] = { 0x70, 0x00, 0x02, 0x1C };

    cmd = acc16;
    switch ( reg ) {
    case G_FILT_DYN: /* 0x16 */
        cmd = acc16;
        break;
    case RESCTRL: /* 0x18 */
        cmd = acc18_res;
        break;
    case SET_EOI: /* 0x18 */
        cmd = acc18_eoi;
        break;
    case MODE: /* 0x19 */
        cmd = acc19;
        break;
    case A_FILT_DYN: /* 0x1A */
        cmd = acc1A;
        break;
    case SEL_BANK: /* 0x1F */
        cmd = acc1F;
        break;
    case X_1D: /* 0x1d */
        cmd = set_d;
        break;
    case X_1E: /* 0x1e */
        cmd = set_e;
        break;
    case X_1C: /* 0x1c */
        cmd = set_c;
        break;
    default:
        break;
    }

    switch ( tp ) {
    case SCHA63T_UNO:
#if 0
        ret = dev_accel->transfer_fullduplex(cmd, res, 4);
#else
        memcpy(buf, cmd, 4);
        dev_accel->transfer(buf, 4, buf, 4);
        memcpy(res, buf, 4);
#endif
        break;
    case SCHA63T_DUE:
#if 0
        ret = dev_gyro->transfer_fullduplex(cmd, res, 4);
#else
        memcpy(buf, cmd, 4);
        dev_accel->transfer(buf, 4, buf, 4);
        memcpy(res, buf, 4);
#endif
        break;
    default:
        break;
    }

    return ret;
}
