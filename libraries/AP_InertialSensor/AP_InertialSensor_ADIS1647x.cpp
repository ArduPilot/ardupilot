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
 */

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>

#include "AP_InertialSensor_ADIS1647x.h"

/*
  device registers
 */
#define REG_PROD_ID  0x72
#define  PROD_ID_16470     0x4056
#define  PROD_ID_16477     0x405d
#define  PROD_ID_16507     0x407b

#define REG_GLOB_CMD 0x68
#define  GLOB_CMD_SW_RESET 0x80

#define REG_RANG_MDL 0x5E // 16477 only

#define REG_DATA_CNTR 0x22

#define REG_MSC_CTRL 0x60
# define REG_MSC_CTRL_BURST32  0x200
# define REG_MSC_CTRL_BURSTSEL 0x100
# define REG_MSC_CTRL_GCOMP    0x080
# define REG_MSC_CTRL_PCOMP    0x040
# define REG_MSC_CTRL_SENSBW   0x010
# define REG_MSC_CTRL_DRPOL    0x001

#define REG_DEC_RATE 0x64
# define REG_DEC_RATE_2000Hz 0
# define REG_DEC_RATE_1000Hz 1
# define REG_DEC_RATE_666Hz 2
# define REG_DEC_RATE_500Hz 3
# define REG_DEC_RATE_400Hz 4

#define REG_FILT_CTRL 0x5c

/*
  timings
 */
#define T_STALL_US   20U
#define T_RESET_MS   500U

#define TIMING_DEBUG 0
#if TIMING_DEBUG
#define DEBUG_SET_PIN(n,v) hal.gpio->write(52+n, v)
#define DEBUG_TOGGLE_PIN(n) hal.gpio->toggle(52+n)
#else
#define DEBUG_SET_PIN(n,v)
#define DEBUG_TOGGLE_PIN(n)
#endif

extern const AP_HAL::HAL& hal;

AP_InertialSensor_ADIS1647x::AP_InertialSensor_ADIS1647x(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                         enum Rotation _rotation,
                                                         uint8_t drdy_gpio)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
    , drdy_pin(drdy_gpio)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_ADIS1647x::probe(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation,
                                   uint8_t drdy_gpio)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = NEW_NOTHROW AP_InertialSensor_ADIS1647x(imu, std::move(dev), rotation, drdy_gpio);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_ADIS1647x::start()
{
    if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_ADIS1647X)) ||
        !_imu.register_gyro(gyro_instance, expected_sample_rate_hz,   dev->get_bus_id_devtype(DEVTYPE_INS_ADIS1647X))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    /*
      as the sensor does not have a FIFO we need to jump through some
      hoops to ensure we don't lose any samples. This creates a thread
      to do the capture, running at very high priority
     */
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADIS1647x::loop, void),
                                      "ADIS1647x",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("Failed to create ADIS1647x thread");
    }
}

/*
  check product ID
 */
bool AP_InertialSensor_ADIS1647x::check_product_id(uint16_t &prod_id)
{
    prod_id = read_reg16(REG_PROD_ID);
    switch (prod_id) {
    case PROD_ID_16470:
        // can do up to 40G
        opmode = OpMode::Basic;
        accel_scale = 1.25 * GRAVITY_MSS * 0.001;
        _clip_limit = (40.0f - 0.5f) * GRAVITY_MSS;
        gyro_scale = radians(0.1);
        expected_sample_rate_hz = 2000;
        return true;

    case PROD_ID_16477: {
        // can do up to 40G
        opmode = OpMode::Basic;
        accel_scale = 1.25 * GRAVITY_MSS * 0.001;
        _clip_limit = (40.0f - 0.5f) * GRAVITY_MSS;
        expected_sample_rate_hz = 2000;
        // RANG_MDL register used for gyro range
        uint16_t rang_mdl = read_reg16(REG_RANG_MDL);
        switch ((rang_mdl >> 2) & 3) {
        case 0:
            gyro_scale = radians(1.0/160);
            break;
        case 1:
            gyro_scale = radians(1.0/40);
            break;
        case 3:
            gyro_scale = radians(1.0/10);
            break;
        default:
            return false;
        }
        return true;
    }
        
    case PROD_ID_16507: {
        // can do up to 40G
        opmode = OpMode::Delta32;
        expected_sample_rate_hz = 1200;
        accel_scale = 392.0 / 2097152000.0;
        dvel_scale = 400.0 / (float)0x7FFFFFFF;
        _clip_limit = (40.0f - 0.5f) * GRAVITY_MSS;
        // RANG_MDL register used for gyro range
        uint16_t rang_mdl = read_reg16(REG_RANG_MDL);
        switch ((rang_mdl >> 2) & 3) {
        case 0:
            gyro_scale = radians(125) / 0x4E200000;
            dangle_scale = radians(360.0 / (float)0x7FFFFFFF);
            break;
        case 1:
            gyro_scale = radians(500) / 0x4E200000;
            dangle_scale = radians(720.0 / (float)0x7FFFFFFF);
            break;
        case 3:
            gyro_scale = radians(2000) / 0x4E200000;
            dangle_scale = radians(2160.0 / (float)0x7FFFFFFF);
            break;
        default:
            return false;
        }
        if (opmode == OpMode::Basic) {
            accel_scale *= 0x10000;
            gyro_scale *= 0x10000;
        }
        return true;
    }

    }
    return false;
}


bool AP_InertialSensor_ADIS1647x::init()
{
    WITH_SEMAPHORE(dev->get_semaphore());


    uint8_t tries = 10;
    uint16_t prod_id = 0;
    do {
        // perform software reset
        write_reg16(REG_GLOB_CMD, GLOB_CMD_SW_RESET);
        hal.scheduler->delay(100);
    } while (!check_product_id(prod_id) && --tries);
    if (tries == 0) {
        return false;
    }

    // bring rate down
    if (expected_sample_rate_hz < 450) {
        if (!write_reg16(REG_DEC_RATE, REG_DEC_RATE_400Hz, true)) {
            return false;
        }
    } else if (expected_sample_rate_hz < 600) {
        if (!write_reg16(REG_DEC_RATE, REG_DEC_RATE_500Hz, true)) {
            return false;
        }
    } else if (expected_sample_rate_hz < 700) {
        if (!write_reg16(REG_DEC_RATE, REG_DEC_RATE_666Hz, true)) {
            return false;
        }
    } else if (expected_sample_rate_hz < 1500) {
        if (!write_reg16(REG_DEC_RATE, REG_DEC_RATE_1000Hz, true)) {
            return false;
        }
    }

    if (!write_reg16(REG_FILT_CTRL, 0, true)) {
        return false;
    }
    
    // choose burst type and compensation
    uint16_t msc_ctrl = REG_MSC_CTRL_GCOMP | REG_MSC_CTRL_PCOMP | REG_MSC_CTRL_DRPOL;
    if (opmode == OpMode::Delta32) {
        msc_ctrl |= REG_MSC_CTRL_BURST32 | REG_MSC_CTRL_BURSTSEL;
    } else if (opmode == OpMode::AG32) {
        msc_ctrl |= REG_MSC_CTRL_BURST32;
    }
    if (!write_reg16(REG_MSC_CTRL, msc_ctrl, true)) {
        return true;
    }

#if TIMING_DEBUG
    // useful for debugging scheduling of transfers
    hal.gpio->pinMode(52, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(53, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(54, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(55, HAL_GPIO_OUTPUT);
#endif

    // we need to use low speed for burst transfers
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    return true;
}


/*
  read a 16 bit register value
 */
uint16_t AP_InertialSensor_ADIS1647x::read_reg16(uint8_t regnum) const
{
    uint8_t req[2] = {regnum, 0};
    uint8_t reply[2] {};
    dev->transfer(req, sizeof(req), nullptr, 0);
    hal.scheduler->delay_microseconds(T_STALL_US);
    dev->transfer(nullptr, 0, reply, sizeof(reply));
    uint16_t ret = (reply[0]<<8U) | reply[1];
    return ret;
}


/*
  write a 16 bit register value
 */
bool AP_InertialSensor_ADIS1647x::write_reg16(uint8_t regnum, uint16_t value, bool confirm) const
{
    const uint8_t retries = 16;
    for (uint8_t i=0; i<retries; i++) {
        uint8_t req[2];
        req[0] = (regnum | 0x80);
        req[1] = value & 0xFF;
        dev->transfer(req, sizeof(req), nullptr, 0);
        hal.scheduler->delay_microseconds(T_STALL_US);

        req[0] = ((regnum+1) | 0x80);
        req[1] = (value>>8) & 0xFF;
        dev->transfer(req, sizeof(req), nullptr, 0);
        hal.scheduler->delay_microseconds(T_STALL_US);

        if (!confirm || read_reg16(regnum) == value) {
            return true;
        }
    }
    return false;
}

/*
  read the sensor using 16 bit burst transfer of gyro/accel data
 */
void AP_InertialSensor_ADIS1647x::read_sensor16(void)
{
    struct adis_data {
        uint8_t cmd[2];
        uint16_t diag_stat;
        int16_t  gx;
        int16_t  gy;
        int16_t  gz;
        int16_t  ax;
        int16_t  ay;
        int16_t  az;
        int16_t  temp;
        uint16_t counter;
        uint8_t  pad;
        uint8_t  checksum;
    } data {};

    do {
        WITH_SEMAPHORE(dev->get_semaphore());
        data.cmd[0] = REG_GLOB_CMD;
        DEBUG_SET_PIN(2, 1);
        if (!dev->transfer_fullduplex((uint8_t *)&data, sizeof(data))) {
            break;
        }
        DEBUG_SET_PIN(2, 0);
    } while (be16toh(data.counter) == last_counter);

    DEBUG_SET_PIN(1, 1);

    /*
      check the 8 bit checksum of the packet
     */
    uint8_t sum = 0;
    const uint8_t *b = (const uint8_t *)&data.diag_stat;
    for (uint8_t i=0; i<offsetof(adis_data, pad) - offsetof(adis_data, diag_stat); i++) {
        sum += b[i];
    }
    if (sum != data.checksum) {
        DEBUG_TOGGLE_PIN(3);
        DEBUG_TOGGLE_PIN(3);
        DEBUG_TOGGLE_PIN(3);
        DEBUG_TOGGLE_PIN(3);
        // corrupt data
        return;
    }

    /*
      check if we have lost a sample
     */
    uint16_t counter = be16toh(data.counter);
    if (done_first_read && uint16_t(last_counter+1) != counter) {
        DEBUG_TOGGLE_PIN(3);
    }
    done_first_read = true;
    last_counter = counter;

    Vector3f accel{float(int16_t(be16toh(data.ax))), float(int16_t(be16toh(data.ay))), float(int16_t(be16toh(data.az)))};
    Vector3f gyro{float(int16_t(be16toh(data.gx))), float(int16_t(be16toh(data.gy))), float(int16_t(be16toh(data.gz)))};

    accel *= accel_scale;
    gyro *= gyro_scale;

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    /*
      publish average temperature at 20Hz
     */
    temp_sum += float(int16_t(be16toh(data.temp))*0.1);
    temp_count++;

    if (temp_count == 100) {
        _publish_temperature(accel_instance, temp_sum/temp_count);
        temp_sum = 0;
        temp_count = 0;
    }
    DEBUG_SET_PIN(1, 0);
}


/*
  read the sensor using 32 bit burst transfer of accel/gyro
 */
void AP_InertialSensor_ADIS1647x::read_sensor32(void)
{
    struct adis_data {
        uint8_t cmd[2];
        uint16_t diag_stat;
        uint16_t  gx_low;
        uint16_t  gx_high;
        uint16_t  gy_low;
        uint16_t  gy_high;
        uint16_t  gz_low;
        uint16_t  gz_high;
        uint16_t  ax_low;
        uint16_t  ax_high;
        uint16_t  ay_low;
        uint16_t  ay_high;
        uint16_t  az_low;
        uint16_t  az_high;
        uint16_t  temp;
        uint16_t counter;
        uint8_t  pad;
        uint8_t  checksum;
    } data {};

    do {
        WITH_SEMAPHORE(dev->get_semaphore());
        data.cmd[0] = REG_GLOB_CMD;
        DEBUG_SET_PIN(2, 1);
        if (!dev->transfer_fullduplex((uint8_t *)&data, sizeof(data))) {
            break;
        }
        DEBUG_SET_PIN(2, 0);
    } while (be16toh(data.counter) == last_counter);

    DEBUG_SET_PIN(1, 1);

    /*
      check the 8 bit checksum of the packet
     */
    uint8_t sum = 0;
    const uint8_t *b = (const uint8_t *)&data.diag_stat;
    for (uint8_t i=0; i<offsetof(adis_data, pad) - offsetof(adis_data, diag_stat); i++) {
        sum += b[i];
    }
    if (sum != data.checksum) {
        DEBUG_TOGGLE_PIN(3);
        DEBUG_TOGGLE_PIN(3);
        DEBUG_TOGGLE_PIN(3);
        DEBUG_TOGGLE_PIN(3);
        // corrupt data
        return;
    }

    /*
      check if we have lost a sample
     */
    uint16_t counter = be16toh(data.counter);
    if (done_first_read && uint16_t(last_counter+1) != counter) {
        DEBUG_TOGGLE_PIN(3);
    }
    done_first_read = true;
    last_counter = counter;
    
    Vector3f accel{float(accel_scale*int32_t(be16toh(data.ax_low) | (be16toh(data.ax_high)<<16))),
                  -float(accel_scale*int32_t(be16toh(data.ay_low) | (be16toh(data.ay_high)<<16))),
                  -float(accel_scale*int32_t(be16toh(data.az_low) | (be16toh(data.az_high)<<16)))};
    Vector3f gyro{float(gyro_scale*int32_t(be16toh(data.gx_low) | (be16toh(data.gx_high)<<16))),
                    -float(gyro_scale*int32_t(be16toh(data.gy_low) | (be16toh(data.gy_high)<<16))),
                    -float(gyro_scale*int32_t(be16toh(data.gz_low) | (be16toh(data.gz_high)<<16)))};

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    /*
      publish average temperature at 20Hz
     */
    temp_sum += float(int16_t(be16toh(data.temp))*0.1);
    temp_count++;

    if (temp_count == 100) {
        _publish_temperature(accel_instance, temp_sum/temp_count);
        temp_sum = 0;
        temp_count = 0;
    }
    DEBUG_SET_PIN(1, 0);
}

/*
  read the sensor using 32 bit burst transfer of delta-angle/delta-velocity
 */
void AP_InertialSensor_ADIS1647x::read_sensor32_delta(void)
{
    struct adis_data {
        uint8_t cmd[2];
        uint16_t diag_stat;
        uint16_t  dax_low;
        uint16_t  dax_high;
        uint16_t  day_low;
        uint16_t  day_high;
        uint16_t  daz_low;
        uint16_t  daz_high;
        uint16_t  dvx_low;
        uint16_t  dvx_high;
        uint16_t  dvy_low;
        uint16_t  dvy_high;
        uint16_t  dvz_low;
        uint16_t  dvz_high;
        uint16_t  temp;
        uint16_t counter;
        uint8_t  pad;
        uint8_t  checksum;
    } data {};

    do {
        WITH_SEMAPHORE(dev->get_semaphore());
        data.cmd[0] = REG_GLOB_CMD;
        DEBUG_SET_PIN(2, 1);
        if (!dev->transfer_fullduplex((uint8_t *)&data, sizeof(data))) {
            break;
        }
        DEBUG_SET_PIN(2, 0);
    } while (be16toh(data.counter) == last_counter);

    DEBUG_SET_PIN(1, 1);

    /*
      check the 8 bit checksum of the packet
     */
    uint8_t sum = 0;
    const uint8_t *b = (const uint8_t *)&data.diag_stat;
    for (uint8_t i=0; i<offsetof(adis_data, pad) - offsetof(adis_data, diag_stat); i++) {
        sum += b[i];
    }
    if (sum != data.checksum) {
        DEBUG_TOGGLE_PIN(3);
        DEBUG_TOGGLE_PIN(3);
        DEBUG_TOGGLE_PIN(3);
        DEBUG_TOGGLE_PIN(3);
        // corrupt data
        return;
    }

    /*
      check if we have lost a sample
     */
    uint16_t counter = be16toh(data.counter);
    if (done_first_read && uint16_t(last_counter+1) != counter) {
        DEBUG_TOGGLE_PIN(3);
    }
    done_first_read = true;
    last_counter = counter;
    
    Vector3f dvel{float(dvel_scale*int32_t(be16toh(data.dvx_low) | (be16toh(data.dvx_high)<<16))),
                  -float(dvel_scale*int32_t(be16toh(data.dvy_low) | (be16toh(data.dvy_high)<<16))),
                  -float(dvel_scale*int32_t(be16toh(data.dvz_low) | (be16toh(data.dvz_high)<<16)))};
    Vector3f dangle{float(dangle_scale*int32_t(be16toh(data.dax_low) | (be16toh(data.dax_high)<<16))),
                    -float(dangle_scale*int32_t(be16toh(data.day_low) | (be16toh(data.day_high)<<16))),
                    -float(dangle_scale*int32_t(be16toh(data.daz_low) | (be16toh(data.daz_high)<<16)))};

    // compensate for clock errors, see "DELTA ANGLES" in datasheet
    dangle *= expected_sample_rate_hz / _gyro_raw_sample_rate(gyro_instance);
    dvel *= expected_sample_rate_hz / _accel_raw_sample_rate(gyro_instance);

    _notify_new_delta_velocity(accel_instance, dvel);
    _notify_new_delta_angle(gyro_instance, dangle);

    /*
      publish average temperature at 20Hz
     */
    temp_sum += float(int16_t(be16toh(data.temp))*0.1);
    temp_count++;

    if (temp_count == 100) {
        _publish_temperature(accel_instance, temp_sum/temp_count);
        temp_sum = 0;
        temp_count = 0;
    }
    DEBUG_SET_PIN(1, 0);
}

/*
  sensor read loop
 */
void AP_InertialSensor_ADIS1647x::loop(void)
{
    while (true) {
        uint32_t tstart = AP_HAL::micros();
        // we deliberately set the period a bit fast to ensure we
        // don't lose a sample
        const uint32_t period_us = (1000000UL / expected_sample_rate_hz) - 20U;
        bool wait_ok = false;
        if (drdy_pin != 0) {
            // when we have a DRDY pin then wait for it to go high
            DEBUG_SET_PIN(0, 1);
            wait_ok = hal.gpio->wait_pin(drdy_pin, AP_HAL::GPIO::INTERRUPT_RISING, 2100);
            DEBUG_SET_PIN(0, 0);
        }
        if (opmode == OpMode::Delta32) {
            read_sensor32_delta();
        } else if (opmode == OpMode::AG32) {
            read_sensor32();
        } else {
            read_sensor16();
        }
        uint32_t dt = AP_HAL::micros() - tstart;
        if (dt < period_us) {
            uint32_t wait_us = period_us - dt;
            if (!wait_ok || wait_us > period_us/2) {
                DEBUG_SET_PIN(3, 1);
                hal.scheduler->delay_microseconds(wait_us);
                DEBUG_SET_PIN(3, 0);
            }
        }
    }
}

bool AP_InertialSensor_ADIS1647x::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
