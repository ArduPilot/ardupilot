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

#include "AP_InertialSensor_BMI055.h"

/*
  device registers, names follow datasheet conventions, with REGA_
  prefix for accel, and REGG_ prefix for gyro
 */
#define REGA_BGW_CHIPID    0x00
#define REGA_ACCD_X_LSB    0x02
#define REGA_ACCD_TEMP     0x08
#define REGA_INT_STATUS_0  0x09
#define REGA_INT_STATUS_1  0x0A
#define REGA_INT_STATUS_2  0x0B
#define REGA_INT_STATUS_3  0x0C
#define REGA_FIFO_STATUS   0x0E
#define REGA_PMU_RANGE     0x0F
#define REGA_PMU_BW        0x10
#define REGA_PMU_LPW       0x11
#define REGA_ACCD_HBW      0x13
#define REGA_BGW_SOFTRESET 0x14
#define REGA_OUT_CTRL      0x20
#define REGA_EST_LATCH     0x21
#define REGA_FIFO_CONFIG_0 0x30
#define REGA_PMU_SELF_TEST 0x32
#define REGA_FIFO_CONFIG_1 0x3E
#define REGA_FIFO_DATA     0x3F

#define REGG_CHIPID        0x00
#define REGA_RATE_X_LSB    0x02
#define REGG_INT_STATUS_0  0x09
#define REGG_INT_STATUS_1  0x0A
#define REGG_INT_STATUS_2  0x0B
#define REGG_INT_STATUS_3  0x0C
#define REGG_FIFO_STATUS   0x0E
#define REGG_RANGE         0x0F
#define REGG_BW            0x10
#define REGG_LPM1          0x11
#define REGG_RATE_HBW      0x13
#define REGG_BGW_SOFTRESET 0x14
#define REGG_FIFO_CONFIG_1 0x3E
#define REGG_FIFO_DATA     0x3F

#define ACCEL_BACKEND_SAMPLE_RATE   2000
#define GYRO_BACKEND_SAMPLE_RATE    2000

extern const AP_HAL::HAL& hal;

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))

AP_InertialSensor_BMI055::AP_InertialSensor_BMI055(AP_InertialSensor &imu,
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
AP_InertialSensor_BMI055::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                enum Rotation rotation)
{
    if (!dev_accel || !dev_gyro) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_BMI055(imu, std::move(dev_accel), std::move(dev_gyro), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_BMI055::start()
{
    accel_instance = _imu.register_accel(ACCEL_BACKEND_SAMPLE_RATE, dev_accel->get_bus_id_devtype(DEVTYPE_INS_BMI055));
    gyro_instance = _imu.register_gyro(GYRO_BACKEND_SAMPLE_RATE,   dev_gyro->get_bus_id_devtype(DEVTYPE_INS_BMI055));

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);
    
    // setup callbacks
    dev_accel->register_periodic_callback(1000000UL / ACCEL_BACKEND_SAMPLE_RATE,
                                          FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BMI055::read_fifo_accel, void));
    dev_gyro->register_periodic_callback(1000000UL / GYRO_BACKEND_SAMPLE_RATE,
                                         FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BMI055::read_fifo_gyro, void));
}

/*
  probe and initialise accelerometer
 */
bool AP_InertialSensor_BMI055::accel_init()
{
    dev_accel->get_semaphore()->take_blocking();

    uint8_t v;
    if (!dev_accel->read_registers(REGA_BGW_CHIPID, &v, 1) || v != 0xFA) {
        goto failed;
    }

    if (!dev_accel->write_register(REGA_BGW_SOFTRESET, 0xB6)) {
        goto failed;
    }
    hal.scheduler->delay(10);
    
    dev_accel->setup_checked_registers(5, 20);
    
    // setup 16g range
    if (!dev_accel->write_register(REGA_PMU_RANGE, 0x0C, true)) {
        goto failed;
    }

    // setup filter bandwidth 1kHz
    if (!dev_accel->write_register(REGA_PMU_BW, 0x0F, true)) {
        goto failed;
    }

    // disable low-power mode
    if (!dev_accel->write_register(REGA_PMU_LPW, 0, true)) {
        goto failed;
    }

    // setup for unfiltered data
    if (!dev_accel->write_register(REGA_ACCD_HBW, 0x80, true)) {
        goto failed;
    }

    // setup FIFO for streaming X,Y,Z
    if (!dev_accel->write_register(REGA_FIFO_CONFIG_1, 0x80, true)) {
        goto failed;
    }

    hal.console->printf("BMI055: found accel\n");

    dev_accel->get_semaphore()->give();
    return true;
    
failed:
    dev_accel->get_semaphore()->give();
    return false;
}

/*
  probe and initialise gyro
 */
bool AP_InertialSensor_BMI055::gyro_init()
{
    dev_gyro->get_semaphore()->take_blocking();

    uint8_t v;
    if (!dev_gyro->read_registers(REGG_CHIPID, &v, 1) || v != 0x0F) {
        goto failed;
    }

    if (!dev_gyro->write_register(REGG_BGW_SOFTRESET, 0xB6)) {
        goto failed;
    }
    hal.scheduler->delay(10);

    dev_gyro->setup_checked_registers(5, 20);
    
    // setup 2000dps range
    if (!dev_gyro->write_register(REGG_RANGE, 0x00, true)) {
        goto failed;
    }

    // setup filter bandwidth 230Hz, no decimation
    if (!dev_gyro->write_register(REGG_BW, 0x81, true)) {
        goto failed;
    }

    // disable low-power mode
    if (!dev_gyro->write_register(REGG_LPM1, 0, true)) {
        goto failed;
    }

    // setup for filtered data
    if (!dev_gyro->write_register(REGG_RATE_HBW, 0x00, true)) {
        goto failed;
    }

    // setup FIFO for streaming X,Y,Z
    if (!dev_gyro->write_register(REGG_FIFO_CONFIG_1, 0x80, true)) {
        goto failed;
    }

    hal.console->printf("BMI055: found gyro\n");    

    dev_gyro->get_semaphore()->give();
    return true;
    
failed:
    dev_gyro->get_semaphore()->give();
    return false;
}

bool AP_InertialSensor_BMI055::init()
{
    dev_accel->set_read_flag(0x80);
    dev_gyro->set_read_flag(0x80);

    return accel_init() && gyro_init();
}

/*
  read accel fifo
 */
void AP_InertialSensor_BMI055::read_fifo_accel(void)
{
    uint8_t num_frames;
    if (!dev_accel->read_registers(REGA_FIFO_STATUS, &num_frames, 1)) {
        _inc_accel_error_count(accel_instance);
        return;
    }
    num_frames &= 0x7F;
    
    // don't read more than 8 frames at a time
    if (num_frames > 8) {
        num_frames = 8;
    }

    if (num_frames == 0) {
        return;
    }
    
    uint8_t data[6*num_frames];
    if (!dev_accel->read_registers(REGA_FIFO_DATA, data, num_frames*6)) {
        _inc_accel_error_count(accel_instance);
        return;
    }
    // data is 12 bits with 16g range, 7.81mg/LSB
    const float scale = 7.81 * 0.001 * GRAVITY_MSS / 16.0f;
    for (uint8_t i = 0; i < num_frames; i++) {
        const uint8_t *d = &data[i*6];
        int16_t xyz[3] {
                int16_t(uint16_t((d[0]&0xF0) | (d[1]<<8))),
                int16_t(uint16_t((d[2]&0xF0) | (d[3]<<8))),
                int16_t(uint16_t((d[4]&0xF0) | (d[5]<<8))) };
        Vector3f accel(xyz[0], xyz[1], xyz[2]);

        accel *= scale;

        _rotate_and_correct_accel(accel_instance, accel);
        _notify_new_accel_raw_sample(accel_instance, accel);
    }

    if (temperature_counter++ == 100) {
        temperature_counter = 0;
        int8_t t;
        if (!dev_accel->read_registers(REGA_ACCD_TEMP, (uint8_t *)&t, 1)) {
            _inc_accel_error_count(accel_instance);
        } else {
            float temp_degc = (0.5f * t) + 23.0f;
            _publish_temperature(accel_instance, temp_degc);
        }
    }
    
    if (!dev_accel->check_next_register()) {
        _inc_accel_error_count(accel_instance);
    }
}

/*
  read gyro fifo
 */
void AP_InertialSensor_BMI055::read_fifo_gyro(void)
{
    uint8_t num_frames;
    if (!dev_gyro->read_registers(REGG_FIFO_STATUS, &num_frames, 1)) {
        _inc_gyro_error_count(gyro_instance);
        return;
    }
    num_frames &= 0x7F;
    
    // don't read more than 8 frames at a time
    if (num_frames > 8) {
        num_frames = 8;
    }
    if (num_frames == 0) {
        return;
    }
    uint8_t data[6*num_frames];
    if (!dev_gyro->read_registers(REGG_FIFO_DATA, data, num_frames*6)) {
        _inc_gyro_error_count(gyro_instance);
        return;
    }

    // data is 16 bits with 2000dps range
    const float scale = radians(2000.0f) / 32767.0f;
    for (uint8_t i = 0; i < num_frames; i++) {
        const uint8_t *d = &data[i*6];
        int16_t xyz[3] {
                    int16_t(uint16_t(d[0] | d[1]<<8)),
                    int16_t(uint16_t(d[2] | d[3]<<8)),
                    int16_t(uint16_t(d[4] | d[5]<<8)) };
        Vector3f gyro(xyz[0], xyz[1], xyz[2]);
        gyro *= scale;

        _rotate_and_correct_gyro(gyro_instance, gyro);
        _notify_new_gyro_raw_sample(gyro_instance, gyro);
    }

    if (!dev_gyro->check_next_register()) {
        _inc_gyro_error_count(gyro_instance);
    }
}

bool AP_InertialSensor_BMI055::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
