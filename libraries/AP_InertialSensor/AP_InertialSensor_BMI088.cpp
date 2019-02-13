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
#include <AP_Common/Semaphore.h>

#include "AP_InertialSensor_BMI088.h"

/*
  device registers, names follow datasheet conventions, with REGA_
  prefix for accel, and REGG_ prefix for gyro
 */
#define REGA_CHIPID        0x00
#define REGA_ERR_REG       0x02
#define REGA_STATUS        0x03
#define REGA_X_LSB         0x12
#define REGA_INT_STATUS_1  0x1D
#define REGA_TEMP_LSB      0x22
#define REGA_TEMP_MSB      0x23
#define REGA_CONF          0x40
#define REGA_RANGE         0x41
#define REGA_PWR_CONF      0x7C
#define REGA_PWR_CTRL      0x7D
#define REGA_SOFTRESET     0x7E
#define REGA_FIFO_CONFIG0  0x48
#define REGA_FIFO_CONFIG1  0x49
#define REGA_FIFO_DOWNS    0x45
#define REGA_FIFO_DATA     0x26
#define REGA_FIFO_LEN0     0x24
#define REGA_FIFO_LEN1     0x25

#define REGG_CHIPID        0x00
#define REGA_RATE_X_LSB    0x02
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

extern const AP_HAL::HAL& hal;

AP_InertialSensor_BMI088::AP_InertialSensor_BMI088(AP_InertialSensor &imu,
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
AP_InertialSensor_BMI088::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev_accel,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro,
                                enum Rotation rotation)
{
    if (!dev_accel || !dev_gyro) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_BMI088(imu, std::move(dev_accel), std::move(dev_gyro), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_BMI088::start()
{
    accel_instance = _imu.register_accel(1600, dev_accel->get_bus_id_devtype(DEVTYPE_INS_BMI088));
    gyro_instance = _imu.register_gyro(2000,   dev_gyro->get_bus_id_devtype(DEVTYPE_INS_BMI088));

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);
    
    // setup callbacks
    dev_accel->register_periodic_callback(1000000UL / 1600,
                                          FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BMI088::read_fifo_accel, void));
    dev_gyro->register_periodic_callback(1000000UL / 2000,
                                         FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BMI088::read_fifo_gyro, void));
}

/*
  probe and initialise accelerometer
 */
bool AP_InertialSensor_BMI088::accel_init()
{
    WITH_SEMAPHORE(dev_accel->get_semaphore());

    uint8_t v;

    // dummy ready on accel ChipID to init accel (see section 3 of datasheet)
    dev_accel->read_registers(REGA_CHIPID, &v, 1);

    if (!dev_accel->read_registers(REGA_CHIPID, &v, 1) || v != 0x1E) {
        return false;
    }

    dev_accel->setup_checked_registers(6, 20);
    
    // setup normal mode for DLPF, with 1600Hz ODR
    if (!dev_accel->write_register(REGA_CONF, 0xAC, true)) {
        return false;
    }

    // setup 24g range
    if (!dev_accel->write_register(REGA_RANGE, 0x03, true)) {
        return false;
    }

    // disable low-power mode
    if (!dev_accel->write_register(REGA_PWR_CONF, 0, true)) {
        return false;
    }
    if (!dev_accel->write_register(REGA_PWR_CTRL, 0x04, true)) {
        return false;
    }

    // setup FIFO for streaming X,Y,Z
    if (!dev_accel->write_register(REGA_FIFO_CONFIG0, 0x00, true)) {
        return false;
    }
    if (!dev_accel->write_register(REGA_FIFO_CONFIG1, 0x50, true)) {
        return false;
    }

    hal.console->printf("BMI088: found accel\n");

    return true;
}

/*
  probe and initialise gyro
 */
bool AP_InertialSensor_BMI088::gyro_init()
{
    WITH_SEMAPHORE(dev_gyro->get_semaphore());

    uint8_t v;
    if (!dev_gyro->read_registers(REGG_CHIPID, &v, 1) || v != 0x0F) {
        return false;
    }

    if (!dev_gyro->write_register(REGG_BGW_SOFTRESET, 0xB6)) {
        return false;
    }
    hal.scheduler->delay(10);

    dev_gyro->setup_checked_registers(5, 20);
    
    // setup 2000dps range
    if (!dev_gyro->write_register(REGG_RANGE, 0x00, true)) {
        return false;
    }

    // setup filter bandwidth 230Hz, no decimation
    if (!dev_gyro->write_register(REGG_BW, 0x81, true)) {
        return false;
    }

    // disable low-power mode
    if (!dev_gyro->write_register(REGG_LPM1, 0, true)) {
        return false;
    }

    // setup for filtered data
    if (!dev_gyro->write_register(REGG_RATE_HBW, 0x00, true)) {
        return false;
    }

    // setup FIFO for streaming X,Y,Z
    if (!dev_gyro->write_register(REGG_FIFO_CONFIG_1, 0x80, true)) {
        return false;
    }

    hal.console->printf("BMI088: found gyro\n");    

    return true;
}

bool AP_InertialSensor_BMI088::init()
{
    dev_accel->set_read_flag(0x80);
    dev_gyro->set_read_flag(0x80);

    return accel_init() && gyro_init();
}

/*
  read accel fifo
 */
void AP_InertialSensor_BMI088::read_fifo_accel(void)
{
    uint8_t len[2];
    if (!dev_accel->read_registers(REGA_FIFO_LEN0, len, 2)) {
        _inc_accel_error_count(accel_instance);
        return;
    }
    uint16_t fifo_length = len[0] + (len[1]<<8);
    if (fifo_length & 0x8000) {
        // empty
        return;
    }

    // don't read more than 8 frames at a time
    if (fifo_length > 8*7) {
        fifo_length = 8*7;
    }
    if (fifo_length == 0) {
        return;
    }
    
    uint8_t data[fifo_length];
    if (!dev_accel->read_registers(REGA_FIFO_DATA, data, fifo_length)) {
        _inc_accel_error_count(accel_instance);
        return;
    }
    // assume configured for 24g range
    const float scale = (1.0/32768.0) * GRAVITY_MSS * 24.0;
    const uint8_t *p = &data[0];
    while (fifo_length >= 7) {
        /*
          the fifo frames are variable length, with the frame type in the first byte
         */
        uint8_t frame_len = 2;
        switch (p[0] & 0xFC) {
        case 0x84: {
            // accel frame
            frame_len = 7;
            const uint8_t *d = p+1;
            int16_t xyz[3] {
                int16_t(uint16_t(d[0] | (d[1]<<8))),
                int16_t(uint16_t(d[2] | (d[3]<<8))),
                int16_t(uint16_t(d[4] | (d[5]<<8)))};
            Vector3f accel(xyz[0], xyz[1], xyz[2]);

            accel *= scale;

            _rotate_and_correct_accel(accel_instance, accel);
            _notify_new_accel_raw_sample(accel_instance, accel);
            break;
        }
        case 0x40:
            // skip frame
            frame_len = 2;
            break;
        case 0x44:
            // sensortime frame
            frame_len = 4;
            break;
        case 0x48:
            // fifo config frame
            frame_len = 2;
            break;
        case 0x50:
            // sample drop frame
            frame_len = 2;
            break;
        }
        p += frame_len;
        fifo_length -= frame_len;
    }

    if (temperature_counter++ == 100) {
        temperature_counter = 0;
        uint8_t tbuf[2];
        if (!dev_accel->read_registers(REGA_TEMP_LSB, tbuf, 2)) {
            _inc_accel_error_count(accel_instance);
        } else {
            uint16_t temp_uint11 = (tbuf[0]<<3) | (tbuf[1]>>5);
            int16_t temp_int11 = temp_uint11>1023?temp_uint11-2048:temp_uint11;
            float temp_degc = temp_int11 * 0.125 + 23;
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
void AP_InertialSensor_BMI088::read_fifo_gyro(void)
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
    const float scale = radians(2000.0) / 32767.0;
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

bool AP_InertialSensor_BMI088::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
