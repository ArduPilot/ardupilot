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
#define REGA_TEMP_MSB      0x22
#define REGA_TEMP_LSB      0x23
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

#define ACCEL_BACKEND_SAMPLE_RATE   1600
#define GYRO_BACKEND_SAMPLE_RATE    2000

const uint32_t ACCEL_BACKEND_PERIOD_US = 1000000UL / ACCEL_BACKEND_SAMPLE_RATE;
const uint32_t GYRO_BACKEND_PERIOD_US = 1000000UL / GYRO_BACKEND_SAMPLE_RATE;

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
    auto sensor = NEW_NOTHROW AP_InertialSensor_BMI088(imu, std::move(dev_accel), std::move(dev_gyro), rotation);

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
    if (!_imu.register_accel(accel_instance, ACCEL_BACKEND_SAMPLE_RATE, dev_accel->get_bus_id_devtype(_accel_devtype)) ||
        !_imu.register_gyro(gyro_instance, GYRO_BACKEND_SAMPLE_RATE,   dev_gyro->get_bus_id_devtype(DEVTYPE_INS_BMI088))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    // setup callbacks
    accel_periodic_handle = dev_accel->register_periodic_callback(ACCEL_BACKEND_PERIOD_US,
                                                                  FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BMI088::read_fifo_accel, void));
    gyro_periodic_handle = dev_gyro->register_periodic_callback(GYRO_BACKEND_PERIOD_US,
                                                                FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BMI088::read_fifo_gyro, void));
}

/*
  read from accelerometer registers, special SPI handling needed
*/
bool AP_InertialSensor_BMI088::read_accel_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    // when on I2C we just read normally
    if (dev_accel->bus_type() != AP_HAL::Device::BUS_TYPE_SPI) {
        return dev_accel->read_registers(reg, data, len);
    }
    // for SPI we need to discard the first returned byte. See
    // datasheet for explanation
    uint8_t b[len+2];
    b[0] = reg | 0x80;
    memset(&b[1], 0, len+1);
    if (!dev_accel->transfer_fullduplex(b, len+2)) {
        return false;
    }
    memcpy(data, &b[2], len);
    return true;
}

/*
  write to accel registers with retries. The SPI sensor may take
  several tries to correctly write a register
*/
bool AP_InertialSensor_BMI088::write_accel_register(uint8_t reg, uint8_t v)
{
    for (uint8_t i=0; i<8; i++) {
        dev_accel->write_register(reg, v);
        uint8_t v2 = 0;
        if (read_accel_registers(reg, &v2, 1) && v2 == v) {
            return true;
        }
    }
    return false;
}

static const struct {
    uint8_t reg;
    uint8_t value;
} accel_config[] = {
    // OSR2 gives 234Hz LPF @ 1.6Khz ODR
    { REGA_CONF, 0x9C },
    // setup 24g range (16g for BMI085)
    { REGA_RANGE, 0x03 },
    // disable low-power mode
    { REGA_PWR_CONF, 0 },
    { REGA_PWR_CTRL, 0x04 },
    // setup FIFO for streaming X,Y,Z
    { REGA_FIFO_CONFIG0, 0x02 },
    { REGA_FIFO_CONFIG1, 0x50 },
};

bool AP_InertialSensor_BMI088::setup_accel_config(void)
{
    if (done_accel_config) {
        return true;
    }
    accel_config_count++;
    for (uint8_t i=0; i<ARRAY_SIZE(accel_config); i++) {
        uint8_t v;
        if (!read_accel_registers(accel_config[i].reg, &v, 1)) {
            return false;
        }
        if (v == accel_config[i].value) {
            continue;
        }
        if (!write_accel_register(accel_config[i].reg, accel_config[i].value)) {
            return false;
        }
    }
    done_accel_config = true;
    DEV_PRINTF("BMI088: accel config OK (%u tries)\n", (unsigned)accel_config_count);
    return true;
}

/*
  probe and initialise accelerometer
 */
bool AP_InertialSensor_BMI088::accel_init()
{
    WITH_SEMAPHORE(dev_accel->get_semaphore());

    uint8_t v;

    // dummy ready on accel ChipID to init accel (see section 3 of datasheet)
    read_accel_registers(REGA_CHIPID, &v, 1);

    if (!read_accel_registers(REGA_CHIPID, &v, 1)) {
        return false;
    }

    switch (v) {
        case 0x1E:
            _accel_devtype = DEVTYPE_INS_BMI088;
            accel_range = 24.0;
            hal.console->printf("BMI088: Found device\n");
            break;
        case 0x1F:
            _accel_devtype = DEVTYPE_INS_BMI085;
            accel_range = 16.0;
            hal.console->printf("BMI085: Found device\n");
            break;
        default:
            return false;
    }

    if (!setup_accel_config()) {
        DEV_PRINTF("BMI08x: delaying accel config\n");
    }

    DEV_PRINTF("BMI08x: found accel\n");

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

    /* Soft-reset gyro
        Return value of 'write_register()' is not checked.
        This commands has the tendency to fail upon soft-reset.
    */
    dev_gyro->write_register(REGG_BGW_SOFTRESET, 0xB6);
    hal.scheduler->delay(30);

    dev_gyro->setup_checked_registers(5, 20);
    
    // setup 2000dps range
    if (!dev_gyro->write_register(REGG_RANGE, 0x00, true)) {
        return false;
    }

    // setup filter bandwidth 532Hz, no decimation
    if (!dev_gyro->write_register(REGG_BW, 0x80, true)) {
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

    // setup FIFO for streaming X,Y,Z, with stop-at-full
    if (!dev_gyro->write_register(REGG_FIFO_CONFIG_1, 0x40, true)) {
        return false;
    }

    DEV_PRINTF("BMI088: found gyro\n");    

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
    if (!setup_accel_config()) {
        return;
    }
    uint8_t len[2];
    if (!read_accel_registers(REGA_FIFO_LEN0, len, 2)) {
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
    
    // adjust the periodic callback to be synchronous with the incoming data
    // this means that we rarely run read_fifo_accel() without updating the sensor data
    dev_accel->adjust_periodic_callback(accel_periodic_handle, ACCEL_BACKEND_PERIOD_US);

    uint8_t data[fifo_length];
    if (!read_accel_registers(REGA_FIFO_DATA, data, fifo_length)) {
        _inc_accel_error_count(accel_instance);
        return;
    }

    // use new accel_range depending on sensor type
    const float scale = (1.0/32768.0) * GRAVITY_MSS * accel_range;
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
        if (!read_accel_registers(REGA_TEMP_MSB, tbuf, 2)) {
            _inc_accel_error_count(accel_instance);
        } else {
            uint16_t temp_uint11 = (tbuf[0]<<3) | (tbuf[1]>>5);
            int16_t temp_int11 = temp_uint11>1023?temp_uint11-2048:temp_uint11;
            float temp_degc = temp_int11 * 0.125f + 23;
            _publish_temperature(accel_instance, temp_degc);
        }
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
    const float scale = radians(2000.0f) / 32767.0f;
    const uint8_t max_frames = 8;
    const Vector3i bad_frame{INT16_MIN,INT16_MIN,INT16_MIN};
    Vector3i data[max_frames];

    if (num_frames & 0x80) {
        // fifo overrun, reset, likely caused by scheduling error
        dev_gyro->write_register(REGG_FIFO_CONFIG_1, 0x40, true);
        goto check_next;
    }

    num_frames &= 0x7F;
    
    // don't read more than 8 frames at a time
    num_frames = MIN(num_frames, max_frames);
    if (num_frames == 0) {
        goto check_next;
    }

    // adjust the periodic callback to be synchronous with the incoming data
    // this means that we rarely run read_fifo_gyro() without updating the sensor data
    dev_gyro->adjust_periodic_callback(gyro_periodic_handle, GYRO_BACKEND_PERIOD_US);

    if (!dev_gyro->read_registers(REGG_FIFO_DATA, (uint8_t *)data, num_frames*6)) {
        _inc_gyro_error_count(gyro_instance);
        goto check_next;
    }

    // data is 16 bits with 2000dps range
    for (uint8_t i = 0; i < num_frames; i++) {
        if (data[i] == bad_frame) {
            continue;
        }
        Vector3f gyro(data[i].x, data[i].y, data[i].z);
        gyro *= scale;

        _rotate_and_correct_gyro(gyro_instance, gyro);
        _notify_new_gyro_raw_sample(gyro_instance, gyro);
    }

check_next:
    AP_HAL::Device::checkreg reg;
    if (!dev_gyro->check_next_register(reg)) {
        log_register_change(dev_gyro->get_bus_id(), reg);
        _inc_gyro_error_count(gyro_instance);
    }
}

bool AP_InertialSensor_BMI088::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
