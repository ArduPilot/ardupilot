/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for the ST ISM330DLC 6-axis IMU, SPI only.

  The ISM330DLC shares the LSM6DSL register layout, so the control registers
  match the ASM330 driver, but the FIFO does not: this part uses the older
  decimation-based FIFO rather than a tagged one. With both the gyro and the
  accelerometer batched undecimated and nothing else enabled, the FIFO holds a
  fixed repeating pattern of six 16-bit words - gyro X/Y/Z then accel X/Y/Z -
  which is what this driver relies on to decode it.

  Datasheet: ISM330DLC DS12171 Rev 4.
 */

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_ISM330DLC.h"

#include <utility>

extern const AP_HAL::HAL& hal;

#define ISM330DLC_WHO_AM_I_VALUE        0x6A

#define ISM330DLC_REG_FIFO_CTRL1        0x06
#define ISM330DLC_REG_FIFO_CTRL2        0x07
#define ISM330DLC_REG_FIFO_CTRL3        0x08
#   define FIFO_CTRL3_DEC_GYRO_NONE     (0x1 << 3)
#   define FIFO_CTRL3_DEC_XL_NONE       (0x1 << 0)
#define ISM330DLC_REG_FIFO_CTRL4        0x09
#define ISM330DLC_REG_FIFO_CTRL5        0x0A
#   define FIFO_CTRL5_ODR_1667Hz        (0x8 << 3)
#   define FIFO_CTRL5_MODE_BYPASS       (0x0 << 0)
#   define FIFO_CTRL5_MODE_CONTINUOUS   (0x6 << 0)

#define ISM330DLC_REG_INT1_CTRL         0x0D
#define ISM330DLC_REG_INT2_CTRL         0x0E
#define ISM330DLC_REG_WHO_AM_I          0x0F

#define ISM330DLC_REG_CTRL1_XL          0x10
#   define CTRL1_XL_ODR_1667Hz          (0x8 << 4)
// full-scale encoding is not in numeric order: 00 = 2g, 01 = 16g, 10 = 4g, 11 = 8g
#   define CTRL1_XL_FS_16G              (0x1 << 2)
#define ISM330DLC_REG_CTRL2_G           0x11
#   define CTRL2_G_ODR_1667Hz           (0x8 << 4)
#   define CTRL2_G_FS_2000DPS           (0x3 << 2)
#define ISM330DLC_REG_CTRL3_C           0x12
#   define CTRL3_C_BOOT                 (0x1 << 7)
#   define CTRL3_C_BDU                  (0x1 << 6)
#   define CTRL3_C_IF_INC               (0x1 << 2)
#   define CTRL3_C_SW_RESET             (0x1 << 0)
#define ISM330DLC_REG_CTRL4_C           0x13
#   define CTRL4_C_I2C_DISABLE          (0x1 << 2)
#define ISM330DLC_REG_CTRL5_C           0x14
#define ISM330DLC_REG_CTRL6_C           0x15
#define ISM330DLC_REG_CTRL7_G           0x16
#define ISM330DLC_REG_CTRL8_XL          0x17
#define ISM330DLC_REG_CTRL9_XL          0x18
#define ISM330DLC_REG_CTRL10_C          0x19

#define ISM330DLC_REG_OUT_TEMP_L        0x20

#define ISM330DLC_REG_FIFO_STATUS1      0x3A
#define ISM330DLC_REG_FIFO_DATA_OUT_L   0x3E

// gyro X/Y/Z then accel X/Y/Z, 16 bits each
#define ISM330DLC_FIFO_WORDS_PER_SET    6

// the sensor produces ~1.67 sample sets per 1 kHz poll; allow plenty of head
// room for scheduling jitter while still bounding the work done in one pass
#define ISM330DLC_MAX_SETS_PER_POLL     12

#define ISM330DLC_SAMPLE_RATE           1667

AP_InertialSensor_ISM330DLC::AP_InertialSensor_ISM330DLC(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> device,
                                                         enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(device))
    , rot(rotation)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_ISM330DLC::probe(AP_InertialSensor &imu,
                                                              AP_HAL::OwnPtr<AP_HAL::Device> device,
                                                              enum Rotation rotation)
{
    if (!device) {
        return nullptr;
    }

    AP_InertialSensor_ISM330DLC *sensor =
        NEW_NOTHROW AP_InertialSensor_ISM330DLC(imu, std::move(device), rotation);
    if (!sensor || !sensor->init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_ISM330DLC::init_sensor()
{
    return hardware_init();
}

bool AP_InertialSensor_ISM330DLC::hardware_init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_read_flag(0x80);

    const uint8_t whoami = register_read(ISM330DLC_REG_WHO_AM_I);
    if (whoami != ISM330DLC_WHO_AM_I_VALUE) {
        DEV_PRINTF("ISM330DLC: unexpected WHOAMI 0x%x\n", (unsigned)whoami);
        return false;
    }

    // eleven registers are written checked below (CTRL3_C, CTRL4_C,
    // INT1/2_CTRL, FIFO_CTRL1-5, CTRL2_G, CTRL1_XL) and re-verified
    // periodically at runtime
    dev->setup_checked_registers(11, 20);

    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    bool reset_success = false;
    for (uint8_t tries = 0; tries < 5; tries++) {
        if (chip_reset()) {
            common_init();
            gyro_init();
            accel_init();
            fifo_init();

            hal.scheduler->delay(50);

            // the FIFO only fills once both sensors are actually running, so a
            // non-empty FIFO is the confirmation that configuration took
            uint16_t pattern;
            if (fifo_status(pattern) > 0) {
                reset_success = true;
                break;
            }
        }
    }

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    if (!reset_success) {
        DEV_PRINTF("ISM330DLC: failed to boot\n");
        return false;
    }

    return true;
}

void AP_InertialSensor_ISM330DLC::start(void)
{
    if (!_imu.register_gyro(gyro_instance, ISM330DLC_SAMPLE_RATE,
                            dev->get_bus_id_devtype(DEVTYPE_INS_ISM330DLC)) ||
        !_imu.register_accel(accel_instance, ISM330DLC_SAMPLE_RATE,
                             dev->get_bus_id_devtype(DEVTYPE_INS_ISM330DLC))) {
        return;
    }

    set_gyro_orientation(gyro_instance, rot);
    set_accel_orientation(accel_instance, rot);

    {
        WITH_SEMAPHORE(dev->get_semaphore());
        fifo_reset();
    }

    dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ISM330DLC::poll_data, void));
}

uint8_t AP_InertialSensor_ISM330DLC::register_read(uint8_t reg)
{
    uint8_t val = 0;
    dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_ISM330DLC::register_write(uint8_t reg, uint8_t val, bool checked)
{
    dev->write_register(reg, val, checked);
}

bool AP_InertialSensor_ISM330DLC::chip_reset()
{
    register_write(ISM330DLC_REG_CTRL3_C, CTRL3_C_SW_RESET);

    // the reset bit self-clears when the internal reset completes
    for (uint8_t tries = 0; tries < 5; tries++) {
        if ((register_read(ISM330DLC_REG_CTRL3_C) & CTRL3_C_SW_RESET) == 0) {
            return true;
        }
        hal.scheduler->delay(2);
    }

    return false;
}

void AP_InertialSensor_ISM330DLC::common_init()
{
    // BDU keeps the low and high halves of a sample consistent, and is also
    // what makes the FIFO status registers safe to read as a group. IF_INC is
    // required for the multi-byte reads this driver uses throughout.
    register_write(ISM330DLC_REG_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC, true);

    // this board wires the part to SPI only; disabling the I2C pins stops the
    // unconnected pads floating into a bus state
    register_write(ISM330DLC_REG_CTRL4_C, CTRL4_C_I2C_DISABLE, true);

    // no interrupts are routed: ArduPilot drains the FIFO by polling
    register_write(ISM330DLC_REG_INT1_CTRL, 0x00, true);
    register_write(ISM330DLC_REG_INT2_CTRL, 0x00, true);
}

void AP_InertialSensor_ISM330DLC::fifo_init()
{
    // no watermark: the driver reads whatever whole sample sets are present
    register_write(ISM330DLC_REG_FIFO_CTRL1, 0x00, true);
    register_write(ISM330DLC_REG_FIFO_CTRL2, 0x00, true);

    // batch gyro and accel undecimated and nothing else, which fixes the FIFO
    // pattern at the six words this driver decodes
    register_write(ISM330DLC_REG_FIFO_CTRL3,
                   FIFO_CTRL3_DEC_GYRO_NONE | FIFO_CTRL3_DEC_XL_NONE, true);
    register_write(ISM330DLC_REG_FIFO_CTRL4, 0x00, true);

    // FIFO_CTRL5 holds both the FIFO ODR and the mode, and resets to bypass
    // (FIFO disabled). It must be written last, once the gyro and the
    // accelerometer are already running, or nothing is ever batched.
    register_write(ISM330DLC_REG_FIFO_CTRL5,
                   FIFO_CTRL5_ODR_1667Hz | FIFO_CTRL5_MODE_CONTINUOUS, true);
}

void AP_InertialSensor_ISM330DLC::gyro_init()
{
    register_write(ISM330DLC_REG_CTRL2_G, CTRL2_G_ODR_1667Hz | CTRL2_G_FS_2000DPS, true);
}

void AP_InertialSensor_ISM330DLC::accel_init()
{
    register_write(ISM330DLC_REG_CTRL1_XL, CTRL1_XL_ODR_1667Hz | CTRL1_XL_FS_16G, true);
}

void AP_InertialSensor_ISM330DLC::fifo_reset()
{
    // a pass through bypass mode is the documented way to flush the FIFO, and
    // it also re-aligns the pattern to the start of a sample set
    register_write(ISM330DLC_REG_FIFO_CTRL5, FIFO_CTRL5_MODE_BYPASS);
    register_write(ISM330DLC_REG_FIFO_CTRL5,
                   FIFO_CTRL5_ODR_1667Hz | FIFO_CTRL5_MODE_CONTINUOUS);

    // the datasheet requires the first set acquired across a FIFO mode switch
    // to be thrown away
    discard_set = true;

    notify_accel_fifo_reset(accel_instance);
    notify_gyro_fifo_reset(gyro_instance);
}

/*
  recover from a FIFO left part way through a sample set
 */
void AP_InertialSensor_ISM330DLC::fifo_recover()
{
    fifo_reset();
    _inc_gyro_error_count(gyro_instance);
    _inc_accel_error_count(accel_instance);
}

/*
  return the number of unread 16-bit words held in the FIFO, and the index of
  the word the next read will pop from the recursive pattern
 */
uint16_t AP_InertialSensor_ISM330DLC::fifo_status(uint16_t &pattern)
{
    uint8_t status[4];
    if (!dev->read_registers(ISM330DLC_REG_FIFO_STATUS1, status, sizeof(status))) {
        pattern = 0;
        return 0;
    }
    // DIFF_FIFO is 11 bits split across FIFO_STATUS1 and the low bits of 2,
    // FIFO_PATTERN is 10 bits split the same way across 3 and 4
    pattern = uint16_t(status[2]) | (uint16_t(status[3] & 0x03) << 8);
    return uint16_t(status[0]) | (uint16_t(status[1] & 0x07) << 8);
}

void AP_InertialSensor_ISM330DLC::read_temperature()
{
    uint8_t raw[2];
    if (!dev->read_registers(ISM330DLC_REG_OUT_TEMP_L, raw, sizeof(raw))) {
        DEV_PRINTF("ISM330DLC: error reading temperature\n");
        return;
    }
    const int16_t t = int16_t(uint16_t(raw[0]) | (uint16_t(raw[1]) << 8));
    temperature_degc = temperature_filter.apply(float(t) / 256.0f + 25.0f);
}

void AP_InertialSensor_ISM330DLC::poll_data()
{
    uint16_t pattern;
    uint16_t sets = fifo_status(pattern) / ISM330DLC_FIFO_WORDS_PER_SET;
    if (sets > ISM330DLC_MAX_SETS_PER_POLL) {
        sets = ISM330DLC_MAX_SETS_PER_POLL;
    }

    // the pattern indexes the next word within the gyro-then-accel set, so
    // anything but zero means the halves would be read swapped from here on
    if (pattern != 0) {
        fifo_recover();
        return;
    }

    for (uint16_t set = 0; set < sets; set++) {
        int16_t w[ISM330DLC_FIFO_WORDS_PER_SET];
        bool ok = true;

        // one transfer per 16-bit word: CTRL3_C.IF_INC increments the
        // address on every byte, so a two-byte read pops one word from
        // FIFO_DATA_OUT_L/H while a longer read would carry on into
        // TIMESTAMP0/1 above it and return zeroes.
        for (uint8_t i = 0; i < ISM330DLC_FIFO_WORDS_PER_SET; i++) {
            uint8_t raw[2];
            if (!dev->read_registers(ISM330DLC_REG_FIFO_DATA_OUT_L, raw, sizeof(raw))) {
                ok = false;
                break;
            }
            w[i] = int16_t(uint16_t(raw[0]) | (uint16_t(raw[1]) << 8));
        }
        if (!ok) {
            // the set was left part way popped
            DEV_PRINTF("ISM330DLC: error reading FIFO\n");
            fifo_recover();
            return;
        }

        // popped but not published: the set spanning a FIFO mode switch is
        // the one the datasheet says to throw away
        if (discard_set) {
            discard_set = false;
            continue;
        }

        // the FIFO pattern is the gyro data set followed by the accel one
        Vector3f gyro_data(w[0], w[1], w[2]);
        gyro_data *= gyro_scale;
        _rotate_and_correct_gyro(gyro_instance, gyro_data);
        _notify_new_gyro_raw_sample(gyro_instance, gyro_data);

        Vector3f accel_data(w[3], w[4], w[5]);
        accel_data *= accel_scale;
        _rotate_and_correct_accel(accel_instance, accel_data);
        _notify_new_accel_raw_sample(accel_instance, accel_data);
    }

    if (temperature_counter++ >= 10) {
        temperature_counter = 0;
        read_temperature();
    }

    // check next register value for correctness
    AP_HAL::Device::checkreg reg;
    if (!dev->check_next_register(reg)) {
        log_register_change(dev->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
    }
}

bool AP_InertialSensor_ISM330DLC::update()
{
    update_gyro(gyro_instance);
    update_accel(accel_instance);

    _publish_temperature(accel_instance, temperature_degc);
    return true;
}
