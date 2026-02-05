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

#include "AP_InertialSensor_ADIS16607.h"

/*
  device registers
 */
#define REG_DEV_ID              0x00
#define DEV_ID_16607            0x6000

#define REG_REV_ID              0x01
#define REG_DIAG_STAT           0x05
#define REG_WRITE_LOCK          0x2D
#define REG_USER_GPIO_CFG1      0x2F
#define REG_SPI_FULLDUPLEX_KEY  0x31
#define REG_SPI_HALFDUPLEX_KEY  0x32

#define REG_USER_DATA_CFG               0x34
#define USER_DATA_CFG_X_ACCEL_EN        (1<<0U)
#define USER_DATA_CFG_Y_ACCEL_EN        (1<<1U)
#define USER_DATA_CFG_Z_ACCEL_EN        (1<<2U)
#define USER_DATA_CFG_X_GYRO_EN         (1<<3U)
#define USER_DATA_CFG_Y_GYRO_EN         (1<<4U)
#define USER_DATA_CFG_Z_GYRO_EN         (1<<5U)
#define USER_DATA_CFG_TEMPERATURE_EN    (1<<12U)
#define USER_DATA_CFG_TIME_STAMP_EN     (1<<13U)
#define USER_DATA_CFG_DATA_CNTR_EN      (1<<14U)
#define USER_DATA_CFG_WORD_SIZE_16      (0<<15U)
#define USER_DATA_CFG_WORD_SIZE_32      (1<<15U)

#define REG_USER_FIFO_CFG               0x35
#define USER_FIFO_CFG_CLEAR_FIFO_B      (1<<15U)

#define REG_SOFT_RESET          0x36
#define REG_MSC_CTRL            0x39

#define REG_DEC_RATE            0x3A
#define DEC_RATE_1912Hz         4

extern const AP_HAL::HAL& hal;

AP_InertialSensor_ADIS16607::AP_InertialSensor_ADIS16607(AP_InertialSensor &imu,
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
AP_InertialSensor_ADIS16607::probe(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation,
                                   uint8_t drdy_gpio)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_ADIS16607 *sensor = NEW_NOTHROW AP_InertialSensor_ADIS16607(imu, std::move(dev), rotation, drdy_gpio);

    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_ADIS16607::start()
{
    if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_ADIS16607)) ||
        !_imu.register_gyro(gyro_instance, expected_sample_rate_hz,   dev->get_bus_id_devtype(DEVTYPE_INS_ADIS16607))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    // This creates a thread to do the capture, running at very high priority
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADIS16607::loop, void),
                                      "ADIS16607",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("Failed to create ADIS16607 thread");
    }
}

/**
 * @brief Check dev ID
 */
bool AP_InertialSensor_ADIS16607::check_dev_id()
{
    // Lock the SPI mode
    write_reg16(REG_SPI_HALFDUPLEX_KEY, 0xB4B4, false);

    if (read_reg16(REG_DEV_ID) == DEV_ID_16607) {
        accel_scale = GRAVITY_MSS / 200000.0f;  // Accel: Dynamic Range ±40g. 24-bit data format 200000.0 LSB/g
        gyro_scale = radians(1.0 / 4000.0);    // Gyro: ADIS16607-3, 24-bit data format 4000.0 LSB/°/sec
        expected_sample_rate_hz = 1912;
        _clip_limit = 39.5f * GRAVITY_MSS;
        return true;
    }

    return false;
}

bool AP_InertialSensor_ADIS16607::init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    uint8_t tries = 5;
    do {
        // Enable SPI Writes to Register Map 
        write_reg16(REG_WRITE_LOCK, 0xAAAA, false);
        write_reg16(REG_WRITE_LOCK, 0x5555, false);
        // Perform software reset
        write_reg16(REG_SOFT_RESET, 0x01);
        // Wait 200 ms for part to come out of reset and bootloader to complete.
        hal.scheduler->delay(200);
    } while (!check_dev_id() && --tries);

    if (tries == 0) {
        return false;
    }

    // Ensure DIGITAL_STATUS is 0x0000 for clear possible false errors
    tries = 10;
    uint16_t stat = 0;
    while ((stat = read_reg16(REG_DIAG_STAT)) != 0x0000 && --tries) {
        hal.scheduler->delay(10);
    }

    if (tries == 0) {
        return false;
    }

    // Configure GPIO pins(GPIO3 as DR)
    write_reg16(REG_USER_GPIO_CFG1, 0x0200, false);

    // Init USER_DATA_CFG register if we use burst read mode
    uint16_t user_data_cfg = USER_DATA_CFG_X_ACCEL_EN | USER_DATA_CFG_Y_ACCEL_EN | USER_DATA_CFG_Z_ACCEL_EN |
                             USER_DATA_CFG_X_GYRO_EN | USER_DATA_CFG_Y_GYRO_EN | USER_DATA_CFG_Z_GYRO_EN |
                             USER_DATA_CFG_TEMPERATURE_EN | USER_DATA_CFG_DATA_CNTR_EN | USER_DATA_CFG_WORD_SIZE_32;

    if (!write_reg16(REG_USER_DATA_CFG, user_data_cfg, true)) {
        return false;
    }

    // Clear FIFO data
    if (!write_reg16(REG_USER_FIFO_CFG, USER_FIFO_CFG_CLEAR_FIFO_B, false)) {
        return false;
    }

    // Filter BW. fC=500Hz.
    if (!write_reg16(REG_MSC_CTRL, 0x100, true)) {
        return false;
    }

    /**
     * Bring rate down
     * The actual decimation rate, D, is the DEC_RATE+1. Note that when changing the decimation rate, it is 
     * recommended to first reset DEC_RATE to 0x000 before entering the new value. This 
     * allows the decimation accumulator to reset.
     */
    if (!write_reg16(REG_DEC_RATE, 0, true)) {
        return false;
    }

    if (!write_reg16(REG_DEC_RATE, DEC_RATE_1912Hz, true)) {
        return false;
    }

    // Write lock
    write_reg16(REG_WRITE_LOCK, 0x5555, false);
    write_reg16(REG_WRITE_LOCK, 0xAAAA, false);

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    return true;
}

/**
 * @brief Read a 16 bit register value
 */
inline uint16_t AP_InertialSensor_ADIS16607::read_reg16(uint8_t regnum) const
{
    uint16_t ret = 0;
    uint8_t seq[4] = {0};

    seq[0] = (regnum | 0x80);
    dev->transfer(&seq[0], 1, &seq[1], 3);
    ret = (seq[2]<<8U) | seq[3];

    return ret;
}

/**
 * @brief Write a 16 bit register value
 */
bool AP_InertialSensor_ADIS16607::write_reg16(uint8_t regnum, uint16_t value, bool confirm) const
{
    const uint8_t retries = 8;
    uint8_t seq[3] = {0};

    for (uint8_t i = 0; i < retries; i++) {
        seq[0] = regnum;
        seq[1] = (value>>8) & 0xFF;
        seq[2] = value & 0xFF;
        dev->transfer(seq, sizeof(seq), nullptr, 0);

        if (!confirm || read_reg16(regnum) == value) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Read the sensor register value of accel/gyro
 */
void AP_InertialSensor_ADIS16607::read_sensor(void)
{
    struct adis_data {
        uint8_t cmd[2];
        uint16_t diag_stat;
        uint16_t ax_high;
        uint16_t ax_low;
        uint16_t ay_high;
        uint16_t ay_low;
        uint16_t az_high;
        uint16_t az_low;
        uint16_t gx_high;
        uint16_t gx_low;
        uint16_t gy_high;
        uint16_t gy_low;
        uint16_t gz_high;
        uint16_t gz_low;
        uint16_t temp;
        uint16_t counter;
        uint16_t checksum;
    } data {};

    do {
        WITH_SEMAPHORE(dev->get_semaphore());
        data.cmd[0] = (REG_DIAG_STAT | 0x80);
        if (!dev->transfer_fullduplex((uint8_t *)&data, sizeof(data))) {
            return;
        }
    } while (be16toh(data.counter) == last_counter);

    // Check the 16 bit checksum of the packet
    uint16_t sum = 0;
    const uint16_t *b = (const uint16_t *)&data.diag_stat;
    for (uint8_t i=0; i<((offsetof(adis_data, checksum) - offsetof(adis_data, diag_stat))/sizeof(uint16_t)); i++) {
        sum += be16toh(b[i]);
    }

    // Check brust read checksum
    if (sum != be16toh(data.checksum)) {
        return; // Corrupt data
    }

    data.diag_stat = be16toh(data.diag_stat);

    /**
     * If the DIAG_STAT register is not 0x8000 or 0x0000, the data 
     * outputs are not guaranteed to be valid and should be discarded.
     */
    if ((data.diag_stat != 0x8000) && (data.diag_stat != 0x0000)) {
        return;
    }

    data.counter = be16toh(data.counter);

    // Check if we have lost a sample
    // if (done_first_read && (uint16_t(last_counter+1) != data.counter)) {
    //     hal.console->printf("lost sample\n");
    // }
    // done_first_read = true;
    // last_counter = data.counter;

    Vector3f accel{float(accel_scale * (int32_t((be16toh(data.ax_high) << 16) | be16toh(data.ax_low)) >> 8)),
                   float(accel_scale * (int32_t((be16toh(data.ay_high) << 16) | be16toh(data.ay_low)) >> 8)),
                   float(accel_scale * (int32_t((be16toh(data.az_high) << 16) | be16toh(data.az_low)) >> 8))};
    Vector3f gyro{float(gyro_scale * (int32_t((be16toh(data.gx_high) << 16) | be16toh(data.gx_low)) >> 8)),
                  float(gyro_scale * (int32_t((be16toh(data.gy_high) << 16) | be16toh(data.gy_low)) >> 8)),
                  float(gyro_scale * (int32_t((be16toh(data.gz_high) << 16) | be16toh(data.gz_low)) >> 8))};

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    temp_sum += float(int16_t(be16toh(data.temp)) * 0.005f) + 25.0f;
    temp_count++;

    // Publish average temperature at about 20Hz.
    if (temp_count == 50) {
        _publish_temperature(accel_instance, temp_sum/temp_count);
        temp_sum = 0;
        temp_count = 0;
    }
}

/**
 * @brief Sensor read loop
 */
void AP_InertialSensor_ADIS16607::loop(void)
{
    while (true) {
        uint32_t tstart = AP_HAL::micros();
        // we deliberately set the period a bit fast to ensure we don't lose a sample
        const uint32_t period_us = (1000000UL / expected_sample_rate_hz) - 35U;
        bool wait_ok = false;
        if (drdy_pin != 0) {
            // when we have a DRDY pin then wait for falling edge
            wait_ok = hal.gpio->wait_pin(drdy_pin, AP_HAL::GPIO::INTERRUPT_FALLING, 1100);
        }
        read_sensor();
        uint32_t dt = AP_HAL::micros() - tstart;
        if (dt < period_us) {
            uint32_t wait_us = period_us - dt;
            if (!wait_ok || wait_us > period_us/2) {
                hal.scheduler->delay_microseconds(wait_us);
            }
        }
    }
}

bool AP_InertialSensor_ADIS16607::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
