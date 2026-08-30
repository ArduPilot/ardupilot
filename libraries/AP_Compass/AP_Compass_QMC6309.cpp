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
 * Driver by Takumi Saito, Aug 2026
 */
#include "AP_Compass_QMC6309.h"

#if AP_COMPASS_QMC6309_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

// Register addresses
#define QMC6309_REG_ID          0x00
#define QMC6309_REG_DATA_X      0x01
#define QMC6309_REG_STATUS      0x09
#define QMC6309_REG_CONTROL1    0x0A
#define QMC6309_REG_CONTROL2    0x0B

// WHOAMI value
#define QMC6309_ID              0x90

// Status register bits
#define QMC6309_STATUS_DRDY          (1U << 0)
#define QMC6309_STATUS_OVERFLOW      (1U << 1)
#define QMC6309_STATUS_NVM_READY     (1U << 3)
#define QMC6309_STATUS_NVM_LOAD_DONE (1U << 4)

// Control register 1: measurement mode and over-sampling ratio
#define QMC6309_CONTROL1_NORMAL_MODE 0x01
#define QMC6309_CONTROL1_OSR1_8      (0U << 3)
#define QMC6309_CONTROL1_OSR2_8      (3U << 5)

// Control register 2: set/reset mode, full-scale range, output data rate and soft reset
#define QMC6309_CONTROL2_SET_RESET_ON (0U << 0)
#define QMC6309_CONTROL2_RANGE_8G     (2U << 2)
#define QMC6309_CONTROL2_ODR_100HZ    (3U << 4)
#define QMC6309_CONTROL2_SOFT_RESET   (1U << 7)

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_QMC6309::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
        bool force_external,
        enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_Compass_QMC6309 *sensor = NEW_NOTHROW AP_Compass_QMC6309(std::move(dev), force_external, rotation);
    if (sensor == nullptr || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_QMC6309::AP_Compass_QMC6309(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                       bool force_external,
                                       enum Rotation rotation)
    : _dev(std::move(dev))
    , _rotation(rotation)
    , _force_external(force_external)
{
}

bool AP_Compass_QMC6309::check_whoami()
{
    uint8_t whoami = 0;
    return _dev->read_registers(QMC6309_REG_ID, &whoami, 1) && whoami == QMC6309_ID;
}

bool AP_Compass_QMC6309::reset()
{
    // trigger a soft reset
    if (!_dev->write_register(QMC6309_REG_CONTROL2, QMC6309_CONTROL2_SOFT_RESET)) {
        return false;
    }
    hal.scheduler->delay(1);

    // clear the reset bit so the sensor leaves reset state
    if (!_dev->write_register(QMC6309_REG_CONTROL2, 0)) {
        return false;
    }

    // wait for the sensor to finish reloading its NVM trim values
    const uint32_t start_ms = AP_HAL::millis();
    while (AP_HAL::millis() - start_ms < 20) {
        uint8_t status = 0;
        if (_dev->read_registers(QMC6309_REG_STATUS, &status, 1) &&
            (status & (QMC6309_STATUS_NVM_READY | QMC6309_STATUS_NVM_LOAD_DONE)) ==
            (QMC6309_STATUS_NVM_READY | QMC6309_STATUS_NVM_LOAD_DONE)) {
            return true;
        }
        hal.scheduler->delay(1);
    }

    return false;
}

bool AP_Compass_QMC6309::init()
{
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    // confirm the sensor is present and put it into a known state
    if (!check_whoami() || !reset() || !check_whoami()) {
        return false;
    }

    // configure output data rate, full-scale range and set/reset mode
    const uint8_t control2 = QMC6309_CONTROL2_ODR_100HZ |
                             QMC6309_CONTROL2_RANGE_8G |
                             QMC6309_CONTROL2_SET_RESET_ON;
    // configure measurement mode and over-sampling ratio
    const uint8_t control1 = QMC6309_CONTROL1_OSR2_8 |
                             QMC6309_CONTROL1_OSR1_8 |
                             QMC6309_CONTROL1_NORMAL_MODE;

    if (!_dev->write_register(QMC6309_REG_CONTROL2, control2) ||
        !_dev->write_register(QMC6309_REG_CONTROL1, control1)) {
        return false;
    }

    // lower retries for run
    _dev->set_retries(3);
    _dev->set_device_type(DEVTYPE_QMC6309);

    if (!register_compass(_dev->get_bus_id())) {
        return false;
    }

    set_rotation(_rotation);
    if (_force_external) {
        set_external(true);
    }

    // poll at 100Hz to match the configured output data rate
    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Compass_QMC6309::timer, void));
    return true;
}

void AP_Compass_QMC6309::timer()
{
    // only proceed once new data is ready and not flagged as an overflow
    uint8_t status = 0;
    if (!_dev->read_registers(QMC6309_REG_STATUS, &status, 1) ||
        !(status & QMC6309_STATUS_DRDY) ||
        (status & QMC6309_STATUS_OVERFLOW)) {
        return;
    }

    struct PACKED {
        le16_t x;
        le16_t y;
        le16_t z;
    } sample;

    // burst read X, Y and Z in one transfer
    if (!_dev->read_registers(QMC6309_REG_DATA_X, reinterpret_cast<uint8_t *>(&sample), sizeof(sample))) {
        return;
    }

    // At the selected +/-8 gauss range the sensor produces 4000 LSB/G.
    const float range_scale = 1000.0f / 4000.0f;
    Vector3f field{
        static_cast<float>(static_cast<int16_t>(le16toh(sample.x))) * range_scale,
        static_cast<float>(static_cast<int16_t>(le16toh(sample.y))) * range_scale,
        static_cast<float>(static_cast<int16_t>(le16toh(sample.z))) * range_scale
    };

    accumulate_sample(field, 20);
}

void AP_Compass_QMC6309::read()
{
    drain_accumulated_samples();
}

#endif  // AP_COMPASS_QMC6309_ENABLED
