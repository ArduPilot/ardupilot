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
/*
  Driver by Voltafield, July 2026
 */

#include "AP_Compass_AF9838.h"

#if AP_COMPASS_AF9838_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

AP_Compass_AF9838::AP_Compass_AF9838(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation)
    : AP_Compass_Backend()
    , _dev(std::move(dev))
    , _force_external(force_external)
    , _rotation(rotation)
{
}

AP_Compass_Backend *AP_Compass_AF9838::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
        bool force_external,
        enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_Compass_AF9838 *sensor = NEW_NOTHROW AP_Compass_AF9838(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_Compass_AF9838::init()
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_retries(10);

    uint8_t val = 0;

    if (!_dev->read_registers(REG_PCODE, &val, 1) || val != PCODE_EXPECT) {
        return false;
    }

    if (!_dev->write_register(REG_SWR, SWR_TRIGGER)) {
        return false;
    }
    hal.scheduler->delay(5);

    if (!_dev->write_register(REG_STATE, STATE_SELF_TEST)) {
        return false;
    }
    hal.scheduler->delay(40);

    uint8_t status = 0;

    if (!_dev->read_registers(REG_STATUS, &status, 1) || (status & STATUS_STERR)) {
        return false;
    }

    _dev->set_device_type(DEVTYPE_AF9838);

    if (!register_compass(_dev->get_bus_id())) {
        return false;
    }

    set_rotation(_rotation);

    if (_force_external) {
        set_external(true);
    }

    _dev->set_retries(1);

    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Compass_AF9838::timer, void));

    return true;
}

void AP_Compass_AF9838::timer()
{

    if (!_single_pending) {
        if (!_dev->write_register(REG_STATE, STATE_SINGLE)) {
            return;
        }
        _single_pending = true;
        _single_start_us = AP_HAL::micros();
        return;
    }

    uint8_t status = 0;
    if (!_dev->read_registers(REG_STATUS, &status, 1)) {
        _single_pending = false;
        return;
    }

    if ((status & STATUS_ACQ) == 0) {
        const uint32_t now = AP_HAL::micros();
        if ((now - _single_start_us) > 20000U) {
            _single_pending = false;
        }
        return;
    }

    uint8_t buf[6];
    if (!_dev->read_registers(REG_DATA, buf, sizeof(buf))) {
        _single_pending = false;
        return;
    }

    // Raw samples are little-endian signed 16-bit values.
    const int16_t x = (int16_t)(buf[0] | (uint16_t(buf[1]) << 8));
    const int16_t y = (int16_t)(buf[2] | (uint16_t(buf[3]) << 8));
    const int16_t z = (int16_t)(buf[4] | (uint16_t(buf[5]) << 8));

    if (status & STATUS_HOFL) {
        _single_pending = false;
        return;
    }

    Vector3f field(float(x) * AF9838_MILLIGAUSS_PER_LSB,
                   float(y) * AF9838_MILLIGAUSS_PER_LSB,
                   float(z) * AF9838_MILLIGAUSS_PER_LSB);

    accumulate_sample(field);
    _single_pending = false;
}

void AP_Compass_AF9838::read()
{
    drain_accumulated_samples();
}
#endif  // AP_COMPASS_AF9838_ENABLED
