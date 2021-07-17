/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   AP_RangeFinder_TOF10120_I2C.cpp - rangefinder for Taidacent TOF10120 Time of Flight Sensor
 *
 *   TOF10120 - a low cost and lightweight laser distance sensor with I2C & UART interfaces.
 *
 *   Datasheet: https://github.com/simpleiot/reference/blob/master/sensors/TOF10120.pdf
 *   Datasheet (EN): https://github.com/simpleiot/reference/blob/master/sensors/TOF10120_english.pdf
 */

#include "AP_RangeFinder_TOF10120_I2C.h"

#include <utility>
#include <math.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_RangeFinder_TOF10120_I2C::AP_RangeFinder_TOF10120_I2C(RangeFinder::RangeFinder_State &_state,
                                                           AP_RangeFinder_Params &_params,
                                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state, _params)
    , _dev(std::move(dev))
{
}

/*
   detect if a TOF10120 rangefinder is connected.
*/
AP_RangeFinder_Backend *AP_RangeFinder_TOF10120_I2C::detect(RangeFinder::RangeFinder_State &_state,
																AP_RangeFinder_Params &_params,
                                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_TOF10120_I2C *sensor
        = new AP_RangeFinder_TOF10120_I2C(_state, _params, std::move(dev));
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_TOF10120_I2C::init(void)
{
    _dev->get_semaphore()->take_blocking();

    if (!start_reading()) {
        _dev->get_semaphore()->give();
        return false;
    }

    // give time for the sensor to process the request
    hal.scheduler->delay(100);

    uint16_t reading_cm;
    if (!get_reading(reading_cm)) {
        _dev->get_semaphore()->give();
        return false;
    }

    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(100000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_TOF10120_I2C::timer, void));

    return true;
}

// ask sensor to make a range reading
bool AP_RangeFinder_TOF10120_I2C::start_reading()
{
    uint8_t cmd = AP_RANGE_FINDER_TOF10120_I2C_RANGE_READING_CMD;

    // send command to take reading
    return _dev->transfer(&cmd, sizeof(cmd), nullptr, 0);
}

// read - return last value measured by sensor
bool AP_RangeFinder_TOF10120_I2C::get_reading(uint16_t &reading_cm)
{
    uint16_t distance_mm = 0;
    uint8_t buffer[2];

    // take range reading
    bool ret = _dev->transfer(nullptr, 0, (uint8_t *) &buffer, sizeof(buffer));

    if (ret) {
        distance_mm = (buffer[0] << 8) | buffer[1];

        reading_cm = (int)roundf(distance_mm / 10);

        // trigger a new reading
        start_reading();
    }

    return ret;
}

/*
  timer called at 10Hz
*/
void AP_RangeFinder_TOF10120_I2C::timer(void)
{
    uint16_t d;
    if (get_reading(d)) {
        WITH_SEMAPHORE(_sem);
        distance = d;
        new_distance = true;
        state.last_reading_ms = AP_HAL::millis();
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_TOF10120_I2C::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (new_distance) {
        state.distance_cm = distance;
        new_distance = false;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 300) {
        // if no updates for 0.3 seconds set no-data
        set_status(RangeFinder::Status::NoData);
    }
}

