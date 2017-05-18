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
  driver for trone rangefinder
 */
#include "AP_RangeFinder_trone.h"

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>

extern const AP_HAL::HAL& hal;

#define TRONE_I2C_ADDR 0x30

// registers
#define TRONE_MEASURE 0x00
#define TRONE_WHOAMI  0x01
#define TRONE_WHOAMI_VALUE 0xA1

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_trone::AP_RangeFinder_trone(uint8_t bus, RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state)
    : AP_RangeFinder_Backend(_ranger, instance, _state)
    , dev(hal.i2c_mgr->get_device(bus, TRONE_I2C_ADDR))
{
}

/*
   detect if a trone rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_trone::detect(uint8_t bus, RangeFinder &_ranger, uint8_t instance,
                                                     RangeFinder::RangeFinder_State &_state)
{
    AP_RangeFinder_trone *sensor = new AP_RangeFinder_trone(bus, _ranger, instance, _state);
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
bool AP_RangeFinder_trone::init(void)
{
    if (!dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    dev->set_retries(10);

    // check WHOAMI
    uint8_t whoami;
    if (!dev->read_registers(TRONE_WHOAMI, &whoami, 1) ||
        whoami != TRONE_WHOAMI_VALUE) {
        return false;
    }
    
    if (!measure()) {
        dev->get_semaphore()->give();
        return false;
    }

    // give time for the sensor to process the request
    hal.scheduler->delay(70);

    uint16_t distance_cm;
    if (!collect(distance_cm)) {
        dev->get_semaphore()->give();
        return false;
    }

    dev->get_semaphore()->give();

    dev->set_retries(1);

    dev->register_periodic_callback(50000,
                                    FUNCTOR_BIND_MEMBER(&AP_RangeFinder_trone::timer, void));

    return true;
}

// measure() - ask sensor to make a range reading
bool AP_RangeFinder_trone::measure()
{
    uint8_t cmd = TRONE_MEASURE;
    return dev->transfer(&cmd, 1, nullptr, 0);
}

// collect - return last value measured by sensor
bool AP_RangeFinder_trone::collect(uint16_t &distance_cm)
{
    uint8_t d[3];

    // take range reading and read back results
    if (!dev->transfer(nullptr, 0, d, sizeof(d))) {
        return false;
    }

    if (d[2] != crc_crc8(d, 2)) {
        // bad CRC
        return false;
    }
    
    distance_cm = ((uint16_t(d[0]) << 8) | d[1]) / 10;

    return true;
}

/*
  timer called at 20Hz
*/
void AP_RangeFinder_trone::timer(void)
{
    // take a reading
    uint16_t distance_cm;
    if (collect(distance_cm) && _sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        accum.sum += distance_cm;
        accum.count++;
        _sem->give();
    }

    // and immediately ask for a new reading
    measure();
}


/*
   update the state of the sensor
*/
void AP_RangeFinder_trone::update(void)
{
    if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (accum.count > 0) {
            state.distance_cm = accum.sum / accum.count;
            accum.sum = 0;
            accum.count = 0;
            update_status();
        } else {
            set_status(RangeFinder::RangeFinder_NoData);
        }
         _sem->give();
    }
}
