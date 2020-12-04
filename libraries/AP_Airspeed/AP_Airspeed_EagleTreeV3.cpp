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
  driver for EagleTreeV3 airspeed sensor
  https://www.eagletreesystems.com/Manuals/microsensor-i2c.pdf
 */

#include "AP_Airspeed_EagleTreeV3.h"
#include <AP_Math/AP_Math.h>

#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL &hal;

#define EagleTreeV3_I2C_ADDR 0xEA

#ifdef EagleTreeV3_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif


AP_Airspeed_EagleTreeV3::AP_Airspeed_EagleTreeV3(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{}

// probe and initialise the sensor
bool AP_Airspeed_EagleTreeV3::init()
{
    dev = hal.i2c_mgr->get_device(get_bus(), EagleTreeV3_I2C_ADDR);
    if (!dev) {
        return false;
    }
    WITH_SEMAPHORE(dev->get_semaphore());
    dev->set_retries(2);
    dev->register_periodic_callback(1000000UL/50U,
                                    FUNCTOR_BIND_MEMBER(&AP_Airspeed_EagleTreeV3::timer, void));
    return true;
}

// start_reading() - ask sensor for reading
bool AP_Airspeed_EagleTreeV3::start_reading()
{
    const uint8_t cmd = 0x07;  // provide a reading...

    // send command to take reading
    return dev->transfer(&cmd, sizeof(cmd), nullptr, 0);
}

// read - return last value measured by sensor
bool AP_Airspeed_EagleTreeV3::get_reading(uint16_t &reading_kph)
{
    be16_t val;

    // take range reading and read back results
    bool ret = dev->transfer(nullptr, 0, (uint8_t *) &val, sizeof(val));

    if (ret) {
        reading_kph = be16toh(val);

        // trigger a new reading
        start_reading();
    }

    return ret;
}


// 50Hz timer
void AP_Airspeed_EagleTreeV3::timer()
{
    uint16_t d;
    if (!get_reading(d)) {
        return;
    }

    WITH_SEMAPHORE(sem);
    airspeed_kph = d;

    last_sample_time_ms = AP_HAL::millis();
}

// return airspeed in m/s if available
bool AP_Airspeed_EagleTreeV3::get_airspeed(float& airspeed)
{
    WITH_SEMAPHORE(sem);

    if (!last_sample_time_ms) {
        return false;
    }

    airspeed = airspeed_kph * 1000.0f / 3600.0f;  // kilometres/hour -> m/s
    return true;
}
