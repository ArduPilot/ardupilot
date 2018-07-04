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
  driver for TeraRanger I2C rangefinders
 */
#include "AP_RangeFinder_TeraRangerI2C.h"

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_Common/Semaphore.h>

extern const AP_HAL::HAL& hal;

// registers
#define TR_MEASURE 0x00
#define TR_WHOAMI  0x01
#define TR_WHOAMI_VALUE 0xA1

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_TeraRangerI2C::AP_RangeFinder_TeraRangerI2C(RangeFinder::RangeFinder_State &_state,
															AP_RangeFinder_Params &_params,
                                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> i2c_dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(i2c_dev))
{
}

/*
   detect if a TeraRanger rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_TeraRangerI2C::detect(RangeFinder::RangeFinder_State &_state,
																AP_RangeFinder_Params &_params,
                                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> i2c_dev)
{
    if (!i2c_dev) {
        return nullptr;
    }

    AP_RangeFinder_TeraRangerI2C *sensor = new AP_RangeFinder_TeraRangerI2C(_state, _params, std::move(i2c_dev));
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
bool AP_RangeFinder_TeraRangerI2C::init(void)
{
    if (!dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    dev->set_retries(10);

    // check WHOAMI
    uint8_t whoami;
    if (!dev->read_registers(TR_WHOAMI, &whoami, 1) ||
        whoami != TR_WHOAMI_VALUE) {
        dev->get_semaphore()->give();
        return false;
    }

    if (!measure()) {
        dev->get_semaphore()->give();
        return false;
    }

    // give time for the sensor to process the request
    hal.scheduler->delay(70);

    uint16_t _distance_cm;
    if (!collect_raw(_distance_cm)) {
        dev->get_semaphore()->give();
        return false;
    }

    dev->get_semaphore()->give();

    dev->set_retries(1);

    dev->register_periodic_callback(50000,
                                    FUNCTOR_BIND_MEMBER(&AP_RangeFinder_TeraRangerI2C::timer, void));

    return true;
}

// measure() - ask sensor to make a range reading
bool AP_RangeFinder_TeraRangerI2C::measure()
{
    uint8_t cmd = TR_MEASURE;
    return dev->transfer(&cmd, 1, nullptr, 0);
}

// collect_raw() - return last value measured by sensor
bool AP_RangeFinder_TeraRangerI2C::collect_raw(uint16_t &raw_distance)
{
    uint8_t d[3];

    // Take range reading
    if (!dev->transfer(nullptr, 0, d, sizeof(d))) {
        return false;
    }

    // Check for CRC
    if (d[2] != crc_crc8(d, 2)) {
        return false;
    } else {
        raw_distance = ((uint16_t(d[0]) << 8) | d[1]);
        return true;
    }
}

// Checks for error code and if correct converts to cm
bool AP_RangeFinder_TeraRangerI2C::process_raw_measure(uint16_t raw_distance, uint16_t &output_distance_cm)
{
  // Check for error codes
  if (raw_distance == 0xFFFF) {
      // Too far away is unreliable so we dont enforce max range here
      return false;
  } else if (raw_distance == 0x0000) {
      // Too close
      output_distance_cm =  state.min_distance_cm;
      return true;
  } else if (raw_distance == 0x0001) {
      // Unable to measure
      return false;
  } else {
    output_distance_cm = raw_distance/10; // Conversion to centimeters
    return true;
  }
}

/*
  timer called at 20Hz
*/
void AP_RangeFinder_TeraRangerI2C::timer(void)
{
    // Take a reading
    uint16_t _raw_distance = 0;
    uint16_t _distance_cm = 0;

    if (collect_raw(_raw_distance)) {
        WITH_SEMAPHORE(_sem);

        if (process_raw_measure(_raw_distance, _distance_cm)){
            accum.sum += _distance_cm;
            accum.count++;
        }
    }
    // and immediately ask for a new reading
    measure();
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_TeraRangerI2C::update(void)
{
    WITH_SEMAPHORE(_sem);

    if (accum.count > 0) {
        state.distance_cm = accum.sum / accum.count;
        state.last_reading_ms = AP_HAL::millis();
        accum.sum = 0;
        accum.count = 0;
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
