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
#include "AP_RangeFinder_GP2Y0E03.h"

#include <utility>

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

const uint8_t CMD_SYSTEM_RESET = 0xee;    
const uint8_t CMD_SOFTWARE_RESET = 0x06;
const uint8_t CMD_PULSE_WIDTH = 0x13;
const uint8_t CMD_READ_DISTANCE_1 = 0x5e;
const uint8_t CMD_READ_DISTANCE_2 = 0x5f;
const uint8_t CMD_READ_SHIFT = 0x35;


AP_RangeFinder_GP2Y0E03::AP_RangeFinder_GP2Y0E03(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state, _params)
    , _dev(std::move(dev))
{
}

AP_RangeFinder_Backend *AP_RangeFinder_GP2Y0E03::detect(
        RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_GP2Y0E03 *sensor
        = new AP_RangeFinder_GP2Y0E03(_state, _params, std::move(dev));

    if (!sensor ) {
        return nullptr;
    }
    
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_RangeFinder_GP2Y0E03::init()
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_retries(10);

    /*
     * Reseting the sensor
     */

    if (!_dev->write_register(CMD_SYSTEM_RESET, CMD_SOFTWARE_RESET)) {
        _dev->get_semaphore()->give();
        hal.console->printf(" \n\n ERRO 1 \n\n");
        return false;
    }

    hal.scheduler->delay(100);

    //Reading the attribute Width to check if the sensor is alive
    uint8_t val;    
    
    _dev->set_retries(10);    
    if (!_dev->read_registers(CMD_PULSE_WIDTH, &val,sizeof(val))) {
        _dev->get_semaphore()->give();
        return false;
    }
    
    if (val != 7) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GP2Y0E03: Invalid width %u", val);
        _dev->get_semaphore()->give();
        return false;
    }
    
    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_GP2Y0E03::timer, void));

    return true;
}

void AP_RangeFinder_GP2Y0E03::update()
{
    WITH_SEMAPHORE(_sem);

    if (accum.count > 0) {
        state.distance_cm = accum.sum / accum.count;
        state.last_reading_ms = AP_HAL::millis();
        accum.sum = 0;
        accum.count = 0;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::Status::NoData);
    }
}

void AP_RangeFinder_GP2Y0E03::timer()
{
    uint8_t shift, high, low;
    
    if (!_dev->read_registers(CMD_READ_SHIFT, &shift, sizeof(shift))) {
        return;
    }
    
    if (!_dev->read_registers(CMD_READ_DISTANCE_1, &high, sizeof(high))) {
        return;
    }
    
    if (!_dev->read_registers(CMD_READ_DISTANCE_2, &low, sizeof(low))) {
        return;
    }
    
    const uint16_t data = (((high << 4) | low) / 16) >> shift;
    
    {
        WITH_SEMAPHORE(_sem);
        accum.sum += data;
        accum.count++;
    }
}