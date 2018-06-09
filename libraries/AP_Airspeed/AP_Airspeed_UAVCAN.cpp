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
 *   UAVCAN airspeed driver
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_Airspeed_UAVCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Common/AP_Common.h>
#include "AP_Airspeed.h"

extern const AP_HAL::HAL &hal;

#define debug_airspeed_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)

AP_Airspeed_UAVCAN::AP_Airspeed_UAVCAN(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
    _sem_airspeed = hal.util->new_semaphore();
}

AP_Airspeed_UAVCAN::~AP_Airspeed_UAVCAN()
{
    if (!_initialized) {
        return;
    }
    
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(_manager);
    if (ap_uavcan == nullptr) {
        return;
    }
    
    ap_uavcan->remove_airspeed_listener(this);
    debug_airspeed_uavcan(2, "AP_Airspeed_UAVCAN destructed\n\r");
    
    delete _sem_airspeed;
}

bool AP_Airspeed_UAVCAN::init()
{
    if (AP_BoardConfig_CAN::get_can_num_ifaces() == 0) {
        return false;
    }
    
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        if (hal.can_mgr[i] == nullptr) {
            continue;
        }
        
        AP_UAVCAN *uavcan = hal.can_mgr[i]->get_UAVCAN();
        if (uavcan == nullptr) {
            continue;
        }
        
        uint8_t freeairspeed = uavcan->find_smallest_free_airspeed_node();
        if (freeairspeed == UINT8_MAX) {
            continue;
        }
        
        if (register_uavcan_airspeed(i, freeairspeed)) {
            debug_airspeed_uavcan(2, "AP_Airspeed_UAVCAN probed, drv: %d, node: %d\n\r", i, freeairspeed);
            return true;
        }
    }
    
    return false;
}

bool AP_Airspeed_UAVCAN::register_uavcan_airspeed(uint8_t mgr, uint8_t node)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return false;
    }
    
    _manager = mgr;
    if (ap_uavcan->register_airspeed_listener_to_node(this, node))
    {
        debug_airspeed_uavcan(2, "AP_Airspeed_UAVCAN loaded\n\r");
        
        _initialized = true;
        return true;
    }
    
    return false;
}

// read the airspeed sensor
bool AP_Airspeed_UAVCAN::get_differential_pressure(float &pressure)
{
    if (!_sem_airspeed->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    pressure = _pressure;
    _sem_airspeed->give();
    return true;
}

bool AP_Airspeed_UAVCAN::get_temperature(float &temperature)
{
    if (!_sem_airspeed->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    temperature = _temperature;
    _sem_airspeed->give();
    return true;
}

void AP_Airspeed_UAVCAN::handle_airspeed_msg(float pressure, float temperature)
{
    if (!_sem_airspeed->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }
    _pressure = pressure;
    _temperature = temperature - 273.15f;
    _sem_airspeed->give();
}

#endif // HAL_WITH_UAVCAN
