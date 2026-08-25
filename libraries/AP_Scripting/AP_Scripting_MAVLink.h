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
  Scripting MAVLink class, for easy scripting MAVLink support
 */
 
#pragma once

#include "AP_Scripting/AP_Scripting.h"
#include <AP_HAL/AP_HAL.h>

//#if defined(HAL_BUILD_AP_PERIPH)
//    // Must have at least two CAN ports on Periph
//    #define AP_SCRIPTING_CAN_SENSOR_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS > 1)
//#else
//    #define AP_SCRIPTING_CAN_SENSOR_ENABLED HAL_MAX_CAN_PROTOCOL_DRIVERS
//#endif

//#if AP_SCRIPTING_CAN_SENSOR_ENABLED

#include <AP_CANManager/AP_CANSensor.h>

class ScriptingMAVLinkBuffer {
public: ScriptingMAVLinkBuffer(uint32_t buffer_size):
        buffer(buffer_size)
    {};

    // read a messages from the buffer
    bool read_msg(AP_Scripting::mavlink_msg &msg);

    // recursively add messages to buffer
    void handle_msg(AP_Scripting::mavlink_msg &msg);

    // recursively add new buffer
    void add_buffer(ScriptingMAVLinkBuffer* new_buff);

    // Add a filter to this buffer
    // bool add_filter(uint32_t mask, uint32_t value);

private:

    ObjectBuffer<AP_Scripting::mavlink_msg> buffer;

    ScriptingMAVLinkBuffer *next;

    HAL_Semaphore sem;

    /*
    struct {
        uint32_t mask;
        uint32_t value;
    } filter[8];
    uint8_t num_filters;
    */

};

//#endif // AP_SCRIPTING_CAN_SENSOR_ENABLED
