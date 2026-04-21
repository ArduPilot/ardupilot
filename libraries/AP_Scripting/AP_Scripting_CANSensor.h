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
  Scripting CANSensor class, for easy scripting CAN support
 */
 
#pragma once

#include <AP_HAL/AP_HAL.h>

#if defined(HAL_BUILD_AP_PERIPH)
    // Must have at least two CAN ports on Periph
    #define AP_SCRIPTING_CAN_SENSOR_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS > 1)
#else
    #define AP_SCRIPTING_CAN_SENSOR_ENABLED HAL_MAX_CAN_PROTOCOL_DRIVERS
#endif

#if AP_SCRIPTING_CAN_SENSOR_ENABLED

#include <AP_CANManager/AP_CANSensor.h>

class ScriptingCANBuffer;
class ScriptingCANSensor : public CANSensor {
public:

    ScriptingCANSensor(AP_CAN::Protocol dtype)
        : CANSensor("Script") {
        register_driver(dtype);
    }

    // handler for outgoing frames, using uint32
    bool write_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us);

    // handler for incoming frames, add to buffers
    void handle_frame(AP_HAL::CANFrame &frame) override;

    // add a new buffer to this sensor
    ScriptingCANBuffer* add_buffer(uint32_t buffer_len);

private:

    HAL_Semaphore sem;

    ScriptingCANBuffer *buffer_list;

};

class ScriptingCANBuffer {
public:

    ScriptingCANBuffer(ScriptingCANSensor &_sensor, uint32_t buffer_size):
        buffer(buffer_size),
        sensor(_sensor)
    {};

    // Call main sensor write method
    bool write_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us);

    // read a frame from the buffer
    bool read_frame(AP_HAL::CANFrame &frame);

    // recursively add frame to buffer
    void handle_frame(AP_HAL::CANFrame &frame);

    // recursively add new buffer
    void add_buffer(ScriptingCANBuffer* new_buff);

    // Add a filter to this buffer
    bool add_filter(uint32_t mask, uint32_t value);

private:

    ObjectBuffer<AP_HAL::CANFrame> buffer;

    ScriptingCANSensor &sensor;

    ScriptingCANBuffer *next;

    HAL_Semaphore sem;

    struct {
        uint32_t mask;
        uint32_t value;
    } filter[8];
    uint8_t num_filters;

};

#endif // AP_SCRIPTING_CAN_SENSOR_ENABLED
