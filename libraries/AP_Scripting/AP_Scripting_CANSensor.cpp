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
#include "AP_Scripting_CANSensor.h"
#include <AP_Math/AP_Math.h>

#if AP_SCRIPTING_CAN_SENSOR_ENABLED

// handler for outgoing frames, using uint32
bool ScriptingCANSensor::write_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us)
{
    return CANSensor::write_frame(out_frame, timeout_us);
};

// handler for incoming frames, add to buffers
void ScriptingCANSensor::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(sem);
    if (buffer_list != nullptr) {
        buffer_list->handle_frame(frame);
    }
}

// add a new buffer to this sensor
ScriptingCANBuffer* ScriptingCANSensor::add_buffer(uint32_t buffer_len)
{
    WITH_SEMAPHORE(sem);
    ScriptingCANBuffer *new_buff = NEW_NOTHROW ScriptingCANBuffer(*this, buffer_len);
    if (buffer_list == nullptr) {
        buffer_list = new_buff;
    } else {
        buffer_list->add_buffer(new_buff);
    }
    return new_buff;
}

// Call main sensor write method
bool ScriptingCANBuffer::write_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us)
{
    return sensor.write_frame(out_frame, timeout_us);
};

// read a frame from the buffer
bool ScriptingCANBuffer::read_frame(AP_HAL::CANFrame &frame)
{
    return buffer.pop(frame);
}

// recursively add frame to buffer
void ScriptingCANBuffer::handle_frame(AP_HAL::CANFrame &frame)
{
    // accept everything if no filters are setup
    bool accept = num_filters == 0;

    // Check if frame matches any filters
    for (uint8_t i = 0; i < MIN(num_filters, ARRAY_SIZE(filter)); i++) {
        if ((frame.id & filter[i].mask) == filter[i].value) {
            accept = true;
            break;
        }
    }

    WITH_SEMAPHORE(sem);

    // Add to buffer for scripting to read
    if (accept) {
        buffer.push(frame);
    }

    // filtering is not applied to other buffers
    if (next != nullptr) {
        next->handle_frame(frame);
    }
}

// recursively add new buffer
void ScriptingCANBuffer::add_buffer(ScriptingCANBuffer* new_buff) {
    WITH_SEMAPHORE(sem);
    if (next == nullptr) {
        next = new_buff;
        return;
    }
    next->add_buffer(new_buff);
}

// Add a filter, will pass ID's that match value given the mask
bool ScriptingCANBuffer::add_filter(uint32_t mask, uint32_t value) {

    // Run out of filters
    if (num_filters >= ARRAY_SIZE(filter)) {
        return false;
    }

    // Add to list and increment
    filter[num_filters].mask = mask;
    filter[num_filters].value = value & mask;
    num_filters++;
    return true;
}

#endif // AP_SCRIPTING_CAN_SENSOR_ENABLED
