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

static HAL_Semaphore sensor_sem;
static ScriptingCANSensor *sensors[2];

ScriptingCANSensor *get_scripting_can_sensor(uint8_t index)
{
    if (index >= ARRAY_SIZE(sensors)) {
        return nullptr;
    }
    WITH_SEMAPHORE(sensor_sem);
    if (sensors[index] == nullptr) {
        const auto protocol = index == 0 ? AP_CAN::Protocol::Scripting : AP_CAN::Protocol::Scripting2;
        sensors[index] = NEW_NOTHROW ScriptingCANSensor(protocol);
    }
    return sensors[index];
}

// handler for outgoing frames, using uint32
bool ScriptingCANSensor::write_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us)
{
    return CANSensor::write_frame(out_frame, timeout_us);
};

// handler for incoming frames, add to buffers
void ScriptingCANSensor::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(sem);
    for (ScriptingCANBuffer *current = buffer_list; current != nullptr; current = current->next) {
        current->handle_frame(frame);
    }
}

// add a new buffer to this sensor
ScriptingCANBuffer* ScriptingCANSensor::add_buffer(uint32_t buffer_len)
{
    WITH_SEMAPHORE(sem);
    ScriptingCANBuffer *new_buff = NEW_NOTHROW ScriptingCANBuffer(*this, buffer_len);
    if (new_buff == nullptr) {
        return nullptr;
    }
    new_buff->next = buffer_list;
    buffer_list = new_buff;
    return new_buff;
}

bool ScriptingCANSensor::remove_buffer(ScriptingCANBuffer *buffer)
{
    if (buffer == nullptr) {
        return false;
    }
    WITH_SEMAPHORE(sem);
    ScriptingCANBuffer **link = &buffer_list;
    while (*link != nullptr) {
        if (*link == buffer) {
            *link = buffer->next;
            delete buffer;
            return true;
        }
        link = &(*link)->next;
    }
    return false;
}

// Call main sensor write method
bool ScriptingCANBuffer::write_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us)
{
    return sensor.write_frame(out_frame, timeout_us);
};

// read a frame from the buffer
bool ScriptingCANBuffer::read_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(sem);
    return buffer.pop(frame);
}

// add frame to buffer
void ScriptingCANBuffer::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(sem);

    // accept everything if no filters are setup
    bool accept = num_filters == 0;

    // Check if frame matches any filters
    for (uint8_t i = 0; i < MIN(num_filters, ARRAY_SIZE(filter)); i++) {
        if ((frame.id & filter[i].mask) == filter[i].value) {
            accept = true;
            break;
        }
    }

    // Add to buffer for scripting to read
    if (accept) {
        buffer.push(frame);
    }
}

// Add a filter, will pass ID's that match value given the mask
bool ScriptingCANBuffer::add_filter(uint32_t mask, uint32_t value) {

    WITH_SEMAPHORE(sem);

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
