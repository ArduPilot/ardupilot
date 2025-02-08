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

#include <AP_AHRS/AP_AHRS.h>
#include "AP_InertialSensor_rate_config.h"
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
#include "FastRateBuffer.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
// hal.console can be accessed from bus threads on ChibiOS
#define debug(fmt, args ...)  do {hal.console->printf("IMU: " fmt "\n", ## args); } while(0)
#else
#define debug(fmt, args ...)  do {printf("IMU: " fmt "\n", ## args); } while(0)
#endif

// enable the fast rate buffer and start pushing samples to it
void AP_InertialSensor::enable_fast_rate_buffer()
{
    if (fast_rate_buffer_enabled) {
        return;
    }
    if (fast_rate_buffer == nullptr) {
        fast_rate_buffer = NEW_NOTHROW FastRateBuffer();
    }
    fast_rate_buffer_enabled = fast_rate_buffer != nullptr;
}

// disable the fast rate buffer and stop pushing samples to it
void AP_InertialSensor::disable_fast_rate_buffer()
{
    fast_rate_buffer_enabled = false;
    if (fast_rate_buffer != nullptr) {
        fast_rate_buffer->reset();
    }
}

// get the number of available gyro samples in the fast rate buffer
uint32_t AP_InertialSensor::get_num_gyro_samples()
{
    if (!fast_rate_buffer_enabled || fast_rate_buffer == nullptr) {
        return 0;
    }
    return fast_rate_buffer->get_num_gyro_samples();
}

// set the rate at which samples are collected, unused samples are dropped
void AP_InertialSensor::set_rate_decimation(uint8_t rdec)
{
    if (!fast_rate_buffer_enabled || fast_rate_buffer == nullptr) {
        return;
    }
    fast_rate_buffer->set_rate_decimation(rdec);
}

// whether or not to push the current gyro sample
bool AP_InertialSensor::is_rate_loop_gyro_enabled(uint8_t instance) const
{
    if (!fast_rate_buffer_enabled || fast_rate_buffer == nullptr) {
        return false;
    }
    return fast_rate_buffer->use_rate_loop_gyro_samples() && instance == AP::ahrs().get_primary_gyro_index();
}

// whether or not to use the dynamic fifo
bool AP_InertialSensor::is_dynamic_fifo_enabled(uint8_t instance) const
{
    if (!fast_rate_buffer_enabled || fast_rate_buffer == nullptr) {
        return false;
    }
    return (_fast_sampling_mask & (1U<<instance)) != 0
            && fast_rate_buffer->use_rate_loop_gyro_samples();
}

// get the next available gyro sample from the fast rate buffer
bool AP_InertialSensor::get_next_gyro_sample(Vector3f& gyro)
{
    if (!fast_rate_buffer_enabled || fast_rate_buffer == nullptr) {
        return false;
    }

    return fast_rate_buffer->get_next_gyro_sample(gyro);
}


bool FastRateBuffer::get_next_gyro_sample(Vector3f& gyro)
{
    if (!use_rate_loop_gyro_samples()) {
        return false;
    }

    if (_rate_loop_gyro_window.available() == 0) {
        _notifier.wait_blocking();
    }

    WITH_SEMAPHORE(_mutex);

    return _rate_loop_gyro_window.pop(gyro);
}

void FastRateBuffer::reset()
{
    _rate_loop_gyro_window.clear();
}

bool AP_InertialSensor::push_next_gyro_sample(const Vector3f& gyro)
{
    if (!fast_rate_buffer_enabled || fast_rate_buffer == nullptr) {
        return false;
    }

    if (++fast_rate_buffer->rate_decimation_count < fast_rate_buffer->rate_decimation) {
        return false;
    }
    /*
        tell the rate thread we have a new sample
    */
    WITH_SEMAPHORE(fast_rate_buffer->_mutex);

    if (!fast_rate_buffer->_rate_loop_gyro_window.push(gyro)) {
        debug("dropped rate loop sample");
    }
    fast_rate_buffer->rate_decimation_count = 0;
    fast_rate_buffer->_notifier.signal();
    return true;
}

void AP_InertialSensor::update_backend_filters()
{
    for (uint8_t i=0; i<_backend_count; i++) {
        _backends[i]->update_filters();
    }
}

void AP_InertialSensor_Backend::update_filters()
{
    WITH_SEMAPHORE(_sem);

    update_accel_filters(accel_instance);
    update_gyro_filters(gyro_instance);
}

#endif // AP_INERTIALSENSOR_RATE_LOOP_WINDOW_ENABLED
