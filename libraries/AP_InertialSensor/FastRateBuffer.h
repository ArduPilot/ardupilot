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
#pragma once

#include "AP_InertialSensor.h"

#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED

#define AP_INERTIAL_SENSOR_RATE_LOOP_BUFFER_SIZE 8     // gyro buffer size for rate loop

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/Semaphores.h>

class FastRateBuffer
{
    friend class AP_InertialSensor;
public:
    bool get_next_gyro_sample(Vector3f& gyro);
    uint32_t get_num_gyro_samples() { return _rate_loop_gyro_window.available(); }
    void set_rate_decimation(uint8_t rdec) { rate_decimation = rdec; }
    // whether or not to push the current gyro sample
    bool use_rate_loop_gyro_samples() const { return rate_decimation > 0; }
    bool gyro_samples_available() { return  _rate_loop_gyro_window.available() > 0; }

private:
    /*
      binary semaphore for rate loop to use to start a rate loop when
      we hav finished filtering the primary IMU
     */
    ObjectBuffer<Vector3f> _rate_loop_gyro_window{AP_INERTIAL_SENSOR_RATE_LOOP_BUFFER_SIZE};
    uint8_t rate_decimation; // 0 means off
    uint8_t rate_decimation_count;
    HAL_BinarySemaphore _notifier;
    HAL_Semaphore _mutex;
};
#endif