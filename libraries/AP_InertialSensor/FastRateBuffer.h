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
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <atomic>
#endif

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
    void reset();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // the gyro samples pushed to the rate loop and the motor-output
    // writes it has made for them, counted so that SITL's lockstep can
    // rendezvous on the outputs for everything pushed rather than on
    // the pop of a sample, which precedes its outputs
    uint32_t rate_loop_push_count() const { return _rate_loop_push_count; }
    uint32_t rate_loop_output_count() const { return _rate_loop_output_count; }
    void count_rate_loop_push() { _rate_loop_push_count++; }
    void count_rate_loop_output() { _rate_loop_output_count++; }
#endif

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
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // written on the main thread and the rate thread respectively, and
    // each read from the other, so atomic rather than plain uint32_t
    std::atomic<uint32_t> _rate_loop_push_count{0};
    std::atomic<uint32_t> _rate_loop_output_count{0};
#endif
};
#endif
