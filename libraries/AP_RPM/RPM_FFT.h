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

#include <AP_Math/AP_Math.h>
#include "AP_RPM.h"
#include "RPM_Backend.h"

#define RPM_FFT_WIDTH 256

class AP_RPM_FFT : public AP_RPM_Backend
{
public:
    // constructor
    AP_RPM_FFT(AP_RPM &ranger, uint8_t instance, AP_RPM::RPM_State &_state);

    // update state
    void update(void);

private:

    void fast_timer_update();
    void slow_timer_update();

    uint8_t instance;

    uint32_t last_imu_sample_us;

    // rpm estimator variables
    arm_cfft_radix4_instance_f32 * fft = new arm_cfft_radix4_instance_f32;
    float dt;

    // fft data
    uint16_t nsamples;
    HAL_Semaphore sem;

    float new_rpm;
    bool have_new_rpm;

    float fft_buffer[2 * RPM_FFT_WIDTH];
};
