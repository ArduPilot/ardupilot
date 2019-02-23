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

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include "RPM_FFT.h"

extern const AP_HAL::HAL& hal;

/* 
   open the sensor in constructor
*/
AP_RPM_FFT::AP_RPM_FFT(AP_RPM &_ap_rpm, uint8_t _instance, AP_RPM::RPM_State &_state) :
    AP_RPM_Backend(_ap_rpm, _instance, _state)
{
    instance = _instance;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_RPM_FFT::fast_timer_update, void));
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_RPM_FFT::slow_timer_update, void));
}


/*
  timer function called at 1kHz
*/
void AP_RPM_FFT::fast_timer_update(void)
{
    if (nsamples < ARRAY_SIZE(fft_buffer)) {
        const AP_InertialSensor &ins = AP::ins();
        uint32_t imu_sample_us = ins.get_last_update_usec();
        if (imu_sample_us == last_imu_sample_us) {
            // no new sample from INS
            return;
        }
        dt = 1.0e-6f * (imu_sample_us - last_imu_sample_us);
        last_imu_sample_us = imu_sample_us;

        const Vector3f &accel = ins.get_accel(0);
        // collecting accel data but added a stronger oscillatory component
        // sets real part of complex input data
        fft_buffer[nsamples] = accel.y + 0.2 * sinf(6.28e-6f*40.0f * dt * nsamples/2);
        nsamples++;
        // sets imaginary part of complex input data
        fft_buffer[nsamples] = 0.0f;
        nsamples++;
    }
}

/*
  IO timer function called at low priority to calculate FFT
*/
void AP_RPM_FFT::slow_timer_update(void)
{
    uint16_t max_f;

    if (nsamples != ARRAY_SIZE(fft_buffer)) {
        // not ready yet
        return;
    } 
    if (arm_cfft_radix4_init_f32( fft, (uint16_t)RPM_FFT_WIDTH, 0, 1) == ARM_MATH_SUCCESS) {
        arm_cfft_radix4_f32( fft , fft_buffer);
        //find first peak above a threshold
        float max_value = 0.0f;
        max_f = 0;
        bool first_max = false;
        bool thrsh = false;
        uint16_t j = 0;
        for (uint16_t i = 0; i < 2*RPM_FFT_WIDTH ; i+=2) { 
            cfft[j]= sqrtf(fft_buffer[i]*fft_buffer[i] + fft_buffer[i+1]*fft_buffer[i+1]);
            if (cfft[j] > 0.1f) {
                thrsh = true;
            } else {
                thrsh = false;
            }
            if (thrsh && !first_max && i != 0) {
                if (cfft[j] > max_value) {
                    max_value = cfft[j];
                    max_f = j;
                }                
            } else if (!thrsh && max_f != 0) {
                first_max = true;
            }
            j++;
        }  
    } else {
        max_f = 0;
    }
//    float sample_rate = 1.0f / dt;
//    new_rpm = 60.0f * sample_rate * (float)max_f * 2.0f / ((float)RPM_FFT_WIDTH - 1.0f);   
 
    WITH_SEMAPHORE(sem);
    new_rpm = 60.0f *  400.0f * (float)max_f / ((float)RPM_FFT_WIDTH - 1.0f);
    have_new_rpm = true;
    // reset nsamples
    nsamples = 0;
}

void AP_RPM_FFT::update(void)
{
    WITH_SEMAPHORE(sem);
    if (have_new_rpm) {
        state.rate_rpm = new_rpm * ap_rpm._scaling[state.instance];
        state.signal_quality = 0.5f;
        state.last_reading_ms = AP_HAL::millis();
        have_new_rpm = false;
    }
}
