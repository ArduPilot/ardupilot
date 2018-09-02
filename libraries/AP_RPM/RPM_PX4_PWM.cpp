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

#if (CONFIG_HAL_BOARD == HAL_BOARD_PX4) || ((CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN) && (!defined(CONFIG_ARCH_BOARD_VRBRAIN_V51) && !defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)))
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "RPM_PX4_PWM.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_pwm_input.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <uORB/topics/pwm_input.h>
#include <stdio.h>
#include <errno.h>
#include <cmath>

#define PWM_LOGGING 0

extern const AP_HAL::HAL& hal;

extern "C" {
    int pwm_input_main(int, char **);
};

/* 
   open the sensor in constructor
*/
AP_RPM_PX4_PWM::AP_RPM_PX4_PWM(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State &_state) :
	AP_RPM_Backend(_ap_rpm, instance, _state)
{
#if HAL_PX4_HAVE_PWM_INPUT
    if (AP_BoardConfig::px4_start_driver(pwm_input_main, "pwm_input", "start")) {
        hal.console->printf("started pwm_input driver\n");
    }
#endif
    _fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
    if (_fd == -1) {
        hal.console->printf("Unable to open %s\n", PWMIN0_DEVICE_PATH);
        return;
    }

    // keep a queue of 5 samples to reduce noise by averaging
    if (ioctl(_fd, SENSORIOCSQUEUEDEPTH, 5) != 0) {
        hal.console->printf("Failed to setup RPM queue\n");
        close(_fd);
        _fd = -1;
        return;
    }

    _resolution_usec = PWMIN_MINRPM_TO_RESOLUTION(((uint32_t)(ap_rpm._minimum[state.instance]+0.5f)));
    ioctl(_fd, PWMINIOSRESOLUTION, _resolution_usec);
    
#if PWM_LOGGING
    _logfd = open("/fs/microsd/pwm.log", O_WRONLY|O_CREAT|O_TRUNC, 0644);
#endif
}

/* 
   close the file descriptor
*/
AP_RPM_PX4_PWM::~AP_RPM_PX4_PWM()
{
    if (_fd != -1) {
        close(_fd);
        _fd = -1;
    }
}

void AP_RPM_PX4_PWM::update(void)
{
    if (_fd == -1) {
        return;
    }

    uint32_t newres = PWMIN_MINRPM_TO_RESOLUTION(((uint32_t)(ap_rpm._minimum[state.instance]+0.5f)));
    if (newres != _resolution_usec) {
        ioctl(_fd, PWMINIOSRESOLUTION, newres);
        _resolution_usec = newres;
    }
    
    struct pwm_input_s pwm;
    uint16_t count = 0;
    const float scaling = ap_rpm._scaling[state.instance];
    float maximum = ap_rpm._maximum[state.instance];
    float minimum = ap_rpm._minimum[state.instance];
    float quality = 0;

    while (::read(_fd, &pwm, sizeof(pwm)) == sizeof(pwm)) {
        // the px4 pwm_input driver reports the period in microseconds
        if (pwm.period == 0) {
            continue;
        }
        float rpm = scaling * (1.0e6f * 60) / pwm.period;
        float filter_value = signal_quality_filter.get();
        state.rate_rpm = signal_quality_filter.apply(rpm);
        if ((maximum <= 0 || rpm <= maximum) && (rpm >= minimum)) {
            if (is_zero(filter_value)){
                quality = 0;
            } else {
                quality = 1 - constrain_float((fabsf(rpm-filter_value))/filter_value, 0.0, 1.0);
                quality = powf(quality, 2.0);
            }
            count++;
        } else {
            quality = 0;
        }

#if PWM_LOGGING
        if (_logfd != -1) {
            dprintf(_logfd, "%u %u %u\n",
                    (unsigned)pwm.timestamp/1000,
                    (unsigned)pwm.period,
                    (unsigned)pwm.pulse_width);
        }
#endif

        state.signal_quality = (0.1 * quality) + (0.9 * state.signal_quality);      // simple LPF
    }

    if (count != 0) {
        state.last_reading_ms = AP_HAL::millis();
    }

    // assume we get readings at at least 1Hz, otherwise reset quality to zero
    if (AP_HAL::millis() - state.last_reading_ms > 1000) {
        state.signal_quality = 0;
    }
}

#endif // CONFIG_HAL_BOARD
