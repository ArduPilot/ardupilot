// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
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
#include <math.h>

extern const AP_HAL::HAL& hal;

/*
  don't accept periods below 100us as it is probably ringing of the
  signal. It would represent 600k RPM
 */
#define RPM_MIN_PERIOD_US 100

/* 
   open the sensor in constructor
*/
AP_RPM_PX4_PWM::AP_RPM_PX4_PWM(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State &_state) :
	AP_RPM_Backend(_ap_rpm, instance, _state)
{
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

    struct pwm_input_s pwm;
    uint16_t count = 0;
    const float scaling = ap_rpm._scaling[state.instance];
    float sum = 0;

    while (::read(_fd, &pwm, sizeof(pwm)) == sizeof(pwm)) {
        // the px4 pwm_input driver reports the period in microseconds
        if (pwm.period > RPM_MIN_PERIOD_US) {
            sum += (1.0e6f * 60) / pwm.period;
            count++;
        }
    }

    if (count != 0) {
        state.rate_rpm = scaling * sum / count;
        state.last_reading_ms = hal.scheduler->millis();
    }
}

#endif // CONFIG_HAL_BOARD
