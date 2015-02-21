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

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_RangeFinder_PX4_PWM.h"

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
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_PX4_PWM::AP_RangeFinder_PX4_PWM(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
	AP_RangeFinder_Backend(_ranger, instance, _state),
    _last_timestamp(0),
    _last_pulse_time_ms(0),
    _disable_time_ms(0),
    _good_sample_count(0),
    _last_sample_distance_cm(0)
{
    state.healthy = false;

    _fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
    if (_fd == -1) {
        hal.console->printf("Unable to open PX4 PWM rangefinder\n");
        return;
    }

    // keep a queue of 20 samples
    if (ioctl(_fd, SENSORIOCSQUEUEDEPTH, 20) != 0) {
        hal.console->printf("Failed to setup range finder queue\n");
        return;
    }
}

/* 
   close the file descriptor
*/
AP_RangeFinder_PX4_PWM::~AP_RangeFinder_PX4_PWM()
{
    if (_fd != -1) {
        close(_fd);
    }
}

/* 
   see if the PX4 driver is available
*/
bool AP_RangeFinder_PX4_PWM::detect(RangeFinder &_ranger, uint8_t instance)
{
    int fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
    if (fd == -1) {
        return false;
    }
    close(fd);
    return true;
}

void AP_RangeFinder_PX4_PWM::update(void)
{
    if (_fd == -1) {
        return;
    }

    struct pwm_input_s pwm;
    float sum_cm = 0;
    uint16_t count = 0;
    const float scaling = ranger._scaling[state.instance];
    uint32_t now = hal.scheduler->millis();

    while (::read(_fd, &pwm, sizeof(pwm)) == sizeof(pwm)) {
        // report the voltage as the pulse width, so we get the raw
        // pulse widths in the log
        state.voltage_mv = pwm.pulse_width;

        _last_pulse_time_ms = now;

        // setup for scaling in meters per millisecond
        float distance_cm = pwm.pulse_width * 0.1f * scaling;

        if (distance_cm > ranger._max_distance_cm[state.instance] ||
            distance_cm < ranger._min_distance_cm[state.instance]) {
            _good_sample_count = 0;
            continue;
        }
        
        float distance_delta_cm = fabsf(distance_cm - _last_sample_distance_cm);
        _last_sample_distance_cm = distance_cm;

        if (distance_delta_cm > 100) {
            // varying by more than 1m in a single sample, which means
            // between 50 and 100m/s vertically - discard
            _good_sample_count = 0;
            continue;
        }

        if (_good_sample_count > 1) {
            count++;
            sum_cm += distance_cm;
            _last_timestamp = pwm.timestamp;
        } else {
            _good_sample_count++;
        }
    }

    // if we haven't received a pulse for 1 second then we may need to
    // reset the timer
    int8_t stop_pin = ranger._stop_pin[state.instance];
    uint16_t settle_time_ms = (uint16_t)ranger._settle_time_ms[state.instance];

    if (stop_pin != -1 && out_of_range()) {
        // we are above the power saving range. Disable the sensor
        hal.gpio->pinMode(stop_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(stop_pin, false);
        state.healthy = false;
        state.distance_cm = 0;
        state.voltage_mv = 0;
        return;
    }

    // consider the range finder healthy if we got a reading in the last 0.2s
    state.healthy = (hal.scheduler->micros64() - _last_timestamp < 200000);

    /* if we haven't seen any pulses for 0.5s then the sensor is
       probably dead. Try resetting it. Tests show the sensor takes
       about 0.2s to boot, so 500ms offers some safety margin
    */
    if (now - _last_pulse_time_ms > 500U && _disable_time_ms == 0) {
        ioctl(_fd, SENSORIOCRESET, 0);
        _last_pulse_time_ms = now;

        // if a stop pin is configured then disable the sensor for the
        // settle time
        if (stop_pin != -1) {
            hal.gpio->pinMode(stop_pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(stop_pin, false);            
            _disable_time_ms = now;
        }
    }

    /* the user can configure a settle time. This controls how 
       long the sensor is disabled for using the stop pin when it is
       reset. This is used both to make sure the sensor is properly
       reset, and also to allow for power management by running a low
       duty cycle when it has no signal 
    */
    if (stop_pin != -1 && _disable_time_ms != 0 && 
        (now - _disable_time_ms > settle_time_ms)) {
        hal.gpio->write(stop_pin, true);        
        _disable_time_ms = 0;
        _last_pulse_time_ms = now;
    }

    if (count != 0) {
        state.distance_cm = sum_cm / count;
    }
}

#endif // CONFIG_HAL_BOARD
