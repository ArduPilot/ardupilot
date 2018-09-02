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

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AP_WheelEncoder/AP_WheelEncoder.h>

// wheel rate control defaults
#define AP_WHEEL_RATE_MAX_DEFAULT   12.0f  // maximum wheel rotation rate in rad/sec (about 115rpm, 687deg/sec)
#define AP_WHEEL_RATE_CONTROL_FF    8.00f
#define AP_WHEEL_RATE_CONTROL_P     2.00f
#define AP_WHEEL_RATE_CONTROL_I     2.00f
#define AP_WHEEL_RATE_CONTROL_IMAX  1.00f
#define AP_WHEEL_RATE_CONTROL_D     0.01f
#define AP_WHEEL_RATE_CONTROL_FILT  0.00f
#define AP_WHEEL_RATE_CONTROL_DT    0.02f
#define AP_WHEEL_RATE_CONTROL_TIMEOUT_MS 200

class AP_WheelRateControl {

public:

    AP_WheelRateControl(const AP_WheelEncoder &wheel_encoder_ref);

    // returns true if a wheel encoder and rate control PID are available for this instance
    bool enabled(uint8_t instance);

    // get throttle output in the range -100 to +100 given a desired rate expressed as a percentage of the rate_max (-100 to +100)
    // instance can be 0 or 1
    float get_rate_controlled_throttle(uint8_t instance, float desired_rate_pct, float dt);

    // get rate maximum in radians/sec
    float get_rate_max_rads() const { return MAX(_rate_max, 0.0f); }

    // get pid objects for reporting
    AC_PID& get_pid(uint8_t instance);

    static const struct AP_Param::GroupInfo        var_info[];

private:

    // parameters
    AP_Int8         _enabled;   // top level enable/disable control
    AP_Float        _rate_max;  // wheel maximum rotation rate in rad/s
    AC_PID          _rate_pid0 = AC_PID(AP_WHEEL_RATE_CONTROL_P, AP_WHEEL_RATE_CONTROL_I, AP_WHEEL_RATE_CONTROL_D, AP_WHEEL_RATE_CONTROL_IMAX, AP_WHEEL_RATE_CONTROL_FILT, AP_WHEEL_RATE_CONTROL_DT, AP_WHEEL_RATE_CONTROL_FF);
    AC_PID          _rate_pid1 = AC_PID(AP_WHEEL_RATE_CONTROL_P, AP_WHEEL_RATE_CONTROL_I, AP_WHEEL_RATE_CONTROL_D, AP_WHEEL_RATE_CONTROL_IMAX, AP_WHEEL_RATE_CONTROL_FILT, AP_WHEEL_RATE_CONTROL_DT, AP_WHEEL_RATE_CONTROL_FF);

    // limit flags
    struct AP_MotorsUGV_limit {
        bool    lower;  // reached this instance's lower limit on last iteration
        bool    upper;  // reached this instance's upper limit on last iteration
    } _limit[2];

    // internal variables
    const AP_WheelEncoder&  _wheel_encoder;     // pointer to accompanying wheel encoder
    uint32_t                _last_update_ms;    // system time of last call to get_rate_controlled_throttle
};
