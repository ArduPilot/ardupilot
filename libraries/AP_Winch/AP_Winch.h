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

// winch rate control default gains
#define AP_WINCH_POS_P      1.00f
#define AP_WINCH_RATE_P     1.00f
#define AP_WINCH_RATE_I     0.50f
#define AP_WINCH_RATE_IMAX  1.00f
#define AP_WINCH_RATE_D     0.00f
#define AP_WINCH_RATE_FILT  5.00f
#define AP_WINCH_RATE_DT    0.10f

class AP_Winch_Backend;

class AP_Winch {
    friend class AP_Winch_Backend;
    friend class AP_Winch_Servo;

public:
    AP_Winch();

    // indicate whether this module is enabled
    bool enabled() const;

    // initialise the winch
    void init(const AP_WheelEncoder *wheel_encoder = nullptr);

    // update the winch
    void update();

    // relax the winch so it does not attempt to maintain length or rate
    void relax() { config.state = STATE_RELAXED; }

    // get current line length
    float get_line_length() const { return config.length_curr; }

    // release specified length of cable (in meters) at the specified rate
    // if rate is zero, the RATE_MAX parameter value will be used
    void release_length(float length, float rate = 0.0f);

    // deploy line at specified speed in m/s (+ve deploys line, -ve retracts line, 0 stops)
    void set_desired_rate(float rate);

    // get rate maximum in m/s
    float get_rate_max() const { return MAX(config.rate_max, 0.0f); }

    static const struct AP_Param::GroupInfo        var_info[];

private:

    // parameters
    AP_Int8     _enabled;               // grabber enable/disable

    // winch states
    typedef enum {
        STATE_RELAXED = 0,              // winch is not operating
        STATE_POSITION,                 // moving or maintaining a target length
        STATE_RATE,                     // deploying or retracting at a target rate
    }  winch_state;

    struct Backend_Config {
        AP_Int8     type;               // winch type
        AP_Float    rate_max;           // deploy or retract rate maximum (in m/s).
        AP_Float    pos_p;              // position error P gain
        AC_PID      rate_pid = AC_PID(AP_WINCH_RATE_P, AP_WINCH_RATE_I, AP_WINCH_RATE_D, AP_WINCH_RATE_IMAX, AP_WINCH_RATE_FILT, AP_WINCH_RATE_DT);           // rate control PID
        winch_state state;              // state of winch control (using target position or target rate)
        float       length_curr;        // current length of the line (in meters) that has been deployed
        float       length_desired;     // target desired length (in meters)
        float       rate_desired;       // target deploy rate (in m/s, +ve = deploying, -ve = retracting)
    } config;

    AP_Winch_Backend *backend;
};
