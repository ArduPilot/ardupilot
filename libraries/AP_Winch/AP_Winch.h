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

#include "AP_Winch_config.h"

#if AP_WINCH_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AP_Logger/AP_Logger_config.h>

class AP_Winch_Backend;

class AP_Winch {
    friend class AP_Winch_Backend;
    friend class AP_Winch_PWM;
    friend class AP_Winch_Daiwa;

public:
    AP_Winch();

    // Do not allow copies
    CLASS_NO_COPY(AP_Winch);

    // indicate whether this module is enabled
    bool enabled() const;

    // true if winch is healthy
    bool healthy() const;

    // initialise the winch
    void init();

    // update the winch
    void update();

    // relax the winch so it does not attempt to maintain length or rate
    void relax() { config.control_mode = ControlMode::RELAXED; }

    // release specified length of cable (in meters)
    void release_length(float length);

    // deploy line at specified speed in m/s (+ve deploys line, -ve retracts line, 0 stops)
    void set_desired_rate(float rate);

    // get rate maximum in m/s
    float get_rate_max() const { return MAX(config.rate_max, 0.0f); }

    // send status to ground station
    void send_status(const class GCS_MAVLINK &channel);

#if HAL_LOGGING_ENABLED
    // write log
    void write_log();
#endif

    // returns true if pre arm checks have passed
    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const;

    static AP_Winch *get_singleton();

    static const struct AP_Param::GroupInfo        var_info[];

private:

    enum class WinchType {
        NONE = 0,
        PWM = 1,
        DAIWA = 2
    };

    // enum for OPTIONS parameter
    enum class Options : int16_t {
        SpinFreelyOnStartup = (1U << 0),    // winch allows line to be manually pulled out soon after startup
        VerboseOutput = (1U << 1),          // verbose output of winch state sent to GCS
        RetryIfStuck = (1U << 2),           // retries to raise or lower if winch stops
    };

    // winch states
    enum class ControlMode : uint8_t {
        RELAXED = 0,    // winch is realxed
        POSITION,       // moving or maintaining a target length (from an external source)
        RATE,           // extending or retracting at a target rate (from an external source)
        RATE_FROM_RC    // extending or retracting at a target rate (from RC input)
    };

    struct Backend_Config {
        AP_Int8     type;               // winch type
        AP_Float    rate_max;           // deploy or retract rate maximum (in m/s).
        AP_Float    pos_p;              // position error P gain
        AP_Int16    options;            // options bitmask
        ControlMode control_mode;       // state of winch control (using target position or target rate)
        float       length_desired;     // target desired length (in meters)
        float       rate_desired;       // target deploy rate (in m/s, +ve = deploying, -ve = retracting)
    } config;

    AP_Winch_Backend *backend;

    static AP_Winch *_singleton;
};

namespace AP {
    AP_Winch *winch();
};

#endif  // AP_WINCH_ENABLED
