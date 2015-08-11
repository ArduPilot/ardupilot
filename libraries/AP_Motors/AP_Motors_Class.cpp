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

/*
 *       AP_Motors.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include "AP_Motors_Class.h"
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

// initialise motor map
    const uint8_t AP_Motors::_motor_to_channel_map[AP_MOTORS_MAX_NUM_MOTORS] PROGMEM = {MOTOR_TO_CHANNEL_MAP};

// Constructor
AP_Motors::AP_Motors(uint16_t loop_rate, uint16_t speed_hz) :
    _roll_control_input(0.0f),
    _pitch_control_input(0.0f),
    _throttle_control_input(0.0f),
    _yaw_control_input(0.0f),
    _throttle_pwm_scalar(1.0f),
    _rpy_pwm_scalar(0.074f),
    _loop_rate(loop_rate),
    _speed_hz(speed_hz),
    _throttle_radio_min(1100),
    _throttle_radio_max(1900),
    _throttle_in(0.0f),
    _throttle_filter(),
    _batt_voltage(0.0f),
    _batt_current(0.0f),
    _air_density_ratio(1.0f)
{
    // init other flags
    _flags.armed = false;
    _flags.stabilizing = false;
    _flags.frame_orientation = AP_MOTORS_X_FRAME;
    _flags.interlock = false;

    // setup throttle filtering
    _throttle_filter.set_cutoff_frequency(0.0f);
    _throttle_filter.reset(0.0f);

    // init limit flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = true;
};

void AP_Motors::armed(bool arm)
{
    _flags.armed = arm;
    AP_Notify::flags.armed = arm;
};
