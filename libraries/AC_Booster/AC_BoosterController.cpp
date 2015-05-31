// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "AC_BoosterController.h"

const AP_Param::GroupInfo AC_BoosterController::var_info[] PROGMEM = {
    // @Param: FWD_PIT
    // @DisplayName: Booster forward pitch
    // @Description: The forward pitch where the booster will take over for fast forward flight.
    // @Range: 0 4500
    // @Units: Centi-degrees
    // @User: Advanced
    AP_GROUPINFO("FWD_PIT", 0, AC_BoosterController, _forward_pitch, BOOSTER_CONTROLLER_FORWARD_PITCH),

    AP_GROUPEND
};

AC_BoosterController::AC_BoosterController (AC_BoosterBackend* booster)
{
    AP_Param::setup_object_defaults(this, var_info);

    _booster = booster;
};

void AC_BoosterController::set_boost (int16_t boost_in)
{
    if (is_active()) {
        _booster->set_boost(boost_in);
    }
}

float AC_BoosterController::set_boost_and_scale_pitch (float pitch_in, float angle_max)
{
    float pitch_out = pitch_in;
    int16_t boost_out = 0;

    calculate_boost_and_scale_pitch (pitch_out, angle_max, boost_out);

    set_boost(boost_out);

    return pitch_out;
}

void AC_BoosterController::calculate_boost_and_scale_pitch(float &pitch, float angle_max, int16_t &boost)
{
    // if the requested pitch is greater than the configured forward pitch, calculate
    // the boost to substitute the excess pitch

    if (is_active() && pitch < -_forward_pitch) {
        boost = abs(1000 * (pitch + _forward_pitch)/angle_max);
        pitch = -_forward_pitch;
    } else {
        boost = 0;
    }
}

void AC_BoosterController::set_active (bool active)
{
    // always set boost to zero when deactivating

    if (!active) {
      set_boost (0);
    }

    _active = active;
}

bool AC_BoosterController::is_active ()
{
    if (_booster == NULL || !_booster->is_enabled()) {
        return false;
    }

    return _active;
}