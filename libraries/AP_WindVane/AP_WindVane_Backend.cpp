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

#include "AP_WindVane.h"
#include "AP_WindVane_Backend.h"

// base class constructor.
AP_WindVane_Backend::AP_WindVane_Backend(AP_WindVane &frontend) :
        _frontend(frontend)
{
}

// update speed to frontend
void AP_WindVane_Backend::speed_update_frontend(float apparent_speed_in)
{
    // apply low pass filter if enabled
    if (is_positive(_frontend._speed_filt_hz)) {
        _speed_filt.set_cutoff_frequency(_frontend._speed_filt_hz);
        _frontend._speed_apparent = _speed_filt.apply(apparent_speed_in, 0.02f);
    } else {
        _frontend._speed_apparent = apparent_speed_in;
    }
}

// update direction to frontend
void AP_WindVane_Backend::direction_update_frontend(float apparent_angle_ef)
{
    // apply low pass filter if enabled
    if (is_positive(_frontend._dir_filt_hz)) {
        _dir_sin_filt.set_cutoff_frequency(_frontend._dir_filt_hz);
        _dir_cos_filt.set_cutoff_frequency(_frontend._dir_filt_hz);
        // https://en.wikipedia.org/wiki/Mean_of_circular_quantities
        const float filtered_sin = _dir_sin_filt.apply(sinf(apparent_angle_ef), 0.05f);
        const float filtered_cos = _dir_cos_filt.apply(cosf(apparent_angle_ef), 0.05f);
        _frontend._direction_apparent_ef = atan2f(filtered_sin, filtered_cos);
    } else {
        _frontend._direction_apparent_ef = apparent_angle_ef;
    }

    // make sure between -pi and pi
    _frontend._direction_apparent_ef = wrap_PI(_frontend._direction_apparent_ef);
}

// calibrate WindVane
void AP_WindVane_Backend::calibrate()
{
    gcs().send_text(MAV_SEVERITY_INFO, "WindVane: No cal required");
    _frontend._calibration.set_and_save(0);
    return;
}
