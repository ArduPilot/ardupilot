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

#include "AP_VisualOdom_Backend.h"

#if HAL_VISUALODOM_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_VisualOdom_Backend::AP_VisualOdom_Backend(AP_VisualOdom &frontend) :
    _frontend(frontend)
{
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_VisualOdom_Backend::healthy() const
{
    // healthy if we have received sensor messages within the past 300ms
    return ((AP_HAL::millis() - _last_update_ms) < AP_VISUALODOM_TIMEOUT_MS);
}

void AP_VisualOdom_Backend::handle_vision_position_delta_estimate(const uint64_t remote_time_us, const uint32_t time_ms, const uint64_t time_delta_usec,
                                    const Vector3f &angle_delta, const Vector3f &position_delta, const float confidence)
{
    Vector3f angle_d = angle_delta;
    Vector3f position_d = position_delta;
    
    // apply rotation to angle and position delta
    const enum Rotation rot = _frontend.get_orientation();
    angle_d.rotate(rot);
    position_d.rotate(rot);

    // send to EKF
    const float time_delta_sec = time_delta_usec / 1000000.0f;
    AP::ahrs_navekf().writeBodyFrameOdom(confidence,
                                         position_d,
                                         angle_d,
                                         time_delta_sec,
                                         time_ms,
                                         _frontend.get_delay_ms(),
                                         _frontend.get_pos_offset());

    // record time for health monitoring
    _last_update_ms = AP_HAL::millis();

    // log sensor data
    Write_VisualOdom(time_delta_sec,
                                  angle_d,
                                  position_d,
                                  confidence);
}

// returns the system time of the last reset if reset_counter has not changed
// updates the reset timestamp to the current system time if the reset_counter has changed
uint32_t AP_VisualOdom_Backend::get_reset_timestamp_ms(uint8_t reset_counter)
{
    // update reset counter and timestamp if reset_counter has changed
    if (reset_counter != _last_reset_counter) {
        _last_reset_counter = reset_counter;
        _reset_timestamp_ms = AP_HAL::millis();
    }
    return _reset_timestamp_ms;
}

#endif
