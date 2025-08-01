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

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

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

#if HAL_GCS_ENABLED
// consume vision_position_delta mavlink messages
void AP_VisualOdom_Backend::handle_vision_position_delta_msg(const mavlink_message_t &msg)
{
    // decode message
    mavlink_vision_position_delta_t packet;
    mavlink_msg_vision_position_delta_decode(&msg, &packet);

    // apply rotation to angle and position delta
    const enum Rotation rot = _frontend.get_orientation();
    Vector3f angle_delta = Vector3f(packet.angle_delta[0], packet.angle_delta[1], packet.angle_delta[2]);
    angle_delta.rotate(rot);
    Vector3f position_delta = Vector3f(packet.position_delta[0], packet.position_delta[1], packet.position_delta[2]);
    position_delta.rotate(rot);

    const uint32_t now_ms = AP_HAL::millis();
    _last_update_ms = now_ms;

    // send to EKF
#if AP_AHRS_ENABLED || HAL_LOGGING_ENABLED
    const float time_delta_sec = packet.time_delta_usec * 1.0E-6;
#endif
#if AP_AHRS_ENABLED
    AP::ahrs().writeBodyFrameOdom(packet.confidence,
                                  position_delta,
                                  angle_delta,
                                  time_delta_sec,
                                  now_ms,
                                  _frontend.get_delay_ms(),
                                  _frontend.get_pos_offset());
#endif

#if HAL_LOGGING_ENABLED
    // log sensor data
    Write_VisualOdom(time_delta_sec,
                                  angle_delta,
                                  position_delta,
                                  packet.confidence);
#endif
}
#endif  // HAL_GCS_ENABLED

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

// align position with ahrs position by updating _pos_correction
// sensor_pos should be the position directly from the sensor with only scaling applied (i.e. no yaw or position corrections)
bool AP_VisualOdom_Backend::align_position_to_ahrs(const Vector3f &sensor_pos, bool align_xy, bool align_z)
{
    // fail immediately if ahrs cannot provide position
    Vector3f ahrs_pos_ned;
    if (!AP::ahrs().get_relative_position_NED_origin_float(ahrs_pos_ned)) {
        return false;
    }

    align_position(sensor_pos, ahrs_pos_ned, align_xy, align_z);
    return true;
}

// apply rotation and correction to position
void AP_VisualOdom_Backend::rotate_and_correct_position(Vector3f &position) const
{
    if (_use_posvel_rotation) {
        position = _posvel_rotation * position;
    }
    position += _pos_correction;
}

// align position with a new position by updating _pos_correction
// sensor_pos should be the position directly from the sensor with only scaling applied (i.e. no yaw or position corrections)
// new_pos should be a NED position offset from the EKF origin
void AP_VisualOdom_Backend::align_position(const Vector3f &sensor_pos, const Vector3f &new_pos, bool align_xy, bool align_z)
{
    // calculate position with current rotation and correction
    Vector3f pos_orig = sensor_pos;
    rotate_and_correct_position(pos_orig);

    // update position correction
    if (align_xy) {
        _pos_correction.x += (new_pos.x - pos_orig.x);
        _pos_correction.y += (new_pos.y - pos_orig.y);
    }
    if (align_z) {
        _pos_correction.z += (new_pos.z - pos_orig.z);
    }
}
#endif
