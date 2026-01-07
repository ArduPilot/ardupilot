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
 *  AHRS View class - for creating a 2nd view of the vehicle attitude
 *
 */

#include "AP_AHRS_View.h"
#include <stdio.h>

AP_AHRS_View::AP_AHRS_View(AP_AHRS &_ahrs, enum Rotation _rotation, float pitch_trim_deg) :
    rotation(_rotation),
    ahrs(_ahrs)
{
    switch (rotation) {
    case ROTATION_NONE:
        y_angle = 0;
        break;
    case ROTATION_PITCH_90:
        y_angle = 90;
        break;
    case ROTATION_PITCH_270:
        y_angle =  270;
        break;
    default:
        AP_HAL::panic("Unsupported AHRS view %u\n", (unsigned)rotation);
    }

    _pitch_trim_deg = pitch_trim_deg;
    // Add pitch trim
    rot_view.from_euler(0, radians(wrap_360(y_angle + pitch_trim_deg)), 0);
    rot_view_T = rot_view;
    rot_view_T.transpose();

    // setup initial state
    update();
}

// apply pitch trim
void AP_AHRS_View::set_pitch_trim(float trim_deg) {
    _pitch_trim_deg = trim_deg; 
    rot_view.from_euler(0, radians(wrap_360(y_angle + _pitch_trim_deg)), 0);
    rot_view_T = rot_view;
    rot_view_T.transpose();
};

// update state
void AP_AHRS_View::update()
{
    rot_body_to_ned = ahrs.get_rotation_body_to_ned();
    gyro = ahrs.get_gyro();

    if (!is_zero(y_angle + _pitch_trim_deg)) {
        rot_body_to_ned = rot_body_to_ned * rot_view_T;
        gyro = rot_view * gyro;
    }

    // compute full-range pitch and fall back for singularity
    const float ax = rot_body_to_ned.a.x;
    const float bx = rot_body_to_ned.b.x;
    const float cx = rot_body_to_ned.c.x;

    // safe denominator (cos(pitch) approx)
    const float denom = sqrtf(ax*ax + bx*bx);

    if (denom > 1e-6f) {
        // full-range pitch using atan2(sin,cos) -> returns (-PI, PI)
        pitch = atan2F(-cx, denom);
        // roll and yaw as before
        roll  = atan2F(rot_body_to_ned.c.y, rot_body_to_ned.c.z);
        yaw   = atan2F(rot_body_to_ned.b.x, rot_body_to_ned.a.x);
    } else {
        // near gimbal lock: denom ~ 0 => cos(pitch) ~ 0 (pitch ~ +/-90°)
        // choose a stable convention: set roll to 0 and recover yaw from another pair
        pitch = (cx > 0) ? -M_PI_2 : M_PI_2; // match existing sign convention
        roll  = 0.0f;
        // use an alternate element to estimate yaw in singular case
        yaw   = atan2F(-rot_body_to_ned.a.y, rot_body_to_ned.b.y);
    }

    roll_sensor  = degrees(roll) * 100;
    pitch_sensor = degrees(pitch) * 100;
    yaw_sensor   = degrees(yaw) * 100;
    if (yaw_sensor < 0) {
        yaw_sensor += 36000;
    }

    ahrs.calc_trig(rot_body_to_ned,
                   trig.cos_roll, trig.cos_pitch, trig.cos_yaw,
                   trig.sin_roll, trig.sin_pitch, trig.sin_yaw);
}

// return a smoothed and corrected gyro vector using the latest ins data (which may not have been consumed by the EKF yet)
Vector3f AP_AHRS_View::get_gyro_latest(void) const {
    return rot_view * ahrs.get_gyro_latest();
}

// rotate a 2D vector from earth frame to body frame
Vector2f AP_AHRS_View::earth_to_body2D(const Vector2f &ef) const
{
    return Vector2f(ef.x * trig.cos_yaw + ef.y * trig.sin_yaw,
                    -ef.x * trig.sin_yaw + ef.y * trig.cos_yaw);
}

// rotate a 2D vector from earth frame to body frame
Vector2f AP_AHRS_View::body_to_earth2D(const Vector2f &bf) const
{
    return Vector2f(bf.x * trig.cos_yaw - bf.y * trig.sin_yaw,
                    bf.x * trig.sin_yaw + bf.y * trig.cos_yaw);
}

// Rotate vector from AHRS reference frame to AHRS view reference frame
void AP_AHRS_View::rotate(Vector3f &vec) const
{
    vec = rot_view * vec;
}
