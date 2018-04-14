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

AP_AHRS_View::AP_AHRS_View(AP_AHRS &_ahrs, enum Rotation _rotation) :
    rotation(_rotation),
    ahrs(_ahrs)
{
    switch (rotation) {
    case ROTATION_NONE:
        rot_view.identity();
        break;
    case ROTATION_PITCH_90:
        rot_view.from_euler(0, radians(90), 0);
        break;
    case ROTATION_PITCH_270:
        rot_view.from_euler(0, radians(270), 0);
        break;
    default:
        AP_HAL::panic("Unsupported AHRS view %u\n", (unsigned)rotation);
    }

    // setup initial state
    update();
}

// update state
void AP_AHRS_View::update(bool skip_ins_update)
{
    rot_body_to_ned = ahrs.get_rotation_body_to_ned();
    gyro = ahrs.get_gyro();

    if (rotation != ROTATION_NONE) {
        Matrix3f &r = rot_body_to_ned;
        r.transpose();
        r = rot_view * r;
        r.transpose();
        gyro.rotate(rotation);
    }

    rot_body_to_ned.to_euler(&roll, &pitch, &yaw);

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
    Vector3f gyro_latest = ahrs.get_gyro_latest();
    gyro_latest.rotate(rotation);
    return gyro_latest;
}

// rotate a 2D vector from earth frame to body frame
Vector2f AP_AHRS_View::rotate_earth_to_body2D(const Vector2f &ef) const
{
    return Vector2f(ef.x * trig.cos_yaw + ef.y * trig.sin_yaw,
                    -ef.x * trig.sin_yaw + ef.y * trig.cos_yaw);
}

// rotate a 2D vector from earth frame to body frame
Vector2f AP_AHRS_View::rotate_body_to_earth2D(const Vector2f &bf) const
{
    return Vector2f(bf.x * trig.cos_yaw - bf.y * trig.sin_yaw,
                    bf.x * trig.sin_yaw + bf.y * trig.cos_yaw);
}
