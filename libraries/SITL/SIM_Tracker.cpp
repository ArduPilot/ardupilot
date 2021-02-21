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
  antenna-tracker simulator class
*/

#include "SIM_Tracker.h"

#include <stdio.h>

namespace SITL {

/*
  update function for position (normal) servos.
*/
void Tracker::update_position_servos(float delta_time, float &yaw_rate, float &pitch_rate) const
{
    float pitch_target = pitch_input*pitch_range;
    float yaw_target = yaw_input*yaw_range;

    pitch_rate = constrain_float(pitch_target - pitch_current_relative, -pitchrate, pitchrate);
    yaw_rate   = constrain_float(yaw_target - yaw_current_relative, -yawrate, yawrate);
}

/*
  update function for onoff servos.
  These servos either move at a constant rate or are still
  Returns (yaw_rate,pitch_rate) tuple
*/
void Tracker::update_onoff_servos(float &yaw_rate, float &pitch_rate) const
{
    if (fabsf(yaw_input) < 0.1) {
        yaw_rate = 0;
    } else if (yaw_input >= 0.1) {
        yaw_rate = yawrate;
    } else {
        yaw_rate = -yawrate;
    }

    if (fabsf(pitch_input) < 0.1) {
        pitch_rate = 0;
    } else if (pitch_input >= 0.1) {
        pitch_rate = pitchrate;
    } else {
        pitch_rate = -pitchrate;
    }
}

/*
  update state of tracker
 */
void Tracker::update(const struct sitl_input &input)
{
    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    float yaw_rate = 0.0f, pitch_rate = 0.0f;

    yaw_input = (input.servos[0]-1500)/500.0f;
    pitch_input = (input.servos[1]-1500)/500.0f;

    // implement yaw and pitch limits
    float r, p, y;
    dcm.to_euler(&r, &p, &y);

    pitch_current_relative = degrees(p) - zero_pitch;
    yaw_current_relative = degrees(y) - zero_yaw;
    float roll_current = degrees(r);
    if (yaw_current_relative > 180) {
        yaw_current_relative -= 360;
    }
    if (yaw_current_relative < -180) {
        yaw_current_relative += 360;
    }
    if (yaw_rate > 0 && yaw_current_relative >= yaw_range) {
        yaw_rate = 0;
    }
    if (yaw_rate < 0 && yaw_current_relative <= -yaw_range) {
        yaw_rate = 0;
    }
    if (pitch_rate > 0 && pitch_current_relative >= pitch_range) {
        pitch_rate = 0;
    }
    if (pitch_rate < 0 && pitch_current_relative <= -pitch_range) {
        pitch_rate = 0;
    }

    if (onoff) {
        update_onoff_servos(yaw_rate, pitch_rate);
    } else {
        update_position_servos(delta_time, yaw_rate, pitch_rate);
    }


    // keep it level
    float roll_rate = 0 - roll_current;

    if (time_now_us - last_debug_us > 2e6f && !onoff) {
        last_debug_us = time_now_us;
        printf("roll=%.1f pitch=%.1f yaw=%.1f rates=%.1f/%.1f/%.1f in=%.3f,%.3f\n",
               roll_current,
               pitch_current_relative,
               yaw_current_relative,
               roll_rate, pitch_rate, yaw_rate,
               yaw_input, pitch_input);
    }

    gyro = Vector3f(radians(roll_rate),radians(pitch_rate),radians(yaw_rate));

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    Vector3f accel_earth = Vector3f(0, 0, -GRAVITY_MSS);
    accel_body = dcm.transposed() * accel_earth;

    // new velocity vector
    velocity_ef.zero();
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
