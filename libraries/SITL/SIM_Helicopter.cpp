/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  helicopter simulator class
*/

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "SIM_Helicopter.h"
#include <stdio.h>

/*
  constructor
 */
Helicopter::Helicopter(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str)
{
    mass = 2.13f;

    /*
       scaling from motor power to Newtons. Allows the copter
       to hover against gravity when the motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;

    // calculate lateral thrust ratio for tail rotor
    tail_thrust_scale = sinf(radians(hover_lean)) * thrust_scale / yaw_zero;

    frame_height = 0.1;
}

/*
  update the helicopter simulation by one time step
 */
void Helicopter::update(const struct sitl_input &input)
{
    float swash1 = (input.servos[0]-1000) / 1000.0f;
    float swash2 = (input.servos[1]-1000) / 1000.0f;
    float swash3 = (input.servos[2]-1000) / 1000.0f;
    float tail_rotor = (input.servos[3]-1000) / 1000.0f;
    float rsc = (input.servos[7]-1000) / 1000.0f;

    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    float thrust = (rsc/rsc_setpoint)*(swash1+swash2+swash3)/3.0f;

    // very simplistic mapping to body euler rates
    float roll_rate = swash1 - swash2;
    float pitch_rate = (swash1 + swash2)/2.0f - swash3;
    float yaw_rate = tail_rotor - 0.5f;

    float rsc_scale = rsc/rsc_setpoint;

    roll_rate *= rsc_scale;
    pitch_rate *= rsc_scale;
    yaw_rate *= rsc_scale;

    // rotational acceleration, in rad/s/s, in body frame
    Vector3f rot_accel;
    rot_accel.x = roll_rate * roll_rate_max;
    rot_accel.y = pitch_rate * pitch_rate_max;
    rot_accel.z = yaw_rate * yaw_rate_max;

    // rotational air resistance
    rot_accel.x -= gyro.x * radians(5000.0) / terminal_rotation_rate;
    rot_accel.y -= gyro.y * radians(5000.0) / terminal_rotation_rate;
    rot_accel.z -= gyro.z * radians(400.0)  / terminal_rotation_rate;

    // torque effect on tail
    rot_accel.z += (rsc_scale+thrust) * rotor_rot_accel;

    // update rotational rates in body frame
    gyro += rot_accel * delta_time;

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // air resistance
    Vector3f air_resistance = -velocity_ef * (GRAVITY_MSS/terminal_velocity);

    // scale thrust to newtons
    thrust *= thrust_scale;

    accel_body = Vector3f(0, yaw_rate * rsc_scale * tail_thrust_scale, -thrust / mass);
    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);
    accel_earth += air_resistance;

    // if we're on the ground, then our vertical acceleration is limited
    // to zero. This effectively adds the force of the ground on the aircraft
    if (on_ground(position) && accel_earth.z > 0) {
        accel_earth.z = 0;
    }

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // add some noise
    add_noise(thrust / thrust_scale);

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    Vector3f old_position = position;
    position += velocity_ef * delta_time;

    // assume zero wind for now
    airspeed = velocity_ef.length();

    // constrain height to the ground
    if (on_ground(position)) {
        if (!on_ground(old_position)) {
            printf("Hit ground at %f m/s\n", velocity_ef.z);

            velocity_ef.zero();

            // zero roll/pitch, but keep yaw
            float r, p, y;
            dcm.to_euler(&r, &p, &y);
            dcm.from_euler(0, 0, y);

            position.z = -(ground_level + frame_height - home.alt*0.01f);
        }
    }

    // update lat/lon/altitude
    update_position();
}
#endif // CONFIG_HAL_BOARD
