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
  singlecopter simulator class
*/

#include "SIM_SingleCopter.h"

#include <stdio.h>

using namespace SITL;

SingleCopter::SingleCopter(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str)
{
    mass = 2.0f;

    if (strstr(frame_str, "coax")) {
        frame_type = FRAME_COAX;
    } else {
        frame_type = FRAME_SINGLE;
    }
    
    /*
       scaling from motor power to Newtons. Allows the copter
       to hover against gravity when the motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    frame_height = 0.1;
}

/*
  update the copter simulation by one time step
 */
void SingleCopter::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    float actuator[4];
    for (uint8_t i=0; i<4; i++) {
        actuator[i] = constrain_float((input.servos[i]-1500) / 500.0f, -1, 1);
    }
    float thrust;
    float yaw_thrust;
    float roll_thrust;
    float pitch_thrust;

    switch (frame_type) {
    case FRAME_SINGLE:
        thrust = constrain_float((input.servos[4]-1000) / 1000.0f, 0, 1);
        yaw_thrust   = -(actuator[0] + actuator[1] + actuator[2] + actuator[3]) * 0.25f * thrust + thrust * rotor_rot_accel;
        roll_thrust  = (actuator[0] - actuator[2]) * 0.5f * thrust;
        pitch_thrust = (actuator[1] - actuator[3]) * 0.5f * thrust;
        break;

    case FRAME_COAX:
    default: {
        float motor1 = constrain_float((input.servos[4]-1000) / 1000.0f, 0, 1);
        float motor2 = constrain_float((input.servos[5]-1000) / 1000.0f, 0, 1);
        thrust = 0.5f*(motor1 + motor2);
        yaw_thrust   = -(actuator[0] + actuator[1] + actuator[2] + actuator[3]) * 0.25f * thrust + (motor2 - motor1) * rotor_rot_accel;
        roll_thrust  = (actuator[0] - actuator[2]) * 0.5f * thrust;
        pitch_thrust = (actuator[1] - actuator[3]) * 0.5f * thrust;
        break;
    }
    }        
    
    // rotational acceleration, in rad/s/s, in body frame
    Vector3f rot_accel(roll_thrust * roll_rate_max,
                       pitch_thrust * pitch_rate_max,
                       yaw_thrust * yaw_rate_max);

    // rotational air resistance
    rot_accel.x -= gyro.x * radians(5000.0) / terminal_rotation_rate;
    rot_accel.y -= gyro.y * radians(5000.0) / terminal_rotation_rate;
    rot_accel.z -= gyro.z * radians(400.0)  / terminal_rotation_rate;

    // air resistance
    Vector3f air_resistance = -velocity_air_ef * (GRAVITY_MSS/terminal_velocity);

    // scale thrust to newtons
    thrust *= thrust_scale;

    accel_body = Vector3f(0, 0, -thrust / mass);
    accel_body += dcm.transposed() * air_resistance;

    update_dynamics(rot_accel);
    
    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
