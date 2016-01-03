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
  very simple plane simulator class. Not aerodynamically accurate,
  just enough to be able to debug control logic for new frame types
*/

#include "SIM_Plane.h"

#include <stdio.h>

using namespace SITL;

Plane::Plane(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str)
{
    mass = 1.0f;

    /*
       scaling from motor power to Newtons. Allows the plane to hold
       vertically against gravity when the motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    frame_height = 0.1f;
}

/*
  calculate lift in neutons
 */
float Plane::calculate_lift(void) const
{
    // simple lift equation from http://wright.nasa.gov/airplane/lifteq.html
    const float max_angle = radians(30);
    const float max_angle_delta = radians(10);
    const float clift_at_max = coefficient.lift * 2 * M_PI_F * max_angle;
    float Cl = coefficient.lift * 2 * M_PI_F * angle_of_attack;
    if (fabsf(angle_of_attack) > max_angle+max_angle_delta) {
        return 0;
    }
    if (angle_of_attack > max_angle) {
        Cl = clift_at_max * (1-(angle_of_attack - max_angle)/max_angle_delta);
    } else if (angle_of_attack < -max_angle) {
        Cl = -clift_at_max * (1+(angle_of_attack - max_angle)/max_angle_delta);
    }
    float lift = 0.5 * Cl * air_density * sq(airspeed) * wing_area;
    return lift;
}


/*
  calculate induced drag in neutons
 */
float Plane::calculate_drag_induced(void) const
{
    float lift = calculate_lift();
    
    // simple induced drag from https://en.wikipedia.org/wiki/Lift-induced_drag
    if (airspeed < 0.1) {
        return 0;
    }
    float drag_i = coefficient.lift_drag * sq(lift) / (0.25 * sq(air_density) * sq(airspeed) * wing_area * M_PI_F * wing_efficiency * aspect_ratio);
    return drag_i;
}

/*
  calculate form drag in neutons
 */
float Plane::calculate_drag_form(void) const
{
    // simple form drag
    float drag_f = 0.5 * air_density * sq(airspeed) * coefficient.drag;
    return drag_f;
}

/*
  calculate lift+drag in neutons in body frame
 */
Vector3f Plane::calculate_lift_drag(void) const
{
    if (velocity_ef.is_zero()) {
        return Vector3f(0,0,0);
    }
    float lift = calculate_lift();
    float drag = calculate_drag_induced() + calculate_drag_form();
    return velocity_bf.normalized()*(-drag) + Vector3f(0, 0, -lift);
}

void Plane::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    float aileron  = (input.servos[0]-1500)/500.0f;
    float elevator = (input.servos[1]-1500)/500.0f;
    float rudder   = (input.servos[3]-1500)/500.0f;
    float throttle = constrain_float((input.servos[2]-1000)/1000.0f, 0, 1);
    float speed_scaling = airspeed / cruise_airspeed;
    
    float thrust     = throttle;
    float roll_rate  = aileron * speed_scaling;
    float pitch_rate = elevator * speed_scaling;
    float yaw_rate   = rudder * speed_scaling;

    // rotational acceleration, in rad/s/s, in body frame
    rot_accel.x = roll_rate  * max_rates.x;
    rot_accel.y = pitch_rate * max_rates.y;
    rot_accel.z = yaw_rate   * max_rates.z;

    // rotational air resistance
    rot_accel.x -= gyro.x * radians(800.0) / terminal_rotation_rate.x;
    rot_accel.y -= gyro.y * radians(800.0) / terminal_rotation_rate.y;
    rot_accel.z -= gyro.z * radians(1200.0)  / terminal_rotation_rate.z;

    // add torque of stabilisers
    rot_accel.z += velocity_bf.y * speed_scaling * coefficient.vertical_stabiliser;
    rot_accel.y -= velocity_bf.z * speed_scaling * coefficient.horizontal_stabiliser;

    // calculate angle of attack
    angle_of_attack = atan2f(velocity_bf.z, velocity_bf.x);
    beta = atan2f(velocity_bf.y,velocity_bf.x);
    
    // add dihedral
    rot_accel.x -= beta * airspeed * coefficient.dihedral;
    
    // velocity in body frame
    velocity_bf = dcm.transposed() * velocity_ef;

    // get lift and drag in body frame, in neutons
    Vector3f lift_drag = calculate_lift_drag();
    
    // scale thrust to newtons
    thrust *= thrust_scale;

    accel_body = Vector3f(thrust/mass, 0, 0);
    accel_body += lift_drag/mass;

    // add some noise
    add_noise(thrust / thrust_scale);
}
    
/*
  update the plane simulation by one time step
 */
void Plane::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    calculate_forces(input, rot_accel, accel_body);
    
    update_dynamics(rot_accel);
    
    // update lat/lon/altitude
    update_position();
}
