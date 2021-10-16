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
  Blimp simulator class
*/

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "SIM_Blimp.h"

#include <stdio.h>

using namespace SITL;

extern const AP_HAL::HAL& hal;

Blimp::Blimp(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = 0.07;
    radius = 0.25;
    moment_of_inertia = {0.004375, 0.004375, 0.004375}; //m*r^2 for hoop...
    k_tan = 5.52e-4; //Tangential (thrust) multiplier
    drag_constant = 0.05;
    drag_gyr_constant = 0.08;

    lock_step_scheduled = true;
    ::printf("Starting Blimp AirFish model...\n");
}

// calculate rotational and linear accelerations
void Blimp::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
  if (!hal.scheduler->is_system_initialized()) {
      return;
  }

  //all fin setup
  for (uint8_t i=0; i<4; i++) {
    fin[i].last_angle = fin[i].angle;
    fin[i].angle = filtered_servo_angle(input, i)*radians(75.0f); //for servo range of -75 deg to +75 deg
    
    if (fin[i].angle < fin[i].last_angle) fin[i].dir = 0; //thus 0 = "angle is reducing"
    else fin[i].dir = 1;
    
    fin[i].vel = (fin[i].angle - fin[i].last_angle)/delta_time; //rad/s
    fin[i].vel = constrain_float(fin[i].vel, radians(-450), radians(450));
    fin[i].T = pow(fin[i].vel,2) * k_tan;
    
    fin[i].Fx = 0;
    fin[i].Fy = 0;
    fin[i].Fz = 0;
  }

  //TODO: Add normal force calculations and include roll & pitch oscillation.
  //Back fin
  fin[0].Fx =  fin[0].T*cos(fin[0].angle); //causes forward movement
  fin[0].Fz =  fin[0].T*sin(fin[0].angle); //causes height change

  //Front fin
  fin[1].Fx = -fin[1].T*cos(fin[1].angle); //causes backward movement
  fin[1].Fz =  fin[1].T*sin(fin[1].angle); //causes height change

  //Right fin
  fin[2].Fy = -fin[2].T*cos(fin[2].angle); //causes left movement
  fin[2].Fx =  fin[2].T*sin(fin[2].angle); //causes yaw

  //Left fin
  fin[3].Fy =  fin[3].T*cos(fin[3].angle); //causes right movement
  fin[3].Fx = -fin[3].T*sin(fin[3].angle); //causes yaw

  Vector3f force_bf{0,0,0}; 
  for (uint8_t i=0; i<4; i++) {
    force_bf.x = force_bf.x + fin[i].Fx;
    force_bf.y = force_bf.y + fin[i].Fy;
    force_bf.z = force_bf.z + fin[i].Fz;
  }

  //mass in kg, thus accel in m/s/s
  body_accel.x = force_bf.x/mass;
  body_accel.y = force_bf.y/mass;
  body_accel.z = force_bf.z/mass;

  Vector3f rot_T{0,0,0};
  rot_T.z = fin[2].Fx * radius + fin[3].Fx * radius;//in N*m (Torque = force * lever arm)

  //rot accel = torque / moment of inertia
  rot_accel.x = 0;
  rot_accel.y = 0;
  rot_accel.z = rot_T.z / moment_of_inertia.z;
}

/*
  update the blimp simulation by one time step
 */
void Blimp::update(const struct sitl_input &input)
{
  delta_time = frame_time_us * 1.0e-6f;

  Vector3f rot_accel = Vector3f(0,0,0);
  calculate_forces(input, rot_accel, accel_body);

  if (hal.scheduler->is_system_initialized()) {
    float gyr_sq = gyro.length_squared();
    if (is_positive(gyr_sq)) {
        Vector3f force_gyr = (gyro.normalized() * drag_gyr_constant * gyr_sq);
        Vector3f ef_drag_accel_gyr = -force_gyr / mass;
        Vector3f bf_drag_accel_gyr = dcm.transposed() * ef_drag_accel_gyr;
        rot_accel += bf_drag_accel_gyr;
    }
  }

  // update rotational rates in body frame
  gyro += rot_accel * delta_time;
  gyro.x = constrain_float(gyro.x, -radians(2000.0f), radians(2000.0f));
  gyro.y = constrain_float(gyro.y, -radians(2000.0f), radians(2000.0f));
  gyro.z = constrain_float(gyro.z, -radians(2000.0f), radians(2000.0f));

  // update attitude
  dcm.rotate(gyro * delta_time);
  dcm.normalize();

  if (hal.scheduler->is_system_initialized()) {
    float speed_sq = velocity_ef.length_squared();
    if (is_positive(speed_sq)) {
        Vector3f force = (velocity_ef.normalized() * drag_constant * speed_sq);
        Vector3f ef_drag_accel = -force / mass;
        Vector3f bf_drag_accel = dcm.transposed() * ef_drag_accel;
        accel_body += bf_drag_accel;
    }

    // add lifting force exactly equal to gravity, for neutral buoyancy
    accel_body += dcm.transposed() * Vector3f(0,0,-GRAVITY_MSS);
  }
  
  Vector3f accel_earth = dcm * accel_body;
  accel_earth += Vector3f(0.0f, 0.0f, GRAVITY_MSS); //add gravity
  velocity_ef += accel_earth * delta_time;
  position += (velocity_ef * delta_time).todouble(); //update position vector

  update_position(); //updates the position from the Vector3f position
  time_advance();
  update_mag_field_bf();
}
