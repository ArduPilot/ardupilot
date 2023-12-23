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

#include "SIM_Blimp.h"
#include <AP_Logger/AP_Logger.h>

#include <stdio.h>

using namespace SITL;

extern const AP_HAL::HAL& hal;

Blimp::Blimp(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = 0.07;
    radius = 0.25;
    moment_of_inertia = {0.004375, 0.004375, 0.004375}; //m*r^2 for hoop...
    cog = {0, 0, 0.1}; //10 cm down from center (i.e. center of buoyancy), for now
    k_tan = 0.6e-7; //Tangential (thrust) and normal force multipliers
    k_nor = 0;//3.4e-7;
    drag_constant = 0.05;
    drag_gyr_constant = 0.15;

    lock_step_scheduled = true;
    ::printf("Starting Blimp model\n");
}

// calculate rotational and linear accelerations
void Blimp::calculate_forces(const struct sitl_input &input, Vector3f &body_acc, Vector3f &rot_accel)
{
  float delta_time = frame_time_us * 1.0e-6f;

  if (!hal.scheduler->is_system_initialized()) {
      return;
  }

  //all fin setup
  for (uint8_t i=0; i<4; i++) {
    fin[i].last_angle = fin[i].angle;
    if (input.servos[i] == 0) {
        fin[i].angle = 0;
        fin[1].servo_angle = 0;
    } else {
        fin[i].angle = filtered_servo_angle(input, i)*radians(45.0f)+radians(13.5); //for servo range of -75 deg to +75 deg
        fin[i].servo_angle = filtered_servo_angle(input, i);
    }

    if (fin[i].angle < fin[i].last_angle) fin[i].dir = 0; //thus 0 = "angle is reducing"
    else fin[i].dir = 1;

    fin[i].vel = degrees(fin[i].angle - fin[i].last_angle)/delta_time; //Could also do multi-point derivative filter - DerivativeFilter.cpp
    //deg/s (should really be rad/s, but that would require modifying k_tan, k_nor)
    //all other angles should be in radians.
    fin[i].vel = constrain_float(fin[i].vel, -450, 450);
    fin[i].T = sq(fin[i].vel) * k_tan;
    fin[i].N = sq(fin[i].vel) * k_nor;
    if (fin[i].dir == 0) fin[i].N = -fin[1].N; //normal force flips when fin changes direction

    fin[i].Fx = 0;
    fin[i].Fy = 0;
    fin[i].Fz = 0;
  }

  //Back fin
  fin[0].Fx =  fin[0].T*cos(fin[0].angle);// + fin[0].N*sin(fin[0].angle); //causes forward movement
  fin[0].Fz =  fin[0].T*sin(fin[0].angle);// - fin[0].N*cos(fin[0].angle); //causes height & wobble in y

  //Front fin
  fin[1].Fx = -fin[1].T*cos(fin[1].angle);// - fin[1].N*sin(fin[1].angle); //causes backward movement
  fin[1].Fz =  fin[1].T*sin(fin[1].angle);// - fin[1].N*cos(fin[1].angle); //causes height & wobble in y

  //Right fin
  fin[2].Fy = -fin[2].T*cos(fin[2].angle);// - fin[2].N*sin(fin[2].angle); //causes left movement
  fin[2].Fx =  fin[2].T*sin(fin[2].angle);// - fin[2].N*cos(fin[2].angle); //cause yaw & wobble in z

  //Left fin
  fin[3].Fy =  fin[3].T*cos(fin[3].angle);// + fin[3].N*sin(fin[3].angle); //causes right movement
  fin[3].Fx =  fin[3].T*sin(fin[3].angle);// + fin[3].N*cos(fin[3].angle); //causes yaw & wobble in z

  Vector3f F_BF{0,0,0};
  for (uint8_t i=0; i<4; i++) {
    F_BF.x = F_BF.x + fin[i].Fx;
    F_BF.y = F_BF.y + fin[i].Fy;
    F_BF.z = F_BF.z + fin[i].Fz;
  }

  body_acc.x = F_BF.x/mass; //mass in kg, thus accel in m/s/s
  body_acc.y = F_BF.y/mass;
  body_acc.z = F_BF.z/mass;

  Vector3f rot_T{0,0,0};

  AP::logger().WriteStreaming("SFT", "TimeUS,f0,f1,f2,f3",
                              "Qffff",
                              AP_HAL::micros64(),
                              fin[0].T, fin[1].T, fin[2].T, fin[3].T);
  AP::logger().WriteStreaming("SFN", "TimeUS,n0,n1,n2,n3",
                              "Qffff",
                              AP_HAL::micros64(),
                              fin[0].N, fin[1].N, fin[2].N, fin[3].N);
  AP::logger().WriteStreaming("SBA1", "TimeUS,ax,ay,az",
                              "Qfff",
                              AP_HAL::micros64(),
                              body_acc.x, body_acc.y, body_acc.z);
  AP::logger().WriteStreaming("SFA1", "TimeUS,f0,f1,f2,f3",
                              "Qffff",
                              AP_HAL::micros64(),
                              fin[0].angle, fin[1].angle, fin[2].angle, fin[3].angle);
  AP::logger().WriteStreaming("SFAN", "TimeUS,f0,f1,f2,f3",
                              "Qffff",
                              AP_HAL::micros64(),
                              fin[0].servo_angle, fin[1].servo_angle, fin[2].servo_angle, fin[3].servo_angle);
  AP::logger().WriteStreaming("SSAN", "TimeUS,f0,f1,f2,f3",
                              "QHHHH",
                              AP_HAL::micros64(),
                              input.servos[0], input.servos[1], input.servos[2], input.servos[3]);
  AP::logger().WriteStreaming("SFV1", "TimeUS,f0,f1,f2,f3",
                              "Qffff",
                              AP_HAL::micros64(),
                              fin[0].vel, fin[1].vel, fin[2].vel, fin[3].vel);
  AP::logger().WriteStreaming("SRT1", "TimeUS,rtx,rty,rtz",
                              "Qfff",
                              AP_HAL::micros64(),
                              rot_T.x, rot_T.y, rot_T.z);

#if 0 //"Wobble" attempt
  rot_T.y = fin[0].Fz * radius + fin[1].Fz * radius;
  AP::logger().WriteStreaming("SRT2", "TimeUS,rtx,rty,rtz",
                              "Qfff",
                              AP_HAL::micros64(),
                              rot_T.x, rot_T.y, rot_T.z);
  // the blimp has pendulum stability due to the centre of gravity being lower than the centre of buoyancy
  Vector3f ang; //x,y,z correspond to roll, pitch, yaw.
  dcm.to_euler(&ang.x, &ang.y, &ang.z); //rpy in radians
  Vector3f ang_ef = dcm * ang;
  rot_T.x -= mass*GRAVITY_MSS*sinf(M_PI-ang_ef.x)/cog.z;
  rot_T.y -= mass*GRAVITY_MSS*sinf(M_PI-ang_ef.y)/cog.z;
  AP::logger().WriteStreaming("SRT3", "TimeUS,rtx,rty,rtz",
                              "Qfff",
                              AP_HAL::micros64(),
                              rot_T.x, rot_T.y, rot_T.z);
  AP::logger().WriteStreaming("SAN1", "TimeUS,anx,any,anz",
                              "Qfff",
                              AP_HAL::micros64(),
                              ang.x, ang.y, ang.z);
  AP::logger().WriteStreaming("SAN2", "TimeUS,anx,any,anz",
                              "Qfff",
                              AP_HAL::micros64(),
                              ang_ef.x, ang_ef.y, ang_ef.z);
  AP::logger().WriteStreaming("SAF1", "TimeUS,afx,afy,afz",
                              "Qfff",
                              AP_HAL::micros64(),
                              sinf(ang.x), sinf(ang.y), sinf(ang.z));
  AP::logger().WriteStreaming("SMGC", "TimeUS,m,g,cz",
                              "Qfff",
                              AP_HAL::micros64(),
                              mass, GRAVITY_MSS, cog.z);
#endif

  rot_T.z = fin[2].Fx * radius - fin[3].Fx * radius;//in N*m (Torque = force * lever arm)
  //rot accel = torque / moment of inertia
  //Torque = moment force.
  rot_accel.x = rot_T.x / moment_of_inertia.x;
  rot_accel.y = rot_T.y / moment_of_inertia.y;
  rot_accel.z = rot_T.z / moment_of_inertia.z;
}

/*
  update the blimp simulation by one time step
 */
void Blimp::update(const struct sitl_input &input)
{
  float delta_time = frame_time_us * 1.0e-6f;

  Vector3f rot_accel = Vector3f(0,0,0);
  calculate_forces(input, accel_body, rot_accel);

  if (hal.scheduler->is_system_initialized()) {
    float gyr_sq = gyro.length_squared();
    if (is_positive(gyr_sq)) {
        Vector3f force_gyr = (gyro.normalized() * drag_gyr_constant * gyr_sq);
        Vector3f ef_drag_accel_gyr = -force_gyr / mass;
        Vector3f bf_drag_accel_gyr = dcm.transposed() * ef_drag_accel_gyr;
        rot_accel += bf_drag_accel_gyr;
    }
  }

#if 0
  AP::logger().WriteStreaming("SBLM", "TimeUS,RAx,RAy,RAz",
                                "Qfff",
                                AP_HAL::micros64(),
                                rot_accel.x, rot_accel.y, rot_accel.z);
#endif

  // update rotational rates in body frame
  gyro += rot_accel * delta_time;

  gyro.x = constrain_float(gyro.x, -radians(2000.0f), radians(2000.0f));
  gyro.y = constrain_float(gyro.y, -radians(2000.0f), radians(2000.0f));
  gyro.z = constrain_float(gyro.z, -radians(2000.0f), radians(2000.0f));

  // Vector3f ang; //x,y,z correspond to roll, pitch, yaw.
  // dcm.to_euler(&ang.x, &ang.y, &ang.z); //rpy in radians
  // dcm.from_euler(0.0f, 0.0f, ang.z);
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

    // add lifting force exactly equal to gravity, for neutral buoyancy (buoyancy in ef)
    accel_body += dcm.transposed() * Vector3f(0,0,-GRAVITY_MSS);
  }

  Vector3f accel_earth = dcm * accel_body;
  accel_earth += Vector3f(0.0f, 0.0f, GRAVITY_MSS); //add gravity
  velocity_ef += accel_earth * delta_time;
  position += (velocity_ef * delta_time).todouble(); //update position vector

  update_position(); //updates the position from the Vector3f position
  time_advance();
  update_mag_field_bf();
  rate_hz = sitl->loop_rate_hz;

}
