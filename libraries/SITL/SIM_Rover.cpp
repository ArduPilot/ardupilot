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
  rover simulator class
*/

#include "SIM_Rover.h"

#include <string.h>
#include <stdio.h>

#include <AP_Math/AP_Math.h>

namespace SITL {

SimRover::SimRover(const char *frame_str) :
    Aircraft(frame_str)
{
    skid_steering = strstr(frame_str, "skid") != nullptr;

    if (skid_steering) {
        printf("SKID Steering Rover Simulation Started\n");
        // these are taken from a 6V wild thumper with skid steering,
        // with a sabertooth controller
        max_accel = 14;
        max_speed = 4;
        return;
    }

    vectored_thrust = strstr(frame_str, "vector") != nullptr;
    if (vectored_thrust) {
        printf("Vectored Thrust Rover Simulation Started\n");
    }

    omni3 = strstr(frame_str, "omni3mecanum") != nullptr;
    if (omni3) {
        printf("Omni3 Mecanum Rover Simulation Started\n");
    }

    lock_step_scheduled = true;
}

/*
  return turning circle (diameter) in meters for steering angle proportion in degrees
*/
float SimRover::turn_circle(float steering) const
{
    if (fabsf(steering) < 1.0e-6) {
        return 0;
    }
    return turning_circle * sinf(radians(max_wheel_turn)) / sinf(radians(steering*max_wheel_turn));
}

/*
   return yaw rate in degrees/second given steering_angle and speed
*/
float SimRover::calc_yaw_rate(float steering, float speed)
{
    if (skid_steering) {
        return constrain_float(steering * skid_turn_rate, -MAX_YAW_RATE, MAX_YAW_RATE);
    }
    if (vectored_thrust) {
        return constrain_float(steering * vectored_turn_rate_max, -MAX_YAW_RATE, MAX_YAW_RATE);
    }
    if (fabsf(steering) < 1.0e-6 or fabsf(speed) < 1.0e-6) {
        return 0;
    }
    float d = turn_circle(steering);
    float c = M_PI * d;
    float t = c / speed;
    float rate = constrain_float(360.0f / t, -MAX_YAW_RATE, MAX_YAW_RATE);
    return rate;
}

/*
  return lateral acceleration in m/s/s
*/
float SimRover::calc_lat_accel(float steering_angle, float speed)
{
    float yaw_rate = calc_yaw_rate(steering_angle, speed);
    float accel = radians(yaw_rate) * speed;
    return accel;
}

/*
  update the rover simulation by one time step
 */
void SimRover::update(const struct sitl_input &input)
{
    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    // update gyro and accel_body according to frame type
    if (omni3) {
        update_omni3(input, delta_time);
    } else {
        update_ackermann_or_skid(input, delta_time);
    }

    // common to all rovers

    // now in earth frame
    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    position += (velocity_ef * delta_time).todouble();

    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

/*
  update the ackermann or skid rover simulation by one time step
 */
void SimRover::update_ackermann_or_skid(const struct sitl_input &input, float delta_time)
{
    float steering, throttle;

    // if in skid steering mode the steering and throttle values are used for motor1 and motor2
    if (skid_steering) {
        const float motor1 = input.servos[0] ? normalise_servo_input(input.servos[0]) : 0;
        const float motor2 = input.servos[2] ? normalise_servo_input(input.servos[2]) : 0;
        steering = motor1 - motor2;
        throttle = 0.5*(motor1 + motor2);
    } else {
        // steering here should really be "old steering" as no-pulses
        // should yield no servo movement
        steering = input.servos[0] ? normalise_servo_input(input.servos[0]) : 0;
        throttle = input.servos[2] ? normalise_servo_input(input.servos[2]) : 0;

        // vectored thrust conversion
        if (vectored_thrust) {
            const float steering_angle_rad = radians(steering * vectored_angle_max);
            steering = sinf(steering_angle_rad) * throttle;
            throttle *= cosf(steering_angle_rad);
        }
    }

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // speed along x axis, +ve is forward
    float speed = velocity_body.x;

    // yaw rate in degrees/s
    float yaw_rate = calc_yaw_rate(steering, speed);

    // target speed with current throttle
    float target_speed = throttle * max_speed;

    // linear acceleration in m/s/s - very crude model
    float accel = max_accel * (target_speed - speed) / max_speed;

    gyro = Vector3f(0,0,radians(yaw_rate));

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due to motor (excluding gravity)
    accel_body = Vector3f(accel, 0, 0);

    // add in accel due to direction change
    accel_body.y += radians(yaw_rate) * speed;
}

/*
  update the omni3 rover simulation by one time step
 */
void SimRover::update_omni3(const struct sitl_input &input, float delta_time)
{
    // in omni3 mode the first three servos are motor speeds

    // use forward kinematics to calculate body frame velocity
    Vector3f wheel_ang_vel;
    for (uint8_t i=0; i<3; i++) {
        wheel_ang_vel[i] = input.servos[i] ? normalise_servo_input(input.servos[i]) : 0;
    };
    wheel_ang_vel *= omni3_wheel_max_ang_vel;

    // derivation of forward kinematics for an Omni3Mecanum rover
    // A. Gfrerrer. "Geometry and kinematics of the Mecanum wheel",
    // Computer Aided Geometric Design 25 (2008) 784–791.
    // Retrieved from https://www.geometrie.tugraz.at/gfrerrer/publications/MecanumWheel.pdf.
    //
    // the frame is equilateral triangle
    //
    // d[i] = 0.18 m is distance from frame centre to each wheel 
    // r_w = 0.04725 m is the wheel radius.
    // delta = radians(-45) is angle of the roller to the direction of forward rotation
    // alpha[i] is the angle the wheel axis is rotated about the body z-axis
    // c[i] = cos(alpha[i] + delta)
    // s[i] = sin(alpha[i] + delta)
    //
    // wheel  d[i]  alpha[i]  a_x[i]   a_y[i]      c[i]      s[i]
    //     1  0.18   1.04719    0.09  0.15588  0.965925  0.258819
    //     2  0.18   3.14159   -0.18  0.0     -0.707106  0.707106
    //     3  0.18   5.23598    0.09 -0.15588 -0.258819 -0.965925
    //
    //  k = 1/(r_w * sin(delta)) = -29.930445 is a scale factor
    //
    // inverse kinematic matrix
    // M[i, 0] = k * c[i]
    // M[i, 1] = k * s[i]
    // M[i, 2] = k * (a_x[i] s[i] - a_y[i] c[i])
    //
    // forward kinematics matrix: Minv = M^-1
    constexpr Matrix3f Minv(
      -0.0215149, 0.01575, 0.0057649,
      -0.0057649, -0.01575, 0.0215149,
      0.0875, 0.0875, 0.0875);

    // twist - this is the target linear and angular velocity
    Vector3f twist = Minv * wheel_ang_vel;

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // linear acceleration in m/s/s - very crude model
    float accel_x = omni3_max_accel * (twist.x - velocity_body.x) / omni3_max_speed;
    float accel_y = omni3_max_accel * (twist.y - velocity_body.y) / omni3_max_speed;

    gyro = Vector3f(0, 0, twist.z);

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due to motors (excluding gravity)
    accel_body = Vector3f(accel_x, accel_y, 0);
}

} // namespace SITL
