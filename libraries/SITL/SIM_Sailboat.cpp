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
    Sailboat simulator class

    see explanation of lift and drag explained here: https://en.wikipedia.org/wiki/Forces_on_sails

    To-Do: add heel handling by calculating lateral force from wind vs gravity force from heel to arrive at roll rate or acceleration
*/

#include "SIM_Sailboat.h"
#include <AP_Math/AP_Math.h>
#include <string.h>
#include <stdio.h>

namespace SITL {

Sailboat::Sailboat(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    max_wheel_turn(35),
    turning_circle(1.8)
{
}

// calculate the lift and drag as values from 0 to 1
// given an apparent wind speed in m/s and angle-of-attack in degrees
void Sailboat::calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float& lift, float& drag)
{
    // check extremes
    if (angle_of_attack_deg <= 0.0f) {
        lift = lift_curve[0];
        drag = drag_curve[0];
        return;
    }
    if (angle_of_attack_deg >= 170.0f) {
        lift = lift_curve[17];
        drag = drag_curve[17];
        return;
    }

    uint8_t index = constrain_int16(angle_of_attack_deg / 10, 0, 17);
    float remainder = angle_of_attack_deg - (index * 10.0f);
    lift = linear_interpolate(lift_curve[index], lift_curve[index+1], remainder, 0.0f, 10.0f);
    drag = linear_interpolate(drag_curve[index], drag_curve[index+1], remainder, 0.0f, 10.0f);

    // apply scaling by wind speed
    lift *= wind_speed;
    drag *= wind_speed;
}

/*
  return turning circle (diameter) in meters for steering angle proportion in degrees
*/
float Sailboat::turn_circle(float steering)
{
    if (fabsf(steering) < 1.0e-6) {
        return 0;
    }
    return turning_circle * sinf(radians(max_wheel_turn)) / sinf(radians(steering*max_wheel_turn));
}

/*
   return yaw rate in degrees/second given steering_angle and speed
*/
float Sailboat::calc_yaw_rate(float steering, float speed)
{
    if (fabsf(steering) < 1.0e-6 or fabsf(speed) < 1.0e-6) {
        return 0;
    }
    float d = turn_circle(steering);
    float c = M_PI * d;
    float t = c / speed;
    float rate = 360.0f / t;
    return rate;
}

/*
  return lateral acceleration in m/s/s
*/
float Sailboat::calc_lat_accel(float steering_angle, float speed)
{
    float yaw_rate = calc_yaw_rate(steering_angle, speed);
    float accel = radians(yaw_rate) * speed;
    return accel;
}

/*
  update the sailboat simulation by one time step
 */
void Sailboat::update(const struct sitl_input &input)
{
    // update wind
    update_wind(input);

    // in sailboats the steering controls the rudder, the throttle controls the main sail position
    float steering = 2*((input.servos[0]-1000)/1000.0f - 0.5f);

    // calculate mainsail angle from servo output 4, 0 to 90 degrees
    float mainsail_angle_bf = constrain_float((input.servos[3]-1000)/1000.0f * 90.0f, 0.0f, 90.0f);

    // calculate apparent wind in earth-frame (this is the direction the wind is coming from)
    Vector3f wind_apparent_ef = wind_ef + velocity_ef;
    const float wind_apparent_dir_ef = degrees(atan2f(wind_apparent_ef.y, wind_apparent_ef.x));
    const float wind_apparent_speed = safe_sqrt(sq(wind_apparent_ef.x)+sq(wind_apparent_ef.y));

    // calculate angle-of-attack from wind to mainsail
    float aoa_deg = MAX(fabsf(wrap_180(wind_apparent_dir_ef - degrees(AP::ahrs().yaw))) - mainsail_angle_bf, 0);

    // calculate Lift force (perpendicular to wind direction) and Drag force (parallel to wind direction)
    float lift_wf, drag_wf;
    calc_lift_and_drag(wind_apparent_speed, aoa_deg, lift_wf, drag_wf);

    // rotate lift and drag from wind frame into body frame
    const float wind_to_veh_rot_angle_deg = wrap_180(180 + wind_apparent_dir_ef - degrees(AP::ahrs().yaw));
    const float sin_rot_rad = sinf(radians(wind_to_veh_rot_angle_deg));
    const float cos_rot_rad = cosf(radians(wind_to_veh_rot_angle_deg));
    const float force_fwd = fabsf((lift_wf * sin_rot_rad)) + (drag_wf * cos_rot_rad);

    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // speed along x axis, +ve is forward
    float speed = velocity_body.x;

    // yaw rate in degrees/s
    float yaw_rate = calc_yaw_rate(steering, speed);

    gyro = Vector3f(0,0,radians(yaw_rate));

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due acceleration from sail and deceleration from hull friction
    accel_body = Vector3f((force_fwd * 1.0f) - (velocity_body.x * 0.5f), 0, 0);

    // add in accel due to direction change
    accel_body.y += radians(yaw_rate) * speed;

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
    position += velocity_ef * delta_time;

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
