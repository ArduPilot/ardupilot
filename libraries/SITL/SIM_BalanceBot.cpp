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
  Balance Bot simulator class
*/

#include "SIM_BalanceBot.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

BalanceBot::BalanceBot(const char *frame_str) :
    Aircraft(frame_str),
    skid_turn_rate(0.15708) // meters/sec
{
    dcm.from_euler(0,0,0); // initial yaw, pitch and roll in radians
    lock_step_scheduled = true;
    printf("Balance Bot Simulation Started\n");
}

/*
   return yaw rate in degrees/second given steering_angle
*/
float BalanceBot::calc_yaw_rate(float steering) const
{
    float wheel_base_length = 0.15f;
    return steering * degrees( skid_turn_rate/wheel_base_length );
}


/*
  update the Balance Bot simulation by one time step
 */
/*
 * The balance bot is physically modeled as an inverted pendulum(cuboid) on wheels
 * Further details on the equations used can be found here:
 * 1) http://robotics.ee.uwa.edu.au/theses/2003-Balance-Ooi.pdf page 33 onwards
 * 2) http://journals.sagepub.com/doi/pdf/10.5772/63933
 */
void BalanceBot::update(const struct sitl_input &input)
{
    // pendulum/chassis constants
    const float m_p = 3.0f; //pendulum mass(kg)
    // const float width = 0.0650f; //width(m)
    // const float height = 0.240f; //height(m)
    const float l = 0.10f; //height of center of mass from base(m)
    const float i_p = 0.01250f; //Moment of inertia about pitch axis(SI units)

    // wheel constants
    const float r_w = 0.05f; //wheel radius(m)
    const float m_w = 0.1130f; //wheel mass(kg)
    const float i_w = 0.00015480f; // moment of inertia of wheel(SI units)

    // motor constants
    const float R = 3.0f; //Winding resistance(ohm)
    const float k_e = 0.240f; //back-emf constant(SI units)
    const float k_t = 0.240f; //torque constant(SI units)
    const float v_max = 12.0f; //max input voltage(V)
    const float gear_ratio = 50.0f;

    // balance bot uses skid steering
    const float motor1 = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
    const float motor2 = 2*((input.servos[2]-1000)/1000.0f - 0.5f);
    const float steering = motor1 - motor2;
    const float throttle = 0.5 * (motor1 + motor2);

    // motor input voltage: (throttle/max_throttle)*v_max
    const float v = throttle*v_max;

    // how much time has passed?
    const float delta_time = frame_time_us * 1.0e-6f;

    // yaw rate in degrees/s
    const float yaw_rate = calc_yaw_rate(steering);

    // obtain roll, pitch, yaw from dcm
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    float theta = p; //radians

    float ang_vel = gyro.y; //radians/s
    if (!hal.util->get_soft_armed()) {
        // simulated fingers uprighting the vehicle
        const float p_gain = 200;
        const float pitch_response = -sin(p) * p_gain * delta_time;
        ang_vel += pitch_response;
    }

    // t1,t2,t3 are terms in the equation to find vehicle frame x acceleration
    const float t1 = ((2.0f*gear_ratio*k_t*v/(R*r_w)) - (2.0f*gear_ratio*k_t*k_e*velocity_vf_x/(R*r_w*r_w)) - (m_p*l*ang_vel*ang_vel*sin(theta))) * (i_p + m_p*l*l);
    const float t2 = -m_p*l*cos(theta)*((2.0f*gear_ratio*k_t*k_e*velocity_vf_x/(R*r_w)) - (2.0f*gear_ratio*k_t*v/(R)) + (m_p*GRAVITY_MSS*l*sin(theta)));
    const float t3 = ( ((2.0f*m_w + 2.0f*i_w/(r_w*r_w) + m_p) * (i_p + m_p*l*l)) - (m_p*m_p*l*l*cos(theta)*cos(theta)) );

    //vehicle frame x acceleration
    const float accel_vf_x = (t1-t2)/t3;

    const float angular_accel_bf_y = ((2.0f*gear_ratio*k_t*k_e*velocity_vf_x/(R*r_w)) - (2.0f*gear_ratio*k_t*v/(R)) + m_p*l*accel_vf_x*cos(theta) + m_p*GRAVITY_MSS*l*sin(theta))
                                     / (i_p + m_p*l*l);

    // accel in body frame due to motor
    accel_body = Vector3f(accel_vf_x*cos(theta), 0, -accel_vf_x*sin(theta));

    // update theta and angular velocity
    ang_vel += angular_accel_bf_y * delta_time;
    theta += ang_vel * delta_time;
    theta = fmod(theta, radians(360));

    gyro = Vector3f(0, ang_vel, radians(yaw_rate));

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // add in accel due to direction change
    accel_body.y += radians(yaw_rate) * velocity_vf_x;

    // update x velocity in vehicle frame
    velocity_vf_x += accel_vf_x * delta_time;

    // now in earth frame
    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0;

    if (!hal.util->get_soft_armed() &&
        p < radians(2)) {
        // reset to vertical when not armed for faster testing
        accel_earth.zero();
        velocity_ef.zero();
        dcm.identity();
        gyro.zero();
        velocity_vf_x =0;
    }
    
    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body += dcm.transposed() * (Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    position += (velocity_ef * delta_time).todouble();

    // neglect roll
    dcm.to_euler(&r, &p, &y);
    dcm.from_euler(0.0f, p, y);
    use_smoothing = true;

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

}// namespace SITL
