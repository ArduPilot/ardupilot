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
  Submarine simulator class
*/

#include "SIM_Submarine.h"
#include <AP_Motors/AP_Motors.h>
#include "Frame_Vectored.h"

#include <stdio.h>

using namespace SITL;

static Thruster vectored_thrusters[] =
{
       Thruster(0, MOT_1_ROLL_FACTOR, MOT_1_PITCH_FACTOR, MOT_1_YAW_FACTOR, MOT_1_THROTTLE_FACTOR, MOT_1_FORWARD_FACTOR, MOT_1_STRAFE_FACTOR),
       Thruster(1, MOT_2_ROLL_FACTOR, MOT_2_PITCH_FACTOR, MOT_2_YAW_FACTOR, MOT_2_THROTTLE_FACTOR, MOT_2_FORWARD_FACTOR, MOT_2_STRAFE_FACTOR),
       Thruster(2, MOT_3_ROLL_FACTOR, MOT_3_PITCH_FACTOR, MOT_3_YAW_FACTOR, MOT_3_THROTTLE_FACTOR, MOT_3_FORWARD_FACTOR, MOT_3_STRAFE_FACTOR),
       Thruster(3, MOT_4_ROLL_FACTOR, MOT_4_PITCH_FACTOR, MOT_4_YAW_FACTOR, MOT_4_THROTTLE_FACTOR, MOT_4_FORWARD_FACTOR, MOT_4_STRAFE_FACTOR),
       Thruster(4, MOT_5_ROLL_FACTOR, MOT_5_PITCH_FACTOR, MOT_5_YAW_FACTOR, MOT_5_THROTTLE_FACTOR, MOT_5_FORWARD_FACTOR, MOT_5_STRAFE_FACTOR),
       Thruster(5, MOT_6_ROLL_FACTOR, MOT_6_PITCH_FACTOR, MOT_6_YAW_FACTOR, MOT_6_THROTTLE_FACTOR, MOT_6_FORWARD_FACTOR, MOT_6_STRAFE_FACTOR)

};

Submarine::Submarine(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    frame(NULL)
{
    frame_height = 0.0;
    ground_behavior = GROUND_BEHAVIOR_NONE;
}

// calculate rotational and linear accelerations
void Submarine::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    rot_accel = Vector3f(0,0,0);

    // slight positive buoyancy
    body_accel = Vector3f(0, 0, -calculate_buoyancy_acceleration());

    for (int i = 0; i < 6; i++) {
        Thruster t = vectored_thrusters[i];
        int16_t pwm = input.servos[t.servo];
        float output = 0;
        if (pwm < 2000 && pwm > 1000) {
         output = (pwm - 1500) / 400.0; // range -1~1
        }

        // 2.5 scalar for approximate real-life performance of T200 thruster
        body_accel += t.linear * output * 2.5;

        rot_accel += t.rotational * output;
    }

    // Limit movement at the sea floor
    if (position.z > 100 && body_accel.z > -GRAVITY_MSS) {
    	body_accel.z = -GRAVITY_MSS;
    }

    // Calculate linear drag forces
    Vector3f linear_drag_forces;
    calculate_drag_force(velocity_air_bf, frame_property.linear_drag_coefficient, linear_drag_forces);
    // Add forces in body frame accel
    body_accel -= linear_drag_forces / frame_property.weight;

    // Calculate angular drag forces
    Vector3f angular_drag_forces;
    calculate_drag_force(gyro, frame_property.angular_drag_coefficient, angular_drag_forces);
    // Add forces in body frame accel
    rot_accel -= angular_drag_forces / frame_property.weight;
}

/**
 * @brief Calculate drag force against body
 *
 * @param velocity Body frame velocity of fluid
 * @param drag_coefficient Drag coefficient of body
 * @param force Output forces
 * $ F_D = rho * v^2 * A * C_D / 2 $
 * rho = water density (kg/m^3), V = velocity (m/s), A = area (m^2), C_D = drag_coefficient
 */
void Submarine::calculate_drag_force(const Vector3f &velocity, const Vector3f &drag_coefficient, Vector3f &force)
{
    /**
     * @brief It's necessary to keep the velocity orientation from the body frame.
     *     To do so, a mathematical artifice is used to do velocity square but without loosing the direction.
     *  $(|V|/V)*V^2$ = $|V|*V$
     */
    const Vector3f velocity_2(
        fabsf(velocity.x) * velocity.x,
        fabsf(velocity.y) * velocity.y,
        fabsf(velocity.z) * velocity.z
    );

    force = (velocity_2 * water_density) * frame_property.equivalent_sphere_area / 2.0f;
    force *= drag_coefficient;
}

/**
* @brief Calculate buoyancy force of the frame
*
* @return float
*/
float Submarine::calculate_buoyancy_acceleration()
{
    float below_water_level = position.z - frame_property.height/2;

    // Completely above water level
    if (below_water_level < 0) {
        return 0.0f;
    }

    // Completely below water level
    if (below_water_level > frame_property.height/2) {
        return frame_property.buoyancy_acceleration;
    }

    // bouyant force is proportional to fraction of height in water
    return frame_property.buoyancy_acceleration * below_water_level/frame_property.height;
};

/*
  update the Submarine simulation by one time step
 */
void Submarine::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel;

    calculate_forces(input, rot_accel, accel_body);

    update_dynamics(rot_accel);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

/*
   return true if we are on the ground
*/
bool Submarine::on_ground() const
{
	return false;
}
