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

#include <stdio.h>

using namespace SITL;

static Thruster vectored_thrusters[] =
{      //       Motor #     Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor
       Thruster(0,          0,              0,              1.0f,           0,                  -1.0f,              1.0f),
       Thruster(1,          0,              0,              -1.0f,          0,                  -1.0f,              -1.0f),
       Thruster(2,          0,              0,              -1.0f,          0,                  1.0f,               1.0f),
       Thruster(3,          0,              0,              1.0f,           0,                  1.0f,               -1.0f),
       Thruster(4,          1.0f,           0,              0,              -1.0f,              0,                  0),
       Thruster(5,          -1.0f,          0,              0,              -1.0f,              0,                  0)
};


static Thruster vectored_6dof_thrusters[] =
{
       //       Motor #     Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor
       Thruster(0,          0,              0,              1.0f,           0,                  -1.0f,              1.0f),
       Thruster(1,          0,              0,              -1.0f,          0,                  -1.0f,              -1.0f),
       Thruster(2,          0,              0,              -1.0f,          0,                  1.0f,               1.0f),
       Thruster(3,          0,              0,              1.0f,           0,                  1.0f,               -1.0f),
       Thruster(4,          1.0f,           -1.0f,          0,              -1.0f,              0,                  0),
       Thruster(5,          -1.0f,          -1.0f,          0,              -1.0f,              0,                  0),
       Thruster(6,          1.0f,           1.0f,           0,              -1.0f,              0,                  0),
       Thruster(7,          -1.0f,          1.0f,           0,              -1.0f,              0,                  0)
};

Submarine::Submarine(const char *frame_str) :
    Aircraft(frame_str),
    frame(NULL)
{
    frame_height = 0.0;
    ground_behavior = GROUND_BEHAVIOR_NONE;

    // default to vectored frame
    thrusters = vectored_thrusters;
    n_thrusters = 6;

    if (strstr(frame_str, "vectored_6dof")) {
        thrusters = vectored_6dof_thrusters;
        n_thrusters = 8;
    }
}

// calculate rotational and linear accelerations
void Submarine::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    rot_accel = Vector3f(0,0,0);

    // slight positive buoyancy
    body_accel = dcm.transposed() *  Vector3f(0, 0, -calculate_buoyancy_acceleration());

    for (int i = 0; i < n_thrusters; i++) {
        Thruster t = thrusters[i];
        int16_t pwm = input.servos[t.servo];
        float output = 0;
        // if valid pwm and not in the esc deadzone
        // TODO: extract deadzone from parameters/vehicle code
        if (pwm < 2000 && pwm > 1000 && (pwm < 1475 || pwm > 1525)) {
            output = (pwm - 1500) / 400.0; // range -1~1
        }

        float thrust = output * fabs(output) * frame_property.thrust; // approximate pwm to thrust function using a quadratic curve
        body_accel += t.linear * thrust / frame_property.weight;
        rot_accel += t.rotational * thrust * frame_property.thruster_mount_radius / frame_property.moment_of_inertia;
    }

    float floor_depth = calculate_sea_floor_depth(position);
    range = floor_depth - position.z;
    // Limit movement at the sea floor
    if (position.z > floor_depth && body_accel.z > -GRAVITY_MSS) {
    	body_accel.z = -GRAVITY_MSS;
    }

    // Calculate linear drag forces
    Vector3f linear_drag_forces;
    calculate_drag_force(velocity_air_bf, frame_property.linear_drag_coefficient, linear_drag_forces);
    // Add forces in body frame accel
    body_accel -= linear_drag_forces / frame_property.weight;

    // Calculate angular drag forces
    // TODO: This results in the wrong units. Fix the math.
    Vector3f angular_drag_torque;
    calculate_angular_drag_torque(gyro, frame_property.angular_drag_coefficient, angular_drag_torque);

    // Calculate torque induced by buoyancy foams on the frame
    Vector3f buoyancy_torque;
    calculate_buoyancy_torque(buoyancy_torque);
    // Add forces in body frame accel
    rot_accel -= angular_drag_torque / frame_property.moment_of_inertia;
    rot_accel += buoyancy_torque / frame_property.moment_of_inertia;
    add_shove_forces(rot_accel, body_accel);
}


/**
 * @brief Calculate the torque induced by buoyancy foam
 *
 * @param torque Output torques
 */
void Submarine::calculate_buoyancy_torque(Vector3f &torque)
{
    // Let's assume 2 Liters water displacement at the top, and ~ 2kg of weight at the bottom.
    const Vector3f force_up(0,0,-40); // 40 N upwards
    const Vector3f force_position = dcm.transposed() * Vector3f(0, 0, 0.15); // offset in meters
    torque = force_position % force_up;
}


/**
 * @brief Calculate sea floor depth from submarine position
 *          This creates a non planar floor for rangefinder sensor test
 *          TODO: Create a better sea floor with procedural generatation
 *
 * @param position
 * @return float
 */
float Submarine::calculate_sea_floor_depth(const Vector3f &/*position*/)
{
    return 50;
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
 * @brief Calculate angular drag torque using the equivalente sphere area and assuming a laminar external flow.
 *
 *  $F_D = C_D*A*\rho*V^2/2$
 * where:
 *      $F_D$ is the drag force
 *      $C_D$ is the drag coefficient
 *      $A$ is the surface area in contact with the fluid
 *      $/rho$ is the fluid density (1000kg/m³ for water)
 *      $V$ is the fluid velocity velocity relative to the surface
 *
 * @param angular_velocity Body frame velocity of fluid
 * @param drag_coefficient Rotational drag coefficient of body
 */
void Submarine::calculate_angular_drag_torque(const Vector3f &angular_velocity, const Vector3f &drag_coefficient, Vector3f &torque)
{
     /**
     * @brief It's necessary to keep the velocity orientation from the body frame.
     *     To do so, a mathematical artifice is used to do velocity square but without loosing the direction.
     *  $(|V|/V)*V^2$ = $|V|*V$
     */
    Vector3f v_2(
        fabsf(angular_velocity.x) * angular_velocity.x,
        fabsf(angular_velocity.y) * angular_velocity.y,
        fabsf(angular_velocity.z) * angular_velocity.z
    );
    Vector3f f_d = v_2 *= drag_coefficient * frame_property.equivalent_sphere_area * 1000 / 2;
    torque = f_d * frame_property.equivalent_sphere_radius;
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
        return GRAVITY_MSS + sitl->buoyancy / frame_property.mass;
    }

    // bouyant force is proportional to fraction of height in water
    return GRAVITY_MSS + (sitl->buoyancy * below_water_level/frame_property.height) / frame_property.mass;
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
