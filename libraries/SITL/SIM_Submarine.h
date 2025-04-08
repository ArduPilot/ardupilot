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
  ROV/AUV/Submarine simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_Motor.h"
#include "SIM_Frame.h"

namespace SITL {

/*
  a submarine simulator
 */


class Thruster {
public:
    Thruster(int8_t _servo, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac, float lat_fac) :
        servo(_servo)
    {
        linear = Vector3f(forward_fac, lat_fac, -throttle_fac);
        rotational = Vector3f(roll_fac, pitch_fac, yaw_fac);
    };
    int8_t servo;
    Vector3f linear;
    Vector3f rotational;
};

class Submarine : public Aircraft {
public:
    Submarine(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW Submarine(frame_str);
    }


protected:
    const float water_density = 1023.6; // (kg/m^3) At a temperature of 25 °C, salinity of 35 g/kg and 1 atm pressure

    const struct {
        float length = 0.457; // x direction (meters)
        float width  = 0.338; // y direction (meters)
        float height = 0.254; // z direction (meters)
        float weight = 10.5;  // (kg)
        float thrust = 51.48; // (N)
        float thruster_mount_radius = 0.25; // distance in meters from thrusters to center of mass. Used to calculate torque.
        float equivalent_sphere_radius = 0.2;
        // volume = 4.pi.r³/3
        float volume = 4 * M_PI * powf(equivalent_sphere_radius, 3) / 3;
        float density = 500;
        float mass = volume * density; // 16.75 kg
        // Moment of Inertia (I)(kg.m²) approximated with a sphere with a 25 cm radius (r) and same density as water
        // I = 2.m.r²/5
        float moment_of_inertia =  2 * (mass * powf(equivalent_sphere_radius, 2) / 5);

        // Frame drag coefficient
        const Vector3f linear_drag_coefficient = Vector3f(1.4, 1.8, 2.0);
        // Angular drag coefficient CD for a cube is 1.05. This is subject to change based on experimentation.
        const Vector3f angular_drag_coefficient = Vector3f(1.05, 1.05, 1.05);
        // Calculate equivalent sphere area for drag force
        // $ A = pi * r^2 / 4 $
        // $ V = 4 * pi * r^3 / 3 $
        // $ r ^2 = (V * 3 / 4) ^ (2/3) $
        // A = area (m^3), r = sphere radius (m)
        float equivalent_sphere_area = M_PI_4 * powf(volume * 3.0f / 4.0f, 2.0f / 3.0f);

    } frame_property;

    bool on_ground() const override;

    float rangefinder_beam_width() const override { return 10; }

    float perpendicular_distance_to_rangefinder_surface() const override;

    // calculate sea floor depth based for terrain follow
    float calculate_sea_floor_depth(const Vector3d &/*position*/) const;
    // calculate rotational and linear accelerations
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
    // calculate buoyancy
    float calculate_buoyancy_acceleration();
    // calculate drag from velocity and drag coefficient
    void calculate_drag_force(const Vector3f &velocity, const Vector3f &drag_coefficient, Vector3f &force) const;
    // calculate torque water resistance
    void calculate_angular_drag_torque(const Vector3f &angular_velocity, const Vector3f &drag_coefficient, Vector3f &torque) const;
    // calculate torque induced by buoyancy foams
    void calculate_buoyancy_torque(Vector3f &torque);

    Frame *frame;
    Thruster* thrusters;
    uint8_t n_thrusters;
};
}
