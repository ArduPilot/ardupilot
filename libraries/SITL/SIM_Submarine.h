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


class Submarine : public Aircraft {
public:
    Submarine(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Submarine(home_str, frame_str);
    }


protected:
    const float water_density = 1023.6; // (kg/m^3) At a temperature of 25 °C, salinity of 35 g/kg and 1 atm pressure

    const struct {
        float length = 0.457; // x direction (meters)
        float width  = 0.338; // y direction (meters)
        float height = 0.254; // z direction (meters)
        float weight = 10.5;  // (kg)
        float net_bouyancy = 2.0; // (N)

        float bouyancy_acceleration = GRAVITY_MSS + net_bouyancy/weight;

        // Frame drag coefficient
        const Vector3f linear_drag_coefficient = Vector3f(0.2, 0.25, 0.3);
        const Vector3f angular_drag_coefficient = Vector3f(0.25, 0.3, 0.2);
        // Calculate total volume from water buoyancy
        // $ V = F_b / (rho * g) $
        // V = volume (m^3), rho = water density (kg/m^3), g = gravity (m/s^2), F_b = force (N)
        float volume = buoyancy_acceleration * weight / (GRAVITY_MSS * 1023.6f);
        // Calculate equivalent sphere area for drag force
        // $ A = pi * r^2 / 4 $
        // $ V = 4 * pi * r^3 / 3 $
        // $ r ^2 = (V * 3 / 4) ^ (2/3) $
        // A = area (m^3), r = sphere radius (m)
        float equivalent_sphere_area = M_PI_4 * pow(volume * 3 / 4, 2 / 3);

    } frame_property;

    bool on_ground() const override;

    // calculate rotational and linear accelerations
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
    // calculate buoyancy
    float calculate_buoyancy_acceleration();
    // calculate drag from velocity and drag coefficient
    void calculate_drag_force(const Vector3f &velocity, const Vector3f &drag_coefficient, Vector3f &force);

    Frame *frame;
};

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
}
