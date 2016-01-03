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
  simple plane simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a very simple plane simulator
 */
class Plane : public Aircraft {
public:
    Plane(const char *home_str, const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Plane(home_str, frame_str);
    }

protected:
    const float hover_throttle = 1.2f;
    const float cruise_airspeed = 20;
    const float cruise_pitch = radians(4);
    const float wing_efficiency = 0.9;
    const float wing_span = 2.0;
    const float wing_chord = 0.15;
    const float aspect_ratio = wing_span / wing_chord;
    const float wing_area = wing_span * wing_chord;
    const float air_density = 1.225; // kg/m^3 at sea level, ISA conditions
    float angle_of_attack;
    float beta;
    Vector3f velocity_bf;

    // manually tweaked coefficients. Not even close to reality
    struct {
        float drag = 0.005;
        float lift = 2.0;
        float lift_drag = 0.5;
        float vertical_stabiliser = 0.1;
        float horizontal_stabiliser = 2;
        float dihedral = 0.1;
    } coefficient;

    float thrust_scale;
    Vector3f terminal_rotation_rate{radians(170), radians(200), radians(180)};
    Vector3f max_rates{radians(350), radians(250), radians(100)};

    float calculate_lift(void) const;
    float calculate_drag_induced(void) const;
    float calculate_drag_form(void) const;
    Vector3f calculate_lift_drag(void) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
};

} // namespace SITL
