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
  Glider model for high altitude balloon drop
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_GLIDER_ENABLED

#include "SIM_Aircraft.h"
#include <AP_Param/AP_Param.h>

namespace SITL {

/*
  a very simple plane simulator
 */
class Glider : public Aircraft {
public:
    Glider(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Glider(frame_str);
    }

    bool on_ground() const override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    float alpharad;
    float betarad;

    AP_Float balloon_burst_amsl;
    AP_Float balloon_rate;

    /*
      parameters that define the glider model
     */
    const struct Model {
        // total vehicle mass
        float mass = 9.07441; // kg

        // reference area
        float Sref = 0.92762; // m^2

        float refSpan = 1.827411; // m
        float refChord = 0.507614; // m
        float IXX = 0.234; // kg-m^2
        float IYY = 1.85; // kg-m^2
        float IZZ = 2.04; // kg-m^2

        // CN is coefficients for forces on +Z axis
        // quadratic in alpharad
        float CN2 = -0.5771;
        float CN1 = 3.9496;
        float CN0 = 0;

        // CA is the coefficients for forces on +X axis
        // quadratic in alpharad
        float CA2 = -1.6809;
        float CA1 = -0.0057;
        float CA0 = 0.0150;

        // CY is the coefficients for forces on the +Y axis
        // quadratic in alpharad, with betarad factor
        float CY2 = -3.342;
        float CY1 = 0.0227;
        float CY0 = -0.4608;

        // Cl is the coefficients for moments on X axis
        // quadratic in alpharad, with betarad factor
        float Cl2 = 0.2888;
        float Cl1 = -0.8518;
        float Cl0 = -0.0491;

        // Cm is the coefficients for moments on Y axis
        // quadratic in alpharad
        float Cm2 = 0.099;
        float Cm1 = -0.6506;
        float Cm0 = -0.0005;

        // Cn is the coefficients for moments on Z axis
        // quadratic in alpharad, with betarad factor
        float Cn2 = 0.0057;
        float Cn1 = -0.0101;
        float Cn0 = 0.1744;

        // controls neutral dynamic derivatives
        // p, q, r are gyro rates
        float Cmq = -6.1866;

        float Clp2 = 0.156;
        float Clp1 = 0.0129;
        float Clp0 = -0.315;

        float Clr2 = -0.0284;
        float Clr1 = 0.2641;
        float Clr0 = 0.0343;

        float Cnp2 = 0.0199;
        float Cnp1 = -0.315;
        float Cnp0 = -0.013;

        float Cnr2 = 0.1297;
        float Cnr1 = 0.0343;
        float Cnr0 = -0.264;

        // elevator
        float elevatorDeflectionLimitDeg = -12.5;
        float deltaCNperRadianElev = -0.7;
        float deltaCAperRadianElev = 0.12;
        float deltaCmperRadianElev = 1.39;
        float deltaCYperRadianElev = 0;
        float deltaClperRadianElev = 0;
        float deltaCnperRadianElev = 0;

        // rudder
        float rudderDeflectionLimitDeg = 18.0;
        float deltaCNperRadianRud = 0;
        float deltaCAperRadianRud = 0.058;
        float deltaCmperRadianRud = 0;
        float deltaCYperRadianRud = 0.31;
        float deltaClperRadianRud = 0.038;
        float deltaCnperRadianRud = -0.174;

        // aileron
        float aileronDeflectionLimitDeg = 15.5;
        float deltaCNperRadianAil = 0;
        float deltaCAperRadianAil = 0.016;
        float deltaCmperRadianAil = 0;
        float deltaCYperRadianAil = -0.015;

        // quadratic in alpharad
        float deltaClperRadianAil0 = 0.09191;
        float deltaClperRadianAil1 = 0.0001;
        float deltaClperRadianAil2 = -0.08645;

        // quadratic in alpharad
        float deltaCnperRadianAil0 = 0.00789;
        float deltaCnperRadianAil1 = 0.00773;
        float deltaCnperRadianAil2 = -0.01162;

        // Forces in the +X direction are –CA * q * Sref
        // Forces in the +Y direction are  +CY * q * Sref
        // Forces in the +Z direction are  –CN * q *Sref
        // Moments about the X axis are +Cl * q * Sref * RefSpan
        // Moments about the Y axis are +Cm * q * Sref * RefChord
        // Moments about the Z axis are +Cn * q * Sref * RefSpan

        // low altitude
        float alphaRadMax = 0.209;
        float betaRadMax = 0.209;

        // balloon launch parameters
        float tetherLength = 50.0f;       // length of tether from balloon to aircraft (m)
        float tetherPogoFreq = 2.0f;      // measured vertical frequency of on tether (Hz)

    } model;

    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder);
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, const Vector3f &force) const;
    bool update_balloon(float balloon, Vector3f &force, Vector3f &rot_accel);
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);

    Vector3f balloon_velocity;           // balloon velocity NED
    Vector3f balloon_position{0.0f, 0.0f, -45.0f}; // balloon position NED from origin

    enum class carriageState {
        NONE = 0, // no carriage option available
        WAITING_FOR_PICKUP = 1, // in launch cradle waiting to be picked up by launch vehicle
        WAITING_FOR_RELEASE = 2, // being carried by luanch vehicle waitng to be released
        PRE_RELEASE = 3, // had been released by launch vehicle
        RELEASED = 4 // had been released by launch vehicle
    } carriage_state;
    bool plane_air_release;    // true when plane has separated from the airborne launching platform

    uint32_t last_drag_ms;
    float sim_LD;
};

} // namespace SITL

#endif // AP_SIM_GLIDER_ENABLED
