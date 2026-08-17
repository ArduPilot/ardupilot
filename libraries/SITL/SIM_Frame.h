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
  multicopter simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_Motor.h"
#include <AP_JSON/AP_JSON.h>
#include <AP_Param/AP_Param.h>

#ifndef SIM_FRAME_MAX_ACTUATORS
#define SIM_FRAME_MAX_ACTUATORS 32
#endif

namespace SITL {

/*
  class to describe a multicopter frame type
 */
class Frame {
public:
    const char *name;
    uint8_t num_motors;
    Motor *motors;

    Frame(const char *_name,
          uint8_t _num_motors,
          Motor *_motors) :
          name(_name),
          num_motors(_num_motors),
          motors(_motors) {}

#if AP_SIM_ENABLED
    // create a frame by name from its template
    static Frame *create_frame(const char *name);
    
    // initialise frame
    void init(const char *frame_str);

    // calculate rotational and linear accelerations
    void calculate_forces(const Aircraft &aircraft,
                          const struct sitl_input &input,
                          Vector3f &rot_accel, Vector3f &body_accel, float* rpm);
#endif // AP_SIM_ENABLED

    static const struct AP_Param::GroupInfo var_info[];

    float terminal_velocity;
    float terminal_rotation_rate;
    uint8_t motor_offset;

    float get_current_amp(void);

    // get mass in kg
    float get_mass(void) const {
        return mass;
    }

    // scale factor on model mass, used by quadplane to allow for plane components
    void set_mass_scale(float scale) {
        mass_scale = scale;
    }

    float get_model_batt_max_voltage(void) const { return model.maxVoltage; }
    float get_model_batt_capacity_ah(void) const { return model.battCapacityAh; }
    float get_model_batt_resistance_ohm(void) const { return model.refBatRes; }

    // returns true once when the battery model values have changed
    bool battery_changed(void);
    
private:
    /*
      parameters that define the multicopter model. The scalars are
      exposed as SIM_FRM_ parameters, defaults come from the parameter
      table and can be overridden by loading a json model file
     */
    struct Model {
        // model mass kg
        AP_Float mass;

        // diameter of model
        AP_Float diagonal_size;

        /*
          the ref values are for a test at fixed angle, used to estimate drag
         */
        AP_Float refSpd; // m/s
        AP_Float refAngle;  // degrees
        AP_Float refVoltage; // Volts
        AP_Float refCurrent; // Amps
        AP_Float refAlt; // altitude AMSL
        float refTempC = 25; // temperature C, unused

        // battery resistance reference value in Ohms
        AP_Float refBatRes;

        // full pack voltage
        AP_Float maxVoltage;

        // battery capacity in Ah. Use zero for unlimited
        AP_Float battCapacityAh;

        // CTUN.ThO at hover at refAlt
        AP_Float hoverThrOut;

        // MOT_THST_EXPO
        AP_Float propExpo;

        // scaling factor for yaw response, deg/sec
        AP_Float refRotRate;

        // MOT params are from the reference test
        // MOT_PWM_MIN
        AP_Float pwmMin;
        // MOT_PWM_MAX
        AP_Float pwmMax;
        // MOT_SPIN_MIN
        AP_Float spin_min;
        // MOT_SPIN_MAX
        AP_Float spin_max;

        // maximum slew rate of motors
        AP_Float slew_max;

        // rotor disc area in m**2
        // Note that coaxial rotors count as one rotor only when calculating effective disc area
        AP_Float disc_area;

        // momentum drag coefficient
        AP_Float mdrag_coef;

        // bluff body drag scaling
        AP_Float bbdrag_coef;

        // if zero value will be estimated from mass
        Vector3f moment_of_inertia;

        Vector3f motor_pos[SIM_FRAME_MAX_ACTUATORS];
        Vector3f motor_thrust_vec[SIM_FRAME_MAX_ACTUATORS];
        float yaw_factor[SIM_FRAME_MAX_ACTUATORS] {0,};

        // number of motors
        float num_motors = 4;
    };

protected:
    // load frame parameters from a json model file
    void load_frame_params(const char *model_json);

    // get air density in kg/m^3
    float get_air_density(float alt_amsl) const;

    struct Model model;

private:
    // apply parameter based settings, called on each physics step so
    // that SIM_FRM_ parameter changes take effect
    void update_parameters(void);

    // exposed area times coefficient of drag
    float areaCd;
    float mass;
    float mass_scale = 1.0;

    // effective momentum drag coefficient, possibly scaled down from model.mdrag_coef
    float mdrag_coef;

    // moment of inertia in use, from model or estimated from mass
    Vector3f moment_of_inertia;

    // last printed EK3 drag suggestions
    float last_drag_bcoef;
    float last_drag_mcoef;

    // battery model change detection
    bool battery_dirty;
    float last_batt_voltage;
    float last_batt_cap;
    float last_batt_res;

    // json parsing helpers
    void parse_float(AP_JSON::value val, const char* label, float &param);
    void parse_vector3(AP_JSON::value val, const char* label, Vector3f &param);
};
}
