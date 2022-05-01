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

#ifndef USE_PICOJSON
#define USE_PICOJSON (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if USE_PICOJSON
#include "picojson.h"
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
    // find a frame by name
    static Frame *find_frame(const char *name);
    
    // initialise frame
    void init(const char *frame_str, Battery *_battery);

    // calculate rotational and linear accelerations
    void calculate_forces(const Aircraft &aircraft,
                          const struct sitl_input &input,
                          Vector3f &rot_accel, Vector3f &body_accel, float* rpm,
                          bool use_drag=true);
#endif // AP_SIM_ENABLED

    float terminal_velocity;
    float terminal_rotation_rate;
    uint8_t motor_offset;

    // calculate current and voltage
    void current_and_voltage(float &voltage, float &current);

    // get mass in kg
    float get_mass(void) const {
        return mass;
    }

    // set mass in kg
    void set_mass(float new_mass) {
        mass = new_mass;
    }
    
private:
    /*
      parameters that define the multicopter model. Can be loaded from
      a json file to give a custom model
     */
    const struct Model {
        // model mass kg
        float mass = 3.0;

        // diameter of model
        float diagonal_size = 0.35;

        /*
          the ref values are for a test at fixed angle, used to estimate drag
         */
        float refSpd = 15.08; // m/s
        float refAngle = 45;  // degrees
        float refVoltage = 12.09; // Volts
        float refCurrent = 29.3; // Amps
        float refAlt = 593; // altitude AMSL
        float refTempC = 25; // temperature C
        float refBatRes = 0.01; // BAT.Res

        // full pack voltage
        float maxVoltage = 4.2*3;

        // battery capacity in Ah. Use zero for unlimited
        float battCapacityAh = 0.0;

        // CTUN.ThO at hover at refAlt
        float hoverThrOut = 0.39;

        // MOT_THST_EXPO
        float propExpo = 0.65;

        // scaling factor for yaw response, deg/sec
        float refRotRate = 120;

        // MOT params are from the reference test
        // MOT_PWM_MIN
        float pwmMin = 1000;
        // MOT_PWM_MAX
        float pwmMax = 2000;
        // MOT_SPIN_MIN
        float spin_min = 0.15;
        // MOT_SPIN_MAX
        float spin_max = 0.95;

        // maximum slew rate of motors
        float slew_max = 150;

        // rotor disc area in m**2 for 4 x 0.35m dia rotors
        // Note that coaxial rotors count as one rotor only when calculating effective disc area
        float disc_area = 0.385;

        // momentum drag coefficient
        float mdrag_coef = 0.2;

        // if zero value will be estimated from mass
        Vector3f moment_of_inertia;

        // if zero will no be used
        Vector3f motor_pos[12];
        Vector3f motor_thrust_vec[12];
        float yaw_factor[12] = {0};

        // number of motors
        float num_motors = 4;

    } default_model;

protected:
#if USE_PICOJSON
    // load frame parameters from a json model file
    void load_frame_params(const char *model_json);
#endif

    // get air density in kg/m^3
    float get_air_density(float alt_amsl) const;

    struct Model model;

private:
    // exposed area times coefficient of drag
    float areaCd;
    float mass;
    float last_param_voltage;
#if AP_SIM_ENABLED
    Battery *battery;
#endif

    // json parsing helpers
#if USE_PICOJSON
    void parse_float(picojson::value val, const char* label, float &param);
    void parse_vector3(picojson::value val, const char* label, Vector3f &param);
#endif
};
}
