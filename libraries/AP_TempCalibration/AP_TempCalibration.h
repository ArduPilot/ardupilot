#pragma once
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
  temperature calibration library. This monitors temperature changes
  and opportunistically calibrates sensors when the vehicle is still
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

class AP_TempCalibration
{
public:
    // constructor.  This remains because construction of Copter's g2
    // object becomes problematic if we don't have at least one object
    // to initialise
    AP_TempCalibration() {}

    // settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    void update(void);

    /* Do not allow copies */
    AP_TempCalibration(const AP_TempCalibration &other) = delete;
    AP_TempCalibration &operator=(const AP_TempCalibration&) = delete;

    enum {
        TC_DISABLED = 0,
        TC_ENABLE_USE = 1,
        TC_ENABLE_LEARN = 2,
    };
    
private:

    AP_Int8 enabled;
    AP_Int8 temp_min;
    AP_Int8 temp_max;
    AP_Float baro_exponent;
    
    Vector3f last_accels;

    float learn_temp_start;
    float learn_temp_step;
    uint16_t learn_count;
    uint16_t learn_i;
    float *learn_values;
    uint32_t last_learn_ms;

    // temperature at which baro correction starts
    const float Tzero = 25;

    const float exp_limit_max = 2;
    const float exp_limit_min = 0;
    float learn_delta = 0.01f;
    
    // require observation of at least 5 degrees of temp range to
    // start learning
    const float min_learn_temp_range = 7;
    
    void setup_learning(void);
    void learn_calibration(void);
    void apply_calibration(void);
    void calculate_calibration();
    float calculate_correction(float temp, float exponent) const;
    float calculate_p_range(float baro_factor) const;
    
};
