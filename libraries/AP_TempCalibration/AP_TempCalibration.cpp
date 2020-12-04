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
  temperature calibration library
 */

#include "AP_TempCalibration.h"
#include <stdio.h>
#include <AP_Baro/AP_Baro.h>

extern const AP_HAL::HAL& hal;

#define TCAL_DEBUG 0

#if TCAL_DEBUG
# define debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
# define debug(fmt, args ...)
#endif

// table of user settable and learned parameters
const AP_Param::GroupInfo AP_TempCalibration::var_info[] = {

    // @Param: _ENABLED
    // @DisplayName: Temperature calibration enable
    // @Description: Enable temperature calibration. Set to 0 to disable. Set to 1 to use learned values. Set to 2 to learn new values and use the values
    // @Values: 0:Disabled,1:Enabled,2:EnableAndLearn
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLED", 1, AP_TempCalibration, enabled, TC_DISABLED, AP_PARAM_FLAG_ENABLE),

    // @Param: _TEMP_MIN
    // @DisplayName: Temperature calibration min learned temperature
    // @Description: Minimum learned temperature. This is automatically set by the learning process
    // @Units: degC
    // @ReadOnly: True
    // @Volatile: True
    // @User: Advanced
    AP_GROUPINFO("_TEMP_MIN", 2, AP_TempCalibration, temp_min, 0),

    // 3 was used by a duplicated temp_min entry (do not use in the future!)

    // @Param: _TEMP_MAX
    // @DisplayName: Temperature calibration max learned temperature
    // @Description: Maximum learned temperature. This is automatically set by the learning process
    // @Units: degC
    // @ReadOnly: True
    // @Volatile: True
    // @User: Advanced
    AP_GROUPINFO("_TEMP_MAX", 4, AP_TempCalibration, temp_max, 0),

    // @Param: _BARO_EXP
    // @DisplayName: Temperature Calibration barometer exponent
    // @Description: Learned exponent for barometer temperature correction
    // @ReadOnly: True
    // @Volatile: True
    // @User: Advanced
    AP_GROUPINFO("_BARO_EXP", 5, AP_TempCalibration, baro_exponent, 0),
    
    AP_GROUPEND
};

/*
  calculate the correction given an exponent and a temperature 

  This one parameter correction is deliberately chosen to be very
  robust for extrapolation. It fits the characteristics of the
  ICM-20789 barometer nicely.
 */
float AP_TempCalibration::calculate_correction(float temp, float exponent) const
{
    return powf(MAX(temp - Tzero, 0), exponent);
}


/*
  setup for learning
 */
void AP_TempCalibration::setup_learning(void)
{
    learn_temp_start = AP::baro().get_temperature();
    learn_temp_step = 0.25;
    learn_count = 200;
    learn_i = 0;
    if (learn_values != nullptr) {
        delete [] learn_values;
    }
    learn_values = new float[learn_count];
    if (learn_values == nullptr) {
        return;
    }
}

/*
  calculate the sum of squares range of pressure values we get with
  the current data. This is the function we try to minimise in the
  calibration
 */
float AP_TempCalibration::calculate_p_range(float baro_factor) const
{
    float sum = 0;
    float P0 = learn_values[0] + calculate_correction(learn_temp_start, baro_factor);
    for (uint16_t i=0; i<learn_i; i++) {
        if (is_zero(learn_values[i])) {
            // gap in the data
            continue;
        }
        float temp = learn_temp_start + learn_temp_step*i;
        float correction = calculate_correction(temp, baro_factor);
        float P = learn_values[i] + correction;
        sum += sq(P - P0);
    }
    return sum / learn_i;
}

/*
  calculate a calibration value

  This fits a simple single value power function to the baro data to
  find the calibration exponent.
 */
void AP_TempCalibration::calculate_calibration(void)
{
    float current_err = calculate_p_range(baro_exponent);
    float test_exponent = baro_exponent + learn_delta;
    float test_err = calculate_p_range(test_exponent);
    if (test_err >= current_err) {
        test_exponent = baro_exponent - learn_delta;
        test_err = calculate_p_range(test_exponent);
    }
    if (test_exponent <= exp_limit_max &&
        test_exponent >= exp_limit_min &&
        test_err < current_err) {
        // move to new value
        debug("CAL: %.2f\n", test_exponent);
        if (!is_equal(test_exponent, baro_exponent.get())) {
            baro_exponent.set_and_save(test_exponent);
        }
        temp_min.set_and_save_ifchanged(learn_temp_start);
        temp_max.set_and_save_ifchanged(learn_temp_start + learn_i*learn_temp_step);
    }
}

/*
  update calibration learning
 */
void AP_TempCalibration::learn_calibration(void)
{
    // just for first baro now
    const AP_Baro &baro = AP::baro();
    if (!baro.healthy(0) ||
        hal.util->get_soft_armed() ||
        baro.get_temperature(0) < Tzero) {
        return;
    }

    // if we have any movement then we reset learning
    if (learn_values == nullptr ||
        !AP::ins().is_still()) {
        debug("learn reset\n");
        setup_learning();
        if (learn_values == nullptr) {
            return;
        }
    }
    float temp = baro.get_temperature(0);
    float P = baro.get_pressure(0);
    uint16_t idx = (temp - learn_temp_start) / learn_temp_step;
    if (idx >= learn_count) {
        // could change learn_temp_step here
        return;
    }
    if (is_zero(learn_values[idx])) {
        learn_values[idx] = P;
        debug("learning %u %.2f at %.2f\n", idx, learn_values[idx], temp);
    } else {
        // filter in new value
        learn_values[idx] = 0.9 * learn_values[idx] + 0.1 * P;
    }
    learn_i = MAX(learn_i, idx);
    
    uint32_t now = AP_HAL::millis();
    if (now - last_learn_ms > 100 &&
        idx*learn_temp_step > min_learn_temp_range &&
        temp - learn_temp_start > temp_max - temp_min) {
        last_learn_ms = now;
        // run estimation and update parameters
        calculate_calibration();
    }
}

/*
  apply learned calibration for current temperature
 */
void AP_TempCalibration::apply_calibration(void)
{
    AP_Baro &baro = AP::baro();
    // just for first baro now
    if (!baro.healthy(0)) {
        return;
    }
    float temp = baro.get_temperature(0);
    float correction = calculate_correction(temp, baro_exponent);
    baro.set_pressure_correction(0, correction);
}

/*
  called at 10Hz from the main thread. This is called both when armed
  and disarmed. It only does learning while disarmed, but needs to
  supply the corrections to the sensor libraries at all times
 */
void AP_TempCalibration::update(void)
{
    switch (enabled.get()) {
    case TC_DISABLED:
        break;
    case TC_ENABLE_LEARN:
        learn_calibration();
        FALLTHROUGH;
    case TC_ENABLE_USE:
        apply_calibration();
        break;
    }
}
