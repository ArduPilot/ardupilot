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
  IMU temperature calibration handling
 */

#include "AP_InertialSensor.h"

#if HAL_INS_TEMPERATURE_CAL_ENABLE

// temperature calibration parameters, per IMU
const AP_Param::GroupInfo AP_InertialSensor::TCal::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable temperature calibration
    // @Description: Enable the use of temperature calibration parameters for this IMU
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_InertialSensor::TCal, enable,  0, AP_PARAM_FLAG_ENABLE),

    // @Param: TMIN
    // @DisplayName: Temperature calibration min
    // @Description: The minimum temperature that the calibration is valid for
    // @Range: -70 80
    // @Units: degC
    // @User: Advanced
    // @Calibration: 1
    AP_GROUPINFO("TMIN", 2, AP_InertialSensor::TCal, temp_min,  0),

    // @Param: TMAX
    // @DisplayName: Temperature calibration max
    // @Description: The maximum temperature that the calibration is valid for
    // @Range: -70 80
    // @Units: degC
    // @User: Advanced
    // @Calibration: 1
    AP_GROUPINFO("TMAX", 3, AP_InertialSensor::TCal, temp_max,  0),

    // @Param: ACC1_X
    // @DisplayName: Accelerometer 1st order temperature coefficient X axis
    // @Description: This is the 1st order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: ACC1_Y
    // @DisplayName: Accelerometer 1st order temperature coefficient Y axis
    // @Description: This is the 1st order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: ACC1_Z
    // @DisplayName: Accelerometer 1st order temperature coefficient Z axis
    // @Description: This is the 1st order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1
    
    AP_GROUPINFO("ACC1", 4, AP_InertialSensor::TCal, accel_coeff[0], 0),

    // @Param: ACC2_X
    // @DisplayName: Accelerometer 2nd order temperature coefficient X axis
    // @Description: This is the 2nd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: ACC2_Y
    // @DisplayName: Accelerometer 2nd order temperature coefficient Y axis
    // @Description: This is the 2nd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: ACC2_Z
    // @DisplayName: Accelerometer 2nd order temperature coefficient Z axis
    // @Description: This is the 2nd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    AP_GROUPINFO("ACC2", 5, AP_InertialSensor::TCal, accel_coeff[1], 0),

    // @Param: ACC3_X
    // @DisplayName: Accelerometer 3rd order temperature coefficient X axis
    // @Description: This is the 3rd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: ACC3_Y
    // @DisplayName: Accelerometer 3rd order temperature coefficient Y axis
    // @Description: This is the 3rd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: ACC3_Z
    // @DisplayName: Accelerometer 3rd order temperature coefficient Z axis
    // @Description: This is the 3rd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    AP_GROUPINFO("ACC3", 6, AP_InertialSensor::TCal, accel_coeff[2], 0),

    // @Param: GYR1_X
    // @DisplayName: Gyroscope 1st order temperature coefficient X axis
    // @Description: This is the 1st order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: GYR1_Y
    // @DisplayName: Gyroscope 1st order temperature coefficient Y axis
    // @Description: This is the 1st order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: GYR1_Z
    // @DisplayName: Gyroscope 1st order temperature coefficient Z axis
    // @Description: This is the 1st order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1
    
    AP_GROUPINFO("GYR1", 7, AP_InertialSensor::TCal, gyro_coeff[0], 0),

    // @Param: GYR2_X
    // @DisplayName: Gyroscope 2nd order temperature coefficient X axis
    // @Description: This is the 2nd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: GYR2_Y
    // @DisplayName: Gyroscope 2nd order temperature coefficient Y axis
    // @Description: This is the 2nd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: GYR2_Z
    // @DisplayName: Gyroscope 2nd order temperature coefficient Z axis
    // @Description: This is the 2nd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    AP_GROUPINFO("GYR2", 8, AP_InertialSensor::TCal, gyro_coeff[1], 0),

    // @Param: GYR3_X
    // @DisplayName: Gyroscope 3rd order temperature coefficient X axis
    // @Description: This is the 3rd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: GYR3_Y
    // @DisplayName: Gyroscope 3rd order temperature coefficient Y axis
    // @Description: This is the 3rd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    // @Param: GYR3_Z
    // @DisplayName: Gyroscope 3rd order temperature coefficient Z axis
    // @Description: This is the 3rd order temperature coefficient from a temperature calibration
    // @User: Advanced
    // @Calibration: 1

    AP_GROUPINFO("GYR3", 9, AP_InertialSensor::TCal, gyro_coeff[2], 0),

    AP_GROUPEND
};


/*
  evaluate a 3rd order polynomial (without the constant term) given a set of coefficients
 */
Vector3f AP_InertialSensor::TCal::polynomial_eval(float tdiff, const AP_Vector3f coeff[3]) const
{
    // evaluate order 3 polynomial
    const Vector3f *c = (Vector3f *)&coeff[0];
    // this scale factor ensures params are easy to work with in GUI parameter editors
    const float scale_factor = 1.0e-6;
    return (c[0] + (c[1] + c[2]*tdiff)*tdiff)*tdiff*scale_factor;
}

/*
  correct a single sensor for the current temperature
 */
void AP_InertialSensor::TCal::correct_sensor(float temperature, float cal_temp, const AP_Vector3f coeff[3], Vector3f &v) const
{
    if (!enable) {
        return;
    }
    temperature = constrain_float(temperature, temp_min, temp_max);
    cal_temp = constrain_float(cal_temp, temp_min, temp_max);

    const float tmid = (temp_max + temp_min)*0.5;
    if (tmid <= 0) {
        return;
    }

    // get the polynomial correction for the difference between the
    // current temperature and the mid temperature
    v -= polynomial_eval(temperature - tmid, coeff);

    // we need to add the correction for the temperature
    // difference between the tmid, which is the reference used for
    // the calibration process, and the cal_temp, which is the
    // temperature that the offsets and scale factors was setup for
    v += polynomial_eval(cal_temp - tmid, coeff);
}

void AP_InertialSensor::TCal::correct_accel(float temperature, float cal_temp, Vector3f &accel) const
{
    correct_sensor(temperature, cal_temp, accel_coeff, accel);
}

void AP_InertialSensor::TCal::correct_gyro(float temperature, float cal_temp, Vector3f &gyro) const
{
    correct_sensor(temperature, cal_temp, gyro_coeff, gyro);
}

void AP_InertialSensor::TCal::sitl_apply_accel(float temperature, Vector3f &accel) const
{
    Vector3f v;
    correct_sensor(temperature, 0.5*(temp_max+temp_min), accel_coeff, v);
    accel -= v;
}

void AP_InertialSensor::TCal::sitl_apply_gyro(float temperature, Vector3f &gyro) const
{
    Vector3f v;
    correct_sensor(temperature, 0.5*(temp_max+temp_min), gyro_coeff, v);
    gyro -= v;
}

#endif // HAL_INS_TEMPERATURE_CAL_ENABLE
