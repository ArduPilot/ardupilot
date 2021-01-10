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

#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

// this scale factor ensures params are easy to work with in GUI parameter editors
#define SCALE_FACTOR 1.0e6
#define INV_SCALE_FACTOR 1.0e-6
#define TEMP_RANGE_MIN 15

extern const AP_HAL::HAL& hal;

// temperature calibration parameters, per IMU
const AP_Param::GroupInfo AP_InertialSensor::TCal::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable temperature calibration
    // @Description: Enable the use of temperature calibration parameters for this IMU
    // @Values: 0:Disabled,1:Enabled,2:LearnCalibration
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_InertialSensor::TCal, enable,  float(Enable::Disabled), AP_PARAM_FLAG_ENABLE),

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
    return (c[0] + (c[1] + c[2]*tdiff)*tdiff)*tdiff*INV_SCALE_FACTOR;
}

/*
  correct a single sensor for the current temperature
 */
void AP_InertialSensor::TCal::correct_sensor(float temperature, float cal_temp, const AP_Vector3f coeff[3], Vector3f &v) const
{
    if (enable != Enable::Enabled) {
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

AP_InertialSensor::TCal::Learn::Learn(TCal &_tcal, float _start_temp) :
    start_temp(_start_temp),
    tcal(_tcal)
{
    for (uint8_t i=0; i<2; i++) {
        state[i].temp_filter.set_cutoff_frequency(1000, 0.5);
        state[i].temp_filter.reset(start_temp);
        state[i].last_temp = start_temp;
    }
    start_tmax = tcal.temp_max;
}

/*
  update polyfit with new sample
 */
void AP_InertialSensor::TCal::Learn::add_sample(const Vector3f &sample, float temperature, struct LearnState &st)
{
    temperature = st.temp_filter.apply(temperature);

    st.sum += sample;
    st.sum_count++;

    if (st.sum_count < 100 ||
        temperature - st.last_temp < 0.5) {
        // wait for more data
        return;
    }

    st.sum /= st.sum_count;

    const uint8_t si = &st - &state[0];

    const float T = (temperature + st.last_temp) * 0.5;

    if (si == 0) {
        // we use the first accel sample as the zero baseline
        if (accel_start.is_zero()) {
            accel_start = st.sum;
            start_temp = T;
        }
        st.sum -= accel_start;
    }
    
    const float tmid = 0.5 * (tcal.temp_max + start_temp);
    const float tdiff = T - tmid;

    AP::logger().Write("TCLR", "TimeUS,I,Si,Temp,TDiff,X,Y,Z",
                       "s#------",
                       "F0000000",
                       "QBBfffff",
                       AP_HAL::micros64(),
                       instance(),
                       si,
                       T,
                       tdiff,
                       st.sum.x, st.sum.y, st.sum.z);
    
    
    st.pfit[0].update(tdiff, st.sum.x);
    st.pfit[1].update(tdiff, st.sum.y);
    st.pfit[2].update(tdiff, st.sum.z);

    st.sum.zero();
    st.sum_count = 0;
    st.last_temp = temperature;

    if (!is_equal(start_tmax,tcal.temp_max.get())) {
        // user has changed the TMAX. This will give a bad result for
        // online learning as the reference temperature (tmid) will
        // change
        AP_Notify::events.temp_cal_failed = 1;
        tcal.enable.set_and_save(int8_t(TCal::Enable::Disabled));
    }
    
    if (temperature >= tcal.temp_max &&
        temperature - start_temp >= TEMP_RANGE_MIN) {
        finish_calibration(temperature);
    }
}

/*
  update accel temperature compensation learning
 */
void AP_InertialSensor::TCal::update_accel_learning(const Vector3f &accel, float temperature)
{
    if (enable != Enable::LearnCalibration) {
        return;
    }
    if (learn == nullptr && hal.scheduler->is_system_initialized()) {
        learn = new Learn(*this, temperature);
        if (learn) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TCAL[%u]: started calibration t=%.1fC tmax=%.1fC",
                          instance(),
                          temperature, temp_max.get());
            AP_Notify::events.initiated_temp_cal = 1;
        }
    }
    if (learn != nullptr) {
        AP_Notify::flags.temp_cal_running = true;
        learn->add_sample(accel, temperature, learn->state[0]);
    }
}

/*
  update gyro temperature compensation learning
 */
void AP_InertialSensor::TCal::update_gyro_learning(const Vector3f &gyro, float temperature)
{
    if (enable != Enable::LearnCalibration) {
        return;
    }
    if (learn != nullptr) {
        learn->add_sample(gyro, temperature, learn->state[1]);
    }
}

/*
  finish and save calibration
 */
void AP_InertialSensor::TCal::Learn::finish_calibration(float temperature)
{
    Vector3f coefficients[3];

    for (uint8_t i=0; i<3; i++) {
        float p[4];
        if (!state[0].pfit[i].get_polynomial(p)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TCAL[%u]: failed accel fit axis %u", instance(), i);
            AP_Notify::events.temp_cal_failed = 1;
            tcal.enable.set_and_save(int8_t(TCal::Enable::Disabled));
            return;
        }
        for (uint8_t k=0; k<3; k++) {
            coefficients[k][i] = p[2-k] * SCALE_FACTOR;
        }
    }
    for (uint8_t k=0; k<3; k++) {
        tcal.accel_coeff[k].set_and_save(coefficients[k]);
    }

    for (uint8_t i=0; i<3; i++) {
        float p[4];
        if (!state[1].pfit[i].get_polynomial(p)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TCAL[%u]: failed gyro fit axis %u", tcal.instance(), i);
            AP_Notify::events.temp_cal_failed = 1;
            tcal.enable.set_and_save(int8_t(TCal::Enable::Disabled));
            return;
        }
        for (uint8_t k=0; k<3; k++) {
            coefficients[k][i] = p[2-k] * SCALE_FACTOR;
        }
    }
    for (uint8_t k=0; k<3; k++) {
        tcal.gyro_coeff[k].set_and_save(coefficients[k]);
    }
    tcal.temp_min.set_and_save(start_temp);
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TCAL[%u]: completed calibration tmin=%.1f tmax=%.1f",
                  instance(),
                  tcal.temp_min.get(), tcal.temp_max.get());
    tcal.enable.set_and_save(int8_t(TCal::Enable::Enabled));

    if (!AP::ins().temperature_cal_running()) {
        AP_Notify::flags.temp_cal_running = false;
        AP_Notify::events.temp_cal_saved = 1;
    }
}

uint8_t AP_InertialSensor::TCal::instance(void) const
{
    return AP::ins().tcal_instance(*this);
}

#endif // HAL_INS_TEMPERATURE_CAL_ENABLE
