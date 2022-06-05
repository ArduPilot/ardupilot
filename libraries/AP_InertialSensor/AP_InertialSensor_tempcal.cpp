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

#define AP_INLINE_VECTOR_OPS

#include "AP_InertialSensor.h"

#if HAL_INS_TEMPERATURE_CAL_ENABLE
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/ExpandingString.h>

// this scale factor ensures params are easy to work with in GUI parameter editors
#define SCALE_FACTOR 1.0e6
#define INV_SCALE_FACTOR 1.0e-6
#define TEMP_RANGE_MIN 10

// timeout calibration after 10 minutes, if no temperature rise
#define CAL_TIMEOUT_MS (600U*1000U)

/*
  we use a fixed reference temperature of 35C. This has the advantage
  that we don't need to know the final temperature when doing an
  online calibration which allows us to have a calibration timeout
*/
#define TEMP_REFERENCE 35.0

extern const AP_HAL::HAL& hal;

// temperature calibration parameters, per IMU
const AP_Param::GroupInfo AP_InertialSensor::TCal::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable temperature calibration
    // @Description: Enable the use of temperature calibration parameters for this IMU. For automatic learning set to 2 and also set the INS_TCALn_TMAX to the target temperature, then reboot
    // @Values: 0:Disabled,1:Enabled,2:LearnCalibration
    // @User: Advanced
    // @RebootRequired: True
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
    // @Description: The maximum temperature that the calibration is valid for. This must be at least 10 degrees above TMIN for calibration
    // @Range: -70 80
    // @Units: degC
    // @User: Advanced
    // @Calibration: 1
    AP_GROUPINFO("TMAX", 3, AP_InertialSensor::TCal, temp_max,  70),

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


    // get the polynomial correction for the difference between the
    // current temperature and the mid temperature
    v -= polynomial_eval(temperature - TEMP_REFERENCE, coeff);

    // we need to add the correction for the temperature
    // difference between the TREF, which is the reference used for
    // the calibration process, and the cal_temp, which is the
    // temperature that the offsets and scale factors was setup for
    v += polynomial_eval(cal_temp - TEMP_REFERENCE, coeff);
}

void AP_InertialSensor::TCal::correct_accel(float temperature, float cal_temp, Vector3f &accel) const
{
    correct_sensor(temperature, cal_temp, accel_coeff, accel);
}

void AP_InertialSensor::TCal::correct_gyro(float temperature, float cal_temp, Vector3f &gyro) const
{
    correct_sensor(temperature, cal_temp, gyro_coeff, gyro);
}

/*
  for SITL we don't apply the temperature limits and use mid-point as
  reference. This makes the SITL data independent of TEMP_REFERENCE
  and prevents an abrupt change at the endpoints
 */
void AP_InertialSensor::TCal::sitl_apply_accel(float temperature, Vector3f &accel) const
{
    const float tmid = 0.5*(temp_max+temp_min);
    accel += polynomial_eval(temperature - tmid, accel_coeff);
}

void AP_InertialSensor::TCal::sitl_apply_gyro(float temperature, Vector3f &gyro) const
{
    const float tmid = 0.5*(temp_max+temp_min);
    gyro += polynomial_eval(temperature - tmid, gyro_coeff);
}

AP_InertialSensor::TCal::Learn::Learn(TCal &_tcal, float _start_temp) :
    start_temp(_start_temp),
    tcal(_tcal)
{
    reset(_start_temp);
}

/*
  update polyfit with new sample
 */
void AP_InertialSensor::TCal::Learn::add_sample(const Vector3f &sample, float temperature, struct LearnState &st)
{
    temperature = st.temp_filter.apply(temperature);

    st.sum += sample;
    st.sum_count++;

    uint32_t now = AP_HAL::millis();

    if (st.sum_count < 100 ||
        temperature - st.last_temp < 0.5) {
        // check for timeout
        if (st.last_sample_ms != 0 &&
            temperature - start_temp >= TEMP_RANGE_MIN &&
            now - st.last_sample_ms > CAL_TIMEOUT_MS) {
            // we have timed out, finish up now
            finish_calibration(st.last_temp);
        }
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

    const float tdiff = T - TEMP_REFERENCE;
#if HAL_LOGGING_ENABLED
    AP::logger().Write("TCLR", "TimeUS,I,SType,Temp,X,Y,Z,NSamp",
                       "s#------",
                       "F000000-",
                       "QBBffffI",
                       AP_HAL::micros64(),
                       instance(),
                       si,
                       T,
                       st.sum.x, st.sum.y, st.sum.z,
                       st.sum_count);
#endif
    
    
    st.pfit.update(tdiff, st.sum);

    st.sum.zero();
    st.sum_count = 0;
    st.last_temp = temperature;
    st.last_sample_ms = now;

    if (temperature - start_temp >= TEMP_RANGE_MIN) {
        if (temperature >= start_tmax) {
            // we've reached the target temperature
            finish_calibration(temperature);
        } else if (now - last_save_ms > 15000) {
            // save partial calibration, so if user stops the cal part
            // way then they still have a useful calibration
            last_save_ms = now;
            save_calibration(st.last_temp);
        }
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
                          instance()+1,
                          temperature, learn->start_tmax);
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
  reset calibration
 */
void AP_InertialSensor::TCal::Learn::reset(float temperature)
{
    memset((void*)&state[0], 0, sizeof(state));
    start_tmax = tcal.temp_max;
    accel_start.zero();
    for (uint8_t i=0; i<ARRAY_SIZE(state); i++) {
        state[i].temp_filter.set_cutoff_frequency(1000, 0.5);
        state[i].temp_filter.reset(temperature);
        state[i].last_temp = temperature;
    }
}

/*
  finish and save calibration
 */
void AP_InertialSensor::TCal::Learn::finish_calibration(float temperature)
{
    if (!save_calibration(temperature)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TCAL[%u]: failed fit", instance()+1);
        AP_Notify::events.temp_cal_failed = 1;
        tcal.enable.set_and_save_ifchanged(int8_t(TCal::Enable::Disabled));
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TCAL[%u]: completed calibration tmin=%.1f tmax=%.1f",
                  instance()+1,
                  tcal.temp_min.get(), tcal.temp_max.get());
    tcal.enable.set_and_save_ifchanged(int8_t(TCal::Enable::Enabled));
}

/*
  save calibration state
 */
bool AP_InertialSensor::TCal::Learn::save_calibration(float temperature)
{
    Vector3f coefficients[3];

    Vector3f p[4];
    if (!state[0].pfit.get_polynomial(p)) {
        return false;
    }
    for (uint8_t k=0; k<3; k++) {
        coefficients[k] = p[2-k] * SCALE_FACTOR;
    }

    for (uint8_t k=0; k<3; k++) {
        tcal.accel_coeff[k].set_and_save_ifchanged(coefficients[k]);
    }

    if (!state[1].pfit.get_polynomial(p)) {
        return false;
    }
    for (uint8_t k=0; k<3; k++) {
        coefficients[k] = p[2-k] * SCALE_FACTOR;
    }

    for (uint8_t k=0; k<3; k++) {
        tcal.gyro_coeff[k].set_and_save_ifchanged(coefficients[k]);
    }
    tcal.temp_min.set_and_save_ifchanged(start_temp);
    tcal.temp_max.set_and_save_ifchanged(temperature);
    return true;
}

uint8_t AP_InertialSensor::TCal::instance(void) const
{
    return AP::ins().tcal_instance(*this);
}

/*
  get a string representation of parameters for this calibration set
*/
void AP_InertialSensor::TCal::get_persistent_params(ExpandingString &str) const
{
    if (enable != TCal::Enable::Enabled) {
        return;
    }
    const uint8_t imu = instance()+1;
    str.printf("INS_TCAL%u_ENABLE=1\n", imu);
    str.printf("INS_TCAL%u_TMIN=%.2f\n", imu, temp_min.get());
    str.printf("INS_TCAL%u_TMAX=%.2f\n", imu, temp_max.get());
    for (uint8_t k=0; k<3; k++) {
        const Vector3f &acc = accel_coeff[k].get();
        const Vector3f &gyr = gyro_coeff[k].get();
        str.printf("INS_TCAL%u_ACC%u_X=%f\n", imu, k+1, acc.x);
        str.printf("INS_TCAL%u_ACC%u_Y=%f\n", imu, k+1, acc.y);
        str.printf("INS_TCAL%u_ACC%u_Z=%f\n", imu, k+1, acc.z);
        str.printf("INS_TCAL%u_GYR%u_X=%f\n", imu, k+1, gyr.x);
        str.printf("INS_TCAL%u_GYR%u_Y=%f\n", imu, k+1, gyr.y);
        str.printf("INS_TCAL%u_GYR%u_Z=%f\n", imu, k+1, gyr.z);
    }
}

/*
  get a string representation of parameters that should be made
  persistent across changes of firmware type
*/
void AP_InertialSensor::get_persistent_params(ExpandingString &str) const
{
    bool save_options = false;
    if (uint32_t(tcal_options.get()) & uint32_t(TCalOptions::PERSIST_ACCEL_CAL)) {
        save_options = true;
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            const uint8_t imu = i+1;
            const Vector3f &aoff = _accel_offset[i].get();
            const Vector3f &ascl = _accel_scale[i].get();
            char id[2] = "";
            if (i > 0) {
                id[0] = '1'+i;
            }
            str.printf("INS_ACC%s_ID=%u\n", id, unsigned(_accel_id[i].get()));
            str.printf("INS_ACC%sOFFS_X=%f\n", id, aoff.x);
            str.printf("INS_ACC%sOFFS_Y=%f\n", id, aoff.y);
            str.printf("INS_ACC%sOFFS_Z=%f\n", id, aoff.z);
            str.printf("INS_ACC%sSCAL_X=%f\n", id, ascl.x);
            str.printf("INS_ACC%sSCAL_Y=%f\n", id, ascl.y);
            str.printf("INS_ACC%sSCAL_Z=%f\n", id, ascl.z);
            str.printf("INS_ACC%u_CALTEMP=%.2f\n", imu, caltemp_accel[i].get());
        }
    }
    if (uint32_t(tcal_options.get()) & uint32_t(TCalOptions::PERSIST_TEMP_CAL)) {
        for (auto &tc : tcal) {
            tc.get_persistent_params(str);
        }
        save_options = true;
    }
    if (save_options) {
        /*
          we also have to save the TCAL_OPTIONS parameter so that
          future flashing of the bootloader doesn't cause an erase
         */
        str.printf("INS_TCAL_OPTIONS=%u\n", unsigned(tcal_options.get()));
    }
}


#endif // HAL_INS_TEMPERATURE_CAL_ENABLE
