#include "AP_AHRS_config.h"

#if AP_AHRS_NAVEKF3_ENABLED

#include "AP_AHRS_NavEKF3.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

NavEKF3 AP_AHRS_NavEKF3::EKF3;

void AP_AHRS_NavEKF3::get_results(AP_AHRS_Backend::Estimates &results)
{
    const AP_InertialSensor &_ins = AP::ins();

    /*
     * rotational rate estimates:
     */
    // The backend should tell us what gyro it is using right now.
    // There's no API for that at the moment, so we assume the
    // gyro corresponds to the IMU index.
    const int8_t ek3_primary_gyro = EKF3.getPrimaryCoreIMUIndex();
    if (ek3_primary_gyro == -1) {
        results.primary_gyro = _ins.get_first_usable_gyro();
    } else {
        results.primary_gyro = ek3_primary_gyro;
    }

    // get gyro bias for primary EKF and change sign to give gyro drift
    // Note sign convention used by EKF is bias = measurement - truth
    Vector3f drift;
    EKF3.getGyroBias(-1, drift);
    results.gyro_drift = -drift;

    // use the same IMU as the primary EKF and correct for gyro drift
    results.gyro_estimate = _ins.get_gyro(results.primary_gyro) + results.gyro_drift;

    /*
     * attitude estimates:
     */
    EKF3.getRotationBodyToNED(results.dcm_matrix);

    Vector3f eulers;
    EKF3.getEulerAngles(eulers);
    results.roll_rad  = eulers.x;
    results.pitch_rad = eulers.y;
    results.yaw_rad   = eulers.z;

    EKF3.getQuaternion(results.quaternion);
    results.quaternion.rotate(-AP::ahrs().get_trim());

    results.attitude_valid = started;

    /*
     * acceleration estimates
     */

    // The backend should tell us what accelerometer it is using
    // right now.  There's no API for that at the moment, so we assume
    // the accelerometer corresponds to the IMU index.
    const int8_t ek3_primary_accel = EKF3.getPrimaryCoreIMUIndex();
    if (ek3_primary_accel == -1) {
        results.primary_accel = _ins.get_first_usable_accel();
    } else {
        results.primary_accel = ek3_primary_accel;
    }

    // get 3-axis accel bias estimates for active EKF (this is usually
    // for the primary IMU)
    EKF3.getAccelBias(-1, results.accel_bias);

    // use the primary IMU for accel earth frame
    Vector3f accel = _ins.get_accel(results.primary_accel);
    accel -= results.accel_bias;
    results.accel_ef = results.dcm_matrix * AP::ahrs().get_rotation_autopilot_body_to_vehicle_body() * accel;

    /*
     * velocity estimates
     */
    EKF3.getVelNED(results.velocity_NED);
    results.velocity_NED_valid = true;
    results.vert_pos_rate_D = EKF3.getPosDownDerivative();
    results.vert_pos_rate_D_valid = true;

    // ground velocity estimate in meters/second, in North/East order
    results.velocity_NE = results.velocity_NED.xy();

    /*
     * position estimates
     */
    results.location_valid = EKF3.getLLH(results.location);

    results.hagl_valid = EKF3.getHAGL(results.hagl);

    /*
     * Sensor-related information
     */
    // true if the estimator will use GPS data in creating its
    // estimate when the data is good:
    results.configured_to_use_gps = EKF3.using_gps();
    // true if GPS is configured as the horizontal position source
    // for this estimator.  Used to decide whether GPS will set
    // the navigation origin:
    results.configured_to_use_gps_for_pos_XY = EKF3.configuredToUseGPSForPos();
}

bool AP_AHRS_NavEKF3::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    if (!started) {
        hal.util->snprintf(failure_msg, failure_msg_len, "EKF3 not started");
        return false;
    }
    return EKF3.pre_arm_check(requires_position, failure_msg, failure_msg_len);
}

#endif  // AP_AHRS_NAVEKF3_ENABLED
