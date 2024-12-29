#include "AP_AHRS_config.h"

#if AP_AHRS_NAVEKF2_ENABLED

#include "AP_AHRS_NavEKF2.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

void AP_AHRS_NavEKF2::get_results(AP_AHRS_Backend::Estimates &results)
{
    /*
     * attitude estimates:
     */
    EKF2.getRotationBodyToNED(results.dcm_matrix);
    Vector3f eulers;
    EKF2.getEulerAngles(eulers);
    results.roll_rad  = eulers.x;
    results.pitch_rad = eulers.y;
    results.yaw_rad   = eulers.z;

    EKF2.getQuaternion(results.quaternion);
    results.quaternion.rotate(-AP::ahrs().get_trim());

    results.attitude_valid = started;

    /*
     * rotational rate estimates:
     */
    // Use the primary EKF to select the primary gyro
    const AP_InertialSensor &_ins = AP::ins();
    const int8_t primary_imu = EKF2.getPrimaryCoreIMUIndex();
    const uint8_t primary_gyro = primary_imu>=0?primary_imu:_ins.get_first_usable_gyro();
    const uint8_t primary_accel = primary_imu>=0?primary_imu:_ins.get_first_usable_accel();

    // get gyro bias for primary EKF and change sign to give gyro drift
    // Note sign convention used by EKF is bias = measurement - truth
    Vector3f drift;
    EKF2.getGyroBias(drift);
    results.gyro_drift = -drift;

    // use the same IMU as the primary EKF and correct for gyro drift
    results.gyro_estimate = _ins.get_gyro(primary_gyro) + results.gyro_drift;

    /*
     * acceleration estimates
     */
    // get z accel bias estimate from active EKF (this is usually for the primary IMU)
    EKF2.getAccelZBias(results.accel_bias.z);

    // This EKF is currently using primary_imu, and a bias applies to only that IMU
    Vector3f accel = _ins.get_accel(primary_accel);
    accel.z -= results.accel_bias.z;
    results.accel_ef = results.dcm_matrix * AP::ahrs().get_rotation_autopilot_body_to_vehicle_body() * accel;

    /*
     * velocity estimates
     */
    EKF2.getVelNED(results.velocity_NED);
    results.velocity_NED_valid = true;
    results.vert_pos_rate_D = EKF2.getPosDownDerivative();
    results.vert_pos_rate_D_valid = true;

    /*
     * position estimates
     */
    results.location_valid = EKF2.getLLH(results.location);
}

bool AP_AHRS_NavEKF2::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    if (!started) {
        hal.util->snprintf(failure_msg, failure_msg_len, "EKF2 not started");
        return false;
    }
    return EKF2.pre_arm_check(requires_position, failure_msg, failure_msg_len);
}

#endif  // AP_AHRS_NAVEKF2_ENABLED
