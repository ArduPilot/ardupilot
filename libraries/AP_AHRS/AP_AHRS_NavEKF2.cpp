#include "AP_AHRS_config.h"

#if AP_AHRS_NAVEKF2_ENABLED

#include "AP_AHRS_NavEKF2.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

NavEKF2 AP_AHRS_NavEKF2::EKF2;

bool AP_AHRS_NavEKF2::start()
{
    const auto now_ms = AP_HAL::millis();

    if (start_time_ms == 0) {
        start_time_ms = now_ms;
    }

#if HAL_LOGGING_ENABLED
    // if we're doing Replay logging then don't allow any data
    // into the EKF yet.  Don't allow it to block us for long.
    if (!hal.util->was_watchdog_reset()) {
        if (now_ms - start_time_ms < 5000) {
            if (!AP::logger().allow_start_ekf()) {
                return false;
            }
        }
    }
#endif

    // wait 1 second for DCM to output a valid tilt error estimate
    // FIXME: work out whether this is still required!
    if (now_ms - start_time_ms <= 1000) {
        return false;
    }

    // try to start the filter:
    return EKF2.InitialiseFilter();
}

void AP_AHRS_NavEKF2::update()
{
    if (!started) {
        started = start();
    }
    if (!started) {
        return;
    }
    EKF2.UpdateFilter();

    // check the current primary core; if it has changed then assume
    // our attitude is reset:
    const int8_t primary_core = EKF2.getPrimaryCoreIndex();
    if (old_primary_core != primary_core) {
        old_primary_core = primary_core;
        attitude_reset_count++;
        LOGGER_WRITE_ERROR(LogErrorSubsystem::EKF_PRIMARY, LogErrorCode(primary_core));
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "EKF2 primary changed:%d", (unsigned)primary_core);
    }
}

void AP_AHRS_NavEKF2::get_results(AP_AHRS_Backend::Estimates &results)
{
    const auto now_ms = AP_HAL::millis();

    // initialisation complete some time after ekf has started
    results.initialised = (started && (now_ms - start_time_ms > 20000));

    results.healthy = started && EKF2.healthy();

    const AP_InertialSensor &_ins = AP::ins();

    /*
     * rotational rate estimates:
     */

    // The backend should tell us what gyro it is using right now.
    // There's no API for that at the moment, so we assume the
    // gyro corresponds to the IMU index.
    const int8_t ek2_primary_gyro = EKF2.getPrimaryCoreIMUIndex();
    if (ek2_primary_gyro == -1) {
        results.primary_gyro = _ins.get_first_usable_gyro();
    } else {
        results.primary_gyro = ek2_primary_gyro;
    }

    // get gyro bias for primary EKF and change sign to give gyro drift
    // Note sign convention used by EKF is bias = measurement - truth
    Vector3f drift;
    EKF2.getGyroBias(drift);
    results.gyro_drift = -drift;

    // use the same IMU as the primary EKF and correct for gyro drift
    results.gyro_estimate = _ins.get_gyro(results.primary_gyro) + results.gyro_drift;

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

    results.attitude_reset_count = attitude_reset_count;

    /*
     * acceleration estimates
     */

    // The backend should tell us what accelerometer it is using
    // right now.  There's no API for that at the moment, so we assume
    // the accelerometer corresponds to the IMU index.
    const int8_t ek2_primary_accel = EKF2.getPrimaryCoreIMUIndex();
    if (ek2_primary_accel == -1) {
        results.primary_accel = _ins.get_first_usable_accel();
    } else {
        results.primary_accel = ek2_primary_accel;
    }

    // get z accel bias estimate from active EKF (this is usually for the primary IMU)
    EKF2.getAccelZBias(results.accel_bias.z);

    // This EKF is currently using primary_imu, and a bias applies to only that IMU
    Vector3f accel = _ins.get_accel(results.primary_accel);
    accel.z -= results.accel_bias.z;
    results.accel_ef = results.dcm_matrix * AP::ahrs().get_rotation_autopilot_body_to_vehicle_body() * accel;

    /*
     * velocity estimates
     */
    EKF2.getVelNED(results.velocity_NED);
    results.velocity_NED_valid = true;
    results.vert_pos_rate_D = EKF2.getPosDownDerivative();
    results.vert_pos_rate_D_valid = true;

    // ground velocity estimate in meters/second, in North/East order
    results.velocity_NE = results.velocity_NED.xy();

    /*
     * position estimates
     */
    results.location_valid = EKF2.getLLH(results.location);

    // origin-relative functions
    results.provides_common_origin = true;

    // origin-relative position:
    results.position_NE_valid = EKF2.getPosNE(results.position_NE);
    results.position_D_valid = EKF2.getPosD(results.position_D);

    results.hagl_valid = EKF2.getHAGL(results.hagl);

    /*
     * air data estimates
     */
    EKF2.getWind(results.wind);
    results.wind_valid = true;

    /*
     * Sensor-related information
     */
    // true if the estimator will use GPS data in creating its
    // estimate when the data is good:
    results.configured_to_use_gps = EKF2.using_gps();
    // true if GPS is configured as the horizontal position source for
    // this estimator.  Used to decide whether GPS will set the
    // navigation origin:
    results.configured_to_use_gps_for_pos_XY = EKF2.configuredToUseGPSForPosXY();

    // are we consuming yaw from an external (e.g. vision-based) source?
    results.using_extnav_for_yaw = EKF2.isExtNavUsedForYaw();

    // are we consuming yaw from a source which is *not* a compass
    results.using_noncompass_for_yaw = EKF2.isExtNavUsedForYaw();

#if AP_AHRS_GET_MAG_DATA_ENABLED
    // estimators can provide their predicted magnetic fields:
    EKF2.getMagNED(results.mag_field_NED);
    results.mag_field_NED_valid = true;
    EKF2.getMagXYZ(results.mag_field_corrections);
    results.mag_field_corrections_valid = true;
#endif  // AP_AHRS_GET_MAG_DATA_ENABLED

    /*
     * filter status and estimates quality values:
     */
    EKF2.getFilterStatus(results.filter_status);
    results.filter_status_valid = true;

    EKF2.getFilterFaults(results.filter_faults);

    // provides the innovations normalised between 0 and 1:
    Vector2f offset;
    results.variances_valid = EKF2.getVariances(results.velVar, results.posVar, results.hgtVar, results.magVar, results.tasVar, offset);

    results.terrain_alt_variance_valid = EKF2.getTerrainAltVariance(results.terrain_alt_variance);
}

bool AP_AHRS_NavEKF2::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    if (!started) {
        hal.util->snprintf(failure_msg, failure_msg_len, "EKF2 not started");
        return false;
    }
    return EKF2.pre_arm_check(failure_msg, failure_msg_len);
}

#endif  // AP_AHRS_NAVEKF2_ENABLED
