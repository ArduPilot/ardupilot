#include "AP_AHRS_External.h"

#if AP_AHRS_EXTERNAL_ENABLED

#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_AHRS/AP_AHRS.h>

void AP_AHRS_External::update()
{
    AP::externalAHRS().update();
}

void AP_AHRS_External::get_results(AP_AHRS_Backend::Estimates &results)
{
    auto &extahrs = AP::externalAHRS();

    results.initialised = extahrs.initialised();

    results.healthy = extahrs.healthy();

#if AP_INERTIALSENSOR_ENABLED
    const AP_InertialSensor &_ins = AP::ins();
    // not using specific sensors:
    results.primary_gyro = _ins.get_first_usable_gyro();
    results.primary_accel = _ins.get_first_usable_accel();
#endif  // AP_INERTIALSENSOR_ENABLED

    if (!extahrs.get_quaternion(results.quaternion)) {
        results.attitude_valid = false;
        return;
    }
    results.attitude_valid = true;
    results.quaternion.rotation_matrix(results.dcm_matrix);
    results.dcm_matrix.to_euler(&results.roll_rad, &results.pitch_rad, &results.yaw_rad);

    results.gyro_drift.zero();
    if (!extahrs.get_gyro(results.gyro_estimate)) {
#if AP_INERTIALSENSOR_ENABLED
        results.gyro_estimate = _ins.get_gyro();
#endif  // AP_INERTIALSENSOR_ENABLED
    }

    Vector3f accel;
    if (!extahrs.get_accel(accel)) {
#if AP_INERTIALSENSOR_ENABLED
        accel = _ins.get_accel();
#endif  // AP_INERTIALSENSOR_ENABLED
    }

    /*
     * acceleration estimates
     */
    // results.accel_bias = {} - External does not estimate accel bias
    const Vector3f accel_ef = results.dcm_matrix * AP::ahrs().get_rotation_autopilot_body_to_vehicle_body() * accel;
    results.accel_ef = accel_ef;

    results.velocity_NED_valid = AP::externalAHRS().get_velocity_NED(results.velocity_NED);
    // a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    results.vert_pos_rate_D_valid = AP::externalAHRS().get_speed_down(results.vert_pos_rate_D);

    // ground velocity estimate in meters/second, in North/East order
    results.velocity_NE = AP::externalAHRS().get_groundspeed_vector();

    /*
     * position estimates
     */
    results.location_valid = AP::externalAHRS().get_location(results.location);

    // origin-relative functions
    results.provides_common_origin = true;

    // origin-relative position:
    Location orgn;
    if (extahrs.get_origin(orgn) &&
        results.location_valid) {
        const Vector3p posNED = orgn.get_distance_NED_postype(results.location);
        results.position_NE = posNED.xy();
        results.position_NE_valid = true;
        results.position_D = posNED.z;
        results.position_D_valid = true;
    }

    // hagl is not supplied:
    // results.hagl_valid = false;
    // results.hagl = 0;

    /*
     * air data estimates
     */
    // wind estimate is not supplied:
    // results.wind = {};
    // results.wind_valid = false;

    /*
     * Sensor-related information
     */
    // true if the estimator will use GPS data in creating its
    // estimate when the data is good:
    results.configured_to_use_gps = true;  // massive assumption here
    // true if GPS is configured as the horizontal position source
    // for this estimator.  Used to decide whether GPS will set
    // the navigation origin.
    results.configured_to_use_gps_for_pos_XY = true;

    // are we consuming yaw from an external (e.g. vision-based) source?
    // this relates only to external sources being passed in via mavlink
    // results.using_extnav_for_yaw = false;

    // are we consuming yaw from a source which is *not* a compass
    // results.using_noncompass_for_yaw = false;

#if AP_AHRS_GET_MAG_DATA_ENABLED
    // estimators can provide their predicted magnetic fields:
    // ... but External does not, and probably should not as the
    // external unit will be experiencing a different magnetic
    // environment to the autopilot.
    // results.mag_field_NED = {};
    // results.mag_field_NED_valid = false;
    // results.mag_field_corrections = {};
    // results.mag_field_corrections_valid = false;
#endif  // AP_AHRS_GET_MAG_DATA_ENABLED

    /*
     * filter status and estimates quality values:
     */
    AP::externalAHRS().get_filter_status(results.filter_status);
    results.filter_status_valid = true;

    // provides the innovations normalised between 0 and 1:
    results.variances_valid = AP::externalAHRS().get_variances(results.velVar, results.posVar, results.hgtVar, results.magVar, results.tasVar);

    results.terrain_alt_variance = 0;
    results.terrain_alt_variance_valid = true;

    // no limit on gains, large vel limit
    results.control_ground_speed_limit = 400.0;
    results.control_gain_scaler_XY = 1;
    results.control_gain_scaler_Z = 1;

    // control height is ever limited:
    // results.control_height_limit_valid = false;
    // results.control_height_limit = 0;
}

bool AP_AHRS_External::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    return AP::externalAHRS().pre_arm_check(failure_msg, failure_msg_len);
}

bool AP_AHRS_External::get_origin(Location &ret) const
{
    return AP::externalAHRS().get_origin(ret);
}

bool AP_AHRS_External::set_origin(const Location &loc)
{
    return AP::externalAHRS().set_origin(loc);
}

#endif
