#include "AP_AHRS_SIM.h"

#if AP_AHRS_SIM_ENABLED

#include "AP_AHRS.h"

bool AP_AHRS_SIM::get_location(Location &loc) const
{
    if (_sitl == nullptr) {
        return false;
    }

    const struct SITL::sitl_fdm &fdm = _sitl->state;
    loc = {};
    loc.lat = fdm.latitude * 1e7;
    loc.lng = fdm.longitude * 1e7;
    loc.alt = fdm.altitude*100;

    return true;
}

bool AP_AHRS_SIM::get_velocity_NED(Vector3f &vec) const
{
    if (_sitl == nullptr) {
        return false;
    }

    const struct SITL::sitl_fdm &fdm = _sitl->state;
    vec = Vector3f(fdm.speedN, fdm.speedE, fdm.speedD);

    return true;
}

bool AP_AHRS_SIM::wind_estimate(Vector3f &wind) const
{
    if (_sitl == nullptr) {
        return false;
    }

    wind = _sitl->state.wind_ef;
    return true;
}

bool AP_AHRS_SIM::airspeed_estimate(float &airspeed_ret) const
{
    if (_sitl == nullptr) {
        return false;
    }

    airspeed_ret = _sitl->state.airspeed;

    return true;
}

bool AP_AHRS_SIM::airspeed_estimate(uint8_t index, float &airspeed_ret) const
{
    return airspeed_estimate(airspeed_ret);
}

bool AP_AHRS_SIM::get_quaternion(Quaternion &quat) const
{
    if (_sitl == nullptr) {
        return false;
    }

    const struct SITL::sitl_fdm &fdm = _sitl->state;
    quat = fdm.quaternion;

    return true;
}

Vector2f AP_AHRS_SIM::groundspeed_vector(void)
{
    if (_sitl == nullptr) {
        return Vector2f{};
    }

    const struct SITL::sitl_fdm &fdm = _sitl->state;

    return Vector2f(fdm.speedN, fdm.speedE);
}

bool AP_AHRS_SIM::get_vert_pos_rate(float &velocity) const
{
    if (_sitl == nullptr) {
        return false;
    }

    velocity = _sitl->state.speedD;

    return true;
}

bool AP_AHRS_SIM::get_hagl(float &height) const
{
    if (_sitl == nullptr) {
        return false;
    }

    height = _sitl->state.altitude - AP::ahrs().get_home().alt*0.01f;

    return true;
}

bool AP_AHRS_SIM::get_relative_position_NED_origin(Vector3f &vec) const
{
    if (_sitl == nullptr) {
        return false;
    }

    Location loc, orgn;
    if (!get_location(loc) ||
        !get_origin(orgn)) {
        return false;
    }

    const Vector2f diff2d = orgn.get_distance_NE(loc);
    const struct SITL::sitl_fdm &fdm = _sitl->state;
    vec = Vector3f(diff2d.x, diff2d.y,
                   -(fdm.altitude - orgn.alt*0.01f));

    return true;
}

bool AP_AHRS_SIM::get_relative_position_NE_origin(Vector2f &posNE) const
{
    Location loc, orgn;
    if (!get_location(loc) ||
        !get_origin(orgn)) {
        return false;
    }
    posNE = orgn.get_distance_NE(loc);

    return true;
}

bool AP_AHRS_SIM::get_relative_position_D_origin(float &posD) const
{
    if (_sitl == nullptr) {
        return false;
    }
    const struct SITL::sitl_fdm &fdm = _sitl->state;
    Location orgn;
    if (!get_origin(orgn)) {
        return false;
    }
    posD = -(fdm.altitude - orgn.alt*0.01f);

    return true;
}

bool AP_AHRS_SIM::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    status.flags.attitude = 1;
    status.flags.horiz_vel = 1;
    status.flags.vert_vel = 1;
    status.flags.horiz_pos_rel = 1;
    status.flags.horiz_pos_abs = 1;
    status.flags.vert_pos = 1;
    status.flags.pred_horiz_pos_rel = 1;
    status.flags.pred_horiz_pos_abs = 1;
    status.flags.using_gps = 1;

    return true;
}

void AP_AHRS_SIM::get_control_limits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    // same as EKF2 for no optical flow
    ekfGndSpdLimit = 400.0f;
    ekfNavVelGainScaler = 1.0f;
}

bool AP_AHRS_SIM::get_mag_offsets(uint8_t mag_idx, Vector3f &magOffsets) const
{
    magOffsets.zero();

    return true;
}

void AP_AHRS_SIM::send_ekf_status_report(GCS_MAVLINK &link) const
{
    // send status report with everything looking good
    const uint16_t flags =
        EKF_ATTITUDE | /* Set if EKF's attitude estimate is good. | */
        EKF_VELOCITY_HORIZ | /* Set if EKF's horizontal velocity estimate is good. | */
        EKF_VELOCITY_VERT | /* Set if EKF's vertical velocity estimate is good. | */
        EKF_POS_HORIZ_REL | /* Set if EKF's horizontal position (relative) estimate is good. | */
        EKF_POS_HORIZ_ABS | /* Set if EKF's horizontal position (absolute) estimate is good. | */
        EKF_POS_VERT_ABS | /* Set if EKF's vertical position (absolute) estimate is good. | */
        EKF_POS_VERT_AGL | /* Set if EKF's vertical position (above ground) estimate is good. | */
        //EKF_CONST_POS_MODE | /* EKF is in constant position mode and does not know it's absolute or relative position. | */
        EKF_PRED_POS_HORIZ_REL | /* Set if EKF's predicted horizontal position (relative) estimate is good. | */
        EKF_PRED_POS_HORIZ_ABS; /* Set if EKF's predicted horizontal position (absolute) estimate is good. | */
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags, 0, 0, 0, 0, 0, 0);
}

bool AP_AHRS_SIM::get_origin(Location &ret) const
{
    if (_sitl == nullptr) {
        return false;
    }

    ret = _sitl->state.home;

    return true;
}

// return the innovations for the specified instance
// An out of range instance (eg -1) returns data for the primary instance
bool AP_AHRS_SIM::get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const
{
    velInnov.zero();
    posInnov.zero();
    magInnov.zero();
    tasInnov = 0.0f;
    yawInnov = 0.0f;

    return true;
}

bool AP_AHRS_SIM::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = 0;
    posVar = 0;
    hgtVar = 0;
    magVar.zero();
    tasVar = 0;

    return true;
}

void AP_AHRS_SIM::get_results(AP_AHRS_Backend::Estimates &results)
{
    if (_sitl == nullptr) {
        _sitl = AP::sitl();
        if (_sitl == nullptr) {
            return;
        }
    }

    const struct SITL::sitl_fdm &fdm = _sitl->state;
    const AP_InertialSensor &_ins = AP::ins();

    fdm.quaternion.rotation_matrix(results.dcm_matrix);
    results.dcm_matrix = results.dcm_matrix * AP::ahrs().get_rotation_vehicle_body_to_autopilot_body();
    results.dcm_matrix.to_euler(&results.roll_rad, &results.pitch_rad, &results.yaw_rad);

    results.gyro_estimate = _ins.get_gyro();
    results.gyro_drift.zero();

    const Vector3f &accel = _ins.get_accel();
    results.accel_ef = results.dcm_matrix * AP::ahrs().get_rotation_autopilot_body_to_vehicle_body() * accel;

#if HAL_NAVEKF3_AVAILABLE
    if (_sitl->odom_enable) {
        // use SITL states to write body frame odometry data at 20Hz
        uint32_t timeStamp_ms = AP_HAL::millis();
        if (timeStamp_ms - _last_body_odm_update_ms > 50) {
            const float quality = 100.0f;
            const Vector3f posOffset(0.0f, 0.0f, 0.0f);
            const float delTime = 0.001f * (timeStamp_ms - _last_body_odm_update_ms);
            _last_body_odm_update_ms = timeStamp_ms;
            timeStamp_ms -= (timeStamp_ms - _last_body_odm_update_ms)/2; // correct for first order hold average delay
            Vector3f delAng = _ins.get_gyro();

            delAng *= delTime;
            // rotate earth velocity into body frame and calculate delta position
            Matrix3f Tbn;
            Tbn.from_euler(radians(fdm.rollDeg),radians(fdm.pitchDeg),radians(fdm.yawDeg));
            const Vector3f earth_vel(fdm.speedN,fdm.speedE,fdm.speedD);
            const Vector3f delPos = Tbn.transposed() * (earth_vel * delTime);
            // write to EKF
            EKF3.writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, 0, posOffset);
        }
    }
#endif // HAL_NAVEKF3_AVAILABLE
}

#endif // AP_AHRS_SIM_ENABLED
