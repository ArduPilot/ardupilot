#include "AP_AHRS_External.h"

#if HAL_EXTERNAL_AHRS_ENABLED

#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_AHRS/AP_AHRS.h>

// true if the AHRS has completed initialisation
bool AP_AHRS_External::initialised(void) const
{
    return AP::externalAHRS().initialised();
}

void AP_AHRS_External::update()
{
    AP::externalAHRS().update();
}

bool AP_AHRS_External::healthy() const {
    return AP::externalAHRS().healthy();
}

void AP_AHRS_External::get_results(AP_AHRS_Backend::Estimates &results)
{
    Quaternion quat;
    if (!AP::externalAHRS().get_quaternion(quat)) {
        return;
    }
    quat.rotation_matrix(results.dcm_matrix);
    results.dcm_matrix = results.dcm_matrix * AP::ahrs().get_rotation_vehicle_body_to_autopilot_body();
    results.dcm_matrix.to_euler(&results.roll_rad, &results.pitch_rad, &results.yaw_rad);

    results.gyro_drift.zero();
    results.gyro_estimate = AP::externalAHRS().get_gyro();

    const Vector3f accel = AP::externalAHRS().get_accel();
    const Vector3f accel_ef = results.dcm_matrix * AP::ahrs().get_rotation_autopilot_body_to_vehicle_body() * accel;
    results.accel_ef = accel_ef;
}


bool AP_AHRS_External::get_location(struct Location &loc) const
{
    return AP::externalAHRS().get_location(loc);
}

bool AP_AHRS_External::get_quaternion(Quaternion &quat) const
{
    return AP::externalAHRS().get_quaternion(quat);
}

Vector2f AP_AHRS_External::groundspeed_vector()
{
    return AP::externalAHRS().get_groundspeed_vector();
}


bool AP_AHRS_External::get_relative_position_NED_origin(Vector3f &vec) const
{
    auto &extahrs = AP::externalAHRS();
    Location loc, orgn;
    if (extahrs.get_origin(orgn) &&
        extahrs.get_location(loc)) {
        const Vector2f diff2d = orgn.get_distance_NE(loc);
        vec = Vector3f(diff2d.x, diff2d.y,
                       -(loc.alt - orgn.alt)*0.01);
        return true;
    }
    return false;
}

bool AP_AHRS_External::get_relative_position_NE_origin(Vector2f &posNE) const
{
    auto &extahrs = AP::externalAHRS();

    Location loc, orgn;
    if (!extahrs.get_location(loc) ||
        !extahrs.get_origin(orgn)) {
        return false;
    }
    posNE = orgn.get_distance_NE(loc);
    return true;
}

bool AP_AHRS_External::get_relative_position_D_origin(float &posD) const
{
    auto &extahrs = AP::externalAHRS();

    Location orgn, loc;
    if (!extahrs.get_origin(orgn) ||
        !extahrs.get_location(loc)) {
        return false;
    }
    posD = -(loc.alt - orgn.alt)*0.01;
    return true;
}

bool AP_AHRS_External::get_velocity_NED(Vector3f &vec) const
{
    return AP::externalAHRS().get_velocity_NED(vec);
}

bool AP_AHRS_External::get_vert_pos_rate_D(float &velocity) const
{
    return AP::externalAHRS().get_speed_down(velocity);
}

bool AP_AHRS_External::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    return AP::externalAHRS().pre_arm_check(failure_msg, failure_msg_len);
}

bool AP_AHRS_External::get_filter_status(nav_filter_status &status) const
{
    AP::externalAHRS().get_filter_status(status);
    return true;
}

void AP_AHRS_External::send_ekf_status_report(GCS_MAVLINK &link) const
{
    AP::externalAHRS().send_status_report(link);
}

bool AP_AHRS_External::get_origin(Location &ret) const
{
    return AP::externalAHRS().get_origin(ret);
}

void AP_AHRS_External::get_control_limits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    // lower gains in VTOL controllers when flying on DCM
    ekfGndSpdLimit = 50.0;
    ekfNavVelGainScaler = 0.5;
}

#endif
