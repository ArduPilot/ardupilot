#pragma once

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
 *  SITL-based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */

#include "AP_AHRS_config.h"

#if AP_AHRS_SIM_ENABLED

#include "AP_AHRS_Backend.h"

#include <GCS_MAVLink/GCS.h>
#include <SITL/SITL.h>

#if HAL_NAVEKF3_AVAILABLE
#include <AP_NavEKF3/AP_NavEKF3.h>
#endif

class AP_AHRS_SIM : public AP_AHRS_Backend {
public:

#if HAL_NAVEKF3_AVAILABLE
    AP_AHRS_SIM(NavEKF3 &_EKF3) :
        AP_AHRS_Backend(),
        EKF3(_EKF3)
        { }
    ~AP_AHRS_SIM() {}
#else
    // a version of the constructor which doesn't take a non-existant
    // NavEKF3 class instance as a parameter.
    AP_AHRS_SIM() : AP_AHRS_Backend() { }
#endif

    CLASS_NO_COPY(AP_AHRS_SIM);

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift() override {};

    // Methods
    void            update() override { }
    void            get_results(Estimates &results) override;
    void            reset() override { return; }

    // dead-reckoning support
    virtual bool get_location(Location &loc) const override;

    // get latest altitude estimate above ground level in meters and validity flag
    bool get_hagl(float &hagl) const override WARN_IF_UNUSED;

    // return a wind estimation vector, in m/s
    bool wind_estimate(Vector3f &wind) const override;

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool airspeed_estimate(float &airspeed_ret) const override;

    // return an airspeed estimate if available. return true
    // if we have an estimate from a specific sensor index
    bool airspeed_estimate(uint8_t airspeed_index, float &airspeed_ret) const override;

    // return a ground vector estimate in meters/second, in North/East order
    Vector2f groundspeed_vector() override;

    bool            use_compass() override { return true; }

    // return the quaternion defining the rotation from NED to XYZ (body) axes
    bool get_quaternion(Quaternion &quat) const override WARN_IF_UNUSED;

    // is the AHRS subsystem healthy?
    bool healthy() const override { return true; }

    bool get_velocity_NED(Vector3f &vec) const override;

    // Get a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    bool get_vert_pos_rate(float &velocity) const override;

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked (not used)
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override { return true; }

    // get_filter_status - returns filter status as a series of flags
    bool get_filter_status(nav_filter_status &status) const override;

    // get compass offset estimates
    // true if offsets are valid
    bool get_mag_offsets(uint8_t mag_idx, Vector3f &magOffsets) const override;

    // relative-origin functions for fallback in AP_InertialNav
    bool get_origin(Location &ret) const override;
    bool get_relative_position_NED_origin(Vector3f &vec) const override;
    bool get_relative_position_NE_origin(Vector2f &posNE) const override;
    bool get_relative_position_D_origin(float &posD) const override;

    void send_ekf_status_report(class GCS_MAVLINK &link) const override;

    void get_control_limits(float &ekfGndSpdLimit, float &controlScaleXY) const override;
    bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

private:

#if HAL_NAVEKF3_AVAILABLE
    // a reference to the EKF3 backend that we can use to send in
    // body-frame-odometry data into the EKF.  Rightfully there should
    // be something over in the SITL directory doing this.
    NavEKF3 &EKF3;
#endif

    class SITL::SIM *_sitl;
    uint32_t _last_body_odm_update_ms;
};

#endif  // AP_AHRS_SIM_ENABLED
