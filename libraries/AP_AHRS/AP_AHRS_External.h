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
 *  External based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */

#include "AP_AHRS_config.h"

#if AP_AHRS_EXTERNAL_ENABLED

#include "AP_AHRS_Backend.h"

class AP_AHRS_External : public AP_AHRS_Backend {
public:

    using AP_AHRS_Backend::AP_AHRS_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS_External);

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift() override {}

    // Methods
    bool            initialised() const override;
    void            update() override;
    void            get_results(Estimates &results) override;
    void            reset() override {}

    // return a wind estimation vector, in m/s
    bool wind_estimate(Vector3f &ret) const override {
        return false;
    }

    // return a ground vector estimate in meters/second, in North/East order
    Vector2f groundspeed_vector() override;

    bool            use_compass() override {
        // this is actually never called at the moment; we use dcm's
        // return value.
        return true;
    }

    // return the quaternion defining the rotation from NED to XYZ (body) axes
    bool get_quaternion(Quaternion &quat) const override WARN_IF_UNUSED;

    void estimate_wind(void);

    // is the AHRS subsystem healthy?
    bool healthy() const override;

    bool get_velocity_NED(Vector3f &vec) const override;

    // Get a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    bool get_vert_pos_rate_D(float &velocity) const override;

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked (not used)
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    // relative-origin functions for fallback in AP_InertialNav
    bool get_origin(Location &ret) const override;
    bool get_relative_position_NED_origin(Vector3f &vec) const override;
    bool get_relative_position_NE_origin(Vector2f &posNE) const override;
    bool get_relative_position_D_origin(float &posD) const override;

    bool get_filter_status(nav_filter_status &status) const override;
    void send_ekf_status_report(class GCS_MAVLINK &link) const override;

    void get_control_limits(float &ekfGndSpdLimit, float &controlScaleXY) const override;
};

#endif
