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
 *  shim AP_NavEKF2 into AP_AHRS_Backend
 */

#include "AP_AHRS_config.h"

#if AP_AHRS_NAVEKF2_ENABLED

#include <AP_NavEKF2/AP_NavEKF2.h>
#include "AP_AHRS_Backend.h"

class AP_AHRS_NavEKF2 : public AP_AHRS_Backend {
public:

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS_NavEKF2);
    AP_AHRS_NavEKF2() {}

    const char *shortname() const override { return "EKF2"; }

    void reset_gyro_drift() override { EKF2.resetGyroBias(); }

    void update() override;

    void get_results(Estimates &results) override;
    void reset() override {
        if (!started) {
            return;
        }
        started = EKF2.InitialiseFilter();
    }

    bool get_origin(Location &ret) const override {
        return EKF2.getOriginLLH(ret);
    }
    bool set_origin(const Location &loc) override {
        return EKF2.setOriginLLH(loc);
    }

    bool use_compass() override {
        return EKF2.use_compass();
    }

    void resetHeightDatum(float origin_alt_tolerance_m) override {
        EKF2.resetHeightDatum(origin_alt_tolerance_m);
    }

    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    void get_control_limits(float &ekfGndSpdLimit, float &controlScaleXY) const override {
        return EKF2.getEkfControlLimits(ekfGndSpdLimit, controlScaleXY);
    }

    // // return the innovations for the specified instance
    // // An out of range instance (eg -1) returns data for the primary instance
    bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const override {
        return EKF2.getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
    }

    void request_yaw_reset(void) override {
        EKF2.requestYawReset();
    }
    void check_lane_switch() override {
        EKF2.checkLaneSwitch();
    }

    // this is out here so parameters can be poked into it
    static NavEKF2 EKF2;

    bool start();
    bool started;
    uint32_t start_time_ms;  // timer used to delay starting the filter

    // a counter which is incremented each time the primary core changes:
    AP_AHRS_ResetCounter<int8_t> attitude_reset_tracker;

    AP_AHRS_ResetCounter<uint16_t> yaw_reset_tracker;
    AP_AHRS_ResetCounter<uint16_t> position_NE_reset_tracker;
    AP_AHRS_ResetCounter<uint16_t> position_D_reset_tracker;
};

#endif  // AP_AHRS_NAVEKF2_ENABLED
