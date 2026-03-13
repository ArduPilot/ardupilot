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
 *  shim AP_NavEKF3 into AP_AHRS_Backend
 */

#include "AP_AHRS_config.h"

#if AP_AHRS_NAVEKF3_ENABLED

#include <AP_NavEKF3/AP_NavEKF3.h>
#include "AP_AHRS_Backend.h"

class AP_AHRS_NavEKF3 : public AP_AHRS_Backend {
public:

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS_NavEKF3);
    AP_AHRS_NavEKF3() {}

    bool healthy(void) const override {
        if (!started) {
            return false;
        }
        if (!EKF3.healthy()) {
            return false;
        }
        return true;
    }

    void reset_gyro_drift() override { EKF3.resetGyroBias(); }

    void update() override { EKF3.UpdateFilter(); }

    void get_results(Estimates &results) override;
    void reset() override {
        if (!started) {
            return;
        }
        started = EKF3.InitialiseFilter();
    }

    bool get_origin(Location &ret) const override {
        return EKF3.getOriginLLH(ret);
    }
    bool set_origin(const Location &loc) override {
        return EKF3.setOriginLLH(loc);
    }

    // return a wind estimation vector, in m/s
    bool wind_estimate(Vector3f &wind) const override {
        return EKF3.getWind(wind);
    }

    Vector2f groundspeed_vector(void) override {
        Vector3f vec;
        EKF3.getVelNED(vec);
        return vec.xy();
    }

    bool            use_compass() override {
        return EKF3.use_compass();
    }

    uint32_t getLastYawResetAngle(float &yawAng) override {
        return EKF3.getLastYawResetAngle(yawAng);
    };
    uint32_t getLastPosNorthEastReset(Vector2f &pos) override WARN_IF_UNUSED {
        return EKF3.getLastPosNorthEastReset(pos);
    };
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const override WARN_IF_UNUSED {
        return EKF3.getLastVelNorthEastReset(vel);
    };
    uint32_t getLastPosDownReset(float &posDelta) override WARN_IF_UNUSED {
        return EKF3.getLastPosDownReset(posDelta);
    };
    void resetHeightDatum(void) override {
        EKF3.resetHeightDatum();
    }
    void request_yaw_reset() override {
        EKF3.requestYawReset();
    }
    // get latest altitude estimate above ground level in meters and validity flag
    bool get_hagl(float &hagl) const override WARN_IF_UNUSED {
        return EKF3.getHAGL(hagl);
    }

    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    void get_control_limits(float &ekfGndSpdLimit, float &controlScaleXY) const override {
        return EKF3.getEkfControlLimits(ekfGndSpdLimit, controlScaleXY);
    }
    void send_ekf_status_report(class GCS_MAVLINK &link) const override {
        EKF3.send_status_report(link);
    }

    // get_filter_status - returns filter status as a series of flags
    bool get_filter_status(nav_filter_status &status) const override {
        EKF3.getFilterStatus(status);
        return true;
    }

    // return the innovations for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const override {
        return EKF3.getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
    }

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override {
        Vector2f offset;
        return EKF3.getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    }

    // this is out here so parameters can be poked into it
    static NavEKF3 EKF3;

    bool started;
};

#endif  // AP_AHRS_NAVEKF3_ENABLED
