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
    void            update() override;
    void            get_results(Estimates &results) override;
    void            reset() override {}

    bool            use_compass() override {
        // this is actually never called at the moment; we use dcm's
        // return value.
        return true;
    }

    void estimate_wind(void);

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked (not used)
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    bool get_filter_status(nav_filter_status &status) const override;
    void send_ekf_status_report(class GCS_MAVLINK &link) const override;

private:

    bool get_relative_position_NED_origin(Vector3f &vec) const;

};

#endif
