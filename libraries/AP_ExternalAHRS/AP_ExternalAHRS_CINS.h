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
  CINS external AHRS implementation
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_CINS_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_CINS/AP_CINS.h>

class AP_ExternalAHRS_CINS : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_CINS(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state, AP_CINS *&cins_ptr);

    // accessors for AP_AHRS
    bool healthy(void) const override {
        return true;
    }
    bool initialised(void) const override {
        return true;
    }
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override {
        return true;
    }

    void get_filter_status(nav_filter_status &status) const override;

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data
    void update() override;

    // Get model/type name
    const char* get_name() const override {
        return "CINS";
    }

private:
    AP_CINS cins;
};

#endif  // AP_EXTERNAL_AHRS_CINS_ENABLED
