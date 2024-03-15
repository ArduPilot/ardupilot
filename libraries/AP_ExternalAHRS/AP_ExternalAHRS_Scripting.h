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
  support for scripting based external AHRS
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SCRIPTING_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_Scripting : public AP_ExternalAHRS_backend {

public:
    using AP_ExternalAHRS_backend::AP_ExternalAHRS_backend;

    // get serial port number, assume first port
    int8_t get_port(void) const override { return 0; }

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override {
        // do pre-arm via scripting arming checks
        return true;
    }
    void get_filter_status(nav_filter_status &status) const override;

    // check for new data, done by script
    void update() override {}

    // Get model/type name
    const char* get_name() const override {
        return "Scripting";
    }

    // handle input from scripting backend
    bool handle_scripting(const AP_ExternalAHRS::state_t &_state, nav_filter_status_flags_t &_filter_status) override;

private:
    uint32_t last_update_ms;
    nav_filter_status filter_status;
};

#endif  // AP_EXTERNAL_AHRS_SCRIPTING_ENABLED

