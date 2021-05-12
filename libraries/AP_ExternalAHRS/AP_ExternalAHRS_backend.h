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
  parent class for ExternalAHRS backends
 */

#pragma once

#include "AP_ExternalAHRS.h"

#if HAL_EXTERNAL_AHRS_ENABLED

class AP_ExternalAHRS_backend {
public:
    AP_ExternalAHRS_backend(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    virtual int8_t get_port(void) const { return -1; }

    // accessors for AP_AHRS
    virtual bool healthy(void) const = 0;
    virtual bool initialised(void) const = 0;
    virtual bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const = 0;
    virtual void get_filter_status(nav_filter_status &status) const {}
    virtual void send_status_report(mavlink_channel_t chan) const {}

    // check for new data
    virtual void update() = 0;
    
protected:
    AP_ExternalAHRS::state_t &state;
    uint16_t get_rate(void) const;

private:
    AP_ExternalAHRS &frontend;
};

#endif  // HAL_EXTERNAL_AHRS_ENABLED

