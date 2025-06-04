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
  Support for Trimble PX-1 GNSS-RTX-INS.
 */


#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_GSOF_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_GSOF/AP_GSOF.h>

class AP_ExternalAHRS_GSOF: public AP_ExternalAHRS_backend, public AP_GSOF
{
public:

    AP_ExternalAHRS_GSOF(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // Get model/type name
    const char* get_name() const override;

    // get serial port number, -1 for not enabled
    int8_t get_port() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;

    // check for new data
    void update() override
    {
        // This driver update runs in a dedicated thread, skip this.
    };

protected:

    uint8_t num_gps_sensors(void) const override
    {
        return 0;
    }

private:

    void update_thread();
    void check_initialise_state();
    bool times_healthy() const;
    bool filter_healthy() const;
    void post_filter() const;

    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    uint32_t baudrate;
    uint8_t *pktbuf;

    HAL_Semaphore sem;

    // Used to monitor initialization state.
    bool last_init_state = false;

    // The last time we received a message from the GNSS
    // containing AP_GSOF::POS_TIME.
    uint32_t last_pos_time_ms;
    // The last time we received a message from the GNSS
    // containing AP_GSOF::INS_FULL_NAV.
    uint32_t last_ins_full_nav_ms;
    // The last time we received a message from the GNSS
    // containing AP_GSOF::INS_RMS.
    uint32_t last_ins_rms_ms;
    // The last time we received a message from the GNSS
    // containing AP_GSOF::LLH_MSL.
    uint32_t last_llh_msl_ms;

    // The last calculated undulation, which is computed only at the rate both inputs are received.
    // The undulation is necessary to convert the INS full nav solution from ellipsoid to MSL altitude.
    double undulation;

    AP_ExternalAHRS::gps_data_message_t gps_data;

};

#endif // AP_EXTERNAL_AHRS_GSOF_ENABLED
