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
  support for MicroStrain GQ7 serially connected AHRS Systems
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_MICROSTRAIN7_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include "MicroStrain_common.h"

class AP_ExternalAHRS_MicroStrain7: public AP_ExternalAHRS_backend, public AP_MicroStrain
{
public:

    AP_ExternalAHRS_MicroStrain7(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(class GCS_MAVLINK &link) const override;

    // check for new data
    void update() override
    {
        build_packet();
    };

private:

    // GQ7 Filter States
    // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/data/mip_field_filter_status.htm
    enum class FilterState {
        GQ7_INIT = 0x01,
        GQ7_VERT_GYRO = 0x02,
        GQ7_AHRS = 0x03,
        GQ7_FULL_NAV = 0x04
    };

    uint32_t baudrate;
    int8_t port_num;
    bool port_open = false;



    void build_packet();

    void post_imu() const;
    void post_gnss() const;
    void post_filter() const;

    void update_thread();
    void check_initialise_state();

    // Returns true when data is not stale.
    bool times_healthy() const;

    // Returns true when the filter is currently healthy.
    bool filter_healthy() const;

    // Only some of the fix types satisfy a healthy filter.
    // GQ7_VERT_GYRO is NOT considered healthy for now.
    // This may be vehicle-dependent in the future.
    static bool filter_state_healthy(FilterState state) WARN_IF_UNUSED;

    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;

    // Used to monitor initialization state.
    bool last_init_state = false;

};

#endif // AP_EXTERNAL_AHRS_MICROSTRAIN7_ENABLED
