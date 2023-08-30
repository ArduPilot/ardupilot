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
  suppport for MicroStrain CX5/GX5-45 serially connected AHRS Systems
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include "MicroStrain_common.h"

class AP_ExternalAHRS_MicroStrain5: public AP_ExternalAHRS_backend, public AP_MicroStrain
{
public:

    AP_ExternalAHRS_MicroStrain5(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

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
    void update() override {
        build_packet();
    };

private:

    uint32_t baudrate;
    int8_t port_num;
    bool port_open = false;
    


    void build_packet();

    void post_imu() const;
    void post_gnss() const;
    void post_filter() const;

    void update_thread();

    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;

};

#endif // AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED
