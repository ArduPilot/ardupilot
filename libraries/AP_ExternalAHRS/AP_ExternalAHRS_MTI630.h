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
  suppport for serial connected AHRS systems
 */

#pragma once

#include "AP_ExternalAHRS_backend.h"

#if AP_EXTERNAL_AHRS_MTI630_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ExternalAHRS_MTI630 : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_MTI630(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

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
        check_uart();
    }

private:
    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    bool port_opened;
    uint32_t baudrate;
    uint16_t rate;
    int32_t _last_update_ms;

    bool isxdigit(int c);
    bool parse_hex(const char* str, uint8_t* out);

    bool parse_floats(const char* str, float* out, int count);

    void update_thread();
    bool check_uart();

    void post_imu() const;

      // try and decode NMEA message
    bool decode(char c);

    // Struct to hold PSONCMS message data
    struct psoncms_t {
        Quaternion quat;
        Vector3f accel;
        Vector3f gyro;
        Vector3f mag;
        float temperature;
    } psoncms;

};

#endif  // AP_EXTERNAL_AHRS_MTI630_ENABLED

