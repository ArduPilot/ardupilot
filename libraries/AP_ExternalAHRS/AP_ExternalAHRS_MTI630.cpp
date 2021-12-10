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
  suppport for serial connected external AHRS MTI630
  https://www.xsens.com/hubfs/Downloads/Leaflets/MTi%20600-series%20Datasheet.pdf
  It needs to be previously configured using the configuration software
  so it outputs PSONCMS messages.
  Tested with messages coming at 50Hz, 115200 baud
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_MTI630.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/NMEA.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#if HAL_EXTERNAL_AHRS_ENABLED

extern const AP_HAL::HAL &hal;

// constructor
AP_ExternalAHRS_MTI630::AP_ExternalAHRS_MTI630(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_MTI630::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS initialised");
}


void AP_ExternalAHRS_MTI630::update_thread()
{
    if (!port_opened) {
        // open port in the thread
        port_opened = true;
        uart->begin(baudrate, 1024, 512);
    }

    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay(1);
        }

    }
}

// Get model/type name
const char* AP_ExternalAHRS_MTI630::get_name() const
{
    return "MTI630";
}

/*
  check the UART for more data
  returns true if the function should be called again straight away
 */
bool AP_ExternalAHRS_MTI630::check_uart()
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the windvane
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        if (decode(c)) {
            WITH_SEMAPHORE(state.sem);
            state.accel = psoncms.accel;
            state.gyro = psoncms.gyro;
            state.quat = psoncms.quat;
            state.have_location = false;
            state.have_origin = false;
            state.have_quaternion = true;
            state.have_velocity = false;
            post_imu();
            _last_update_ms = AP_HAL::millis();
        }
    }
    return false;
}


// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_MTI630::post_imu() const
{
    AP_ExternalAHRS::ins_data_message_t ins {
        accel: Vector3f(
            psoncms.accel.x,
            -psoncms.accel.z,
            psoncms.accel.y
        ),
        gyro: Vector3f(
            psoncms.gyro.x,
            -psoncms.gyro.z,
            psoncms.gyro.y
        ),
        temperature: psoncms.temperature
    };
    AP::ins().handle_external(ins);

    AP_ExternalAHRS::mag_data_message_t mag {
        field: Vector3f(
            // The datasheet says the compass outputs are "Arbitrary units"
            // in testings these gains made the values match the values of the internal compasses
            // The signals make the rotations match
            psoncms.mag.x * 500,
            psoncms.mag.y * -500,
            psoncms.mag.z * -500)
    };
    AP::compass().handle_external(mag);

    // @LoggerMessage: EAH1
    // @Description: External AHRS data
    // @Field: TimeUS: Time since system startup
    // @Field: Roll: euler roll
    // @Field: Pitch: euler pitch
    // @Field: Yaw: euler yaw
    float roll, pitch, yaw;
    state.quat.to_euler(roll, pitch, yaw);
    AP::logger().WriteStreaming("MTI", "TimeUS,Roll,Pitch,Yaw,xmag,ymag,zmag",
                       "sdddGGG", "F000000",
                       "Qffffff",
                       AP_HAL::micros64(),
                       degrees(roll), degrees(pitch), degrees(yaw), psoncms.mag.x, psoncms.mag.y, psoncms.mag.z);
}

// Utility function to parse hex values
bool AP_ExternalAHRS_MTI630::parse_hex(const char* str, uint8_t* out) {
    if (!isxdigit(str[0]) || !isxdigit(str[1])) {
        return false;
    }

    char buffer[3] = {str[0], str[1], '\0'};
    *out = (uint8_t)strtol(buffer, NULL, 16);
    return true;
}

bool AP_ExternalAHRS_MTI630::isxdigit(int c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

// Utility function to parse floats from a string
bool AP_ExternalAHRS_MTI630::parse_floats(const char* str, float* out, int count) {
    char* endptr;

    for (int i = 0; i < count; i++) {
        float val = strtof(str, &endptr);
        if (endptr == str) {
            // Conversion failed
            return false;
        }

        // Assign the parsed value
        out[i] = val;

        // Move to the next number
        str = endptr;

        if (i < count - 1) {  // Not for the last number
            if (*str != ',') {
                // Not enough numbers
                return false;
            }
            str++;  // Skip the comma
        }
    }

    return true;
}

bool AP_ExternalAHRS_MTI630::decode(char c)
{
    static char buffer[256];
    static uint16_t idx = 0;
    static bool start_msg = false;

    // If we have a new line or carriage return
    if (c == '\n' || c == '\r') {
        if (start_msg) {
            buffer[idx] = 0;  // null terminate the string
            start_msg = false;

            // Now process buffer
            if (strncmp(buffer, "$PSONCMS,", 9) == 0) { // Verify it's the right message type
                // Calculate the checksum
                uint8_t chksum = 0;
                for (uint16_t i = 1; i < idx - 3; i++) {  // Exclude "$" and "*hh"
                    chksum ^= buffer[i];
                }

                // Read the sent checksum
                uint8_t sent_chksum;
                if (!parse_hex(buffer + (idx - 2), &sent_chksum)) {
                    // Invalid checksum
                    return false;
                }

                // Verify checksums match
                if (chksum == sent_chksum) {
                    // Checksums match, process data
                    float values[14];
                    if (!parse_floats(buffer + 9, values, 14)) {
                        // Invalid data
                        return false;
                    }

                    // ENU to NED
                    psoncms.quat = Quaternion(values[0], values[1], -values[2], -values[3]);

                    psoncms.accel = Vector3f(values[4], values[5], values[6]);
                    psoncms.gyro = Vector3f(values[7], values[8], values[9]);
                    psoncms.mag = Vector3f(values[10], values[11], values[12]);
                    psoncms.temperature = values[13];

                    return true;
                }
            }
        }
        idx = 0;
    } else if (c == '$') { // Start of a new message
        start_msg = true;
    }

    if (start_msg) {
        buffer[idx++] = c;
        if (idx >= sizeof(buffer)) {
            // Buffer overflow, this shouldn't happen if data is well formed
            idx = 0;
            start_msg = false;
        }
    }

    return false;
}


bool AP_ExternalAHRS_MTI630::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - _last_update_ms) < 500;
    return true;
}

bool AP_ExternalAHRS_MTI630::initialised(void) const
{
    return true;
}

bool AP_ExternalAHRS_MTI630::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "MTI630 unhealthy");
        return false;
    }
    return true;
}

/*
  Assume it is ok if we are getting data
 */
void AP_ExternalAHRS_MTI630::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    status.flags.initalized = 1;
    if (healthy()) {
        status.flags.attitude = 1;
        status.flags.vert_vel = 0;
        status.flags.vert_pos = 0;

    }
    return;
}

// send an EKF_STATUS message to GCS
void AP_ExternalAHRS_MTI630::send_status_report(class GCS_MAVLINK &link) const
{
    // TODO: implement this
    return;
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_MTI630::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

#endif // AP_EXTERNAL_AHRS_MTI630_ENABLED
