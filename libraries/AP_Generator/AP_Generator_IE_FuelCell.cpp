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

#pragma GCC optimize("Os")

#include "AP_Generator_IE_FuelCell.h"

#if AP_GENERATOR_IE_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

// Initialize the fuelcell object and prepare it for use
void AP_Generator_IE_FuelCell::init()
{
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Generator, 0);

    if (_uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Generator: No serial port found");
        return;
    }
    _uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Generator, 0));
    _health_warn_last_ms = AP_HAL::millis();
}

// Update fuelcell, expected to be called at 10hz
void AP_Generator_IE_FuelCell::update()
{
    if (_uart == nullptr) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

    // Read any available data
    for (uint8_t i = 0; i < UINT8_MAX; i++) {  // process at most n bytes
        uint8_t c;
        if (!_uart->read(c)) {
            break;
        }

        if (!decode(c)) {
            // Sentence not yet valid, don't assign state and output
            continue;
        }

        // We have a valid sentence, write the parsed values to unit specific measurements
        assign_measurements(now);
    }

    _healthy = (now - _last_time_ms) < HEALTHY_TIMEOUT_MS;

    // Check if we should notify gcs off any change of fuel cell state
    check_status(now);

    update_frontend();

#if HAL_LOGGING_ENABLED
    log_write();
#endif
}

// Add a single character to the buffer and attempt to decode
// Returns true if a complete sentence was successfully decoded
bool AP_Generator_IE_FuelCell::decode(char c)
{
    // Start of a string
    if ((c == '<') || (c == '[')) {
        _start_char = c;
        _sentence_valid = false;
        _data_valid = true;
        _term_number = 0;
        _term_offset = 0;
        _in_string = true;
        _checksum = c;
        return false;
    }
    if (!_in_string) {
        return false;
    }

    // End of a string
    const char end_char = (_start_char == '[') ? ']' : '>';
    if (c == end_char) {
        decode_latest_term();
        _in_string = false;

        return _sentence_valid;
    }

    // End of a term in the string
    if (c == ',') {
        decode_latest_term();
        _checksum += c;
        return false;
    }

    // Otherwise add the char to the current term
    _term[_term_offset++] = c;
    _checksum += c;

    // We have overrun the expected sentence
    if (_term_offset >TERM_BUFFER) {
        _in_string = false;
    }

    return false;
}

// Check for arming
bool AP_Generator_IE_FuelCell::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    // Refuse arming if not healthy
    if (!healthy()) {
        strncpy(failmsg, "Not healthy", failmsg_len);
        return false;
    }

    // Refuse arming if not in running state
    if (!is_running()) {
        strncpy(failmsg, "Status not running", failmsg_len);
        return false;
    }

    // Check for error codes
    if (check_for_err_code(failmsg, failmsg_len)) {
        return false;
    }

    return true;
}

// Lookup table for running state.  State code is the same for all IE units.
const AP_Generator_IE_FuelCell::Lookup_State AP_Generator_IE_FuelCell::lookup_state[] = {
    { State::STARTING,"Starting"},
    { State::READY,"Ready"},
    { State::RUNNING,"Running"},
    { State::FAULT,"Fault"},
    { State::BATTERY_ONLY,"Battery Only"},
};

// Check for any change in error state or status and report to gcs
void AP_Generator_IE_FuelCell::check_status(const uint32_t now)
{
    // Check driver health
    if (!healthy() && (!_health_warn_last_ms || (now - _health_warn_last_ms >= 20000))) {
        // Don't spam GCS with unhealthy message
        _health_warn_last_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Generator: Not healthy");

    } else if (healthy()) {
        _health_warn_last_ms = 0;
    }

    // If fuel cell state has changed send gcs message
    update_state_msg();

    // Check error codes
    check_for_err_code_if_changed();
}

// Check error codes and populate message with error code
void AP_Generator_IE_FuelCell::check_for_err_code_if_changed()
{
    // Only check if there has been a change in error code
    if ((_err_code == _last_err_code) && (_sub_err_code == _last_sub_err_code)) {
        return;
    }

#if HAL_GCS_ENABLED
    char msg_txt[64];
    if (check_for_err_code(msg_txt, sizeof(msg_txt)) || check_for_warning_code(msg_txt, sizeof(msg_txt))) {
        GCS_SEND_TEXT(get_mav_severity(_err_code), "%s", msg_txt);

    } else if ((_err_code == 0) && (_sub_err_code == 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Fuel cell error cleared");

    }
#endif

    _last_err_code = _err_code;
    _last_sub_err_code = _sub_err_code;

}

// Return true is fuel cell is in running state suitable for arming
bool AP_Generator_IE_FuelCell::is_running() const
{
    return _state == State::RUNNING;
}

// Print msg to user updating on state change
void AP_Generator_IE_FuelCell::update_state_msg()
{
    // If fuel cell state has changed send gcs message
    if (_state != _last_state) {
        for (const struct Lookup_State entry : lookup_state) {
            if (_state == entry.option) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Generator: %s", entry.msg_txt);
                break;
            }
        }
        _last_state = _state;
    }
}

#endif  // AP_GENERATOR_IE_ENABLED
