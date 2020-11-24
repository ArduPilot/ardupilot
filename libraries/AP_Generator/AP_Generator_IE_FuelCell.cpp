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

#include "AP_Generator_IE_FuelCell.h"
#include <AP_SerialManager/AP_SerialManager.h>

#if GENERATOR_ENABLED

// Initialize the fuelcell object and prepare it for use
void AP_Generator_IE_FuelCell::init()
{
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Generator, 0);

    if (_uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Generator: No serial port found");
        return;
    }
    _uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Generator, 0));

    _health_warn_sent = false;
}

// Update fuelcell, expected to be called at 20hz
void AP_Generator_IE_FuelCell::update()
{
    if (_uart == nullptr) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

   // Read any available data
    uint32_t nbytes = MIN(_uart->available(),30u);
    while (nbytes-- > 0) {
        const int16_t c = _uart->read();
        if (c < 0) {
            // Nothing to decode
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
    check_status();

    update_frontend();

    log_write();
}

// Add a single character to the buffer and attempt to decode
// Returns true if a complete sentence was successfully decoded
bool AP_Generator_IE_FuelCell::decode(char c)
{
    // Start of a string
    if (c == '<') {
        _sentence_valid = false;
        _data_valid = true;
        _term_number = 0;
        _term_offset = 0;
        _in_string = true;
        return false;
    }
    if (!_in_string) {
        return false;
    }

    // End of a string
    if (c == '>') {
        decode_latest_term();
        _in_string = false;

        return _sentence_valid;
    }

    // End of a term in the string
    if (c == ',') {
        decode_latest_term();
        return false;
    }

    // Otherwise add the char to the current term
    _term[_term_offset++] = c;

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
        strncpy(failmsg, "Driver not healthy", failmsg_len);
        return false;
    }

    // Refuse arming if not in running state
    if (_state != State::RUNNING) {
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
void AP_Generator_IE_FuelCell::check_status()
{
    // Check driver health
    if (!healthy() && !_health_warn_sent) {
        // Don't spam GCS with unhealthy message
        _health_warn_sent = true;
        gcs().send_text(MAV_SEVERITY_ALERT, "Generator: Driver not healthy");

    } else if (healthy()) {
        _health_warn_sent = false;
    }

    // If fuel cell state has changed send gcs message
    if (_state != _last_state) {
        for (const struct Lookup_State entry : lookup_state) {
            if (_state == entry.option) {
                gcs().send_text(MAV_SEVERITY_INFO, "Generator: %s", entry.msg_txt);
                break;
            }
        }
        _last_state = _state;
    }

    // Check error codes
    char msg_txt[32];
    if (check_for_err_code_if_changed(msg_txt, sizeof(msg_txt))) {
        gcs().send_text(MAV_SEVERITY_ALERT, "%s", msg_txt);
    }
}

// Check error codes and populate message with error code
bool AP_Generator_IE_FuelCell::check_for_err_code_if_changed(char* msg_txt, uint8_t msg_len)
{
    // Only check if there has been a change in error code
    if (_err_code == _last_err_code) {
        return false;
    }

    if (check_for_err_code(msg_txt, msg_len)) {
        _last_err_code = _err_code;
        return true;
    }

    return false;
}
#endif
