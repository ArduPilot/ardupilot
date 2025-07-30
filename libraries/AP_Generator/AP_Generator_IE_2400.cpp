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

#include "AP_Generator_IE_2400.h"

#if AP_GENERATOR_IE_2400_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void AP_Generator_IE_2400::init()
{
    // Call init from base class to do common setup
    AP_Generator_IE_FuelCell::init();

    // Tell frontend what measurements are available for this generator
    _frontend._has_current = true;
    _frontend._has_consumed_energy = true;
    _frontend._has_fuel_remaining = true;
}

// Assigns the unit specific measurements once a valid sentence is obtained
void AP_Generator_IE_2400::assign_measurements(const uint32_t now)
{

    if (_type == PacketType::V2_INFO) {
        // Got info packet
        if (_had_info) {
            // Not expecting the version to change
            return;
        }
        _had_info = true;

        // Info tells us the protocol version, so lock on straight away
        if (_version == ProtocolVersion::DETECTING) {
            if (strcmp(_info.Protocol_version, "4") == 0) {
                _version = ProtocolVersion::V2;
            } else {
                // Got a valid info packet, but don't know this protocol version
                // Give up
                _version = ProtocolVersion::UNKNOWN;
            }
        }

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IE Fuel cell detected, PCM: %s, Ver: %s, SN: %s", _info.PCM_number, _info.Software_version, _info.Serial_number);

        return;
    }

    // Try and lock onto version
    if (_version == ProtocolVersion::DETECTING) {
        ProtocolVersion new_version = ProtocolVersion::DETECTING;
        switch (_type) {
            case PacketType::NONE:
                // Should not get a valid packet of type none
                _last_version_packet_count = 0;
                return;

            case PacketType::LEGACY_DATA:
                new_version = ProtocolVersion::LEGACY;
                break;

            case PacketType::V2_DATA:
            case PacketType::V2_INFO:
                new_version = ProtocolVersion::V2;
                break;
        }

        if (_last_version == new_version) {
            _last_version_packet_count++;
        } else {
            _last_version_packet_count = 0;
        }
        _last_version = new_version;

        // If received 20 valid packets for a single protocol version then lock on
        if (_last_version_packet_count > 20) {
            _version = new_version;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Generator: IE using %s protocol", (_version == ProtocolVersion::V2) ? "V2" : "legacy" );

        } else {
            // Don't record any data during version detection
            return;
        }
    }

    if (_type == PacketType::V2_DATA) {
        memcpy(&_valid_V2, &_parsed_V2, sizeof(_valid_V2));
    }

    // Update internal fuel cell state
    _pwr_out = _parsed.pwr_out;
    _spm_pwr = _parsed.spm_pwr;
    _battery_pwr = _parsed.battery_pwr;

    _state = (State)_parsed.state;
    _v2_state = (V2_State)_parsed.state;
    _err_code = _parsed.err_code;
    _sub_err_code = _parsed.sub_err_code;

    _fuel_remaining = _fuel_rem;

    // Update battery voltage
    _voltage = _parsed.battery_volt;

    /* Calculate battery current. Convention: +current battery is discharging, -current
    battery is charging.  This is aligned with normal AP behaviour.  This is the opposite
    of IE's convention hence *-1 */
    if (_parsed.battery_volt > 0) {
        _current = -1 * _battery_pwr / _parsed.battery_volt;
    } else {
        _current = 0;
    }

    // Calculate consumed current
    _consumed_mah += _current * (now - _last_time_ms) * AMS_TO_MAH;

    _last_time_ms = now;
}

// Process characters received and extract terms for IE 2.4kW
void AP_Generator_IE_2400::decode_latest_term()
{
    // Null terminate and move onto the next term
    _term[_term_offset] = 0;
    _term_offset = 0;
    _term_number++;
    _type = PacketType::NONE;

    if (_start_char == '<') {
        decode_data_packet();

    } else if (_start_char == '[') {
        decode_info_packet();

    } else {
        _sentence_valid = false;

    }
}

void AP_Generator_IE_2400::decode_data_packet()
{
    // Try and decode both protocol versions until locked on
    if ((_version == ProtocolVersion::LEGACY) || (_version == ProtocolVersion::DETECTING)) {
        decode_legacy_data();
    }
    if ((_version == ProtocolVersion::V2) || (_version == ProtocolVersion::DETECTING)) {
        decode_v2_data();
    }
}

void AP_Generator_IE_2400::decode_legacy_data()
{
    switch (_term_number) {
        case 1: {
            // Float
            _parsed.tank_bar = strtof(_term, NULL);

            // Scale tank pressure linearly to a value between 0 and 1
            // Min = 5 bar, max = 300 bar, PRESS_GRAD = 1/295.
            const float PRESS_GRAD = 0.003389830508f;
            _fuel_rem = constrain_float((_parsed.tank_bar-5)*PRESS_GRAD,0,1);
            break;
        }
        case 2:
            // Float
            _parsed.battery_volt = strtof(_term, NULL);
            break;

        case 3:
            // Signed int base 10
            _parsed.pwr_out = strtol(_term, nullptr, 10);
            break;

        case 4:
            // Unsigned int base 10
            _parsed.spm_pwr = strtoul(_term, nullptr, 10);
            break;

        case 5:
            // Signed int base 10
            _parsed.battery_pwr = strtol(_term, nullptr, 10);
            break;

        case 6:
            // Unsigned int base 10
            _parsed.state = strtoul(_term, nullptr, 10);
            break;

        case 7:
            // Unsigned int base 10
            _parsed.err_code = strtoul(_term, nullptr, 10);
            // Sentence only declared valid when we have the expected number of terms
            _sentence_valid = true;
            _type = PacketType::LEGACY_DATA;
            break;

        default:
            // We have received more terms than, something has gone wrong with telemetry data, mark invalid sentence
            _sentence_valid = false;
            break;
    }
}

void AP_Generator_IE_2400::decode_v2_data()
{
    switch (_term_number) {
        case 1:
            _fuel_rem = strtof(_term, NULL) * 0.01;
            break;

        case 2:
            _parsed_V2.inlet_press = strtof(_term, NULL);
            break;

        case 3:
            _parsed.battery_volt = strtof(_term, NULL);
            break;

        case 4:
            _parsed.pwr_out = strtol(_term, nullptr, 10);
            break;

        case 5:
            _parsed.spm_pwr = strtoul(_term, nullptr, 10);
            break;

        case 6:
            _parsed_V2.unit_fault = strtoul(_term, nullptr, 10);
            break;

        case 7:
            _parsed.battery_pwr = strtol(_term, nullptr, 10);
            break;

        case 8:
            _parsed.state = strtoul(_term, nullptr, 10);
            break;

        case 9:
            _parsed.err_code = strtoul(_term, nullptr, 10);
            break;

        case 10:
            _parsed.sub_err_code = strtoul(_term, nullptr, 10);
            break;

        case 11:
            strncpy(_parsed_V2.info_str, _term, ARRAY_SIZE(_parsed_V2.info_str));
            break;

        case 12: {
            // The inverted checksum is sent, un-invert it
            uint8_t checksum = ~strtoul(_term, nullptr, 10);

            // Sent checksum only included characters up to the checksum term
            // Add on the checksum terms to match our running total
            for (uint8_t i = 0; i < ARRAY_SIZE(_term); i++) {
                if (_term[i] == 0) {
                    break;
                }
                checksum += _term[i];
            }

            _sentence_valid = checksum == _checksum;
            _type = PacketType::V2_DATA;
            break;
        }

        default:
            // We have received more terms than, something has gone wrong with telemetry data, mark invalid sentence
            _sentence_valid = false;
            break;
    }
}


void AP_Generator_IE_2400::decode_info_packet()
{

    switch (_term_number) {
        case 1:
            // PCM software number
            strncpy(_info.PCM_number, _term, ARRAY_SIZE(_info.PCM_number));
            break;

        case 2:
            // Software version
            strncpy(_info.Software_version, _term, ARRAY_SIZE(_info.Software_version));
            break;

        case 3:
            // protocol version
            strncpy(_info.Protocol_version, _term, ARRAY_SIZE(_info.Protocol_version));
            break;

        case 4:
            // Hardware serial number
            strncpy(_info.Serial_number, _term, ARRAY_SIZE(_info.Serial_number));
            break;

        case 5: {
            // The inverted checksum is sent, un-invert it
            uint8_t checksum = ~strtoul(_term, nullptr, 10);

            // Sent checksum only included characters up to the checksum term
            // Add on the checksum terms to match our running total
            for (uint8_t i = 0; i < ARRAY_SIZE(_term); i++) {
                if (_term[i] == 0) {
                    break;
                }
                checksum += _term[i];
            }

            _sentence_valid = checksum == _checksum;
            _type = PacketType::V2_INFO;
            break;
        }

        default:
            // We have received more terms than, something has gone wrong with telemetry data, mark invalid sentence
            _sentence_valid = false;
            break;
    }

}

// Check for failsafes
AP_BattMonitor::Failsafe AP_Generator_IE_2400::update_failsafes() const
{
    // Check for error codes that lead to critical action battery monitor failsafe
    if (is_critical_error(_err_code)) {
        return AP_BattMonitor::Failsafe::Critical;
    }

    // Check for error codes that lead to low action battery monitor failsafe
    if (is_low_error(_err_code)) {
        return AP_BattMonitor::Failsafe::Low;
    }

    return AP_BattMonitor::Failsafe::None;
}

// Check for error codes that are deemed critical
bool AP_Generator_IE_2400::is_critical_error(const uint32_t err_in) const
{
    // V2 protocol
    if (_version == ProtocolVersion::V2) {
        return err_in >= 30;
    }

    // V1 protocol
    switch ((ErrorCode)err_in) {
        // Error codes that lead to critical action battery monitor failsafe
        case ErrorCode::BATTERY_CRITICAL:
        case ErrorCode::PRESSURE_CRITICAL:
        case ErrorCode::SYSTEM_CRITICAL:
            return true;

        default:
            // Minor internal error is always ignored and caught by the default
            return false;
    }
}

// Check for error codes that are deemed severe and would be cause to trigger a battery monitor low failsafe action
bool AP_Generator_IE_2400::is_low_error(const uint32_t err_in) const
{
    // V2 protocol
    if (_version == ProtocolVersion::V2) {
        return (err_in > 20) && (err_in < 30);
    }

    // V1 protocol
    switch ((ErrorCode)err_in) {
        // Error codes that lead to critical action battery monitor failsafe
        case ErrorCode::START_DENIED:
        case ErrorCode::PRESSURE_ALERT:
        case ErrorCode::BATTERY_LOW:
        case ErrorCode::PRESSURE_LOW:
        case ErrorCode::SPM_LOST:
            return true;

        default:
            // Minor internal error is always ignored and caught by the default
            return false;
    }
}

// Check error codes and populate message with error code
bool AP_Generator_IE_2400::check_for_err_code(char* msg_txt, uint8_t msg_len) const
{
    // Check if we have received an error code
    if (!is_critical_error(_err_code) && !is_low_error(_err_code)) {
        return false;
    }

    if (_version == ProtocolVersion::V2) {
        hal.util->snprintf(msg_txt, msg_len, "Fuel cell err %u.%u: %s", (unsigned)_err_code, (unsigned)_sub_err_code, _valid_V2.info_str);
        return true;
    }

    hal.util->snprintf(msg_txt, msg_len, "Fuel cell err code <%u>", (unsigned)_err_code);
    return true;
}

bool AP_Generator_IE_2400::check_for_warning_code(char* msg_txt, uint8_t msg_len) const
{
    if (_err_code == 0) {
        // No error nothing to do.
        return false;
    }
    if (is_critical_error(_err_code) || is_low_error(_err_code)) {
        // Critical or low error are already reported
        return false;
    }

    switch (_version) {
        case ProtocolVersion::DETECTING:
        case ProtocolVersion::UNKNOWN:
            break;

        case ProtocolVersion::LEGACY:
            if ((ErrorCode)_err_code == ErrorCode::REDUCED_POWER) {
                hal.util->snprintf(msg_txt, msg_len, "Fuel cell reduced power <%u>", (unsigned)_err_code);
                return true;
            }
            break;

        case ProtocolVersion::V2:
            hal.util->snprintf(msg_txt, msg_len, "Fuel cell warning %u.%u: %s", (unsigned)_err_code, (unsigned)_sub_err_code, _valid_V2.info_str);
            return true;
    }

    hal.util->snprintf(msg_txt, msg_len, "Fuel cell warning code <%u>", (unsigned)_err_code);
    return true;
}

// Check if we should notify on any change of fuel cell state
void AP_Generator_IE_2400::check_status(const uint32_t now)
{
    if (ErrorCode(_err_code) == ErrorCode::REDUCED_POWER) {
        if (now - _last_low_power_warning_ms > 5000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Fuel cell: reduced power; conditioning needed?");
            _last_low_power_warning_ms = now;
        }
    }
    return AP_Generator_IE_FuelCell::check_status(now);
}

#if HAL_LOGGING_ENABLED
// log generator status to the onboard log
void AP_Generator_IE_2400::log_write()
{
#define MASK_LOG_ANY    0xFFFF
    if (!AP::logger().should_log(MASK_LOG_ANY)) {
        return;
    }

    switch (_version) {
        case ProtocolVersion::DETECTING:
        case ProtocolVersion::UNKNOWN:
            return;

        case ProtocolVersion::LEGACY:
            // @LoggerMessage: IE24
            // @Description: Intelligent Energy Fuel Cell generator (legacy protocol)
            // @URL: https://ardupilot.org/copter/docs/common-ie24-fuelcell.html
            // @Field: TimeUS: Time since system startup
            // @Field: FUEL: Fuel remaining
            // @Field: SPMPWR: stack power module power draw
            // @Field: POUT: output power
            // @Field: ERR: error codes
            // @FieldValueEnum: ERR: AP_Generator_IE_2400::ErrorCode
            AP::logger().WriteStreaming(
                "IE24",
                "TimeUS,FUEL,SPMPWR,POUT,ERR",
                "s%WW-",
                "F2---",
                "Qfiii",
                AP_HAL::micros64(),
                _fuel_remaining,
                _spm_pwr,
                _pwr_out,
                _err_code
                );
            break;

        case ProtocolVersion::V2:
            // @LoggerMessage: IEFC
            // @Description: Intelligent Energy Fuel Cell generator
            // @URL: https://ardupilot.org/copter/docs/common-ie24-fuelcell.html
            // @Field: TimeUS: Time since system startup
            // @Field: Tank: Fuel remaining
            // @Field: Inlet: Inlet pressure
            // @Field: BattV: battery voltage
            // @Field: OutPwr: output power
            // @Field: SPMPwr: stack power module power draw
            // @Field: FNo: fault number
            // @Field: BPwr: battery power draw
            // @Field: State: generator state
            // @FieldValueEnum: State: AP_Generator_IE_2400::V2_State
            // @Field: F1: error code
            // @FieldValueEnum: F1: AP_Generator_IE_2400::ErrorCode
            // @Field: F2: sub-error code
            AP::logger().WriteStreaming(
                "IEFC",
                "TimeUS,Tank,Inlet,BattV,OutPwr,SPMPwr,FNo,BPwr,State,F1,F2",
                "s%-vWW-W---",
                "F----------",
                "QfffhHBhBII",
                AP_HAL::micros64(),
                _fuel_remaining,
                _valid_V2.inlet_press,
                _voltage,
                _pwr_out,
                _spm_pwr,
                _valid_V2.unit_fault,
                _battery_pwr,
                uint8_t(_v2_state),
                _err_code,
                _sub_err_code
                );
            break;
    }

}
#endif  // HAL_LOGGING_ENABLED

// Return true is fuel cell is in running state suitable for arming
bool AP_Generator_IE_2400::is_running() const
{
    switch (_version) {
        case ProtocolVersion::DETECTING:
        case ProtocolVersion::UNKNOWN:
            return false;

        case ProtocolVersion::LEGACY:
            // Can use the base class method
            return AP_Generator_IE_FuelCell::is_running();

        case ProtocolVersion::V2:
            return _v2_state == V2_State::Running;
    }

    return false;
}

// Lookup table for running state.  State code is the same for all IE units.
const AP_Generator_IE_2400::Lookup_State_V2 AP_Generator_IE_2400::lookup_state_V2[] = {
    { V2_State::FCPM_Off, "FCPM Off"},
    { V2_State::Starting, "Starting"},
    { V2_State::Running, "Running"},
    { V2_State::Stopping, "Stopping"},
    { V2_State::Go_to_Sleep, "Sleep"},
};

// Print msg to user updating on state change
void AP_Generator_IE_2400::update_state_msg()
{
    switch (_version) {
        case ProtocolVersion::DETECTING:
        case ProtocolVersion::UNKNOWN:
            break;

        case ProtocolVersion::LEGACY:
            // Can use the base class method
            AP_Generator_IE_FuelCell::update_state_msg();
            break;

        case ProtocolVersion::V2: {
            // If fuel cell state has changed send gcs message
            if (_v2_state != _last_v2_state) {
                for (const struct Lookup_State_V2 entry : lookup_state_V2) {
                    if (_v2_state == entry.option) {
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Generator: %s", entry.msg_txt);
                        break;
                    }
                }
                _last_v2_state = _v2_state;
            }
            break;
        }
    }
}

#if HAL_GCS_ENABLED
// Get the MAV_SEVERITY level of a given error code
MAV_SEVERITY AP_Generator_IE_2400::get_mav_severity(uint32_t err_code) const
{
    if (err_code <= 9) {
        return MAV_SEVERITY_INFO;
    }
    if (err_code <= 20) {
        return MAV_SEVERITY_WARNING;
    }
    return MAV_SEVERITY_CRITICAL;
}
#endif // HAL_GCS_ENABLED

#endif  // AP_GENERATOR_IE_2400_ENABLED
