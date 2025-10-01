
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
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include "AP_GPS_UBLOX_CFGv2.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_GPS/AP_GPS_UBLOX.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

#if 0
#define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
#define Debug(fmt, args ...)  do {} while(0)
#endif

const char* const AP_GPS_UBLOX_CFGv2::CONSTELLATION_NAMES[] = {
    "GPS",   // 0
    "SBAS",  // 1
    "GAL",   // 2
    "BDS",   // 3
    "",      // 4 (IMES, unused)
    "QZSS",  // 5
    "GLO",   // 6
    "NAVIC", // 7
};

AP_GPS_UBLOX_CFGv2::AP_GPS_UBLOX_CFGv2(AP_GPS_UBLOX &_ubx_backend)
    : ubx_backend(_ubx_backend)
{
    static_assert(ARRAY_SIZE(AP_GPS_UBLOX_CFGv2::CONSTELLATION_NAMES) == AP_GPS_UBLOX::GNSS_LAST, "missing constellation name or ubx_gnss_identifier");
}

void AP_GPS_UBLOX_CFGv2::update()
{
    // if (ubx_backend._class == AP_GPS_UBLOX::CLASS_ACK) {
    //     if (ubx_backend._msg_id == AP_GPS_UBLOX::MSG_ACK_ACK) {
    //         Debug("GPS %d: ACK for class 0x%02x id 0x%02x",
    //               ubx_backend.state.instance + 1,
    //               ubx_backend._buffer.ack.clsID,
    //               ubx_backend._buffer.ack.msgID);
    //     } else if (ubx_backend._msg_id == AP_GPS_UBLOX::MSG_ACK_NACK) {
    //         Debug("GPS %d: NACK for class 0x%02x id 0x%02x",
    //                 ubx_backend.state.instance + 1,
    //                 ubx_backend._buffer.nack.clsID,
    //                 ubx_backend._buffer.nack.msgID);
    //     }
    // }

    switch (curr_state) {
        case States::IDENTIFY_MODULE: {
            if (AP_HAL::millis() - _last_request_time > 200) {
                _last_request_time = AP_HAL::millis();
                ubx_backend._request_version();
            }
            if (_identify_module()) {
                _last_request_time = 0; // immediate next state
                curr_state = States::IDENTIFY_SIGNALS;
            } else {
                // stay in this state
                break;
            }
        } // fallthrough to next state
        case States::IDENTIFY_SIGNALS:
            if (AP_HAL::millis() - _last_request_time > 550) {
                _last_request_time = AP_HAL::millis();
                _request_cfg_group(ConfigKey::CFG_SIGNAL_GPS_ENA, ConfigLayer::RAM);
            }
            break;
        case States::CONFIGURE_SIGNALS:
            if (_send_signal_cfg()) {
                curr_state = States::IDENTIFY_SIGNALS;
                // this makes sure that we request the current signal config again
                // after some time, to allow GNSS system to reset
                _last_request_time = AP_HAL::millis();
                break;
            } else {
                // continue to next state
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                 "GPS %d: u-blox signals configured",
                                 ubx_backend.state.instance + 1);
                curr_state = States::IDENTIFY_UART_PORT;
                _last_request_time = 0; // immediate next state
            }
        case States::IDENTIFY_UART_PORT: {
            // transition occurs when ubx_backend sets _ublox_port from MON-COMMS
            if (ubx_backend._ublox_port < UBLOX_MAX_PORTS) {
                curr_state = States::FETCH_COMMON_CONFIG;
                _last_request_time = 0; // immediate next state
            } else if (AP_HAL::millis() - _last_request_time > 250) {
                _last_request_time = AP_HAL::millis();
                // Poll only MON-COMMS to detect output port interface
                ubx_backend._send_message(AP_GPS_UBLOX::CLASS_MON, AP_GPS_UBLOX::MSG_MON_COMMS, nullptr, 0);
                break;
            }
        }
        case States::FETCH_COMMON_CONFIG:
            if (AP_HAL::millis() - _last_request_time > 200) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                 "GPS %d: u-blox fetching config",
                                 ubx_backend.state.instance + 1);
                _last_request_time = AP_HAL::millis();
                _request_common_cfg();
            }
            break;
        case States::SET_COMMON_CONFIG:
            if (AP_HAL::millis() - _last_request_time > 200) {
                _last_request_time = AP_HAL::millis();
                if (set_common_cfg()) {
                    // after sending, re-fetch to verify
                    curr_state = States::FETCH_COMMON_CONFIG;
                }
            }
            break;
        default:
            break;
    }
}

bool AP_GPS_UBLOX_CFGv2::_identify_module()
{
    if (strlen(ubx_backend._module) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "GPS %d: u-blox module: %s",
                    ubx_backend.state.instance + 1,
                    ubx_backend._module);

        // check if module contains the model string
        if (strstr(ubx_backend._module, "M9") != nullptr) {
            module = Module::M9;
            // _hardware_generation defined in AP_GPS_UBLOX.cpp
        } else if (strstr(ubx_backend._module, "NEO-F9") != nullptr) {
            module = Module::NEO_F9;
            // _hardware_generation defined in AP_GPS_UBLOX.cpp
        } else if (strstr(ubx_backend._module, "ZED-F9") != nullptr) {
            module = Module::ZED_F9;
            // _hardware_generation defined in AP_GPS_UBLOX.cpp
        } else if (strstr(ubx_backend._module, "M10") != nullptr) {
            module = Module::M10;
            // _hardware_generation defined in AP_GPS_UBLOX.cpp
        } else if (strstr(ubx_backend._module, "F10") != nullptr) {
            module = Module::F10;
            ubx_backend._hardware_generation = AP_GPS_UBLOX::UBLOX_F10;
        } else if (strstr(ubx_backend._module, "F20") != nullptr) {
            module = Module::F20;
            ubx_backend._hardware_generation = AP_GPS_UBLOX::UBLOX_F20;
        } else if (strstr(ubx_backend._module, "X20") != nullptr) {
            module = Module::X20;
            ubx_backend._hardware_generation = AP_GPS_UBLOX::UBLOX_X20;
        } else {
            module = Module::UNKNOWN;
        }
    }

    // check protocol version
    if (strlen(ubx_backend._protver) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "GPS %d: u-blox protocol: %s",
                    ubx_backend.state.instance + 1,
                    ubx_backend._protver);
        // convert the version string to a float
        float protver = atof(ubx_backend._protver);
        if (protver >= 23.01f) {
            // protocol version 23.01 and above supports CFG-VALGET/VALSET
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                        "GPS %d: u-blox protocol supports CFG-VALGET/VALSET",
                        ubx_backend.state.instance + 1);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                        "GPS %d: u-blox protocol too old for CFG-VALGET/VALSET",
                        ubx_backend.state.instance + 1);
            // we cannot proceed with configuration
            curr_state = States::FAILED;
            return false;
        }
        if (protver >= 40.00f) {
            // protocol version 40.00 and above only support CFG-VALGET/VALSET
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                        "GPS %d: Legacy config unsupported",
                        ubx_backend.state.instance + 1);
            ubx_backend._legacy_cfg_unsupported = true;
        }
        // move to next state
        return true;
    }

    return false;
}

// get value by key
bool AP_GPS_UBLOX_CFGv2::UBXPackedCfg::get(ConfigKey Key, uint64_t &value) const {
    uint32_t key;
    // find key in buffer
    for (uint16_t i = 0; i < _size; i += sizeof(uint32_t) + config_key_size((ConfigKey)key)) {
        memcpy(&key, _buf + i, sizeof(key));
        if (key == Key) {
            memcpy(&value, _buf + i + sizeof(key), config_key_size(Key));
            return true;
        }
    }
    // key not found
    return false;
}

// set value by key
bool AP_GPS_UBLOX_CFGv2::UBXPackedCfg::set(ConfigKey Key, uint64_t value) {
    uint32_t key;
    // find key in buffer
    for (uint16_t i = 0; i < _size; i += sizeof(uint32_t) + config_key_size((ConfigKey)key)) {
        memcpy(&key, _buf + i, sizeof(key));
        if (key == Key) {
            // write value bytes in little-endian at the correct value offset
            size_t index = i + sizeof(uint32_t);
            switch (config_key_size(Key)) {
                case 1:
                    append_value_le_bytes<uint8_t>(_buf, index, (uint8_t)value);
                    break;
                case 2:
                    append_value_le_bytes<uint16_t>(_buf, index, (uint16_t)value);
                    break;
                case 4:
                    append_value_le_bytes<uint32_t>(_buf, index, (uint32_t)value);
                    break;
                case 8:
                    append_value_le_bytes<uint64_t>(_buf, index, (uint64_t)value);
                    break;
                default:
                    return false;
            }
            return true;
        }
    }
    // key not found
    return false;
}

// 
// Streaming VALGET byte-wise processing
// 
void AP_GPS_UBLOX_CFGv2::process_valget_byte(uint8_t byte)
{
    // We stream parse the VALGET payload only. The driver calls us for each
    // payload byte while the outer parser maintains UBX checksums.
    // Payload layout (after UBX length field):
    //   ubx_cfg_valget { uint8 version; uint8 layers; uint8 res[2]; }
    //   followed by repeated: [key:uint32 little-endian][value:N bytes]
    // where N is determined by key bits 30..28.

    if (!_valget_in_progress) {
        // starting new VALGET payload
        _valget_in_progress = true;
        _valget_hdr_count = 0;
        _valget_key_count = 0;
        _valget_value_expected = 0;
        _valget_value_count = 0;
        _valget_key = 0;
        _valget_value = 0;
    }

    if (_valget_abort) {
        // aborting this VALGET; ignore all bytes until process_valget_complete()
        return;
    }

    // First consume the 4-byte VALGET header
    if (_valget_hdr_count < 4) {
        // We don't need to store header, just count it
        _valget_hdr_count++;
        return;
    }

    // Read 4-byte key (little-endian)
    if (_valget_key_count < 4) {
        _valget_key |= (uint32_t)byte << (8 * _valget_key_count);
        _valget_key_count++;
        if (_valget_key_count == 4) {
            // determine expected value length from key size bits
            const uint8_t size_code = (uint8_t)((_valget_key >> 28) & 0x07U);
            switch (size_code) {
            case UBX_CFG_KEY_SIZE_CODE_BIT: // 1 bit, stored in 1 byte
            case UBX_CFG_KEY_SIZE_CODE_1B:
                _valget_value_expected = 1; break;
            case UBX_CFG_KEY_SIZE_CODE_2B:
                _valget_value_expected = 2; break;
            case UBX_CFG_KEY_SIZE_CODE_4B:
                _valget_value_expected = 4; break;
            case UBX_CFG_KEY_SIZE_CODE_8B:
                _valget_value_expected = 8; break;
            default:
                // invalid size; abort this VALGET
                Debug("GPS %d: valget invalid key size code %u for key 0x%lx",
                        ubx_backend.state.instance + 1,
                        size_code,
                        _valget_key);
                _valget_abort = true;
                return;
            }
            _valget_value = 0;
            _valget_value_count = 0;
        }
        return;
    }

    // Read value bytes (little-endian)
    if (_valget_value_count < _valget_value_expected) {
        _valget_value |= (uint64_t)byte << (8 * _valget_value_count);
        _valget_value_count++;
        if (_valget_value_count == _valget_value_expected) {
            // Deliver this key/value
            _handle_valget_kv((ConfigKey)_valget_key, _valget_value, _valget_value_expected);
            // reset for next pair
            _valget_key = 0;
            _valget_value = 0;
            _valget_key_count = 0;
            _valget_value_expected = 0;
            _valget_value_count = 0;
        }
        return;
    }
}

bool AP_GPS_UBLOX_CFGv2::is_common_cfg_needed()
{
    #define CFG_NEED(KEY_CLASS, KEY, TYPE, VAL) \
        if (_processing_cfg.KEY##_val != (TYPE)VAL) { need = true; }

    bool need = false;
    if (module <= Module::F10) {
        // Check constant config values for single-UART modules
        need = UBX_CFG_COMMON_UART(CFG_NEED);
    } else {
        // Check constant config values
        switch (ubx_backend._ublox_port) {
        case 2: // UART1
            need = UBX_CFG_COMMON_UART1(CFG_NEED);
            break;
        case 3: // UART2
            need = UBX_CFG_COMMON_UART2(CFG_NEED);
            break;
        default:
            // should not happen
            break;
        }
    }

    // check variable settings
    need = need || (_cfg.meas_rate != ubx_backend.params.rate_ms) ||
                   (_cfg.dynmodel != ubx_backend.gps._navfilter);

                   if (ubx_backend.gps._min_elevation != 100) {
        // do not set min_elevation if user has not set it
        need = need || (_processing_cfg.min_elevation != _cfg.min_elevation);
    }
    if (_cfg.gps_l5_health_ovrd_exists) {
        need = need || (_processing_cfg.gps_l5_health_ovrd != _cfg.gps_l5_health_ovrd);
    }

    return need;
}

void AP_GPS_UBLOX_CFGv2::process_valget_complete(bool success)
{
    // Reset state regardless of success; higher-level logic handles retry.
    _valget_in_progress = false;
    _valget_abort = false;

    if (success) {
        // processing complete
        // move processing config to active config
        _cfg = _processing_cfg;

        switch (curr_state) {
            case States::IDENTIFY_SIGNALS:
                // show a list of supported constellations
                _publish_supported_constellations();
                curr_state = States::CONFIGURE_SIGNALS;
                break;
            case States::FETCH_COMMON_CONFIG: {
                // Decide whether to set config based on fetched rates/model
                curr_state = is_common_cfg_needed() ? States::SET_COMMON_CONFIG : States::CONFIGURED;
                if (curr_state == States::CONFIGURED) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                 "GPS %d: u-blox configured",
                                 ubx_backend.state.instance + 1);
                }
                _last_request_time = 0; // immediate next state
                break;
            }
            default:
                break;
        }
    }
}

void AP_GPS_UBLOX_CFGv2::_handle_valget_kv(ConfigKey key, uint64_t value, uint8_t value_len)
{
    Debug("GPS %d: valget key 0x%lx value 0x%llx len %u",
            ubx_backend.state.instance + 1,
            (uint32_t)key,
            (uint64_t)value,
            (size_t)value_len);
    // First, check for signal-related keys and update masks
    _parse_signal_kv(key, value, value_len);

    // Capture common configuration when fetching
    _parse_common_cfg(key, value);
}

// Request a configuration group via CFG-VALGET
bool AP_GPS_UBLOX_CFGv2::_request_cfg_group(ConfigKey group, ConfigLayer layer)
{
    struct {
        struct AP_GPS_UBLOX::ubx_cfg_valget msg;
        uint32_t key;
    } msg {};
    if (ubx_backend.port->txspace() < (uint16_t)(sizeof(AP_GPS_UBLOX::ubx_header)+sizeof(msg)+2)) {
        return false;
    }
    msg.msg.version = 0;
    msg.msg.layers = layer; // ram
    msg.key = group | 0xFFFFLU;
    return ubx_backend._send_message(AP_GPS_UBLOX::CLASS_CFG, AP_GPS_UBLOX::MSG_CFG_VALGET, &msg, sizeof(msg));
}

// send a CFG-VALSET using provided [key:uint32][value:N] pairs without extra allocation
bool AP_GPS_UBLOX_CFGv2::_send_valset_bytes(const uint8_t *bytes, uint16_t size, ConfigLayer layers)
{
    if (bytes == nullptr || size == 0) {
        return false;
    }
    // Check tx space for header + fixed valset header (without key) + bytes + checksum
    const uint16_t payload_len = (uint16_t)(sizeof(AP_GPS_UBLOX::ubx_cfg_valset) - sizeof(uint32_t)) + size;
    const uint16_t total_len = (uint16_t)(sizeof(AP_GPS_UBLOX::ubx_header) + payload_len + 2U);
    if (ubx_backend.port->txspace() < total_len) {
        return false;
    }

    // Prepare UBX header
    AP_GPS_UBLOX::ubx_header header{};
    header.preamble1 = AP_GPS_UBLOX::PREAMBLE1;
    header.preamble2 = AP_GPS_UBLOX::PREAMBLE2;
    header.msg_class = AP_GPS_UBLOX::CLASS_CFG;
    header.msg_id    = AP_GPS_UBLOX::MSG_CFG_VALSET;
    header.length    = payload_len;

    // Prepare VALSET fixed part
    AP_GPS_UBLOX::ubx_cfg_valset valset{};
    valset.version = 1;
    valset.layers = layers;
    valset.transaction = 0;
    valset.reserved[0] = 0;

    // Compute checksum over class..length + payload
    uint8_t ck_a = 0, ck_b = 0;
    AP_GPS_UBLOX::_update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2, ck_a, ck_b);
    AP_GPS_UBLOX::_update_checksum((uint8_t *)&valset, sizeof(valset)-sizeof(uint32_t), ck_a, ck_b);
    AP_GPS_UBLOX::_update_checksum((uint8_t *)bytes, size, ck_a, ck_b);

    // Write in segments
    ubx_backend.port->write((const uint8_t *)&header, sizeof(header));
    ubx_backend.port->write((const uint8_t *)&valset, sizeof(valset)-sizeof(uint32_t));
    ubx_backend.port->write(bytes, size);
    ubx_backend.port->write(&ck_a, 1);
    ubx_backend.port->write(&ck_b, 1);
    return true;
}

// Map signal CFG keys to SupportedSignals indices and update masks
void AP_GPS_UBLOX_CFGv2::_parse_signal_kv(ConfigKey key, uint64_t value, uint8_t value_len)
{
    if (value_len < 1) {
        return;
    }
    const bool ena = (value & 0x1U) != 0;

    auto set_gnss = [&](uint8_t idx, bool enabled){
        _processing_cfg.supported_gnss |= (1U << idx);
        if (enabled) {
            _processing_cfg.enabled_gnss |= (1U << idx);
        } else {
            _processing_cfg.enabled_gnss &= ~(1U << idx);
        }
    };

    auto set_signal = [&](uint32_t idx, bool enabled){
        _processing_cfg.supported_signals |= (1U << idx);
        if (enabled) {
            _processing_cfg.enabled_signals |= (1U << idx);
        } else {
            _processing_cfg.enabled_signals &= ~(1U << idx);
        }
    };

    switch (key) {
    // Process GNSS enables
    #define GNSS_X(NAME, INDEX) case ConfigKey::CFG_SIGNAL_##NAME##_ENA: set_gnss(AP_GPS_UBLOX::GNSS_##NAME, ena); Debug("GPS %d: GNSS %s %d", ubx_backend.state.instance + 1, #NAME, ena); break;
    UBLOX_GNSS(GNSS_X)
    #undef GNSS_X
    default:
        break;
    }

    switch (key) {
    // Process Signal enables
    #define SIG_X(NAME, INDEX) case ConfigKey::CFG_SIGNAL_##NAME##_ENA: set_signal(INDEX, ena); Debug("GPS %d: Signal %s %d", ubx_backend.state.instance + 1, #NAME, ena); break;
    UBLOX_SIGNALS(SIG_X)
    #undef SIG_X
    default:
        break;
    }
}

// send a list of supported constellation name with enabled/disabled status, use snprintf formatting
void AP_GPS_UBLOX_CFGv2::_publish_supported_constellations()
{
    char buf[100] = {};
    snprintf(buf, sizeof(buf), "GPS %d: GNSS Mode: ", ubx_backend.state.instance + 1);
    // UBLOX_GNSS indexes are 0..7 (with 4 unused for IMES), and we track GNSS in 8-bit masks
    for (uint8_t i = 0; i < 8; i++) {
        if ((_cfg.supported_gnss & (1U << i)) && strlen(buf) < sizeof(buf) - 10) {
            const char *name = CONSTELLATION_NAMES[i];
            if (name[0] == '\0') { continue; }
            const bool enabled = (_cfg.enabled_gnss & (1U << i)) != 0;
            snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "%s[%d] ", name, enabled ? 1 : 0);
        }
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", buf);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                 "GPS %d: Signal Mode: 0x%08lx/0x%08lx",
                 ubx_backend.state.instance + 1,
                 (unsigned long)_cfg.enabled_signals,
                 (unsigned long)_cfg.supported_signals);

    // update available masks, so user can see what is possible
    ubx_backend.params.sig_avail.set(_cfg.supported_signals);
    ubx_backend.params.gnss_avail.set(_cfg.supported_gnss);
}

bool AP_GPS_UBLOX_CFGv2::is_signal_cfg_needed()
{
    return _cfg.enabled_gnss != ubx_backend.params.gnss_mode ||
           _cfg.enabled_signals != ubx_backend.params.sig_mode;
}

// send signal configuration if needed, return true if more to do
// returns false if configuration is unchanged
bool AP_GPS_UBLOX_CFGv2::_send_signal_cfg()
{
    if ((_cfg.enabled_gnss == ubx_backend.params.gnss_mode && _cfg.enabled_signals == ubx_backend.params.sig_mode)
         || ubx_backend.params.gnss_mode == 0) {
        return false;
    }

    // Clear unavailable GNSS and signals from parameters
    uint8_t desired_gnss = (uint8_t)ubx_backend.params.gnss_mode;
    uint32_t desired_signals = (uint32_t)ubx_backend.params.sig_mode;

    if (desired_signals == 0) {
        // enable all supported signals
        desired_signals = _cfg.supported_signals;
    }

    // Clear unavailable GNSS bits
    desired_gnss &= _cfg.supported_gnss;
    
    // Clear unavailable signal bits
    desired_signals &= _cfg.supported_signals;
    
    // Validate signal mask against enabled GNSS using UBLOX_SIG_MASK
    uint32_t allowed_signals = 0;
    #define ACCUM_ALLOWED(NAME, MASK) \
        if (desired_gnss & (1U << AP_GPS_UBLOX::GNSS_##NAME)) { allowed_signals |= (uint32_t)(MASK); }
    UBLOX_SIG_MASK(ACCUM_ALLOWED)
    #undef ACCUM_ALLOWED
    desired_signals &= allowed_signals;

    // Update parameters if` they were modified
    if (desired_gnss != ubx_backend.params.gnss_mode) {
        ubx_backend.params.gnss_mode.set_and_save(desired_gnss);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "GPS %d: GNSS mode cleared unavailable constellations: 0x%02x -> 0x%02x",
                    ubx_backend.state.instance + 1,
                    (unsigned)ubx_backend.params.gnss_mode,
                    (unsigned)desired_gnss);
    }

    if (desired_signals != ubx_backend.params.sig_mode) {
        ubx_backend.params.sig_mode.set_and_save(desired_signals);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "GPS %d: Signal mode cleared unavailable signals: 0x%08lx -> 0x%08lx",
                    ubx_backend.state.instance + 1,
                    (unsigned long)ubx_backend.params.sig_mode,
                    (unsigned long)desired_signals);
    }


    // Log configuration changes
    if (desired_gnss != _cfg.enabled_gnss) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "GPS %d: updating GNSS Mode 0x%02x -> 0x%02x",
                    ubx_backend.state.instance + 1,
                    (unsigned)_cfg.enabled_gnss,
                    (unsigned)desired_gnss);
    }
    if (desired_signals != _cfg.enabled_signals) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "GPS %d: updating Signal Mode 0x%08lx -> 0x%08lx",
                    ubx_backend.state.instance + 1,
                    (unsigned long)_cfg.enabled_signals,
                    (unsigned long)desired_signals);
    }

    // Prepare a packed config with the changes
    constexpr uint32_t max_cfg_size = (AP_GPS_UBLOX::GNSS_LAST + SIGNALS_LAST) * (sizeof(uint32_t) + 1);
    uint8_t packed_buf[max_cfg_size] = {};
    UBXPackedCfg packed_cfg(packed_buf, sizeof(packed_buf));

    // Push GNSS enables/disables using current and desired values
    #define PUSH_GNSS(NAME, INDEX) \
        { \
            uint8_t curr_val = (_cfg.enabled_gnss & (1U << AP_GPS_UBLOX::GNSS_##NAME)) ? 1U : 0U; \
            uint8_t desired_val = (desired_gnss & (1U << AP_GPS_UBLOX::GNSS_##NAME)) ? 1U : 0U; \
            if (packed_cfg.push<ConfigKey::CFG_SIGNAL_##NAME##_ENA>(curr_val, desired_val)) { \
                Debug("GPS %d: pushed GNSS %s %d -> %d", ubx_backend.state.instance + 1, #NAME, curr_val, desired_val); \
            } \
        }
    UBLOX_GNSS(PUSH_GNSS)
    #undef PUSH_GNSS

    if (desired_signals == 0) {
        // no signals to be configured
        Debug("GPS %d: no signals to be configured", ubx_backend.state.instance + 1);
        goto exit;
    }

    // Push Signal enables/disables using current and desired values
    #define PUSH_SIG(NAME, INDEX) \
        { \
            uint8_t curr_val = (_cfg.enabled_signals & (1UL << (INDEX))) ? 1U : 0U; \
            uint8_t desired_val = (desired_signals & (1UL << (INDEX))) ? 1U : 0U; \
            if (packed_cfg.push<ConfigKey::CFG_SIGNAL_##NAME##_ENA>(curr_val, desired_val)) { \
                Debug("GPS %d: pushed signal %s %d -> %d", ubx_backend.state.instance + 1, #NAME, curr_val, desired_val); \
            } \
        }
    UBLOX_SIGNALS(PUSH_SIG)
    #undef PUSH_SIG
exit:
    const uint8_t *blob = packed_cfg.get();
    const uint16_t blob_len = (uint16_t)packed_cfg.get_size();
    if (blob_len == 0) {
        return true;
    }

    _send_valset_bytes(blob, blob_len, ConfigLayer::ALL);

    return true;
}

#define CFG_KEYS(KEY_CLASS,KEY, KEYTYPE, VAL) \
    ConfigKey::KEY_CLASS##_##KEY,

#define COMMON_CFG_KEYS_DUAL_UART \
    UBX_CFG_COMMON_UART1(CFG_KEYS) \
    ConfigKey::CFG_NAVSPG_DYNMODEL, \
    ConfigKey::CFG_NAVSPG_INFIL_MINELEV, \
    ConfigKey::CFG_RATE_MEAS,

#define COMMON_CFG_KEYS_SINGLE_UART \
    UBX_CFG_COMMON_UART(CFG_KEYS) \
    ConfigKey::CFG_NAVSPG_DYNMODEL, \
    ConfigKey::CFG_NAVSPG_INFIL_MINELEV, \
    ConfigKey::CFG_RATE_MEAS,

bool AP_GPS_UBLOX_CFGv2::_request_common_cfg()
{
    constexpr uint32_t common_cfg_keys_single_uart[] = {
        COMMON_CFG_KEYS_SINGLE_UART
    };

    // Use ARRAY_SIZE macro to calculate array size at compile time
    static constexpr struct PACKED {
        AP_GPS_UBLOX::ubx_cfg_valget msg;
        uint32_t keys[ARRAY_SIZE(common_cfg_keys_single_uart)];
    } msg_single_uart = {
        .msg = {0, 0, {0, 0}},  // version=0, layers=0, reserved={0,0}
        .keys = {
            COMMON_CFG_KEYS_SINGLE_UART
        }  // Copy from the array
    };

    constexpr uint32_t common_cfg_keys_dual_uart[] = {
        COMMON_CFG_KEYS_DUAL_UART
    };
    static constexpr struct PACKED {
        AP_GPS_UBLOX::ubx_cfg_valget msg;
        uint32_t keys[ARRAY_SIZE(common_cfg_keys_dual_uart)];
    } msg_dual_uart = {
        .msg = {0, 0, {0, 0}},  // version=0, layers=0, reserved={0,0}
        .keys = {
            COMMON_CFG_KEYS_DUAL_UART
        }  // Copy from the array
    };

    if (module <= Module::F10) {
        if (ubx_backend.port->txspace() < (uint16_t)(sizeof(AP_GPS_UBLOX::ubx_header) + sizeof(msg_single_uart) + 2)) {
            return false;
        }

        // send VALGET with multiple keys
        return ubx_backend._send_message(AP_GPS_UBLOX::CLASS_CFG, AP_GPS_UBLOX::MSG_CFG_VALGET, &msg_single_uart, sizeof(msg_single_uart));
    } else {
        if (ubx_backend.port->txspace() < (uint16_t)(sizeof(AP_GPS_UBLOX::ubx_header) + sizeof(msg_dual_uart) + 2)) {
            return false;
        }

        // send VALGET with multiple keys
        return ubx_backend._send_message(AP_GPS_UBLOX::CLASS_CFG, AP_GPS_UBLOX::MSG_CFG_VALGET, &msg_dual_uart, sizeof(msg_dual_uart));
    }
}

#define CASE_UPDATE_VALUES(KEY_CLASS,KEY, KEYTYPE, VAL) \
    case ConfigKey::KEY_CLASS##_##KEY: { \
        _processing_cfg.KEY##_val = reinterpret_cast<KEYTYPE&>(value); \
        break; \
    }

void AP_GPS_UBLOX_CFGv2::_parse_common_cfg(ConfigKey key, uint64_t value)
{
    switch (key) {
    // combined common cfg key cases
    UBX_CFG_COMMON_UART1(CASE_UPDATE_VALUES)
    // variable settings
    case ConfigKey::CFG_RATE_MEAS: {
        _processing_cfg.meas_rate = reinterpret_cast<uint16_t&>(value);
        break;
    }
    case ConfigKey::CFG_NAVSPG_DYNMODEL: {
        _processing_cfg.dynmodel = reinterpret_cast<uint8_t&>(value);
        break;
    }
    case ConfigKey::CFG_NAVSPG_INFIL_MINELEV: {
        _processing_cfg.min_elevation = reinterpret_cast<int8_t&>(value);
        break;
    }
    case ConfigKey::CFG_SIGNAL_L5_HEALTH_OVRD: {
        // we receive this as part of signal config request
        _processing_cfg.gps_l5_health_ovrd = reinterpret_cast<uint8_t&>(value);
        _processing_cfg.gps_l5_health_ovrd_exists = true;
        break;
    }
    default:
        break;
    }
}

#define CFG_PUSH(KEY_CLASS, KEY, TYPE, VAL) \
    packed_cfg.push<ConfigKey::KEY_CLASS##_##KEY>(_cfg.KEY##_val, (TYPE)VAL);

// removed _reset_common_capture; we initialize inline with sentinels
bool AP_GPS_UBLOX_CFGv2::set_common_cfg()
{
    // Compose and send VALSET for differences recorded in _processing_cfg
    // we are not too worried about the blob size, if we exceed we just fail to send
    // and retry later, with less number of changes, but it will be nice to avoid
    uint8_t blob[100] = {};
    UBXPackedCfg packed_cfg(blob, sizeof(blob));

    if (module <= Module::F10) {
        UBX_CFG_COMMON_UART(CFG_PUSH)
    } else {
        // set constant config values for dual-UART modules
        switch (ubx_backend._ublox_port) {
        case 2: // UART1
            UBX_CFG_COMMON_UART1(CFG_PUSH)
            break;
        case 3: // UART2
            UBX_CFG_COMMON_UART2(CFG_PUSH)
            break;
        default:
            break;
        }
    }

    // push variable settings
    packed_cfg.push<ConfigKey::CFG_RATE_MEAS>(_cfg.meas_rate, (uint16_t)ubx_backend.params.rate_ms);
    packed_cfg.push<ConfigKey::CFG_NAVSPG_DYNMODEL>(_cfg.dynmodel, (uint8_t)ubx_backend.gps._navfilter);
    if (ubx_backend.gps._min_elevation != -100) {
        packed_cfg.push<ConfigKey::CFG_NAVSPG_INFIL_MINELEV>((uint8_t)_cfg.min_elevation, (uint8_t)ubx_backend.gps._min_elevation);
    }
    if (_cfg.gps_l5_health_ovrd_exists) {
        packed_cfg.push<ConfigKey::CFG_SIGNAL_L5_HEALTH_OVRD>((uint8_t)_cfg.gps_l5_health_ovrd, (uint8_t)ubx_backend.gps.option_set(AP_GPS::GPSL5HealthOverride));
    }
    const uint8_t *bytes = packed_cfg.get();
    const uint16_t size = (uint16_t)packed_cfg.get_size();
    if (size == 0) {
        return false;
    }

    return _send_valset_bytes(bytes, size, ConfigLayer::ALL);
}

// Append value bytes in little-endian order to dest at index, incrementing index
void AP_GPS_UBLOX_CFGv2::UBXPackedCfg::append_value_le_bytes(uint8_t *dest, size_t &index, uint64_t value, size_t size) {
    if (size >= 1) {
        dest[index++] = (uint8_t)(((uint64_t)value) & 0xFF);
    }
    if (size >= 2) {
        dest[index++] = (uint8_t)((((uint64_t)value) >> 8) & 0xFF);
    }
    if (size >= 4) {
        dest[index++] = (uint8_t)((((uint64_t)value) >> 16) & 0xFF);
        dest[index++] = (uint8_t)((((uint64_t)value) >> 24) & 0xFF);
    }
    if (size >= 8) {
        dest[index++] = (uint8_t)((((uint64_t)value) >> 32) & 0xFF);
        dest[index++] = (uint8_t)((((uint64_t)value) >> 40) & 0xFF);
        dest[index++] = (uint8_t)((((uint64_t)value) >> 48) & 0xFF);
        dest[index++] = (uint8_t)((((uint64_t)value) >> 56) & 0xFF);
    }
}

// Push raw value for a key (length is inferred from key's size bits)
bool AP_GPS_UBLOX_CFGv2::UBXPackedCfg::push(ConfigKey key, uint64_t value)
{
    Debug("GPS: try packed push key 0x%08x value 0x%llx", (uint32_t)key, (uint64_t)value);
    size_t len = config_key_size(key);
    if (_size + sizeof(key) + len > _capacity) {
        return false;
    }
    size_t index = _size;

    append_value_le_bytes<uint32_t>(_buf, index, (uint32_t)key);

    switch (len) {
        case 1:
            append_value_le_bytes<uint8_t>(_buf, index, (uint8_t)value);
            break;
        case 2:
            append_value_le_bytes<uint16_t>(_buf, index, (uint16_t)value);
            break;
        case 4:
            append_value_le_bytes<uint32_t>(_buf, index, (uint32_t)value);
            break;
        case 8:
            append_value_le_bytes<uint64_t>(_buf, index, (uint64_t)value);
            break;
        default:
            return false;
    }
    _size = index;
    return true;
}