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

#include "AP_EZKontrolCAN.h"
#include "AP_EZKontrolCAN_protocol.h"

#include <AP_Common/AP_Common.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_EZKontrolCAN *AP_EZKontrolCAN::_singleton;

const AP_Param::GroupInfo AP_EZKontrolCAN::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: EZKontrol CAN enable
    // @Description: Enable EZKontrol CAN motor backend
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, AP_EZKontrolCAN, _enable, 0),

    // @Param: CAN_PORT
    // @DisplayName: EZKontrol CAN port
    // @Description: CAN port index used for EZKontrol (1 or 2)
    // @Values: 1:CAN1,2:CAN2
    // @User: Advanced
    AP_GROUPINFO("CAN_PORT", 2, AP_EZKontrolCAN, _can_port, 1),

    // @Param: BITRATE
    // @DisplayName: EZKontrol CAN bitrate
    // @Description: CAN bitrate in kbit/s (controller expects protocol 2 for 250k, protocol 102 for 500k)
    // @Values: 250:250k,500:500k
    // @User: Advanced
    AP_GROUPINFO("BITRATE", 3, AP_EZKontrolCAN, _bitrate, 250),

    // @Param: ADDR_L
    // @DisplayName: EZKontrol left motor address
    // @Description: Left motor node address
    // @User: Advanced
    AP_GROUPINFO("ADDR_L", 4, AP_EZKontrolCAN, _addr_left, 0xEF),

    // @Param: ADDR_R
    // @DisplayName: EZKontrol right motor address
    // @Description: Right motor node address
    // @User: Advanced
    AP_GROUPINFO("ADDR_R", 5, AP_EZKontrolCAN, _addr_right, 0xF0),

    // @Param: MODE
    // @DisplayName: EZKontrol control mode
    // @Description: Control mode for EZKontrol commands
    // @Values: 0:Speed,1:Torque
    // @User: Advanced
    AP_GROUPINFO("MODE", 6, AP_EZKontrolCAN, _mode, 0),

    // @Param: MAX_RPM
    // @DisplayName: EZKontrol maximum RPM
    // @Description: Maximum motor RPM used for scaling
    // @Range: 0 5000
    // @User: Advanced
    AP_GROUPINFO("MAX_RPM", 7, AP_EZKontrolCAN, _max_rpm, 0),

    // @Param: MAX_CURR_A
    // @DisplayName: EZKontrol maximum current
    // @Description: Maximum motor current in amps used for scaling
    // @Units: A
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("MAX_CURR_A", 8, AP_EZKontrolCAN, _max_curr_a, 0.0f),

    // @Param: DEBUG
    // @DisplayName: EZKontrol debug messages
    // @Description: Enable periodic GCS debug text
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("DEBUG", 9, AP_EZKontrolCAN, _debug, 0),

    // @Param: TIMEOUT_MS
    // @DisplayName: EZKontrol telemetry timeout
    // @Description: Telemetry timeout in milliseconds for EZKontrol controllers
    // @Units: ms
    // @Range: 100 5000
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT_MS", 10, AP_EZKontrolCAN, _timeout_ms, 500),

    AP_GROUPEND
};

AP_EZKontrolCAN::AP_EZKontrolCAN() :
    _can_iface(nullptr),
    _can_inited(false),
    _healthy(false),
    _left_target(0.0f),
    _right_target(0.0f),
    _armed(false),
    _last_target_ms(0),
    _last_update_ms(0),
    _last_debug_ms(0),
    _last_send_ms(0),
    _rx_count(0),
    _last_rx_debug_count(0),
    _last_rx_id(0),
    _last_rx_dlc(0)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

AP_EZKontrolCAN *AP_EZKontrolCAN::get_singleton()
{
    return _singleton;
}

void AP_EZKontrolCAN::init()
{
    _can_iface = nullptr;
    _can_inited = false;
    _healthy = false;
    reset_controller_state(_left_state);
    reset_controller_state(_right_state);

    if (!enabled()) {
        return;
    }

    const int8_t port = _can_port.get();
    if (port < 1 || port > HAL_NUM_CAN_IFACES) {
        return;
    }

    _can_iface = hal.can[port - 1];
    if (_can_iface == nullptr) {
        return;
    }

    const int16_t bitrate_kbps = _bitrate.get();
    const uint32_t bitrate_bps = (bitrate_kbps == 250) ? 250000U : 500000U;

    // TODO Stage 2: configure filters and protocol-specific options.
    _can_inited = _can_iface->init(bitrate_bps);
    _healthy = _can_inited;

    _left_state.address = uint8_t(_addr_left.get());
    _right_state.address = uint8_t(_addr_right.get());
    const uint32_t now_ms = AP_HAL::millis();
    _left_state.last_handshake_ms = now_ms;
    _right_state.last_handshake_ms = now_ms;

    if (_debug.get() != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EZK CAN%u %uk init:%u L:0x%02x R:0x%02x",
                      unsigned(port),
                      unsigned(bitrate_kbps),
                      _can_inited ? 1U : 0U,
                      unsigned(_left_state.address),
                      unsigned(_right_state.address));
    }
}

void AP_EZKontrolCAN::set_targets(float left_norm, float right_norm, bool armed)
{
    _left_target = constrain_float(left_norm, -1.0f, 1.0f);
    _right_target = constrain_float(right_norm, -1.0f, 1.0f);
    _armed = armed;
    _last_target_ms = AP_HAL::millis();
}

void AP_EZKontrolCAN::update()
{
    _last_update_ms = AP_HAL::millis();

    if (!enabled()) {
        return;
    }

    if (_debug.get() != 0) {
        const uint32_t now_ms = _last_update_ms;
        if (now_ms - _last_debug_ms >= 1000U) {
            _last_debug_ms = now_ms;
            const uint32_t left_telem_age_ms = _left_state.last_telem_ms == 0 ? 9999U : MIN<uint32_t>(now_ms - _left_state.last_telem_ms, 9999U);
            const uint32_t right_telem_age_ms = _right_state.last_telem_ms == 0 ? 9999U : MIN<uint32_t>(now_ms - _right_state.last_telem_ms, 9999U);
            const uint32_t rx_delta = _rx_count - _last_rx_debug_count;
            _last_rx_debug_count = _rx_count;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "EZK hs L:%u R:%u tlm L:%lu R:%lu",
                          _left_state.handshake_complete ? 1U : 0U,
                          _right_state.handshake_complete ? 1U : 0U,
                          (unsigned long)left_telem_age_ms,
                          (unsigned long)right_telem_age_ms);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "EZK rx:%lu init:%u id:%08lx dlc:%u",
                          (unsigned long)rx_delta,
                          _can_inited ? 1U : 0U,
                          (unsigned long)_last_rx_id,
                          unsigned(_last_rx_dlc));
        }
    }

    if (!_can_inited || _can_iface == nullptr) {
        _healthy = false;
        return;
    }

    for (uint16_t i = 0; i < 50; i++) {
        AP_HAL::CANFrame frame {};
        uint64_t timestamp_us = 0;
        AP_HAL::CANIface::CanIOFlags flags {};
        const int16_t read = _can_iface->receive(frame, timestamp_us, flags);
        if (read != 1) {
            break;
        }
        _rx_count++;
        _last_rx_id = frame.id;
        _last_rx_dlc = frame.dlc;
        handle_rx_frame(frame);
    }

    const uint32_t now_ms = _last_update_ms;
    if (_last_send_ms == 0) {
        _last_send_ms = now_ms;
    }
    while (now_ms - _last_send_ms >= AP_EZKontrolCAN_Protocol::HEARTBEAT_PERIOD_MS) {
        _last_send_ms += AP_EZKontrolCAN_Protocol::HEARTBEAT_PERIOD_MS;
        if (_left_state.handshake_complete) {
            send_command(_left_state, _left_target, _armed);
        }
        if (_right_state.handshake_complete) {
            send_command(_right_state, _right_target, _armed);
        }
    }

    update_health();
}

bool AP_EZKontrolCAN::enabled() const
{
    return _enable.get() != 0;
}

bool AP_EZKontrolCAN::healthy() const
{
    return !enabled() || _healthy;
}

AP_EZKontrolCAN *AP::ezkontrol_can()
{
    return AP_EZKontrolCAN::get_singleton();
}

void AP_EZKontrolCAN::reset_controller_state(ControllerState &state)
{
    state.handshake_complete = false;
    state.life_counter = 0;
    state.last_handshake_ms = 0;
    state.last_telem_ms = 0;
    state.last_handshake_warn_ms = 0;
    state.last_telem_warn_ms = 0;
    state.last_fault_warn_ms = 0;
    state.last_error_bits = 0;
    state.debug_handshake_reported = false;
    state.debug_telem_reported = false;
    state.telem = {};
}

AP_EZKontrolCAN::ControllerState *AP_EZKontrolCAN::find_state_by_address(uint8_t address)
{
    if (address == _left_state.address) {
        return &_left_state;
    }
    if (address == _right_state.address) {
        return &_right_state;
    }
    return nullptr;
}

void AP_EZKontrolCAN::handle_rx_frame(const AP_HAL::CANFrame &frame)
{
    if ((frame.id & AP_HAL::CANFrame::FlagEFF) == 0) {
        return;
    }

    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;
    const uint8_t priority = (id >> 24) & 0xFF;
    const uint8_t message = (id >> 16) & 0xFF;
    const uint8_t dest = (id >> 8) & 0xFF;
    const uint8_t src = id & 0xFF;

    if (dest != AP_EZKontrolCAN_Protocol::VCU_ADDRESS) {
        return;
    }

    ControllerState *state = find_state_by_address(src);
    if (state == nullptr) {
        return;
    }

    if (message == AP_EZKontrolCAN_Protocol::MSG_HANDSHAKE &&
        priority == AP_EZKontrolCAN_Protocol::PRIORITY_MCU_TO_VCU &&
        frame.dlc == AP_HAL::CANFrame::NonFDCANMaxDataLen) {
        bool valid = true;
        for (uint8_t i = 0; i < frame.dlc; i++) {
            if (frame.data[i] != AP_EZKontrolCAN_Protocol::HANDSHAKE_START) {
                valid = false;
                break;
            }
        }
        if (valid) {
            send_handshake_ack(src);
            state->handshake_complete = true;
            state->last_handshake_ms = _last_update_ms;
            if (_debug.get() != 0 && !state->debug_handshake_reported) {
                state->debug_handshake_reported = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EZK hs req addr 0x%02x ACK",
                              unsigned(state->address));
            }
            return;
        }
    }

    if (message == AP_EZKontrolCAN_Protocol::MSG_TELEM_1 &&
        priority == AP_EZKontrolCAN_Protocol::PRIORITY_MCU_TO_VCU) {
        state->handshake_complete = true;
        handle_telemetry_1(*state, frame);
        return;
    }

    if (message == AP_EZKontrolCAN_Protocol::MSG_TELEM_2 &&
        priority == AP_EZKontrolCAN_Protocol::PRIORITY_MCU_TO_VCU) {
        state->handshake_complete = true;
        handle_telemetry_2(*state, frame);
        return;
    }
}

void AP_EZKontrolCAN::handle_telemetry_1(ControllerState &state, const AP_HAL::CANFrame &frame)
{
    if (frame.dlc < 8) {
        return;
    }
    const uint16_t bus_voltage_raw = uint16_t(frame.data[0]) | (uint16_t(frame.data[1]) << 8);
    const uint16_t bus_current_raw = uint16_t(frame.data[2]) | (uint16_t(frame.data[3]) << 8);
    const uint16_t phase_current_raw = uint16_t(frame.data[4]) | (uint16_t(frame.data[5]) << 8);
    const uint16_t speed_raw = uint16_t(frame.data[6]) | (uint16_t(frame.data[7]) << 8);

    state.telem.bus_voltage = bus_voltage_raw * 0.1f;
    state.telem.bus_current = bus_current_raw * 0.1f - 3200.0f;
    state.telem.phase_current = phase_current_raw * 0.1f - 3200.0f;
    state.telem.speed_rpm = float(speed_raw) - 32000.0f;
    state.telem.telem_1_valid = true;
    state.telem.valid = true;
    state.last_telem_ms = _last_update_ms;
    publish_esc_telem(state);
    if (_debug.get() != 0 && !state.debug_telem_reported) {
        state.debug_telem_reported = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EZK telem active addr 0x%02x",
                      unsigned(state.address));
    }
}

void AP_EZKontrolCAN::handle_telemetry_2(ControllerState &state, const AP_HAL::CANFrame &frame)
{
    if (frame.dlc < 8) {
        return;
    }
    const int16_t temp_mos = int16_t(frame.data[0]) - 40;
    const int16_t temp_motor = int16_t(frame.data[1]) - 40;
    const uint16_t status_bits = frame.data[2];
    const uint32_t error_bits = uint32_t(frame.data[3]) |
                                (uint32_t(frame.data[4]) << 8) |
                                (uint32_t(frame.data[5]) << 16);

    state.telem.temp_mos_c = temp_mos;
    state.telem.temp_motor_c = temp_motor;
    state.telem.status_bits = status_bits;
    state.telem.error_bits = error_bits;
    state.telem.telem_2_valid = true;
    state.telem.valid = true;
    state.last_telem_ms = _last_update_ms;
    publish_esc_telem(state);
    if (_debug.get() != 0 && !state.debug_telem_reported) {
        state.debug_telem_reported = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EZK telem active addr 0x%02x",
                      unsigned(state.address));
    }

    if (error_bits != 0 && error_bits != state.last_error_bits) {
        state.last_error_bits = error_bits;
        if (_last_update_ms - state.last_fault_warn_ms >= 1000U) {
            state.last_fault_warn_ms = _last_update_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "EZK fault 0x%06x on addr 0x%02x",
                          static_cast<unsigned int>(error_bits & 0xFFFFFFU),
                          static_cast<unsigned int>(state.address));
        }
    }
}

void AP_EZKontrolCAN::send_handshake_ack(uint8_t dest_addr)
{
    AP_HAL::CANFrame frame {};
    frame.id = AP_HAL::CANFrame::FlagEFF |
               AP_EZKontrolCAN_Protocol::make_extended_id(
                   AP_EZKontrolCAN_Protocol::PRIORITY_VCU_TO_MCU,
                   AP_EZKontrolCAN_Protocol::MSG_HANDSHAKE,
                   dest_addr,
                   AP_EZKontrolCAN_Protocol::VCU_ADDRESS);
    frame.dlc = AP_HAL::CANFrame::NonFDCANMaxDataLen;
    for (uint8_t i = 0; i < frame.dlc; i++) {
        frame.data[i] = AP_EZKontrolCAN_Protocol::HANDSHAKE_ACK;
    }

    _can_iface->send(frame, AP_HAL::micros64() + 1000U, AP_HAL::CANIface::AbortOnError);
}

void AP_EZKontrolCAN::send_command(ControllerState &state, float target_norm, bool armed)
{
    AP_HAL::CANFrame frame {};
    frame.id = AP_HAL::CANFrame::FlagEFF |
               AP_EZKontrolCAN_Protocol::make_extended_id(
                   AP_EZKontrolCAN_Protocol::PRIORITY_VCU_TO_MCU,
                   AP_EZKontrolCAN_Protocol::MSG_COMMAND,
                   state.address,
                   AP_EZKontrolCAN_Protocol::VCU_ADDRESS);
    frame.dlc = AP_HAL::CANFrame::NonFDCANMaxDataLen;

    const float max_current_a = MAX<float>(_max_curr_a.get(), 0.0f);
    const float max_speed_rpm = MAX<float>(float(_max_rpm.get()), 0.0f);
    const float target_sign = is_negative(target_norm) ? -1.0f : 1.0f;
    const bool has_target = !is_zero(target_norm);
    float target_current_a = 0.0f;
    float target_speed_rpm = 0.0f;
    if (armed) {
        if (_mode.get() == 0) {
            target_current_a = has_target ? target_sign * max_current_a : 0.0f;
            target_speed_rpm = target_norm * max_speed_rpm;
        } else {
            target_current_a = target_norm * max_current_a;
            target_speed_rpm = has_target ? target_sign * max_speed_rpm : 0.0f;
        }
    }

    const int32_t raw_current = constrain_int32(lroundf((target_current_a + 3200.0f) * 10.0f), 0, 65535);
    const int32_t raw_speed = constrain_int32(lroundf(target_speed_rpm + 32000.0f), 0, 65535);

    frame.data[0] = uint8_t(raw_current & 0xFF);
    frame.data[1] = uint8_t(raw_current >> 8);
    frame.data[2] = uint8_t(raw_speed & 0xFF);
    frame.data[3] = uint8_t(raw_speed >> 8);

    uint8_t flags = 0;
    if (armed) {
        flags |= AP_EZKontrolCAN_Protocol::COMMAND_FLAG_RUN;
    }
    if (_mode.get() == 0) {
        flags |= AP_EZKontrolCAN_Protocol::COMMAND_FLAG_SPEED_MODE;
    }
    frame.data[4] = flags;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = state.life_counter;

    state.life_counter++;

    _can_iface->send(frame, AP_HAL::micros64() + 1000U, AP_HAL::CANIface::AbortOnError);
}

void AP_EZKontrolCAN::publish_esc_telem(const ControllerState &state) const
{
#if HAL_WITH_ESC_TELEM
    if (!state.telem.valid) {
        return;
    }

    const uint8_t esc_index = (&state == &_right_state) ? 1U : 0U;

    if (state.telem.telem_1_valid) {
        AP::esc_telem().update_rpm(esc_index, state.telem.speed_rpm, 0.0f);
    }

    AP_ESC_Telem_Backend::TelemetryData esc_telem {};
    uint16_t data_mask = 0;

    if (state.telem.telem_1_valid) {
        esc_telem.voltage = state.telem.bus_voltage;
        esc_telem.current = state.telem.bus_current;
        data_mask |= AP_ESC_Telem_Backend::TelemetryType::VOLTAGE |
                     AP_ESC_Telem_Backend::TelemetryType::CURRENT;
    }

    if (state.telem.telem_2_valid) {
        esc_telem.temperature_cdeg = int16_t(constrain_int32(lroundf(state.telem.temp_mos_c * 100.0f), INT16_MIN, INT16_MAX));
        esc_telem.motor_temp_cdeg = int16_t(constrain_int32(lroundf(state.telem.temp_motor_c * 100.0f), INT16_MIN, INT16_MAX));
        data_mask |= AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
                     AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE;

#if AP_EXTENDED_ESC_TELEM_ENABLED
        esc_telem.flags = (state.telem.error_bits << 8) | state.telem.status_bits;
        data_mask |= AP_ESC_Telem_Backend::TelemetryType::FLAGS;
#endif
    }

    if (data_mask != 0) {
        AP::esc_telem().update_telem_data(esc_index, esc_telem, data_mask);
    }
#endif
}

void AP_EZKontrolCAN::update_health()
{
    const uint32_t now_ms = _last_update_ms;
    const uint32_t timeout_ms = MAX<uint32_t>(100U, uint32_t(_timeout_ms.get()));
    const uint32_t handshake_warn_ms = 2000U;

    auto update_state = [&](ControllerState &state) {
        if (!state.handshake_complete) {
            if (now_ms - state.last_handshake_warn_ms >= 1000U &&
                state.last_handshake_ms != 0 &&
                now_ms - state.last_handshake_ms >= handshake_warn_ms) {
                state.last_handshake_warn_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "EZK handshake pending addr 0x%02x",
                              static_cast<unsigned int>(state.address));
            }
        } else {
            if (state.last_telem_ms != 0 && now_ms - state.last_telem_ms > timeout_ms) {
                if (now_ms - state.last_telem_warn_ms >= 1000U) {
                    state.last_telem_warn_ms = now_ms;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "EZK telemetry timeout addr 0x%02x",
                                  static_cast<unsigned int>(state.address));
                }
            }
        }
    };

    update_state(_left_state);
    update_state(_right_state);

    const bool left_ok = _left_state.handshake_complete &&
                         _left_state.last_telem_ms != 0 &&
                         (now_ms - _left_state.last_telem_ms) <= timeout_ms;
    const bool right_ok = _right_state.handshake_complete &&
                          _right_state.last_telem_ms != 0 &&
                          (now_ms - _right_state.last_telem_ms) <= timeout_ms;

    _healthy = _can_inited && left_ok && right_ok;
}
